// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos8895 S3CFB front-end: the fbdev ABI of the Samsung vendor DECON driver,
 * served as an in-kernel DRM client of the DECON0 CRTC. No DECON register is
 * touched here.
 */

#include <linux/build_bug.h>
#include <linux/compat.h>
#include <linux/dma-buf.h>
#include <linux/dma-fence.h>
#include <linux/fb.h>
#include <linux/file.h>
#include <linux/hrtimer.h>
#include <linux/ion_legacy.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/sync_file.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_uapi.h>
#include <drm/drm_blend.h>
#include <drm/drm_client.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_mode_config.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_lock.h>
#include <drm/drm_plane.h>
#include <drm/drm_prime.h>
#include <drm/drm_print.h>

#include "exynos8895_drm_decon.h"
#include "exynos8895_s3cfb.h"

/* the UAPI of the vendor DECON driver */

#define MAX_DECON_WIN		6
#define MAX_PLANE_CNT		3
#define MIN_WIN_BLOCK_WIDTH	8
#define MIN_WIN_BLOCK_HEIGHT	1
#define DECON_WIN_UPDATE_IDX	6
#define DECON_WIN_DSU_ID	(DECON_WIN_UPDATE_IDX + 1)
#define HDR_CAPA_NUM		4

enum {
	DECON_WIN_STATE_DISABLED = 0,
	DECON_WIN_STATE_COLOR,
	DECON_WIN_STATE_BUFFER,
	DECON_WIN_STATE_UPDATE,
};

enum decon_idma_type {
	IDMA_G0 = 0,	/* dedicated to window 5 */
	IDMA_G1,
	IDMA_VG0,
	IDMA_VG1,
	IDMA_VGF0,
	IDMA_VGF1,
};

enum decon_pixel_format {
	DECON_PIXEL_FORMAT_ARGB_8888 = 0,
	DECON_PIXEL_FORMAT_ABGR_8888,
	DECON_PIXEL_FORMAT_RGBA_8888,
	DECON_PIXEL_FORMAT_BGRA_8888,
	DECON_PIXEL_FORMAT_XRGB_8888,
	DECON_PIXEL_FORMAT_XBGR_8888,
	DECON_PIXEL_FORMAT_RGBX_8888,
	DECON_PIXEL_FORMAT_BGRX_8888,
	DECON_PIXEL_FORMAT_RGBA_5551,
	DECON_PIXEL_FORMAT_RGB_565,
	DECON_PIXEL_FORMAT_NV16,
	DECON_PIXEL_FORMAT_NV61,
	DECON_PIXEL_FORMAT_YVU422_3P,
	DECON_PIXEL_FORMAT_NV12,
	DECON_PIXEL_FORMAT_NV21,
	DECON_PIXEL_FORMAT_NV12M,
	DECON_PIXEL_FORMAT_NV21M,
	DECON_PIXEL_FORMAT_YUV420,
	DECON_PIXEL_FORMAT_YVU420,
	DECON_PIXEL_FORMAT_YUV420M,
	DECON_PIXEL_FORMAT_YVU420M,
	DECON_PIXEL_FORMAT_NV12N,
	DECON_PIXEL_FORMAT_NV12N_10B,
	DECON_PIXEL_FORMAT_MAX,
};

enum decon_blending {
	DECON_BLENDING_NONE = 0,
	DECON_BLENDING_PREMULT = 1,
	DECON_BLENDING_COVERAGE = 2,
	DECON_BLENDING_MAX = 3,
};

enum doze_mode {
	DECON_PWR_OFF = 0,
	DECON_PWR_DOZE,
	DECON_PWR_NORMAL,
	DECON_PWR_DOZE_SUSPEND,
	DECON_PWR_MAX,
};

/* The enum fields of the vendor structures are 4-byte unsigned, u32 here. */

struct decon_win_rect {
	int x;
	int y;
	u32 w;
	u32 h;
};

/* dma_addr_t is 64-bit on arm64 */
struct dpp_params {
	u64 addr[MAX_PLANE_CNT];
	u32 flip;		/* enum dpp_flip */
	u32 eq_mode;		/* enum dpp_csc_eq */
	u32 comp_src;		/* enum dpp_comp_src */
};

struct decon_frame {
	int x;
	int y;
	u32 w;
	u32 h;
	u32 f_w;
	u32 f_h;
};

struct decon_win_config {
	u32 state;
	union {
		u32 color;
		struct {
			int fd_idma[3];
			int fence_fd;
			int plane_alpha;
			u32 blending;		/* enum decon_blending */
			u32 idma_type;		/* enum decon_idma_type */
			u32 format;		/* enum decon_pixel_format */
			struct dpp_params dpp_parm;
			struct decon_win_rect block_area;
			struct decon_win_rect transparent_area;
			struct decon_win_rect opaque_area;
			struct decon_frame src;
		};
		struct {	/* DSU */
			int left;
			int top;
			int right;
			int bottom;
			int mode;
		};
	};
	struct decon_frame dst;
	bool protection;
	bool compression;
};

/* config[6] = window update, config[7] = DSU */
struct decon_win_config_data {
	int fence;
	int fd_odma;
	struct decon_win_config config[MAX_DECON_WIN + 2];
};

struct decon_hdr_capabilities {
	unsigned int out_types[HDR_CAPA_NUM];
};

struct decon_hdr_capabilities_info {
	int out_num;
	int max_luminance;
	int max_average_luminance;
	int min_luminance;
};

#define S3CFB_SET_VSYNC_INT		_IOW('F', 206, __u32)
#define S3CFB_DECON_SELF_REFRESH	_IOW('F', 207, __u32)
#define S3CFB_WIN_CONFIG		_IOW('F', 209, struct decon_win_config_data)
#define S3CFB_POWER_MODE		_IOW('F', 223, __u32)
#define S3CFB_GET_HDR_CAPABILITIES	_IOW('F', 400, struct decon_hdr_capabilities)
#define S3CFB_GET_HDR_CAPABILITIES_NUM	_IOW('F', 401, struct decon_hdr_capabilities_info)

/* the layout and the ioctl numbers of the vendor arm64 ABI */
static_assert(sizeof(struct dpp_params) == 40);
static_assert(sizeof(struct decon_win_config) == 184);
static_assert(sizeof(struct decon_win_config_data) == 1480);
static_assert(offsetof(struct decon_win_config, state) == 0);
static_assert(offsetof(struct decon_win_config, color) == 8);
static_assert(offsetof(struct decon_win_config, fd_idma) == 8);
static_assert(offsetof(struct decon_win_config, fence_fd) == 20);
static_assert(offsetof(struct decon_win_config, plane_alpha) == 24);
static_assert(offsetof(struct decon_win_config, blending) == 28);
static_assert(offsetof(struct decon_win_config, idma_type) == 32);
static_assert(offsetof(struct decon_win_config, format) == 36);
static_assert(offsetof(struct decon_win_config, dpp_parm) == 40);
static_assert(offsetof(struct decon_win_config, block_area) == 80);
static_assert(offsetof(struct decon_win_config, transparent_area) == 96);
static_assert(offsetof(struct decon_win_config, opaque_area) == 112);
static_assert(offsetof(struct decon_win_config, src) == 128);
static_assert(offsetof(struct decon_win_config, left) == 8);
static_assert(offsetof(struct decon_win_config, dst) == 152);
static_assert(offsetof(struct decon_win_config, protection) == 176);
static_assert(offsetof(struct decon_win_config, compression) == 177);
static_assert(offsetof(struct decon_win_config_data, config) == 8);
static_assert(sizeof(struct decon_hdr_capabilities) == 16);
static_assert(sizeof(struct decon_hdr_capabilities_info) == 16);
static_assert(S3CFB_SET_VSYNC_INT == 0x400446ce);
static_assert(S3CFB_DECON_SELF_REFRESH == 0x400446cf);
static_assert(S3CFB_WIN_CONFIG == 0x45c846d1);
static_assert(S3CFB_POWER_MODE == 0x400446df);
static_assert(S3CFB_GET_HDR_CAPABILITIES == 0x40104790);
static_assert(S3CFB_GET_HDR_CAPABILITIES_NUM == 0x40104791);

/* panel and front-end constants */

#define S3CFB_XRES		1440
#define S3CFB_YRES		2960
#define S3CFB_WIDTH_MM		71
#define S3CFB_HEIGHT_MM		145
#define S3CFB_BPP		32
#define S3CFB_PSR_MODE		2	/* command mode */
#define S3CFB_DSC_WIDTH		720
#define S3CFB_DSC_HEIGHT	40
#define S3CFB_DSC_EN		1
#define S3CFB_WIN_G0		5	/* the only window of G0 */

#define S3CFB_VSYNC_TIMEOUT_MS	200
#define S3CFB_FENCE_TIMEOUT_MS	900
#define S3CFB_SOFT_VSYNC_NS	16666667	/* one 60 Hz frame */
#define S3CFB_CACHE_SIZE	32
#define S3CFB_CACHE_IDLE	120	/* frames without use before eviction */

/* one imported gralloc buffer */
struct s3cfb_import {
	struct dma_buf *buf;		/* one reference; NULL: free slot */
	u32 handle;			/* GEM handle in client.file */
	u64 last_frame;
	/* framebuffer cached with the plane-0 entry, one reference */
	struct drm_framebuffer *fb;
	u32 fb_fourcc, fb_w, fb_h;
	struct dma_buf *fb_buf1;	/* second-plane buffer of fb, or NULL */
};

/* one window of a frame */
struct s3cfb_win {
	u32 state;			/* DECON_WIN_STATE_* */
	u32 ch;				/* idma_type = DPP channel = plane index */
	struct drm_framebuffer *fb;	/* BUFFER: one reference */
	struct dma_fence *acquire;	/* BUFFER: one reference, or NULL */
	struct decon_frame src, dst;
	u16 alpha;			/* DRM plane alpha */
	u16 blend;			/* DRM_MODE_BLEND_* */
	u32 argb;			/* COLOR */
	bool comp;			/* compression (AFBC) */
};

struct s3cfb;

struct s3cfb_frame {
	struct kthread_work work;
	struct s3cfb *s;
	struct s3cfb_win win[MAX_DECON_WIN];
	struct dsu_info dsu;
};

struct s3cfb_fence {
	struct dma_fence base;		/* first: dma_fence_free() frees this struct */
	struct list_head node;
};

struct s3cfb {
	struct kref ref;		/* the DRM client and the registered fb_info */
	struct drm_client_dev client;
	struct exynos8895_decon_handle h;
	struct drm_display_mode *mode;
	struct fb_info *info;
	struct platform_device *sdev;
	struct kernfs_node *vsync_kn;
	struct kthread_worker *worker;
	bool torn_down;

	/* ioctl side: power flag, import cache, frame count */
	struct mutex lock;
	bool power;			/* written under lock */
	bool dead;			/* torn down; written under the fb_info lock */
	bool dsr_on;
	u64 frame;
	struct s3cfb_import cache[S3CFB_CACHE_SIZE];

	/* release fence timeline */
	spinlock_t fence_lock;
	u64 fence_ctx;
	u64 tl_value, tl_max;
	struct list_head fences;	/* unsignalled, in seqno order, one reference each */
	bool release_armed;		/* a commit's frame start owes the previous frame's step */

	/* vsync */
	spinlock_t vs_lock;
	ktime_t vsync_ts;
	bool vsync_on;
	bool te_live;			/* a TE arrived since the CRTC was last enabled */
	bool soft_on;			/* the software vsync timer is armed */
	wait_queue_head_t vsync_wait;
	struct hrtimer soft;

	/* colour windows of the last successful commit (worker, or blank after a flush) */
	struct exynos8895_color_win cw[MAX_DECON_WIN];
	u8 cw_on;

	/* current DSU mode and active resolution */
	struct dsu_info dsu;
};

/* the one front-end (DECON0 has one fb0) */
static struct s3cfb *s3cfb_inst;

static inline struct s3cfb *s3cfb_of(struct fb_info *info)
{
	return *(struct s3cfb **)info->par;
}

static void s3cfb_free(struct kref *ref)
{
	struct s3cfb *s = container_of(ref, struct s3cfb, ref);

	mutex_destroy(&s->lock);
	kfree(s);
}

/* release fences */

static const char *s3cfb_fence_driver_name(struct dma_fence *f)
{
	return "exynos8895-s3cfb";
}

static const char *s3cfb_fence_timeline_name(struct dma_fence *f)
{
	return "decon0";
}

static const struct dma_fence_ops s3cfb_fence_ops = {
	.get_driver_name	= s3cfb_fence_driver_name,
	.get_timeline_name	= s3cfb_fence_timeline_name,
};

/*
 * A fence at the next timeline maximum. The timeline list keeps one
 * reference until it is signalled; the caller gets another.
 */
static struct dma_fence *s3cfb_fence_create(struct s3cfb *s)
{
	struct s3cfb_fence *f = kzalloc(sizeof(*f), GFP_KERNEL);
	unsigned long flags;

	if (!f)
		return NULL;
	spin_lock_irqsave(&s->fence_lock, flags);
	dma_fence_init(&f->base, &s3cfb_fence_ops, &s->fence_lock, s->fence_ctx, ++s->tl_max);
	list_add_tail(&f->node, &s->fences);
	dma_fence_get(&f->base);
	spin_unlock_irqrestore(&s->fence_lock, flags);

	return &f->base;
}

/*
 * Step the timeline by one, or with @all to its maximum (teardown: no frame
 * follows), and signal every fence whose seqno it reached.
 */
static void s3cfb_timeline_inc(struct s3cfb *s, bool all)
{
	struct s3cfb_fence *f, *tmp;
	unsigned long flags;
	LIST_HEAD(done);

	spin_lock_irqsave(&s->fence_lock, flags);
	if (all)
		s->tl_value = s->tl_max;
	else if (s->tl_value < s->tl_max)
		s->tl_value++;
	list_for_each_entry_safe(f, tmp, &s->fences, node) {
		if (f->base.seqno > s->tl_value)
			break;
		dma_fence_signal_locked(&f->base);
		list_move_tail(&f->node, &done);
	}
	spin_unlock_irqrestore(&s->fence_lock, flags);

	list_for_each_entry_safe(f, tmp, &done, node) {
		list_del(&f->node);
		dma_fence_put(&f->base);
	}
}

/*
 * The previous frame's buffers are released at the next frame's start, not
 * when the blocking commit returns a panel transfer later. If no frame
 * starts, the worker takes the step after its commit instead.
 */
static void s3cfb_release_arm(struct s3cfb *s)
{
	unsigned long flags;

	spin_lock_irqsave(&s->fence_lock, flags);
	s->release_armed = true;
	spin_unlock_irqrestore(&s->fence_lock, flags);
}

static bool s3cfb_release_take(struct s3cfb *s)
{
	unsigned long flags;
	bool armed;

	spin_lock_irqsave(&s->fence_lock, flags);
	armed = s->release_armed;
	s->release_armed = false;
	spin_unlock_irqrestore(&s->fence_lock, flags);
	return armed;
}

/* hard interrupt context */
static void s3cfb_frame_start(void *data)
{
	struct s3cfb *s = data;

	if (s3cfb_release_take(s))
		s3cfb_timeline_inc(s, false);
}

/*
 * The fence of one WIN_CONFIG as a sync_file on a reserved fd, installed only
 * after the copy-back; no O_CLOEXEC.
 */
static int s3cfb_fence_fd(struct s3cfb *s, struct sync_file **sfp)
{
	struct dma_fence *f = s3cfb_fence_create(s);
	struct sync_file *sf;
	int fd;

	if (!f)
		return -ENOMEM;
	sf = sync_file_create(f);
	dma_fence_put(f);
	if (!sf)
		return -ENOMEM;
	fd = get_unused_fd_flags(0);
	if (fd < 0) {
		fput(sf->file);
		return fd;
	}
	*sfp = sf;

	return fd;
}

/* vsync */

/* the TE time, its waiters and, while vsync is on, a POLLPRI on "vsync" */
static void s3cfb_vsync_event_locked(struct s3cfb *s, ktime_t ts)
{
	s->vsync_ts = ts;
	wake_up_interruptible_all(&s->vsync_wait);
	if (s->vsync_on)
		kernfs_notify(s->vsync_kn);	/* any context */
}

/* from the TE hard interrupt */
static void s3cfb_te(void *data, ktime_t ts)
{
	struct s3cfb *s = data;
	unsigned long flags;

	spin_lock_irqsave(&s->vs_lock, flags);
	s->te_live = true;
	s3cfb_vsync_event_locked(s, ts);
	spin_unlock_irqrestore(&s->vs_lock, flags);
}

/*
 * Software vsync while powered but not scanning out: the vendor driver had
 * the panel on at unblank, so TE flowed before the first frame. It stops at
 * the first TE and on blank.
 */
static enum hrtimer_restart s3cfb_soft_vsync(struct hrtimer *t)
{
	struct s3cfb *s = container_of(t, struct s3cfb, soft);
	unsigned long flags;
	bool run;

	spin_lock_irqsave(&s->vs_lock, flags);
	run = READ_ONCE(s->power) && s->vsync_on && !s->te_live;
	if (run) {
		s3cfb_vsync_event_locked(s, ktime_get());
		hrtimer_forward_now(t, ns_to_ktime(S3CFB_SOFT_VSYNC_NS));
	} else {
		s->soft_on = false;
	}
	spin_unlock_irqrestore(&s->vs_lock, flags);

	return run ? HRTIMER_RESTART : HRTIMER_NORESTART;
}

static void s3cfb_soft_vsync_update(struct s3cfb *s)
{
	unsigned long flags;

	spin_lock_irqsave(&s->vs_lock, flags);
	if (READ_ONCE(s->power) && s->vsync_on && !s->te_live && !s->soft_on) {
		s->soft_on = true;
		hrtimer_start(&s->soft, ns_to_ktime(S3CFB_SOFT_VSYNC_NS), HRTIMER_MODE_REL);
	}
	spin_unlock_irqrestore(&s->vs_lock, flags);
}

static ktime_t s3cfb_vsync_ts(struct s3cfb *s)
{
	unsigned long flags;
	ktime_t ts;

	spin_lock_irqsave(&s->vs_lock, flags);
	ts = s->vsync_ts;
	spin_unlock_irqrestore(&s->vs_lock, flags);

	return ts;
}

/*
 * 0 at once while off; else the next timestamp within 200 ms, -ETIMEDOUT
 * otherwise. An interrupted wait returns 0, as in the vendor driver.
 */
static int s3cfb_wait_for_vsync(struct s3cfb *s)
{
	ktime_t ts;
	long ret;

	if (!READ_ONCE(s->power))
		return 0;
	ts = s3cfb_vsync_ts(s);
	ret = wait_event_interruptible_timeout(s->vsync_wait, s3cfb_vsync_ts(s) != ts,
					       msecs_to_jiffies(S3CFB_VSYNC_TIMEOUT_MS));

	return ret == 0 ? -ETIMEDOUT : 0;
}

/* import cache and framebuffers */

/*
 * Drop the cache's references of @e: its framebuffer, any framebuffer that uses
 * its buffer as second plane, its GEM handle and its dma-buf. A frame queued or
 * on screen holds its own framebuffer reference, so nothing in use is freed.
 */
static void s3cfb_evict(struct s3cfb *s, struct s3cfb_import *e)
{
	int i;

	for (i = 0; i < S3CFB_CACHE_SIZE; i++) {
		struct s3cfb_import *o = &s->cache[i];

		if (o->fb && o->fb_buf1 == e->buf) {
			drm_framebuffer_put(o->fb);
			o->fb = NULL;
		}
	}
	if (e->fb)
		drm_framebuffer_put(e->fb);
	drm_gem_handle_delete(s->client.file, e->handle);
	dma_buf_put(e->buf);
	memset(e, 0, sizeof(*e));
}

static void s3cfb_cache_flush(struct s3cfb *s)
{
	int i;

	for (i = 0; i < S3CFB_CACHE_SIZE; i++)
		if (s->cache[i].buf)
			s3cfb_evict(s, &s->cache[i]);
}

/* entries unused for S3CFB_CACHE_IDLE frames */
static void s3cfb_cache_age(struct s3cfb *s)
{
	int i;

	for (i = 0; i < S3CFB_CACHE_SIZE; i++)
		if (s->cache[i].buf && s->frame - s->cache[i].last_frame > S3CFB_CACHE_IDLE)
			s3cfb_evict(s, &s->cache[i]);
}

/*
 * The cache keeps each buffer mapped across frames, so the exporter cleans it
 * only at the first use. A CACHED ION buffer without CACHED_NEEDS_SYNC is never
 * synced by user space; the vendor ION cleaned it at every map, so do the same.
 */
static void s3cfb_autosync(struct dma_buf *buf)
{
	if (ionl_dmabuf_is_autosync(buf))
		dma_buf_end_cpu_access(buf, DMA_TO_DEVICE);
}

/*
 * The cache entry of dma-buf fd @fd: a hit reuses its handle; a miss imports it
 * once through the GEM PRIME import on the DECON device, evicting the least
 * recently used entry when the cache is full.
 */
static struct s3cfb_import *s3cfb_import(struct s3cfb *s, int fd)
{
	struct drm_device *drm = s->h.drm;
	struct s3cfb_import *e = NULL, *lru = NULL;
	struct drm_gem_object *obj;
	struct dma_buf *buf;
	u32 handle;
	bool same;
	int i, ret;

	buf = dma_buf_get(fd);
	if (IS_ERR(buf))
		return ERR_CAST(buf);

	for (i = 0; i < S3CFB_CACHE_SIZE; i++) {
		struct s3cfb_import *c = &s->cache[i];

		if (c->buf == buf) {
			dma_buf_put(buf);
			c->last_frame = s->frame;
			s3cfb_autosync(c->buf);
			return c;
		}
		if (!c->buf) {
			if (!e)
				e = c;
		} else if (!lru || c->last_frame < lru->last_frame) {
			lru = c;
		}
	}
	if (!e) {
		/* never an entry of the frame being built: it has at most 12 */
		if (lru->last_frame == s->frame) {
			dma_buf_put(buf);
			return ERR_PTR(-EBUSY);
		}
		s3cfb_evict(s, lru);
		e = lru;
	}

	ret = drm_gem_prime_fd_to_handle(drm, s->client.file, fd, &handle);
	if (ret) {
		dma_buf_put(buf);
		return ERR_PTR(ret);
	}
	/* the fd may have been replaced since dma_buf_get(): the handle must be of @buf */
	obj = drm_gem_object_lookup(s->client.file, handle);
	same = obj && obj->dma_buf == buf;
	if (obj)
		drm_gem_object_put(obj);
	if (!same) {
		drm_gem_handle_delete(s->client.file, handle);
		dma_buf_put(buf);
		return ERR_PTR(-EINVAL);
	}

	e->buf = buf;
	e->handle = handle;
	e->last_frame = s->frame;
	s3cfb_autosync(buf);

	return e;
}

/*
 * The vendor driver scanned each DECON format out with the IDMA code of
 * another name (ARGB_8888 with BGRA8888), so the DRM format is that name.
 * @bpp covers the luma plane only; NV12N is one fd, CbCr after the luma.
 */
static const struct s3cfb_fmt {
	u32 fourcc;
	u8 nfd;
	u8 cpp;
	u8 bpp;
	u8 nv12n;
} s3cfb_formats[DECON_PIXEL_FORMAT_MAX] = {
	[DECON_PIXEL_FORMAT_ARGB_8888] = { DRM_FORMAT_BGRA8888, 1, 4, 4 },
	[DECON_PIXEL_FORMAT_ABGR_8888] = { DRM_FORMAT_RGBA8888, 1, 4, 4 },
	[DECON_PIXEL_FORMAT_RGBA_8888] = { DRM_FORMAT_ABGR8888, 1, 4, 4 },
	[DECON_PIXEL_FORMAT_BGRA_8888] = { DRM_FORMAT_ARGB8888, 1, 4, 4 },
	[DECON_PIXEL_FORMAT_XRGB_8888] = { DRM_FORMAT_BGRX8888, 1, 4, 4 },
	[DECON_PIXEL_FORMAT_XBGR_8888] = { DRM_FORMAT_RGBX8888, 1, 4, 4 },
	[DECON_PIXEL_FORMAT_RGBX_8888] = { DRM_FORMAT_XBGR8888, 1, 4, 4 },
	[DECON_PIXEL_FORMAT_BGRX_8888] = { DRM_FORMAT_XRGB8888, 1, 4, 4 },
	[DECON_PIXEL_FORMAT_RGB_565]   = { DRM_FORMAT_RGB565, 1, 2, 2 },
	[DECON_PIXEL_FORMAT_NV16]      = { DRM_FORMAT_NV16, 2, 1, 2 },
	[DECON_PIXEL_FORMAT_NV61]      = { DRM_FORMAT_NV61, 2, 1, 2 },
	[DECON_PIXEL_FORMAT_NV12]      = { DRM_FORMAT_NV12, 2, 1, 1 },
	[DECON_PIXEL_FORMAT_NV21]      = { DRM_FORMAT_NV21, 2, 1, 1 },
	[DECON_PIXEL_FORMAT_NV12M]     = { DRM_FORMAT_NV12, 2, 1, 1 },
	[DECON_PIXEL_FORMAT_NV21M]     = { DRM_FORMAT_NV21, 2, 1, 1 },
	[DECON_PIXEL_FORMAT_NV12N]     = { DRM_FORMAT_NV12, 1, 1, 1, 1 },
};

#define S3CFB_NV12N_Y_SIZE(w, h)	(ALIGN((w), 16) * ALIGN((h), 16) + 256)

/*
 * The framebuffer of a buffer window, created with the driver's own
 * fb_create, which checks each plane against its GEM size, and cached with
 * the plane-0 entry. Returns a reference for the frame.
 */
static struct drm_framebuffer *s3cfb_fb(struct s3cfb *s, const struct decon_win_config *c,
					const struct s3cfb_fmt *f, struct s3cfb_import **e)
{
	struct drm_device *drm = s->h.drm;
	struct dma_buf *buf1 = f->nfd == 2 ? e[1]->buf : NULL;
	struct drm_mode_fb_cmd2 cmd = { };
	struct drm_framebuffer *fb;
	int i;

	if (e[0]->fb && e[0]->fb_fourcc == f->fourcc && e[0]->fb_w == c->src.f_w &&
	    e[0]->fb_h == c->src.f_h && e[0]->fb_buf1 == buf1) {
		drm_framebuffer_get(e[0]->fb);
		return e[0]->fb;
	}

	cmd.width = c->src.f_w;
	cmd.height = c->src.f_h;
	cmd.pixel_format = f->fourcc;
	for (i = 0; i < f->nfd; i++) {
		cmd.handles[i] = e[i]->handle;
		cmd.pitches[i] = c->src.f_w * (i ? 1 : f->cpp);
	}
	if (f->nv12n) {
		cmd.handles[1] = e[0]->handle;
		cmd.pitches[1] = c->src.f_w;
		cmd.offsets[1] = S3CFB_NV12N_Y_SIZE(c->src.f_w, c->src.f_h);
	}
	fb = drm->mode_config.funcs->fb_create(drm, s->client.file,
					       drm_get_format_info(drm, f->fourcc,
								   DRM_FORMAT_MOD_LINEAR),
					       &cmd);
	if (IS_ERR(fb))
		return fb;

	if (e[0]->fb)
		drm_framebuffer_put(e[0]->fb);
	e[0]->fb = fb;
	e[0]->fb_fourcc = f->fourcc;
	e[0]->fb_w = c->src.f_w;
	e[0]->fb_h = c->src.f_h;
	e[0]->fb_buf1 = buf1;
	drm_framebuffer_get(fb);

	return fb;
}

/* WIN_CONFIG */

static const u16 s3cfb_blend[DECON_BLENDING_MAX] = {
	[DECON_BLENDING_NONE]		= DRM_MODE_BLEND_PIXEL_NONE,
	[DECON_BLENDING_PREMULT]	= DRM_MODE_BLEND_PREMULTI,
	[DECON_BLENDING_COVERAGE]	= DRM_MODE_BLEND_COVERAGE,
};

/*
 * The checks for every window: channel, known blending, plane alpha 0..255,
 * a destination right of and below 0,0 at least 16 wide.
 */
static int s3cfb_check_win(const struct decon_win_config *c)
{
	if (c->idma_type > IDMA_VGF1 || c->blending >= DECON_BLENDING_MAX)
		return -EINVAL;
	if (c->plane_alpha < 0 || c->plane_alpha > 0xff)
		return -EINVAL;
	if (!c->dst.w || !c->dst.h || c->dst.x < 0 || c->dst.y < 0 || c->dst.w < 16)
		return -EINVAL;

	return 0;
}

/* a buffer window also needs IDMA_G0 only in window 5 and a known format */
static int s3cfb_check_limitation(int w, const struct decon_win_config *c)
{
	if (c->idma_type == IDMA_G0 && w != S3CFB_WIN_G0)
		return -EINVAL;
	if (c->format >= DECON_PIXEL_FORMAT_MAX)
		return -EINVAL;

	return s3cfb_check_win(c);
}

/* validate and stage a DSU mode switch request from config[DECON_WIN_DSU_ID] */
static int s3cfb_dsu_check_info(const struct decon_win_config *dsu_info,
				const struct dsu_info_dt *dt_dsu_info)
{
	int actual_mode;
	int width, height;

	if (unlikely(dsu_info->mode < DSU_MODE_1 || dsu_info->mode >= DSU_MODE_MAX)) {
		pr_err("DECON:ERR:%s:invalid dsu mode : %d\n", __func__, dsu_info->mode);
		return -EINVAL;
	}

	actual_mode = dsu_info->mode - DSU_MODE_1;
	if (unlikely(actual_mode >= dt_dsu_info->dsu_number)) {
		pr_err("DECON:ERR:%s:invalid dsu mode : %d\n", __func__, dsu_info->mode);
		return -EINVAL;
	}

	if (dsu_info->top != 0 || dsu_info->left != 0) {
		pr_err("DECON:ERR:%s:wrong dsu info : mode:%d, top:%d, left:%d\n",
		       __func__, dsu_info->mode, dsu_info->top, dsu_info->left);
		return -EINVAL;
	}

	width = dsu_info->right - dsu_info->left;
	height = dsu_info->bottom - dsu_info->top;

	if (dt_dsu_info->res_info[actual_mode].width != width ||
	    dt_dsu_info->res_info[actual_mode].height != height) {
		pr_err("DECON:ERR:%s:wrong dsu info : mode:%d, width:%d, height:%d\n",
		       __func__, dsu_info->mode, width, height);
		return -EINVAL;
	}

	return 0;
}

static int s3cfb_dsu_win_config(struct s3cfb *s, const struct decon_win_config *windata,
				struct dsu_info *regs_dsu)
{
	const struct dsu_info_dt *dt_dsu_info = exynos8895_panel_get_dsu_dt();
	const struct decon_win_config *new_dsu = &windata[DECON_WIN_DSU_ID];
	struct dsu_info *cur_dsu = &s->dsu;
	int ret;

	if (likely(new_dsu->mode == DSU_MODE_NONE))
		return 0;

	if (likely(new_dsu->mode == cur_dsu->mode))
		return 0;

	if (unlikely(!dt_dsu_info || dt_dsu_info->dsu_en == 0)) {
		pr_debug("s3cfb: the panel does not support DSU\n");
		return 0;
	}

	pr_debug("s3cfb: DSU mode %d -> %d (%d,%d-%d,%d)\n",
		 cur_dsu->mode, new_dsu->mode,
		 new_dsu->left, new_dsu->top, new_dsu->right, new_dsu->bottom);

	ret = s3cfb_dsu_check_info(new_dsu, dt_dsu_info);
	if (unlikely(ret != 0))
		return ret;

	regs_dsu->needupdate = 1;
	regs_dsu->mode = new_dsu->mode;
	regs_dsu->top = new_dsu->top;
	regs_dsu->left = new_dsu->left;
	regs_dsu->right = new_dsu->right;
	regs_dsu->bottom = new_dsu->bottom;

	return 0;
}

/* window bounds against the pending DSU mode if any, else the current one */
static int s3cfb_check_dsu_info(const struct s3cfb *s, const struct decon_win_config *config,
				int idx, const struct s3cfb_frame *fr)
{
	const struct dsu_info *dsu = fr->dsu.needupdate ? &fr->dsu : &s->dsu;

	if (config->dst.w > dsu->right || config->dst.h > dsu->bottom) {
		pr_err("DSU ERR : win[%d] size is abnormal (w:%u, h:%u, x:%d, y:%d)\n",
		       idx, config->dst.w, config->dst.h, config->dst.x, config->dst.y);
		pr_err("%s dsu info : right :%d, bottom : %d\n",
		       fr->dsu.needupdate ? "regs" : "decon", dsu->right, dsu->bottom);
		return -EINVAL;
	}

	return 0;
}

/*
 * A colour window: colour map on, no DMA, the alpha byte forced to 0xff. The
 * vendor driver checked nothing here; the buffer-independent checks of a buffer
 * window apply, and it must lie on the screen as its position is written
 * unclipped.
 */
static int s3cfb_prep_color(const struct s3cfb *s, const struct decon_win_config *c,
			    const struct s3cfb_frame *fr, struct s3cfb_win *win)
{
	const struct dsu_info *dsu = fr->dsu.needupdate ? &fr->dsu : &s->dsu;

	if (s3cfb_check_win(c))
		return -EINVAL;
	if ((u64)c->dst.x + c->dst.w > dsu->right || (u64)c->dst.y + c->dst.h > dsu->bottom)
		return -EINVAL;

	win->state = DECON_WIN_STATE_COLOR;
	win->ch = c->idma_type;
	win->dst = c->dst;
	win->argb = c->color | 0xff000000;
	win->alpha = c->plane_alpha * 0x101;
	win->blend = s3cfb_blend[c->blending];

	return 0;
}

/*
 * A buffer window: checks, import of every plane's buffer, acquire fence,
 * buffer size check; then the framebuffer.
 */
static int s3cfb_prep_buffer(struct s3cfb *s, int w, const struct decon_win_config *c,
			     const struct s3cfb_frame *fr, struct s3cfb_win *win)
{
	struct drm_device *drm = s->h.drm;
	struct s3cfb_import *e[2] = { };
	const struct s3cfb_fmt *f;
	struct drm_framebuffer *fb;
	int i, ret;

	ret = s3cfb_check_dsu_info(s, c, w, fr);
	if (ret)
		return ret;
	ret = s3cfb_check_limitation(w, c);
	if (ret)
		return ret;
	f = &s3cfb_formats[c->format];
	if (!f->fourcc)
		return -EINVAL;
	/*
	 * drm_mode_addfb2's size limits, and a source inside the buffer so that
	 * the 16.16 plane source cannot overflow.
	 */
	if (!c->src.f_w || !c->src.f_h || c->src.f_w > drm->mode_config.max_width ||
	    c->src.f_h > drm->mode_config.max_height || c->src.x < 0 || c->src.y < 0 ||
	    (u64)c->src.x + c->src.w > c->src.f_w || (u64)c->src.y + c->src.h > c->src.f_h)
		return -EINVAL;

	for (i = 0; i < f->nfd; i++) {
		e[i] = s3cfb_import(s, c->fd_idma[i]);
		if (IS_ERR(e[i]))
			return PTR_ERR(e[i]);
	}

	if (c->fence_fd >= 0) {
		/* the fd stays the composer's */
		win->acquire = sync_file_get_fence(c->fence_fd);
		if (!win->acquire)
			return -EINVAL;
	}

	/* to avoid a SysMMU fault on a small buffer */
	if ((u64)c->src.f_w * c->src.f_h * f->bpp > e[0]->buf->size)
		return -EINVAL;

	fb = s3cfb_fb(s, c, f, e);
	if (IS_ERR(fb))
		return PTR_ERR(fb);

	win->state = DECON_WIN_STATE_BUFFER;
	win->ch = c->idma_type;
	win->fb = fb;
	win->src = c->src;
	win->dst = c->dst;
	win->alpha = c->plane_alpha * 0x101;
	win->blend = s3cfb_blend[c->blending];
	win->comp = c->compression;

	return 0;
}

static void s3cfb_frame_free(struct s3cfb_frame *fr)
{
	int w;

	for (w = 0; w < MAX_DECON_WIN; w++) {
		if (fr->win[w].fb)
			drm_framebuffer_put(fr->win[w].fb);
		if (fr->win[w].acquire)
			dma_fence_put(fr->win[w].acquire);
	}
	kfree(fr);
}

/*
 * config[7] (DSU) and windows 0..5; config[6] (window update) is ignored. An
 * unknown state fails with -EINVAL.
 */
static int s3cfb_prepare(struct s3cfb *s, const struct decon_win_config_data *d,
			 struct s3cfb_frame *fr)
{
	int w, ret;

	ret = s3cfb_dsu_win_config(s, d->config, &fr->dsu);
	if (ret)
		return ret;

	for (w = 0; w < MAX_DECON_WIN && !ret; w++) {
		const struct decon_win_config *c = &d->config[w];

		switch (c->state) {
		case DECON_WIN_STATE_DISABLED:
			break;
		case DECON_WIN_STATE_COLOR:
			ret = s3cfb_prep_color(s, c, fr, &fr->win[w]);
			break;
		case DECON_WIN_STATE_BUFFER:
			ret = s3cfb_prep_buffer(s, w, c, fr, &fr->win[w]);
			break;
		default:
			ret = -EINVAL;
			break;
		}
	}

	return ret;
}

/* the current DSU mode changes once the frame that requested it is committed */
static void s3cfb_dsu_applied(struct s3cfb *s, const struct dsu_info *dsu)
{
	WRITE_ONCE(s->dsu.mode, dsu->mode);
	WRITE_ONCE(s->dsu.top, dsu->top);
	WRITE_ONCE(s->dsu.bottom, dsu->bottom);
	WRITE_ONCE(s->dsu.left, dsu->left);
	WRITE_ONCE(s->dsu.right, dsu->right);
	if (s->info) {
		s->info->var.xres = dsu->right;
		s->info->var.yres = dsu->bottom;
		s->info->var.xres_virtual = dsu->right;
		s->info->var.yres_virtual = dsu->bottom;
		s->info->fix.line_length = dsu->right * S3CFB_BPP / 8;
	}
}

static void s3cfb_apply(struct kthread_work *work);

/*
 * Under the front-end lock; the frame goes to the "decon0" worker and only
 * the fence is copied back.
 */
static int s3cfb_win_config(struct s3cfb *s, struct decon_win_config_data __user *argp)
{
	struct decon_win_config_data *d;
	struct sync_file *sf = NULL;
	struct s3cfb_frame *fr = NULL;
	int w, n_active = 0, fd = -1, ret = 0;

	d = kmalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	if (copy_from_user(d, argp, sizeof(*d))) {
		kfree(d);
		return -EFAULT;
	}

	mutex_lock(&s->lock);

	/* display off: a fence and one timeline step, no frame */
	if (!s->power) {
		fd = s3cfb_fence_fd(s, &sf);
		s3cfb_timeline_inc(s, false);
		if (fd < 0)
			ret = fd;
		goto unlock;
	}

	/* a fence, or -1 without an active window */
	for (w = 0; w < MAX_DECON_WIN; w++)
		if (d->config[w].state == DECON_WIN_STATE_COLOR ||
		    d->config[w].state == DECON_WIN_STATE_BUFFER)
			n_active++;
	if (n_active) {
		fd = s3cfb_fence_fd(s, &sf);
		if (fd < 0) {
			ret = fd;
			goto unlock;
		}
	} else {
		unsigned long flags;

		spin_lock_irqsave(&s->fence_lock, flags);
		s->tl_max++;
		spin_unlock_irqrestore(&s->fence_lock, flags);
	}

	s->frame++;
	s3cfb_cache_age(s);

	fr = kzalloc(sizeof(*fr), GFP_KERNEL);
	ret = fr ? s3cfb_prepare(s, d, fr) : -ENOMEM;
	if (ret) {
		/* command mode: signal the fence */
		if (fd >= 0) {
			s3cfb_timeline_inc(s, false);
			put_unused_fd(fd);
			fput(sf->file);
			fd = -1;
		}
		if (fr)
			s3cfb_frame_free(fr);
		goto unlock;
	}

	fr->s = s;
	kthread_init_work(&fr->work, s3cfb_apply);
	kthread_queue_work(s->worker, &fr->work);

unlock:
	mutex_unlock(&s->lock);
	kfree(d);
	if (ret)
		return ret;

	if (put_user(fd, &argp->fence)) {
		if (fd >= 0) {
			put_unused_fd(fd);
			fput(sf->file);
		}
		return -EFAULT;
	}
	if (fd >= 0)
		fd_install(fd, sf->file);

	return 0;
}

/* the "decon0" worker: one atomic commit per frame */

static void s3cfb_set_color_wins(struct s3cfb *s, const struct exynos8895_color_win *cw, u8 on)
{
	int w;

	for (w = 0; w < MAX_DECON_WIN; w++)
		exynos8895_decon_set_color_win(s->h.dev, w, (on & BIT(w)) ? &cw[w] : NULL);
}

/*
 * One atomic state on the CRTC and its six planes (plane index = channel):
 * @fr NULL turns everything off; otherwise each buffer window gets its plane,
 * colour windows are set and an inactive CRTC is enabled in the same commit.
 */
static int s3cfb_atomic(struct s3cfb *s, const struct s3cfb_frame *fr,
			struct drm_modeset_acquire_ctx *ctx)
{
	struct drm_crtc *crtc = s->h.crtc;
	struct exynos8895_color_win cw[MAX_DECON_WIN] = { };
	struct drm_atomic_state *state;
	struct drm_crtc_state *cs = NULL;
	int pw[EXYNOS8895_DECON_NR_PLANES];
	u8 cw_on = 0;
	int ch, w, ret;

	/* blank of an inactive CRTC: nothing to turn off */
	if (!fr && !crtc->state->active)
		return 0;

	state = drm_atomic_state_alloc(s->h.drm);
	if (!state)
		return -ENOMEM;
	state->acquire_ctx = ctx;

	if (!fr || !crtc->state->active) {
		cs = drm_atomic_get_crtc_state(state, crtc);
		ret = PTR_ERR_OR_ZERO(cs);
		if (ret)
			goto out;
	}
	if (!fr) {
		cs->active = false;
	} else if (!crtc->state->active) {
		struct drm_connector_state *conn;

		ret = drm_atomic_set_mode_for_crtc(cs, s->mode);
		if (ret)
			goto out;
		cs->active = true;
		conn = drm_atomic_get_connector_state(state, s->h.connector);
		ret = PTR_ERR_OR_ZERO(conn);
		if (ret)
			goto out;
		ret = drm_atomic_set_crtc_for_connector(conn, crtc);
		if (ret)
			goto out;
	}

	/* a channel named by two buffer windows serves the lower one */
	for (ch = 0; ch < EXYNOS8895_DECON_NR_PLANES; ch++)
		pw[ch] = -1;
	for (w = 0; fr && w < MAX_DECON_WIN; w++) {
		const struct s3cfb_win *win = &fr->win[w];

		if (win->state == DECON_WIN_STATE_BUFFER && pw[win->ch] < 0)
			pw[win->ch] = w;
		if (win->state == DECON_WIN_STATE_COLOR) {
			cw[w].dst.x1 = win->dst.x;
			cw[w].dst.y1 = win->dst.y;
			cw[w].dst.x2 = win->dst.x + win->dst.w;
			cw[w].dst.y2 = win->dst.y + win->dst.h;
			cw[w].argb = win->argb;
			cw[w].plane = win->ch;
			cw[w].alpha = win->alpha;
			cw[w].blend = win->blend;
			cw_on |= BIT(w);
		}
	}

	for (ch = 0; ch < EXYNOS8895_DECON_NR_PLANES; ch++) {
		struct drm_plane_state *ps = drm_atomic_get_plane_state(state, s->h.planes[ch]);
		const struct s3cfb_win *win;

		ret = PTR_ERR_OR_ZERO(ps);
		if (ret)
			goto out;
		if (pw[ch] < 0) {
			ret = drm_atomic_set_crtc_for_plane(ps, NULL);
			if (ret)
				goto out;
			drm_atomic_set_fb_for_plane(ps, NULL);
			ret = exynos8895_decon_plane_set_win(ps, -1, false);
			if (ret)
				goto out;
			continue;
		}
		win = &fr->win[pw[ch]];
		ret = drm_atomic_set_crtc_for_plane(ps, crtc);
		if (ret)
			goto out;
		drm_atomic_set_fb_for_plane(ps, win->fb);
		ps->src_x = (u32)win->src.x << 16;
		ps->src_y = (u32)win->src.y << 16;
		ps->src_w = win->src.w << 16;
		ps->src_h = win->src.h << 16;
		ps->crtc_x = win->dst.x;
		ps->crtc_y = win->dst.y;
		ps->crtc_w = win->dst.w;
		ps->crtc_h = win->dst.h;
		ps->alpha = win->alpha;
		ps->pixel_blend_mode = win->blend;
		ret = exynos8895_decon_plane_set_win(ps, pw[ch], win->comp);
		if (ret)
			goto out;
	}

	/* the DECON applies these in the next atomic_flush; undone on failure */
	s3cfb_set_color_wins(s, cw, cw_on);
	exynos8895_decon_set_dsu(s->h.dev, (fr && fr->dsu.needupdate) ? &fr->dsu : NULL);
	ret = drm_atomic_commit(state);	/* blocking */
	if (ret) {
		exynos8895_decon_set_dsu(s->h.dev, NULL);
		s3cfb_set_color_wins(s, s->cw, s->cw_on);
	} else {
		memcpy(s->cw, cw, sizeof(cw));
		s->cw_on = cw_on;
	}
out:
	drm_atomic_state_put(state);
	return ret;
}

static int s3cfb_commit(struct s3cfb *s, const struct s3cfb_frame *fr)
{
	struct drm_modeset_acquire_ctx ctx;
	int ret;

	DRM_MODESET_LOCK_ALL_BEGIN(s->h.drm, ctx, 0, ret);
	ret = s3cfb_atomic(s, fr, &ctx);
	DRM_MODESET_LOCK_ALL_END(s->h.drm, ctx, ret);

	return ret;
}

static void s3cfb_apply(struct kthread_work *work)
{
	struct s3cfb_frame *fr = container_of(work, struct s3cfb_frame, work);
	struct s3cfb *s = fr->s;
	bool buffers = false, step = true;
	int w, ret;

	/*
	 * Acquire fences, 900 ms each; a timed-out window is dropped only if
	 * compressed, as in the vendor driver.
	 */
	for (w = 0; w < MAX_DECON_WIN; w++) {
		struct s3cfb_win *win = &fr->win[w];

		if (!win->acquire)
			continue;
		if (dma_fence_wait_timeout(win->acquire, false,
					   msecs_to_jiffies(S3CFB_FENCE_TIMEOUT_MS)) <= 0) {
			pr_err("s3cfb: win%d acquire fence timeout\n", w);
			if (win->comp) {
				win->state = DECON_WIN_STATE_DISABLED;
				drm_framebuffer_put(win->fb);
				win->fb = NULL;
			}
		}
		dma_fence_put(win->acquire);
		win->acquire = NULL;
	}

	/*
	 * A frame without a buffer window is not committed: the planes on
	 * screen keep their framebuffers.
	 */
	for (w = 0; w < MAX_DECON_WIN; w++)
		if (fr->win[w].state == DECON_WIN_STATE_BUFFER)
			buffers = true;

	/* a rejected frame leaves the previous one on screen */
	if ((buffers || fr->dsu.needupdate) && !READ_ONCE(s->dead)) {
		s3cfb_release_arm(s);
		ret = s3cfb_commit(s, fr);
		if (ret)
			pr_err_ratelimited("s3cfb: WIN_CONFIG rejected by atomic_check: %d\n", ret);
		else if (fr->dsu.needupdate)
			s3cfb_dsu_applied(s, &fr->dsu);
		/* false when this frame's start already signalled (s3cfb_frame_start()) */
		step = s3cfb_release_take(s);
	}

	/* the previous frame's fence, unless the frame start did */
	if (step)
		s3cfb_timeline_inc(s, false);
	s3cfb_frame_free(fr);
}

/* power */

static void s3cfb_power_on(struct s3cfb *s)
{
	mutex_lock(&s->lock);
	WRITE_ONCE(s->power, true);
	mutex_unlock(&s->lock);
	s3cfb_soft_vsync_update(s);
}

/*
 * Power flag off, flush the queued frames, then the CRTC off with every plane
 * off and the import cache released. As in the vendor driver the timeline is
 * left as it is: the last fence signals with the next WIN_CONFIG.
 */
static int s3cfb_power_off(struct s3cfb *s)
{
	unsigned long flags;
	int ret;

	mutex_lock(&s->lock);
	WRITE_ONCE(s->power, false);
	mutex_unlock(&s->lock);

	kthread_flush_worker(s->worker);
	ret = s3cfb_commit(s, NULL);

	mutex_lock(&s->lock);
	s3cfb_cache_flush(s);
	mutex_unlock(&s->lock);

	spin_lock_irqsave(&s->vs_lock, flags);
	s->te_live = false;
	spin_unlock_irqrestore(&s->vs_lock, flags);

	return ret;
}

/* fb_ops */

/*
 * The resolution of the current DSU mode and 32 bpp; every other field as
 * given, for the composer's reserved[0..1].
 */
static int s3cfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct s3cfb *s = s3cfb_of(info);
	u32 xres = READ_ONCE(s->dsu.right);
	u32 yres = READ_ONCE(s->dsu.bottom);

	if (var->xres != xres || var->yres != yres ||
	    var->bits_per_pixel != S3CFB_BPP)
		return -EINVAL;
	var->xres_virtual = max(var->xres_virtual, var->xres);
	var->yres_virtual = max(var->yres_virtual, var->yres);
	info->fix.line_length = var->xres_virtual * S3CFB_BPP / 8;
	return 0;
}

/* the composer composes only through WIN_CONFIG */
static int s3cfb_set_par(struct fb_info *info)
{
	return 0;
}

static bool s3cfb_crtc_active(struct s3cfb *s)
{
	struct drm_crtc *crtc = s->h.crtc;
	bool active;

	drm_modeset_lock(&crtc->mutex, NULL);
	active = crtc->state->active;
	drm_modeset_unlock(&crtc->mutex);

	return active;
}

/*
 * UNBLANK only sets the power flag; the first buffer frame enables the CRTC.
 * The panel is told first, so UNBLANK of an active CRTC wakes a dozing panel.
 */
static int s3cfb_blank(int blank, struct fb_info *info)
{
	struct s3cfb *s = s3cfb_of(info);

	if (s->dead)
		return -ENODEV;

	switch (blank) {
	case FB_BLANK_UNBLANK:
		exynos8895_decon_panel_power(EXYNOS8895_PANEL_UNBLANK, s3cfb_crtc_active(s));
		s3cfb_power_on(s);
		return 0;
	case FB_BLANK_NORMAL:
	case FB_BLANK_POWERDOWN:
		exynos8895_decon_panel_power(EXYNOS8895_PANEL_POWERDOWN, s3cfb_crtc_active(s));
		pm_relax(s->h.dev);
		return s3cfb_power_off(s);
	default:
		return -EINVAL;
	}
}

/*
 * DOZE_SUSPEND flushes the queued frames first, so that a frame posted in
 * DOZE has enabled the CRTC before its state is read. OFF and NORMAL are
 * no-ops.
 */
static int s3cfb_power_mode(struct s3cfb *s, u32 mode)
{
	bool active;

	switch (mode) {
	case DECON_PWR_DOZE:
		active = s3cfb_crtc_active(s);
		exynos8895_decon_panel_power(EXYNOS8895_PANEL_DOZE, active);
		if (!active) {
			pm_stay_awake(s->h.dev);
			s3cfb_power_on(s);
		}
		return 0;
	case DECON_PWR_DOZE_SUSPEND:
		kthread_flush_worker(s->worker);
		active = s3cfb_crtc_active(s);
		exynos8895_decon_panel_power(EXYNOS8895_PANEL_DOZE_SUSPEND, active);
		if (!active)
			return 0;
		pm_relax(s->h.dev);
		return s3cfb_power_off(s);
	default:
		return 0;
	}
}

static int s3cfb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct s3cfb *s = s3cfb_of(info);
	void __user *argp = (void __user *)arg;
	unsigned long flags;
	bool active;
	u32 val;

	if (s->dead)
		return -ENODEV;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		if (get_user(val, (u32 __user *)argp))
			return -EFAULT;
		return val ? -ENODEV : s3cfb_wait_for_vsync(s);

	case S3CFB_SET_VSYNC_INT:	/* one byte, the vendor's bool */
		if (get_user(active, (bool __user *)argp))
			return -EFAULT;
		spin_lock_irqsave(&s->vs_lock, flags);
		s->vsync_on = active;
		spin_unlock_irqrestore(&s->vs_lock, flags);
		s3cfb_soft_vsync_update(s);
		return 0;

	case S3CFB_WIN_CONFIG:
		return s3cfb_win_config(s, argp);

	case S3CFB_DECON_SELF_REFRESH:	/* recorded only */
		if (get_user(active, (bool __user *)argp))
			return -EFAULT;
		s->dsr_on = active;
		return 0;

	case S3CFB_GET_HDR_CAPABILITIES: {	/* no HDR */
		struct decon_hdr_capabilities capa = { };

		return copy_to_user(argp, &capa, sizeof(capa)) ? -EFAULT : 0;
	}

	case S3CFB_GET_HDR_CAPABILITIES_NUM: {
		struct decon_hdr_capabilities_info capa = { };

		return copy_to_user(argp, &capa, sizeof(capa)) ? -EFAULT : 0;
	}

	case S3CFB_POWER_MODE:
		if (get_user(val, (int __user *)argp))
			return -EFAULT;
		if (val >= DECON_PWR_MAX)
			return -EINVAL;
		return s3cfb_power_mode(s, val);

	default:
		return -ENOTTY;
	}
}

#ifdef CONFIG_COMPAT
static int s3cfb_compat_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	return s3cfb_ioctl(info, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static ssize_t s3cfb_read(struct fb_info *info, char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}

static ssize_t s3cfb_write(struct fb_info *info, const char __user *buf, size_t count,
			   loff_t *ppos)
{
	return 0;
}

/* no framebuffer memory: the composer never maps fb0 */
static int s3cfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	return -ENODEV;
}

/* the last reference of the fb_info (open files keep it) */
static void s3cfb_fb_destroy(struct fb_info *info)
{
	struct s3cfb *s = s3cfb_of(info);

	framebuffer_release(info);
	kref_put(&s->ref, s3cfb_free);
}

static const struct fb_ops s3cfb_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_read	= s3cfb_read,
	.fb_write	= s3cfb_write,
	.fb_check_var	= s3cfb_check_var,
	.fb_set_par	= s3cfb_set_par,
	.fb_blank	= s3cfb_blank,
	.fb_ioctl	= s3cfb_ioctl,
#ifdef CONFIG_COMPAT
	.fb_compat_ioctl = s3cfb_compat_ioctl,
#endif
	.fb_mmap	= s3cfb_mmap,
	.fb_destroy	= s3cfb_fb_destroy,
};

/* sysfs of 12860000.decon_f */

static ssize_t vsync_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s3cfb *s = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%llu\n", ktime_to_ns(s3cfb_vsync_ts(s)));
}
static DEVICE_ATTR_RO(vsync);

/*
 * psr_mode, the mode count, then width, height, DSC slice width and height
 * (or MIN_WIN_BLOCK_WIDTH/HEIGHT when uncompressed) and DSC enable per mode.
 */
static ssize_t psr_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	const struct dsu_info_dt *dsu_info = exynos8895_panel_get_dsu_dt();
	unsigned int i;
	ssize_t len;

	if (!dsu_info || !dsu_info->dsu_en || !dsu_info->dsu_number)
		return sysfs_emit(buf, "%d\n%d\n%d\n%d\n%d\n%d\n%d\n", S3CFB_PSR_MODE, 1,
				  S3CFB_XRES, S3CFB_YRES, S3CFB_DSC_WIDTH, S3CFB_DSC_HEIGHT,
				  S3CFB_DSC_EN);

	len = sysfs_emit(buf, "%d\n%u\n", S3CFB_PSR_MODE, dsu_info->dsu_number);
	for (i = 0; i < dsu_info->dsu_number; i++) {
		const struct dsu_res_info *r = &dsu_info->res_info[i];

		len += sysfs_emit_at(buf, len, "%u\n%u\n%u\n%u\n%u\n",
				     r->width, r->height,
				     r->dsc_en ? r->dsc_width : MIN_WIN_BLOCK_WIDTH,
				     r->dsc_en ? r->dsc_height : MIN_WIN_BLOCK_HEIGHT,
				     r->dsc_en);
	}

	return len;
}
static DEVICE_ATTR_RO(psr_info);

static struct attribute *s3cfb_attrs[] = {
	&dev_attr_vsync.attr,
	&dev_attr_psr_info.attr,
	NULL
};
ATTRIBUTE_GROUPS(s3cfb);

/* init and teardown */

/*
 * Everything but the DRM client: stop the ioctls, the vsync sources and the
 * worker, signal every fence and release the import cache. The planes on
 * screen keep their own framebuffer references.
 */
static void s3cfb_teardown(struct s3cfb *s)
{
	unsigned long flags;

	if (s->torn_down)
		return;
	s->torn_down = true;

	lock_fb_info(s->info);
	s->dead = true;
	unlock_fb_info(s->info);
	/* the fb_info reference goes in s3cfb_fb_destroy() */
	unregister_framebuffer(s->info);

	device_init_wakeup(s->h.dev, false);
	exynos8895_decon_set_vsync_cb(s->h.dev, NULL, NULL);
	exynos8895_decon_set_frame_start_cb(s->h.dev, NULL, NULL);
	spin_lock_irqsave(&s->vs_lock, flags);
	s->vsync_on = false;
	spin_unlock_irqrestore(&s->vs_lock, flags);
	hrtimer_cancel(&s->soft);
	sysfs_put(s->vsync_kn);
	platform_device_unregister(s->sdev);

	kthread_destroy_worker(s->worker);
	s3cfb_timeline_inc(s, true);
	mutex_lock(&s->lock);
	s3cfb_cache_flush(s);
	mutex_unlock(&s->lock);
	drm_mode_destroy(s->h.drm, s->mode);
}

/* drm_dev_unregister(): the only place the client may be released */
static void s3cfb_client_unregister(struct drm_client_dev *client)
{
	struct s3cfb *s = container_of(client, struct s3cfb, client);

	if (s3cfb_inst == s)
		s3cfb_inst = NULL;
	s3cfb_teardown(s);
	drm_client_release(client);
	kref_put(&s->ref, s3cfb_free);
}

static const struct drm_client_funcs s3cfb_client_funcs = {
	.owner		= THIS_MODULE,
	.unregister	= s3cfb_client_unregister,
};

/*
 * The panel mode, once: fill_modes under the mode_config mutex as the
 * in-kernel clients do, then a copy of the preferred mode.
 */
static struct drm_display_mode *s3cfb_get_mode(struct s3cfb *s)
{
	struct drm_connector *conn = s->h.connector;
	struct drm_display_mode *m, *mode = NULL;

	mutex_lock(&s->h.drm->mode_config.mutex);
	conn->funcs->fill_modes(conn, S3CFB_XRES, S3CFB_YRES);
	list_for_each_entry(m, &conn->modes, head) {
		if (m->type & DRM_MODE_TYPE_PREFERRED) {
			mode = drm_mode_duplicate(s->h.drm, m);
			break;
		}
	}
	mutex_unlock(&s->h.drm->mode_config.mutex);

	return mode;
}

/*
 * The fb_info of the vendor driver's default window: the panel timing, 32
 * bpp, the panel size and no memory.
 */
static void s3cfb_fill_info(struct fb_info *info)
{
	static const struct fb_videomode vm = {
		.xres		= S3CFB_XRES,
		.yres		= S3CFB_YRES,
		.left_margin	= 2,
		.right_margin	= 2,
		.hsync_len	= 2,
		.upper_margin	= 15,
		.lower_margin	= 3,
		.vsync_len	= 1,
	};
	struct fb_var_screeninfo *var = &info->var;

	fb_videomode_to_var(var, &vm);
	var->bits_per_pixel = S3CFB_BPP;
	var->red.offset = 16;
	var->red.length = 8;
	var->green.offset = 8;
	var->green.length = 8;
	var->blue.offset = 0;
	var->blue.length = 8;
	var->transp.offset = 24;
	var->transp.length = 8;
	var->width = S3CFB_WIDTH_MM;
	var->height = S3CFB_HEIGHT_MM;
	var->activate = FB_ACTIVATE_NOW;
	var->vmode = FB_VMODE_NONINTERLACED;

	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.accel = FB_ACCEL_NONE;
	info->fix.line_length = S3CFB_XRES * S3CFB_BPP / 8;
	info->fbops = &s3cfb_fb_ops;
}

static int s3cfb_create(struct drm_device *drm)
{
	struct s3cfb *s;
	int ret;

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	kref_init(&s->ref);
	mutex_init(&s->lock);
	spin_lock_init(&s->fence_lock);
	spin_lock_init(&s->vs_lock);
	INIT_LIST_HEAD(&s->fences);
	init_waitqueue_head(&s->vsync_wait);
	hrtimer_setup(&s->soft, s3cfb_soft_vsync, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	/* the boot loader left the panel on */
	s->power = true;
	s->dsu.mode = DSU_MODE_1;
	s->dsu.left = 0;
	s->dsu.top = 0;
	s->dsu.right = S3CFB_XRES;
	s->dsu.bottom = S3CFB_YRES;
	s->dsu.needupdate = 0;
	/* the vendor timeline starts at 1 */
	s->fence_ctx = dma_fence_context_alloc(1);
	s->tl_max = 1;

	ret = exynos8895_decon_get(NULL, &s->h);
	if (!ret && s->h.drm != drm)
		ret = -ENODEV;
	if (ret)
		goto err_free;

	s->mode = s3cfb_get_mode(s);
	if (!s->mode) {
		ret = -ENODEV;
		goto err_free;
	}
	if (s->mode->hdisplay != S3CFB_XRES || s->mode->vdisplay != S3CFB_YRES) {
		ret = -EINVAL;
		goto err_mode;
	}

	/* client.file owns the imported handles */
	ret = drm_client_init(drm, &s->client, "s3cfb", &s3cfb_client_funcs);
	if (ret)
		goto err_mode;

	/* SCHED_FIFO, as the vendor decon0 thread */
	s->worker = kthread_run_worker(0, "decon0");
	if (IS_ERR(s->worker)) {
		ret = PTR_ERR(s->worker);
		goto err_client;
	}
	sched_set_fifo(s->worker->task);

	/* the sysfs directory the composer opens */
	s->sdev = platform_device_alloc("12860000.decon_f", PLATFORM_DEVID_NONE);
	if (!s->sdev) {
		ret = -ENOMEM;
		goto err_worker;
	}
	s->sdev->dev.groups = s3cfb_groups;
	platform_set_drvdata(s->sdev, s);
	ret = platform_device_add(s->sdev);
	if (ret) {
		platform_device_put(s->sdev);
		goto err_worker;
	}
	s->vsync_kn = sysfs_get_dirent(s->sdev->dev.kobj.sd, "vsync");
	if (!s->vsync_kn) {
		ret = -ENODEV;
		goto err_sdev;
	}

	s->info = framebuffer_alloc(sizeof(s), s->h.dev);
	if (!s->info) {
		ret = -ENOMEM;
		goto err_kn;
	}
	*(struct s3cfb **)s->info->par = s;
	s3cfb_fill_info(s->info);

	exynos8895_decon_set_vsync_cb(s->h.dev, s3cfb_te, s);
	exynos8895_decon_set_frame_start_cb(s->h.dev, s3cfb_frame_start, s);
	/*
	 * One-shot trigger, as the vendor driver runs: the panel holds the
	 * frame in its memory, and only then does the pipe idle between frames
	 * for display hibernation.
	 */
	exynos8895_decon_set_oneshot(s->h.dev, true);
	/* the DECON's wakeup source the DOZE power mode holds */
	device_init_wakeup(s->h.dev, true);

	kref_get(&s->ref);
	ret = register_framebuffer(s->info);
	if (ret) {
		kref_put(&s->ref, s3cfb_free);
		device_init_wakeup(s->h.dev, false);
		exynos8895_decon_set_vsync_cb(s->h.dev, NULL, NULL);
		exynos8895_decon_set_frame_start_cb(s->h.dev, NULL, NULL);
		framebuffer_release(s->info);
		goto err_kn;
	}

	/* from here on released by s3cfb_client_unregister() */
	drm_client_register(&s->client);
	s3cfb_inst = s;

	pr_debug("exynos8895-s3cfb: fb%d %dx%d on crtc %u, %d planes, psr_mode %d\n",
		 s->info->node, S3CFB_XRES, S3CFB_YRES, drm_crtc_index(s->h.crtc),
		 EXYNOS8895_DECON_NR_PLANES, S3CFB_PSR_MODE);

	return 0;

err_kn:
	sysfs_put(s->vsync_kn);
err_sdev:
	platform_device_unregister(s->sdev);
err_worker:
	kthread_destroy_worker(s->worker);
err_client:
	drm_client_release(&s->client);
err_mode:
	drm_mode_destroy(drm, s->mode);
err_free:
	kref_put(&s->ref, s3cfb_free);
	return ret;
}

/*
 * Called by exynos_drm_bind() right after drm_dev_register(); a failure
 * leaves the DRM device working without fb0.
 */
void exynos8895_s3cfb_init(struct drm_device *drm)
{
	int ret = s3cfb_create(drm);

	if (ret)
		drm_err(drm, "exynos8895-s3cfb: init failed: %d\n", ret);
}

/* Called by exynos_drm_unbind() before drm_dev_unregister() releases the client */
void exynos8895_s3cfb_fini(struct drm_device *drm)
{
	struct s3cfb *s = s3cfb_inst;

	if (!s || s->h.drm != drm)
		return;
	s3cfb_teardown(s);
}
