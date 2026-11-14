// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 8895 (greatlte) /dev/s5p-smem: the physical address of an ION buffer,
 * for the iris trustlet client.  Stock secmem.c (S:drivers/soc/samsung/secmem.c,
 * DZC3 identical) also carries the DRM content path protection controls, which
 * stay out with CONFIG_EXYNOS_CONTENT_PATH_PROTECTION; libIrisTlc issues only
 * SECMEM_IOC_GET_FD_PHYS_ADDR (spec L6N_iris_userspace 3.1).
 */

#include <linux/dma-buf.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ion_legacy.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

/* S:include/soc/samsung/secmem.h:27-30,64; ion_phys_addr_t is unsigned long */
struct secfd_info {
	int fd;
	unsigned long phys;
};

#define SECMEM_IOC_GET_FD_PHYS_ADDR	_IOWR('S', 8, struct secfd_info)

/* S:drivers/soc/samsung/secmem.c:171-218 */
static long secmem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct secfd_info fd_info;
	struct dma_buf *dmabuf;
	phys_addr_t phys;
	size_t len;
	int ret;

	if (cmd != SECMEM_IOC_GET_FD_PHYS_ADDR)
		return -ENOTTY;

	if (copy_from_user(&fd_info, (void __user *)arg, sizeof(fd_info)))
		return -EFAULT;

	dmabuf = dma_buf_get(fd_info.fd);
	if (IS_ERR(dmabuf)) {
		pr_err("%s: Failed to get ion_handle of DRM\n", __func__);
		return -ENOMEM;
	}
	ret = ionl_dmabuf_phys(dmabuf, &phys, &len);
	dma_buf_put(dmabuf);
	if (ret) {
		pr_err("%s: Failed to get phys. addr of DRM\n", __func__);
		return -ENOMEM;
	}

	fd_info.phys = phys;
	if (copy_to_user((void __user *)arg, &fd_info, sizeof(fd_info)))
		return -EFAULT;
	return 0;
}

static const struct file_operations secmem_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= secmem_ioctl,
	.compat_ioctl	= secmem_ioctl,
};

static struct miscdevice secmem = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "s5p-smem",
	.fops	= &secmem_fops,
};

static int __init secmem_init(void)
{
	return misc_register(&secmem);
}
device_initcall(secmem_init);
