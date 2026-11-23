// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos8895 /dev/s5p-smem: the physical address of an ION buffer, for the
 * iris trustlet client. Only SECMEM_IOC_GET_FD_PHYS_ADDR of the vendor secmem
 * driver; its content path protection controls are not implemented.
 */

#include <linux/dma-buf.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ion_legacy.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

/* vendor ABI: phys was an ion_phys_addr_t, an unsigned long */
struct secfd_info {
	int fd;
	unsigned long phys;
};

#define SECMEM_IOC_GET_FD_PHYS_ADDR	_IOWR('S', 8, struct secfd_info)

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
