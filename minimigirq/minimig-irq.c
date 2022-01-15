/*
 *  minimig_irq.c -- Minimig irq driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>
#include <linux/console.h>
#include <linux/debugfs.h>
#include <linux/printk.h> /* printk */
#include "minimig_ioctl.h"

#define DRIVER_NAME          "minimig_irq"

struct minimig_dev {
	struct platform_device *pdev;
	struct resource *minimig_res;
	void __iomem *minimig_base;
	int irq0;
};

static struct dentry *dir;

static wait_queue_head_t vs_wait;
static u32 irq_count = 0;
static irqreturn_t irq_handler(int irq, void *par)
{
	++irq_count;
	wake_up_interruptible(&vs_wait);
	return IRQ_HANDLED;
}

static int minimig_wait_for_irq(u32 arg)
{
	u32 count;
	int ret;

	if (arg != 0) return -ENODEV;

	count = irq_count;
	ret = wait_event_interruptible_timeout(vs_wait,
				       count != irq_count,
				       msecs_to_jiffies(10000000));

	return (!ret) ? -ETIMEDOUT : 0;
}

static long minimig_ioctl(struct file *info, unsigned int cmd, unsigned long arg)
{
	int ret;
	u32 ioarg;

	switch (cmd)
	{
	case MINIMIG_IOC_WAIT_IRQ:
/*		if (get_user(ioarg, (u32 __user *)arg))
		{
			ret = -EFAULT;
			break;
		}*/
		ioarg = 0;

		ret = minimig_wait_for_irq(ioarg);
		break;
	default:
		ret = -ENOTTY;
	}

	return ret;
}

#define pgprot_cached(prot) \
	__pgprot_modify(prot, L_PTE_MT_MASK, L_PTE_MT_DEV_CACHED)

// 511MB ram for linux
// 513MB ram for fpga
// We will only patch the 384MB for the z3 fast ram
// i.e. 0x28000000-0x3FFFFFFF (0x18000000)
//
// We'll patch this too
// fpga->hps bridge minimig mem/hardware window 
// i.e. 0xc0000000-0xc0ffffff (0x1000000 - 24-bit)
//
static int minimig_mmap_writecombine(struct file *filp, struct vm_area_struct * vma)
{
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	printk("mmap of %lx into %lx, with writecombine\n",vma->vm_pgoff << PAGE_SHIFT,vma->vm_start);

	return io_remap_pfn_range(vma,
                           vma->vm_start,
                           vma->vm_pgoff,
                           vma->vm_end - vma->vm_start,
                           vma->vm_page_prot);
}
static int minimig_mmap_cached(struct file *filp, struct vm_area_struct * vma)
{
	vma->vm_page_prot = pgprot_cached(vma->vm_page_prot);
	printk("mmap of %lx into %lx, with cached\n",vma->vm_pgoff << PAGE_SHIFT,vma->vm_start);

	return io_remap_pfn_range(vma,
                           vma->vm_start,
                           vma->vm_pgoff,
                           vma->vm_end - vma->vm_start,
                           vma->vm_page_prot);
}
static int minimig_mmap_noncached(struct file *filp, struct vm_area_struct * vma)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	printk("mmap of %lx into %lx, with noncached\n",vma->vm_pgoff << PAGE_SHIFT,vma->vm_start);

	return io_remap_pfn_range(vma,
                           vma->vm_start,
                           vma->vm_pgoff,
                           vma->vm_end - vma->vm_start,
                           vma->vm_page_prot);
}
static int minimig_mmap_dmacoherent(struct file *filp, struct vm_area_struct * vma)
{
	vma->vm_page_prot = pgprot_dmacoherent(vma->vm_page_prot);
	printk("mmap of %lx into %lx, with dmacoherent\n",vma->vm_pgoff << PAGE_SHIFT,vma->vm_start);

	return io_remap_pfn_range(vma,
                           vma->vm_start,
                           vma->vm_pgoff,
                           vma->vm_end - vma->vm_start,
                           vma->vm_page_prot);
}

static const struct file_operations fopsioctl = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = minimig_ioctl,
};
static const struct file_operations fopswritecombine = {
    .owner = THIS_MODULE,
    .mmap = minimig_mmap_writecombine
};
static const struct file_operations fopscached = {
    .owner = THIS_MODULE,
    .mmap = minimig_mmap_cached
};
static const struct file_operations fopsnoncached = {
    .owner = THIS_MODULE,
    .mmap = minimig_mmap_noncached
};
static const struct file_operations fopsdmacoherent = {
    .owner = THIS_MODULE,
    .mmap = minimig_mmap_dmacoherent
};

static int minimig_probe(struct platform_device *pdev)
{
	struct minimig_dev *minimigdev;

	minimigdev = devm_kzalloc(&pdev->dev, sizeof(*minimigdev), GFP_KERNEL);
	if (!minimigdev)	return -ENOMEM;

	minimigdev->pdev = pdev;

/*	Seems to be for memory ranges - might be handy
 *	minimigdev->minimig_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!minimigdev->minimig_res) return -ENODEV;

	minimigdev->minimig_base = memremap(minimigdev->minimig_res->start, resource_size(minimigdev->minimig_res), MEMREMAP_WT);
	if (IS_ERR(minimigdev->minimig_base))
	{
		dev_err(&pdev->dev, "devm_ioremap_resource fb failed\n");
		return PTR_ERR(fbdev->minimig_base);
	}*/

	platform_set_drvdata(pdev, minimigdev);

	dev_info(&pdev->dev, "minimig_irqs module starting\n");

	minimigdev->irq0 = irq_of_parse_and_map(pdev->dev.of_node, 0);
	dev_info(&pdev->dev, "minimig_irqs: IRQ0=%d\n", minimigdev->irq0);

	init_waitqueue_head(&vs_wait);

	if (minimigdev->irq0 == NO_IRQ || request_irq(minimigdev->irq0, &irq_handler, IRQF_SHARED, "minimig_irq", minimigdev))
	{
		dev_err(&pdev->dev, "request_irq 0 failed.\n");
	}

	dir = debugfs_create_dir("minimig_irq", 0);
	/* ioctl permissions are not automatically restricted by rwx as for read / write,
	 * but we could of course implement that ourselves:
	 * https://stackoverflow.com/questions/29891803/user-permission-check-on-ioctl-command */
	debugfs_create_file("ioctl_dev", 0, dir, NULL, &fopsioctl);
	debugfs_create_file_unsafe("mmap_writecombine", 0, dir, NULL, &fopswritecombine);
	debugfs_create_file_unsafe("mmap_noncached", 0, dir, NULL, &fopsnoncached);
	debugfs_create_file_unsafe("mmap_cached", 0, dir, NULL, &fopscached);
	debugfs_create_file_unsafe("mmap_dmacoherent", 0, dir, NULL, &fopsdmacoherent);

	return 0;
}

static int minimig_remove(struct platform_device *dev)
{
	struct minimig_dev *minimig_dev = platform_get_drvdata(dev);

	if (minimig_dev)
	{
		if(minimig_dev->irq0 != NO_IRQ) free_irq(minimig_dev->irq0, dev);
		minimig_dev->irq0 = NO_IRQ;
	}
	return 0;
}

static struct of_device_id minimig_match[] = {
	{ .compatible = "minimig_irq" },
	{},
};
MODULE_DEVICE_TABLE(of, minimig_match);

static struct platform_driver minimig_driver = {
	.probe = minimig_probe,
	.remove = minimig_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
		.of_match_table = minimig_match,
	},
};
module_platform_driver(minimig_driver);

MODULE_DESCRIPTION("Minimig irq driver");
MODULE_AUTHOR("Mark Watson");
MODULE_LICENSE("GPL v2");

