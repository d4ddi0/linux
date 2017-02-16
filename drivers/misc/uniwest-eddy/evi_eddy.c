/**
 * Copyright (c) 2015 United Western Technologies, Corporation
 *
 * evi_eddy is a multichannel eddy current device
 *
 */

#include <generated/utsrelease.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mman.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>
#include <linux/types.h>
#include <linux/uaccess.h>

/* headers private to the module */
#include "evi_registers.h"
#include "evi_ioctl.h"
#include "smartscanner.h"

#define DEVICE_NAME "evi-eddy"
#define SAMPLE_SIZE  40
#define SAMPLE_WORDS (SAMPLE_SIZE / sizeof(__u32))

struct ef_device {
	void __iomem *base;
	dev_t data_dev_num;
	struct cdev data_cdev;
	struct device *dev;
	struct smartscanner ss;
	struct semaphore access;
	struct resource *res;
	const char *fw_name;
};

static struct class ef_class = {
	.name = "evi-eddy",
	.owner = THIS_MODULE,
};

static const struct of_device_id of_ef_match[] = {
	{ .compatible = "uniwest,evi-eddy", },
	{}
};

/* device major and minor numbers */
static const int ef_major = 42;
static const int ef_minor;


/**
 * Character Device read handler. No read op on test character device.
 */
static ssize_t ef_data_read(struct file *filp, char __user *buf, size_t count,
			    loff_t *f_pos)
{
	int i, j;
	union ef_fifo_status st;

	struct ef_device *evi = filp->private_data;

	st.raw_data = readl_relaxed(evi->base + EVI_FIFO_STATUS);
	count = rounddown(min(count, st.fifo_count * sizeof(__u32)),
			      SAMPLE_SIZE);

	if (st.full)
		dev_dbg_ratelimited(evi->dev, "Buffer filled up");

	for (i = 0; i < count; i += SAMPLE_SIZE) {
		u32 data[SAMPLE_WORDS];

		for (j = 0; j < SAMPLE_WORDS; j++)
			data[j] = readl_relaxed(evi->base + EVI_FIFO);

		if (copy_to_user(buf, (char *)data, SAMPLE_SIZE))
			return -EFAULT;

		buf += SAMPLE_SIZE;
	}

	return i ? i : -EAGAIN;
}

/*
 * Open handler
 */
static int ef_open(struct inode *inode, struct file *filp)
{
	struct ef_device *evi = filp->private_data;

	if (MAJOR(filp->f_path.dentry->d_inode->i_rdev) == ef_major &&
	    MINOR(filp->f_path.dentry->d_inode->i_rdev) == ef_minor) {
		filp->private_data = container_of(inode->i_cdev,
						  struct ef_device, data_cdev);
	} else {
		dev_err(evi->dev, "Error opening device\n");
		return -ENODEV;
	}

	return 0;
}

void evi_read_data(void *dest, size_t len, void __iomem *src)
{
	int i;

	for (i = 0; i < len; i += 4)
		*(int32_t *)(dest + i) = readl_relaxed(src + i);

	barrier();
}

static long ef_start_data_flow(struct ef_device *evi)
{
	char buf[64];

	/* read DIFF_X to clear the irq line */
	evi_read_data(buf, sizeof(buf), evi->base + EVI_DATABUF);
	return 0;
}

static long ef_stop_data_flow(struct ef_device *evi)
{
	return 0;
}

static long ef_start_hw(struct ef_device *evi)
{
	writel_relaxed(0, evi->base + EVI_STARTPROCESSING);
	return 0;
}

static long ef_stop_hw(struct ef_device *evi)
{
	int i;

	writel_relaxed(0, evi->base + EVI_STOPPROCESSING);

	for (i = 0; i < 20; i++) {
		union ef_fifo_status status;

		status.raw_data = readl_relaxed(evi->base +
				EVI_FIFO_STATUS);

		if (status.stopped)
			return 0;

		msleep(50);
	}

	dev_err(evi->dev, "Timed out waiting for the HW to STOP\n");
	return -EAGAIN;
}

static long ef_read_reg(struct ef_device *evi, __user __u32 *reg)
{
	__u32 addr;
	__u32 val;

	if (unlikely(get_user(addr, reg)))
		return -EFAULT;

	val = readl_relaxed(evi->base + addr);
	return put_user(val, reg);
}

static long ef_write_reg(struct ef_device *evi,
			 __user struct reg_write_info *reg)
{
	struct reg_write_info tmp;

	if (unlikely(copy_from_user(&tmp, reg, sizeof(tmp))))
		return -EFAULT;

	writel_relaxed(tmp.value, evi->base + tmp.addr);

	return 0;
}

static long ef_ioctl(struct file *filp, unsigned int command, unsigned long arg)
{
	long ret = 0;
	struct ef_device *evi = filp->private_data;

	if (down_interruptible(&evi->access))
		return -ERESTARTSYS;

	switch (command) {
	case EVI_EDDY_VERSION:
		ret = copy_to_user((char *)arg, UTS_RELEASE,
				   strlen(UTS_RELEASE));
		break;
	case EVI_EDDY_START_DATA_FLOW:
		ret = ef_start_data_flow(evi);
		break;
	case EVI_EDDY_STOP_DATA_FLOW:
		ret = ef_stop_data_flow(evi);
		break;
	case  EVI_EDDY_START_HW:
		ret = ef_start_hw(evi);
		break;
	case EVI_EDDY_STOP_HW:
		ret = ef_stop_hw(evi);
		break;
	case EVI_EDDY_READ_REG:
		ret = ef_read_reg(evi, (__user u32 *)arg);
		break;
	case EVI_EDDY_WRITE_REG:
		ret = ef_write_reg(evi, (__user struct reg_write_info *)arg);
		break;
	default:
		ret = scanner_ioctl(filp, command, arg, &evi->ss);
		break;
	}

	up(&evi->access);

	return ret;
}

static int ef_release(struct inode *inode, struct file *filp)
{
	struct ef_device *evi = filp->private_data;

	ef_stop_hw(evi);
	dev_dbg(evi->dev, "EVi read device closed\n");
	return 0;
}

static void ef_vma_open(struct vm_area_struct *vma)
{
}

static void ef_vma_close(struct vm_area_struct *vma)
{
}

static const struct vm_operations_struct ef_mmap_vm_ops = {
	.open               = ef_vma_open,
	.close              = ef_vma_close,
};

static int ef_mmap_hw(struct file *filp, struct vm_area_struct *vma)
{
	int ret;
	size_t size = vma->vm_end - vma->vm_start;
	struct ef_device *evi = filp->private_data;
	resource_size_t res_size = resource_size(evi->res);

	if (size <= res_size) {
		dev_info(evi->dev, "request mmap 0x%x of 0x%zx bytes\n",
			 size, res_size);
	} else {
		dev_err(evi->dev, "Error: mmap %zx cannot exceed 0x%zx\n",
			size, res_size);
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	ret = io_remap_pfn_range(vma, vma->vm_start,
				 evi->res->start >> PAGE_SHIFT, size,
				 vma->vm_page_prot);
	if (ret) {
		dev_err(evi->dev, "io_remap_pfn_range failed: %d\n", ret);
		return ret;
	}

	return ret;
}

static int ef_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;
	struct ef_device *evi = filp->private_data;

	if (down_interruptible(&evi->access))
		return -ERESTARTSYS;

	vma->vm_private_data = evi;
	vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP | VM_LOCKED;

	switch (vma->vm_pgoff << PAGE_SHIFT) {
	case 0:
		ret = ef_mmap_hw(filp, vma);
		break;

	default:
		dev_err(evi->dev, "Could not map memory. Invalid offset specified (0x%016lX).\n",
			(vma->vm_pgoff << PAGE_SHIFT));
		return -EINVAL;
	}

	if (!ret) {
		vma->vm_ops = &ef_mmap_vm_ops;
		ef_vma_open(vma);
	}

	up(&evi->access);

	return ret;
}

static const struct file_operations ef_data_fops = {
	.owner              = THIS_MODULE,
	.read               = ef_data_read,
	.open               = ef_open,
	.release            = ef_release,
	.llseek             = no_llseek,
	.unlocked_ioctl     = ef_ioctl,
	.mmap               = ef_mmap,
};

static int init_character_device(struct ef_device *evi, struct device *parent)
{
	int ret;
	struct device *device = NULL;

	evi->data_dev_num = MKDEV(ef_major, ef_minor);
	ret = register_chrdev_region(evi->data_dev_num, 1, DEVICE_NAME);
	if (ret < 0) {
		dev_err(parent, "Error registering character device\n");
		return ret;
	}

	cdev_init(&evi->data_cdev, &ef_data_fops);
	evi->data_cdev.owner = THIS_MODULE;
	evi->data_cdev.ops = &ef_data_fops;
	ret = cdev_add(&evi->data_cdev, evi->data_dev_num, 1);
	if (ret < 0) {
		dev_err(parent, "Error adding character device\n");
		goto evi_char_init_dev_add;
	}

	evi->dev = device_create(&ef_class, parent, evi->data_dev_num, evi,
				 DEVICE_NAME "%d", ef_minor);
	if (IS_ERR(device)) {
		ret = PTR_ERR(device);
		dev_err(parent, "Error creating " DEVICE_NAME "%d\": %d\n",
			ef_minor, ret);
		goto evi_char_init_dev_create;
	}

	return ret;

evi_char_init_dev_create:
	cdev_del(&evi->data_cdev);

evi_char_init_dev_add:
	unregister_chrdev_region(evi->data_dev_num, 1);

	return ret;
}

static void ef_free_char_devices(struct ef_device *evi)
{
	device_destroy(&ef_class, evi->data_dev_num);
	cdev_del(&evi->data_cdev);
	unregister_chrdev_region(evi->data_dev_num, 1);
}

static int ef_hw_verify(struct ef_device *evi)
{
	u32 ef_ver = readl_relaxed(evi->base + EVI_VERSION);

	dev_notice(evi->dev, "evi eddy device version: %08X\r\n", ef_ver);
	if (ef_ver == 0xffffffff || ef_ver == 0) {
		dev_err(evi->dev, "HW Version invalid!\n");
		return -EPROBE_DEFER;
	}

	return 0;
}

static int of_ef_probe(struct platform_device *pdev)
{
	int ret = 0;
	const struct of_device_id *match;
	struct ef_device *evi;

	match = of_match_device(of_ef_match, &pdev->dev);
	if (!match)
		return -EINVAL;

	evi = kzalloc(sizeof(*evi), GFP_KERNEL);
	platform_set_drvdata(pdev, evi);
	sema_init(&evi->access, 1);

	ret = init_character_device(evi, &pdev->dev);
	if (ret)
		return ret;

	evi->res = devm_kzalloc(evi->dev, sizeof(*evi->res), GFP_KERNEL);
	ret = of_address_to_resource(pdev->dev.of_node, 0, evi->res);
	if (ret) {
		dev_err(evi->dev, "of_address_to_resource failed: %d\n", ret);
		return ret;
	}

	evi->base = devm_ioremap_resource(evi->dev, evi->res);
	if (IS_ERR(evi->base)) {
		ret = PTR_ERR(evi->base);
		dev_err(evi->dev, "Could not map hw: %d\n", ret);
		return ret;
	}
	if (ret)
		goto ef_hw_chardev_remove;

	ret = ef_hw_verify(evi);
	if (ret)
		goto ef_hw_chardev_remove;

	evi->ss.irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	evi->ss.statusirq = irq_of_parse_and_map(pdev->dev.of_node, 1);
	evi->ss.dev = &pdev->dev;
	evi->ss.status = evi->base + EVI_STATUS;
	evi->ss.base = evi->base + EVI_SCANNERSEND;
	ret = ef_scannerirq_probe(&evi->ss);
	if (ret)
		goto ef_hw_chardev_remove;

	return ret;

ef_hw_chardev_remove:
	ef_free_char_devices(evi);

	return ret;
}

static int of_ef_remove(struct platform_device *pdev)
{
	struct ef_device *evi = platform_get_drvdata(pdev);

	ef_scannerirq_remove(&evi->ss);
	ef_free_char_devices(evi);

	dev_notice(evi->dev, "removed\n");
	return 0;
}

static struct platform_driver of_ef_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_ef_match,
	},
	.probe = of_ef_probe,
	.remove = of_ef_remove,
};

static struct pci_device_id  pci_ef_ids[] = {
	{
		.vendor = 0x1172,
		.device = 0xe003,
		.subvendor = PCI_ANY_ID,
		.subdevice = PCI_ANY_ID,
		.class = 0,
		.class_mask = 0,
	},
	{ 0,}
};
MODULE_DEVICE_TABLE(pci, pci_ef_ids);

#define EVI_PCI_BAR 2

static int pci_ef_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int nvec = 3;
	int ret;
	struct ef_device *evi;

	evi = kzalloc(sizeof(*evi), GFP_KERNEL);
	pci_set_drvdata(pdev, evi);
	sema_init(&evi->access, 1);

	ret = pci_enable_device(pdev);
	if (ret)
		return ret;

	ret = pci_request_region(pdev, EVI_PCI_BAR, "evi_eddy_pcie");
	if (ret)
		goto err_out;

	evi->res = &pdev->resource[EVI_PCI_BAR];
	evi->base = pcim_iomap(pdev, EVI_PCI_BAR, resource_size(evi->res));
	if (ret)
		return ret;

	ret = ef_hw_verify(evi);
	if (ret)
		return ret;

	nvec = pci_alloc_irq_vectors(pdev, 1, nvec, PCI_IRQ_ALL_TYPES);
	if (nvec < 0)
		return nvec;

	evi->ss.statusirq = pci_irq_vector(pdev, 0);
	evi->ss.irq = pci_irq_vector(pdev, 1);
	evi->ss.dev = &pdev->dev;
	evi->ss.status = evi->base + EVI_STATUS;
	evi->ss.base = evi->base + EVI_SCANNERSEND;
	ret = ef_scannerirq_probe(&evi->ss);
	if (ret)
		return ret;

	ret = init_character_device(evi, &pdev->dev);
	if (ret)
		return ret;

	return 0;
err_out:
	pci_free_irq_vectors(pdev);
	pci_disable_device(pdev);
	iounmap(evi->base);
	pci_release_region(pdev, 4);

	return ret;
}

static void pci_ef_remove(struct pci_dev *pdev)
{
	struct ef_device *evi = pci_get_drvdata(pdev);

	pci_free_irq_vectors(pdev);
	pci_disable_device(pdev);
	iounmap(evi->base);
	pci_release_region(pdev, 4);
}

static struct pci_driver pci_ef_driver = {
	.name = "evi-eddy-pcie",
	.id_table = pci_ef_ids,
	.probe = pci_ef_probe,
	.remove = pci_ef_remove,
};

static void __exit ef_exit(void)
{
	platform_driver_unregister(&of_ef_driver);
	pci_unregister_driver(&pci_ef_driver);
	class_unregister(&ef_class);
}

static int __init ef_init(void)
{
	int ret;

	ret = class_register(&ef_class);
	if (ret) {
		pr_err("Register class %s failed: %d\n", ef_class.name, ret);
		goto register_failed;
	}

	ret = platform_driver_register(&of_ef_driver);
	if (ret) {
		pr_err("Register %s failed %d:\n",
		       of_ef_driver.driver.name, ret);
		goto register_failed;
	}

	ret = pci_register_driver(&pci_ef_driver);
	if (ret) {
		pr_err("Register %s failed %d:\n",
		       pci_ef_driver.driver.name, ret);
		goto register_failed;
	}

	return 0;

register_failed:
	ef_exit();
	return ret;
}

module_init(ef_init);
module_exit(ef_exit);

MODULE_DEVICE_TABLE(of, of_ef_match);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("EVi (c) Kernel Module 2013 United Western Technologies, Corporation");
