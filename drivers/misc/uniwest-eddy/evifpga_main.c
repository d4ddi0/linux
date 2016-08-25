/**
 * Copyright (c) 2015 United Western Technologies, Corporation
 *
 * evi-fpga collects eddy current data over weim bus
 * There is also an spi interface used to load the fpga firmware
 *
 */

#include <generated/compile.h>
#include <generated/utsrelease.h>
#include <linux/bug.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/mman.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
//#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

/* headers private to the module */
#include "evi_fpga_registers.h"
#include "evi_ioctl.h"
#include "smartscanner.h"

#define DEVICE_NAME "evifpga"
#define SCANNER_CONNECTED (0x00010000)

struct ef_device {
	void __iomem *fpga_registers;
	uint32_t flags;
	dev_t data_dev_num;
	struct cdev data_cdev;
	struct device *dev;
	struct gpio_desc *statusirq_gpiod;
	unsigned int statusirq;
	struct smartscanner ss;
	u32 battbox_state;
	struct semaphore access;
	struct fpga_manager *fmgr;
	const char *fw_name;
};

static struct ef_device ef_device = {
	.fpga_registers = NULL,
};

static const struct of_device_id of_ef_match[] = {
	{ .compatible = "uniwest,evi-eddy", },
	{}
};

/* device major and minor numbers */
static const int ef_major = 42;
static const int ef_minor;

static struct class *ef_class;
static struct work_struct ef_status_work;

static inline u32 read_weim(const volatile void __iomem *addr)
{
	uint32_t value;

	value = readl_relaxed(addr);

	return value;
}

static inline void write_weim(u32 b, volatile void __iomem *addr)
{
	writel_relaxed(b, addr);
}

void ef_update_status(struct work_struct *work)
{
	struct ef_device *evi = &ef_device;
	uint32_t status, changes;

	status = readl_relaxed(evi->fpga_registers + EVI_FPGA_EIM_BATTBOXRECV);
	changes = status ^ evi->battbox_state;
	evi->battbox_state = status;

	if (!(changes & (1 << 31)))
		return;

	if (status & (1 << 31)) {
		msleep(100);
		status = readl_relaxed(evi->fpga_registers +
					EVI_FPGA_EIM_BATTBOXRECV);
		if (status & (1 << 31)) {
			evi->ss.msg = 0x00ff0000;
			evi->flags |= SCANNER_CONNECTED;
			evi->ss.connected = true;
		}
	} else {
		int i;

		evi->flags = 0;
		evi->ss.connected = false;
		for (i = 0; i < 8; i++) {
			readl_relaxed(evi->fpga_registers +
				      EVI_FPGA_EIM_SCANNERRECV);
		}
	}
}

static irqreturn_t ef_handle_statusirq(int irq, void *dev)
{
	schedule_work(&ef_status_work);
	return IRQ_HANDLED;
}

int ef_statusirq_probe(struct ef_device *evi)
{
	int ret = 0;

	evi->statusirq_gpiod = devm_gpiod_get(evi->dev, "statusirq", GPIOD_IN);
	if (IS_ERR(evi->statusirq_gpiod)) {
		dev_err(evi->dev, "Failed to get statusirq gpio: %ld\n",
			PTR_ERR(evi->statusirq_gpiod));
		return PTR_ERR(evi->statusirq_gpiod);
	}

	evi->statusirq = gpiod_to_irq(evi->statusirq_gpiod);
	irq_set_irq_type(evi->statusirq, IRQ_TYPE_EDGE_RISING);
	ret = request_irq(evi->statusirq, ef_handle_statusirq,
			0, "evifpga-status", evi);
	if (ret) {
		dev_err(evi->dev, "Error requesting statusirq: %d\n", ret);
		return ret;
	}

	INIT_WORK(&ef_status_work, ef_update_status);
	schedule_work(&ef_status_work);

	return ret;
}

static void ef_statusirq_remove(struct ef_device *evi)
{
	disable_irq(evi->statusirq);
	free_irq(evi->statusirq, &evi->statusirq);
}

/**
 * Character Device read handler. No read op on test character device.
 */
static ssize_t ef_data_read(struct file *filp, char __user *buf, size_t count,
			    loff_t *f_pos)
{
	return -EINVAL;
}

/*
 * Open handler
 */
static int ef_open(struct inode *inode, struct file *filp)
{
	if (MAJOR(filp->f_path.dentry->d_inode->i_rdev) == ef_major &&
		MINOR(filp->f_path.dentry->d_inode->i_rdev) == ef_minor) {
		filp->private_data = container_of(inode->i_cdev,
						  struct ef_device, data_cdev);
	} else {
		dev_err(ef_device.dev, "Error opening device\n");
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

/*
 * ef_start_data_flow()
 *
 * Assumes that interrupts are enabled and that FPGA is running
 * and that the interrupt state is FPGA_INTERRUPT_IDLE
 */
static long ef_start_data_flow(struct ef_device *evi)
{
	char buf[64];

	/* read FPGA_DATA_DIFF_X to clear the irq line */
	evi_read_data(buf, sizeof(buf),
			ef_device.fpga_registers + EVI_FPGA_EIM_DATABUF);
	return 0;
}

static long ef_stop_data_flow(struct ef_device *evi)
{
	return 0;
}

static long ef_start_fpga(struct ef_device *evi)
{
	write_weim(0, evi->fpga_registers + EVI_FPGA_EIM_STARTPROCESSING);
	return 0;
}

static long ef_stop_fpga(struct ef_device *evi)
{
	int i;

	write_weim(0, evi->fpga_registers + EVI_FPGA_EIM_STOPPROCESSING);

	for (i = 0; i < 20; i++) {
		union ef_fifo_status status;

		status.raw_data = read_weim(evi->fpga_registers +
				EVI_FPGA_EIM_FIFO_STATUS);

		if (status.stopped)
			return 0;

		msleep(50);
	}

	dev_err(evi->dev, "Timed out waiting for the FPGA to STOP\n");
	return -EAGAIN;
}

static long ef_ioctl(struct file *filp, unsigned int command, unsigned long arg)
{
	long ret = 0;
	struct ef_device *evi = filp->private_data;

	if (down_interruptible(&evi->access))
		return -ERESTARTSYS;

	switch (command) {
	case EVI_FPGA_VERSION:
		ret = copy_to_user((char *)arg, UTS_RELEASE,
				    strlen(UTS_RELEASE));
		break;
	case EVI_FPGA_START_DATA_FLOW:
		ret = ef_start_data_flow(evi);
		break;
	case EVI_FPGA_STOP_DATA_FLOW:
		ret = ef_stop_data_flow(evi);
		break;
	case  EVI_FPGA_START_FPGA:
		ret = ef_start_fpga(evi);
		break;
	case EVI_FPGA_STOP_FPGA:
		ret = ef_stop_fpga(evi);
		break;
	case EVI_FPGA_SCANNER_STATUS:
		ret = copy_to_user((char *)arg, &evi->flags,
				   sizeof(evi->flags));
		break;
	case EVI_FPGA_SCANNER_CMD:
		ret = scanner_cmd(&evi->ss, (void *)arg);
		break;
	case EVI_FPGA_SCANNER_FIRMWARE:
		ret = ef_load_scanner_firmware(&evi->ss, (char *)arg);
		break;
	default:
		dev_warn(evi->dev, "Invalid IOCTL called: %d\n", command);
		ret = -ENOTTY;
		break;
	}

	up(&evi->access);

	return ret;
}

static int ef_release(struct inode *inode, struct file *filp)
{
	ef_stop_fpga(&ef_device);
	dev_dbg(ef_device.dev, "EVi read device closed\n");
	return 0;
}

static void ef_vma_open(struct vm_area_struct *vma)
{
}

static void ef_vma_close(struct vm_area_struct *vma)
{
}

static struct vm_operations_struct ef_mmap_vm_ops = {
	.open               = ef_vma_open,
	.close              = ef_vma_close,
};

static int ef_mmap_eim(struct file *filp, struct vm_area_struct *vma)
{
	int ret;
	size_t size = vma->vm_end - vma->vm_start;
	struct ef_device *evi = filp->private_data;
	struct resource res;

	ret = of_address_to_resource(evi->dev->of_node, 0, &res);
	if (ret) {
		dev_err(evi->dev, "of_address_to_resource failed: %d\n", ret);
		return ret;
	}

	if (size <= resource_size(&res)) {
		dev_err(evi->dev, "request EIM map 0x%x of 0x%zx bytes\n",
			size, resource_size(&res));
	} else {
		dev_err(evi->dev, "Error: EIM mmap %zx cannot exceed 0x%zx\n",
			size, resource_size(&res));
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	ret = io_remap_pfn_range(vma, vma->vm_start, res.start >> PAGE_SHIFT,
			size, vma->vm_page_prot);
	if (ret) {
		dev_err(evi->dev, "io_remap_pfn_range EIM failed: %d\n", ret);
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
	case FPGA_VM_EIM_OFFSET:
		ret = ef_mmap_eim(filp, vma);
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

static int init_character_device(struct ef_device *evi)
{
	int ret_val;
	struct device *device = NULL;

	dev_dbg(evi->dev, "Creating character device(s) - ");
	ef_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (ef_class == NULL) {
		dev_err(evi->dev, "Could not create EVi device class\n");
		return PTR_ERR(ef_class);
	}

	evi->data_dev_num = MKDEV(ef_major, ef_minor);
	ret_val = register_chrdev_region(evi->data_dev_num, 1, "EViFPGA");
	if (ret_val < 0) {
		dev_err(evi->dev, "Error registering character device\n");
		goto evi_char_init_reg_region;
	}

	cdev_init(&evi->data_cdev, &ef_data_fops);
	evi->data_cdev.owner = THIS_MODULE;
	evi->data_cdev.ops = &ef_data_fops;
	ret_val = cdev_add(&evi->data_cdev, evi->data_dev_num, 1);
	if (ret_val < 0) {
		dev_err(evi->dev, "Error adding character device\n");
		goto evi_char_init_dev_add;
	}

	device = device_create(ef_class, NULL, evi->data_dev_num, NULL,
			       DEVICE_NAME "%d", ef_minor);
	if (IS_ERR(device)) {
		ret_val = PTR_ERR(device);
		dev_err(evi->dev,
			"Error trying to create device \"%s%d\" - %d\n",
			DEVICE_NAME, ef_minor, ret_val);
		goto evi_char_init_dev_create;
	}

	return ret_val;

evi_char_init_dev_create:
	cdev_del(&evi->data_cdev);

evi_char_init_dev_add:
	unregister_chrdev_region(evi->data_dev_num, 1);

evi_char_init_reg_region:
	class_destroy(ef_class);

	return ret_val;
}

static void ef_free_char_devices(struct ef_device *evi)
{
	device_destroy(ef_class, evi->data_dev_num);
	cdev_del(&evi->data_cdev);
	unregister_chrdev_region(evi->data_dev_num, 1);
	class_destroy(ef_class);
}

static int ef_eim_map(struct platform_device *pdev)
{
	struct resource res;
	struct ef_device *evi = &ef_device;
	int ret = 0;

	ret = of_address_to_resource(pdev->dev.of_node, 0, &res);
	if (ret) {
		dev_err(&pdev->dev, "of_address_to_resource failed: %d\n", ret);
		return ret;
	}

	/* devm_ioremap_resource automatically unmaps as the device is freed */
	evi->fpga_registers = devm_ioremap_resource(&pdev->dev, &res);
	evi->ss.dev = evi->dev;
	evi->ss.status = evi->fpga_registers + EVI_FPGA_EIM_STATUS;
	evi->ss.base = evi->fpga_registers + EVI_FPGA_EIM_SCANNERSEND;
	evi->ss.user_flags = &(evi->flags);
	if (IS_ERR(evi->fpga_registers)) {
		ret = PTR_ERR(evi->fpga_registers);
		dev_err(evi->dev, "Could not map eim: %d\n", ret);
	}

	return ret;
}

static int of_ef_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	const struct of_device_id *match;
	struct ef_device *evi = &ef_device;
	uint32_t ef_ver;

	match = of_match_device(of_ef_match, &pdev->dev);
	if (!match)
		return -EINVAL;

	ef_device.dev = &pdev->dev;
	sema_init(&evi->access, 1);

	ret_val = init_character_device(&ef_device);
	if (ret_val)
		return ret_val;

	ret_val = ef_eim_map(pdev);
	if (ret_val)
		goto ef_weim_chardev_remove;

	ret_val = ef_scannerirq_probe(&evi->ss);
	if (ret_val)
		goto ef_weim_chardev_remove;

	ret_val = ef_statusirq_probe(evi);
	if (ret_val)
		goto out_ss_irq;

	ef_ver = read_weim(ef_device.fpga_registers +
			EVI_FPGA_EIM_VERSION);

	dev_notice(evi->dev, "evi eddy device version: %08X\r\n", ef_ver);

	return ret_val;

out_ss_irq:
	ef_scannerirq_remove(&evi->ss);
ef_weim_chardev_remove:
	ef_free_char_devices(&ef_device);

	return ret_val;
}

static int of_ef_remove(struct platform_device *pdev)
{
	struct ef_device *evi = &ef_device;

	ef_scannerirq_remove(&ef_device.ss);
	ef_statusirq_remove(&ef_device);
	ef_free_char_devices(&ef_device);

	dev_notice(evi->dev, "removed\n");
	return 0;
}

static struct platform_driver of_ef_driver = {
	.driver = {
		.name = "evi-eddy",
		.owner = THIS_MODULE,
		.of_match_table = of_ef_match,
	},
	.probe = of_ef_probe,
	.remove = of_ef_remove,
};

static int __init ef_init(void)
{
	int ret;

	ret = platform_driver_register(&of_ef_driver);
	if (ret) {
		pr_err("Register %s failed %d:\n",
			of_ef_driver.driver.name, ret);
		return ret;
	}

	return ret;
}

static void __exit ef_exit(void)
{
	platform_driver_unregister(&of_ef_driver);
}

module_init(ef_init);
module_exit(ef_exit);

MODULE_DEVICE_TABLE(of, of_ef_match);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("EVi (c) Kernel Module 2013 United Western Technologies, Corporation");
