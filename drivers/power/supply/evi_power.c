/*
 * Driver for evi power supply board
 *
 * Copyright 2015 United Western Technologies
 *
 */

#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pm.h>
#include <linux/reboot.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#include "evi_power.h"

#define POWERBOARD_IOC_MAGIC	'C'
#define POWERBOARD_IOC_WRITE	_IOWR(POWERBOARD_IOC_MAGIC, 0, unsigned long)
#define POWERBOARD_IOC_READ	_IOWR(POWERBOARD_IOC_MAGIC, 1, unsigned long)
#define POWERBOARD_IOC_VERSION	_IOR(POWERBOARD_IOC_MAGIC, 2, unsigned long)

#define UNHANDLED_BITS 0xFFFFFFC4
#define DEVICE_NAME "powerboard"
int powerboard_major = 44;
int powerboard_minor = 0;

const char driver_name[] = DEVICE_NAME;

struct evi_pb {
	struct semaphore lock;
	struct spi_device *spi;
	struct class *powerboard_class;
	dev_t data_dev_num;
	struct cdev data_cdev;
	struct evi_power_version device_ver;
	struct work_struct work;
	struct pmic_data status;
	struct pmic_data last_status;
	struct motor_data motor_status[3];
	struct motor_data last_motor_status[3];
#if defined(CONFIG_HWMON)
	struct device *hwmon_dev;
#endif
};

struct powerboard_data {
	uint32_t reg_addr;
	union {
		unsigned char data[4];
		uint32_t reg;
	};
};

struct evi_power_cmd {
	uint8_t zero_pad;
	uint8_t start_byte;
	uint8_t reg_addr;
	uint8_t command;
};

static int pb_get_version(struct evi_pb *pb);
static long pb_read32(struct evi_pb *pb, uint32_t addr, void *buf);
static long pb_write32(struct evi_pb *pb, uint32_t addr, const void *buf);
static void pb_power_off(void);
static int pb_probe(struct spi_device *spi);
static int pb_remove(struct spi_device *spi);
static int pb_open(struct inode *inode, struct file *filp);
static long pb_ioctl_read(unsigned long user_addr, struct evi_pb *pb);
static long pb_ioctl_write(unsigned long user_addr, struct evi_pb *pb);
static long pb_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int init_character_device(struct evi_pb *pb);
static void pb_free_char_devices(struct evi_pb *pb);

static unsigned int pb_irq;

static struct evi_pb pb_dev = {
};

const struct file_operations dev_fops = {
	.owner = THIS_MODULE,
	.read = NULL,
	.write = NULL,
	.open = pb_open,
	.unlocked_ioctl = pb_ioctl,
	.llseek = no_llseek,
};

static irqreturn_t pb_handle_irq(int irq, void *spi)
{
	schedule_work(&pb_dev.work);
	return IRQ_HANDLED;
}
static void __pmic_status_msg(struct evi_pb *pb, int st, int last_st, char *msg)
{
	char *state;

	if (st != last_st) {
		if (st)
			state = "FAIL";
		else
			state = "OKAY";
		dev_err(&pb->spi->dev, "[%s] %s\n", state, msg);
	}
}

#define str(s) #s
#define pmic_status_msg(p, f) \
	__pmic_status_msg(p, p->status.reg.f, \
				p->last_status.reg.f, str(f))

static void handle_pmic_fault(struct evi_pb *pb, uint32_t reg)
{

	int ret = pb_read32(pb, reg, &pb->status.raw);

	if (ret) {
		dev_alert(&pb->spi->dev, "pb_read32 failed with %d\n", ret);
		return;
	}

	if (!pb->status.reg.PWRON)
		dev_err(&pb->spi->dev, "PMIC Communication Error\n");

	pmic_status_msg(pb, LOW_Vin);
	pmic_status_msg(pb, Therm110);
	pmic_status_msg(pb, Therm120);
	pmic_status_msg(pb, Therm125);
	pmic_status_msg(pb, Therm130);
	pmic_status_msg(pb, SW1Afault);
	pmic_status_msg(pb, SW1Bfault);
	pmic_status_msg(pb, SW1Cfault);
	pmic_status_msg(pb, SW2fault);
	pmic_status_msg(pb, SW3Afault);
	pmic_status_msg(pb, SW3Bfault);
	pmic_status_msg(pb, SW4fault);
	pmic_status_msg(pb, SWBSTfault);
	pmic_status_msg(pb, VGEN1fault);
	pmic_status_msg(pb, VGEN2fault);
	pmic_status_msg(pb, VGEN3fault);
	pmic_status_msg(pb, VGEN4fault);
	pmic_status_msg(pb, VGEN5fault);
	pmic_status_msg(pb, VGEN6fault);
	pmic_status_msg(pb, OTP_ECC);
	pb->last_status.raw = pb->status.raw;
#if defined(CONFIG_HWMON)
	sysfs_notify(&pb->spi->dev.kobj, NULL, "in0_alarm");
	kobject_uevent(&pb->spi->dev.kobj, KOBJ_CHANGE);
#endif
}

#if defined(CONFIG_HWMON)
static ssize_t show_pmic_alarm(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute *da = to_sensor_dev_attr(attr);
	uint8_t channel = da->index;

	return sprintf(buf, "%d\n", pb_dev.last_status.raw & (1 << channel));
}

static ssize_t show_in1_alarm(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", pb_dev.last_motor_status[0].raw);
}

static ssize_t show_in2_alarm(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", pb_dev.last_motor_status[1].raw);
}

static ssize_t show_in3_alarm(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", pb_dev.last_motor_status[2].raw);
}

static SENSOR_DEVICE_ATTR(in0_lcrit_alarm, S_IRUGO, show_pmic_alarm, NULL, 1);
static SENSOR_DEVICE_ATTR(temp1_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 2);
static SENSOR_DEVICE_ATTR(temp1_crit_alarm, S_IRUGO, show_pmic_alarm, NULL, 3);
static SENSOR_DEVICE_ATTR(temp1_emergency_alarm,
			S_IRUGO,
			show_pmic_alarm,
			NULL,
			5);
static SENSOR_DEVICE_ATTR(curr1_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 8);
static SENSOR_DEVICE_ATTR(curr2_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 9);
static SENSOR_DEVICE_ATTR(curr3_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 10);
static SENSOR_DEVICE_ATTR(curr4_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 11);
static SENSOR_DEVICE_ATTR(curr5_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 12);
static SENSOR_DEVICE_ATTR(curr6_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 14);
static SENSOR_DEVICE_ATTR(curr7_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 15);
static SENSOR_DEVICE_ATTR(curr8_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 17);
static SENSOR_DEVICE_ATTR(curr9_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 24);
static SENSOR_DEVICE_ATTR(curr10_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 25);
static SENSOR_DEVICE_ATTR(curr11_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 26);
static SENSOR_DEVICE_ATTR(curr12_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 27);
static SENSOR_DEVICE_ATTR(curr13_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 28);
static SENSOR_DEVICE_ATTR(curr14_max_alarm, S_IRUGO, show_pmic_alarm, NULL, 29);



static struct attribute *powerboard_attr[] = {
	&sensor_dev_attr_in0_lcrit_alarm.dev_attr.attr,
	&sensor_dev_attr_temp1_max_alarm.dev_attr.attr,
	&sensor_dev_attr_temp1_crit_alarm.dev_attr.attr,
	&sensor_dev_attr_temp1_emergency_alarm.dev_attr.attr,
	&sensor_dev_attr_curr1_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr2_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr3_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr4_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr5_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr6_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr7_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr8_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr9_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr10_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr11_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr12_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr13_max_alarm.dev_attr.attr,
	&sensor_dev_attr_curr14_max_alarm.dev_attr.attr,
	NULL
};

static const struct attribute_group powerboard_group = {
	.name = "pmic0",
	.attrs = powerboard_attr,
};

static const struct attribute_group *powerboard_groups[] = {
	&powerboard_group,
	NULL
};

static void notify_motor_fault(struct evi_pb *pb, uint32_t motor)
{
	switch (motor) {
	case 0:
		sysfs_notify(&pb->spi->dev.kobj, NULL, "in1_alarm");
		kobject_uevent(&pb->spi->dev.kobj, KOBJ_CHANGE);
	break;
	case 1:
		sysfs_notify(&pb->spi->dev.kobj, NULL, "in2_alarm");
		kobject_uevent(&pb->spi->dev.kobj, KOBJ_CHANGE);
	break;
	case 2:
		sysfs_notify(&pb->spi->dev.kobj, NULL, "in3_alarm");
		kobject_uevent(&pb->spi->dev.kobj, KOBJ_CHANGE);
	break;
	}

}
#else
static void notify_motor_fault(struct evi_pb *pb, uint32_t motor)
{
}
#endif

static void __motor_status_msg(struct evi_pb *pb, int st, int last_st, char *msg
				, int motor_num)
{
	char *state;

	if (st != last_st) {
		if (st)
			state = "FAIL";
		else
			state = "OKAY";
		dev_err(&pb->spi->dev, "[%s] %s[%i]\n", state, msg, motor_num);
	}
}

#define motor_status_msg(p, i, f) \
	__motor_status_msg(p, p->motor_status[i].reg.f, \
				p->last_motor_status[i].reg.f, str(f), i)


static void handle_motor_fault(struct evi_pb *pb, uint32_t reg, uint32_t motor)
{
	int ret = pb_read32(pb, reg, &pb->motor_status[motor].raw);

	if (ret) {
		dev_alert(&pb->spi->dev, "pb_read32 failed with %d\n", ret);
		return;
	}

	motor_status_msg(pb, motor, pwr_good);
	motor_status_msg(pb, motor, alert);
	motor_status_msg(pb, motor, pwr_monitor_err);
	pb->last_motor_status[motor].raw = pb->motor_status[motor].raw;
	notify_motor_fault(pb, motor);
}

static void pb_irq_work(struct work_struct *work)
{
	int ret;
	struct evi_pb *pb = &pb_dev;
	struct power_data status = {
		.raw = 0,
	};

	ret = pb_read32(pb, 2, &status.raw);

	if (ret)
		dev_alert(&pb->spi->dev, "pb_read32 failed with %d\n", ret);

	if (status.raw & UNHANDLED_BITS)
		dev_info(&pb->spi->dev, "read 0x%8.8X from power status\n",
			 status.raw);

	if (status.reg.shutting_down) {
		disable_irq(pb_irq);
		orderly_poweroff(true);
	}

	if (status.reg.pmic0_fault)
		handle_pmic_fault(pb, 0x0e);

	if (status.reg.motor0_fault)
		handle_motor_fault(pb, 0x05, 0);

	if (status.reg.motor1_fault)
		handle_motor_fault(pb, 0x08, 1);

	if (status.reg.motor2_fault)
		handle_motor_fault(pb, 0x0B, 2);

}

static int pb_irq_init(struct spi_device *spi)
{
	int ret;

	pb_irq = irq_of_parse_and_map(spi->dev.of_node, 0);
	if (!pb_irq) {
		dev_alert(&spi->dev, "failed to get pb irq\n");
		return -ENODEV;
	}
	ret = request_irq(pb_irq, pb_handle_irq, IRQF_TRIGGER_RISING,
			  "EVi pb request irq", spi);
	if (ret)
		dev_alert(&spi->dev, "power gpio irq request failed with %d\n",
			  ret);

	return ret;
}

/* SPI device code */

static int pb_probe(struct spi_device *spi)
{
	int ret_val = 0;

	dev_dbg(&spi->dev, "Initializing\n");
	memset(&pb_dev, 0, sizeof(struct evi_pb));
	sema_init(&pb_dev.lock, 1);

	pb_dev.spi = spi;
	ret_val = pb_get_version(&pb_dev);
	if (ret_val)
		return ret_val;

	if (down_interruptible(&pb_dev.lock))
		return -EBUSY;

	ret_val = init_character_device(&pb_dev);
	if (ret_val)
		return ret_val;
	pm_power_off = pb_power_off;
	ret_val = pb_irq_init(spi);
	if (ret_val)
		return ret_val;
#if defined(CONFIG_HWMON)
	pb_dev.hwmon_dev = devm_hwmon_device_register_with_groups(
				&spi->dev, "powerboard", &pb_dev,
				powerboard_groups);
#endif

	INIT_WORK(&pb_dev.work, pb_irq_work);
	up(&pb_dev.lock);
	return 0;
}

static int pb_remove(struct spi_device *spi)
{
	if (down_interruptible(&pb_dev.lock))
		return -EBUSY;

	pb_free_char_devices(&pb_dev);
	pb_dev.spi = NULL;
	dev_dbg(&spi->dev, "removed\n");
	up(&pb_dev.lock);
	return 0;
}

static int pb_open(struct inode *inode, struct file *filp)
{
	pr_debug("Powerboard Spi device opened\n");
	filp->private_data = &pb_dev;

	return 0;
}

static long pb_op_init(struct evi_pb *pb, uint8_t reg, uint8_t cmd_type)
{
	char output_reg;
	int ret;

	switch (cmd_type) {
	case EVI_POWER_SPI_READ_REGISTER:
		output_reg = reg << 1;
		break;
	case EVI_POWER_SPI_WRITE_REGISTER:
		output_reg = (reg << 1) + 1;
		break;
	default:
		return -ENOTTY;
	}
	ret = spi_write(pb->spi, &output_reg, 1);
	if (!ret)
		udelay(1000);

	return ret;
}

static long pb_read_bytes(struct evi_pb *pb, char *dest, int bytes)
{
	int i;

	memset(dest, '\0', bytes);
	for (i = 0; i < bytes; i++) {
		if (spi_read(pb->spi, &dest[i], 1) < 0)
			break;
	}
	if (i < bytes)
		pr_err("Read only %d of %d bytes from evi_pb\n", i, bytes);

	return i;
}

static long pb_read32(struct evi_pb *pb, uint32_t addr, void *buf)
{
	long ret;

	if (down_interruptible(&pb->lock))
		return -ERESTARTSYS;

	ret = pb_op_init(pb, (uint8_t)addr, EVI_POWER_SPI_READ_REGISTER);
	if (ret < 0) {
		pr_err("Unable to init evi_pb read: %ld\n", ret);
		goto pb_read32_fail;
	}

	ret = pb_read_bytes(pb, buf, 4) - 4;

pb_read32_fail:
	up(&pb->lock);
	return ret;
}

static long pb_ioctl_read(unsigned long user_addr, struct evi_pb *pb)
{
	long ret = 0;
	struct powerboard_data request;

	if (copy_from_user((void *)&request, (void *)user_addr,
	     sizeof(struct powerboard_data))) {
		pr_err("evi_pb: copy_from_user failed\n");
		ret = -1;
		goto evi_pb_ioctl_read_exit;
	}
	ret = pb_read32(pb, request.reg_addr, request.data);
	if (ret)
		goto evi_pb_ioctl_read_exit;

	if (copy_to_user((void *)user_addr, (void *)&request,
			 sizeof(struct powerboard_data))) {
		pr_err("Unable to copy data read from evi_pb to user-space\n");
		ret = -1;
	}

evi_pb_ioctl_read_exit:
	return ret;
}

static long pb_write32(struct evi_pb *pb, uint32_t addr, const void *buf)
{
	long ret;
	int i;

	if (down_interruptible(&pb->lock))
		return -ERESTARTSYS;

	ret = pb_op_init(pb, (uint8_t)addr, EVI_POWER_SPI_WRITE_REGISTER);
	if (ret < 0) {
		pr_err("Unable to init evi_pb write: %ld\n", ret);
		goto pb_write32_fail;
	}

	for (i = 0; i < 4; ++i) {
		ret = spi_write(pb->spi, buf + i, 1);
		if (ret) {
			pr_err("Unable to write to evi_pb: %ld\n", ret);
			break;
		}
	}

pb_write32_fail:
	up(&pb->lock);
	return ret;
}

static long pb_ioctl_write(unsigned long user_addr, struct evi_pb *pb)
{
	struct powerboard_data request;
	long ret;

	if (copy_from_user((void *)&request, (void *)user_addr,
			    sizeof(struct powerboard_data))) {
		pr_err("Unable to copy evi_pb data from user-space for WRITE\n");
		ret = -1;
		goto evi_pb_ioctl_write_exit;
	}
	ret = pb_write32(pb, request.reg_addr, request.data);

evi_pb_ioctl_write_exit:
	return ret;
}

static long pb_version_ioctl(void __user *user_addr, struct evi_pb *pb)
{
	unsigned long missed_bytes;

	missed_bytes = copy_to_user(user_addr, (void *)&pb->device_ver,
				    sizeof(struct evi_power_version));
	if (missed_bytes) {
		pr_err("evi_pb_version_ioctl failed to copy %lu bytes\n",
			missed_bytes);
		return -EFAULT;
	}
	return 0;
}

static long pb_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct evi_pb *pb = filp->private_data;

	switch (cmd) {
	case POWERBOARD_IOC_READ:
		return pb_ioctl_read(arg, pb);
	case POWERBOARD_IOC_WRITE:
		return pb_ioctl_write(arg, pb);
	case POWERBOARD_IOC_VERSION:
		return pb_version_ioctl((void __user *)arg, pb);
	default:
		dev_err(&pb->spi->dev, "Invalid IOCTL called: %d\n", cmd);
		return -ENOTTY;
	}
	return 0;
}

static int init_character_device(struct evi_pb *pb)
{
	int ret_val;
	struct device *device = NULL;

	pr_debug("Creating character device(s) - ");

	pb->powerboard_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (pb->powerboard_class == NULL) {
		pr_err("Could not create EVi device class\n");
		return PTR_ERR(pb->powerboard_class);
	}
	pb->data_dev_num = MKDEV(powerboard_major, powerboard_minor);

	ret_val =
	    register_chrdev_region(pb->data_dev_num, 1, "EViPowerboard");

	if (ret_val < 0) {
		pr_err("Error registering character device\n");
		goto evi_char_init_reg_region;
	}
	cdev_init(&pb->data_cdev, &dev_fops);

	pb->data_cdev.owner = THIS_MODULE;
	pb->data_cdev.ops = &dev_fops;

	ret_val = cdev_add(&pb->data_cdev, pb->data_dev_num, 1);

	if (ret_val < 0) {
		pr_err("Error adding character device\n");
		goto evi_char_init_dev_add;
	}

	device = device_create(pb->powerboard_class, NULL, pb->data_dev_num,
				NULL, DEVICE_NAME "%d", powerboard_minor);

	if (IS_ERR(device)) {
		ret_val = PTR_ERR(device);
		pr_err("Error trying to create device \"%s%d\" - %d\n",
		       DEVICE_NAME, powerboard_minor, ret_val);
		goto evi_char_init_dev_create;
	}

	return ret_val;

evi_char_init_dev_create:
	cdev_del(&pb->data_cdev);

evi_char_init_dev_add:
	unregister_chrdev_region(pb->data_dev_num, 1);

evi_char_init_reg_region:
	class_destroy(pb->powerboard_class);

	return ret_val;
}

static void pb_free_char_devices(struct evi_pb *pb)
{
	device_destroy(pb->powerboard_class, pb->data_dev_num);
	cdev_del(&pb->data_cdev);
	unregister_chrdev_region(pb->data_dev_num, 1);
	class_destroy(pb->powerboard_class);
}

static int pb_get_version(struct evi_pb *pb)
{
	int ret;

	pb->device_ver.major = 0;
	ret = pb_op_init(pb, 1, EVI_POWER_SPI_READ_REGISTER);
	if (!ret)
		ret = pb_read_bytes(pb, (char *)&pb->device_ver, 4) - 4;

	if (pb->device_ver.major == 0 &&
	    pb->device_ver.minor == 0 &&
	    pb->device_ver.rev == 0 &&
	    pb->device_ver.subrev == 0) {
		/* failed read. try new protocol */
		pr_err("Failed to get to powerboard version with legacy protocol\n");
		pb->device_ver.major = 255;
		ret = pb_op_init(pb, 1, EVI_POWER_SPI_READ_REGISTER);
		if (!ret)
			ret = pb_read_bytes(pb, (char *)&pb->device_ver, 4) - 4;
	}
	if (pb->device_ver.major == 0 &&
	    pb->device_ver.minor == 0 &&
	    pb->device_ver.rev == 0 &&
	    pb->device_ver.subrev == 0) {
		pr_err("Failed to get to powerboard version.\n");
	}
	return ret;
}

/**
 * pb_poweroff - shut off system power from powerboard
 */
static void pb_power_off(void)
{
	char value[] = {0x01, 0x00, 0x00, 0x00};
	long ret;
	uint32_t addr;

	pr_alert("evi_pb_poweroff called\n");
	addr = (pb_dev.device_ver.major >= 1) ? 2 : 3;
	ret = pb_write32(&pb_dev, addr, value);
	if (ret)
		pr_alert("evi_pb_poweroff failed to send: %ld\n", ret);
	msleep(5000);
	pr_alert("evi_pb_poweroff: power not off after 5 seconds!\n");
}

#ifdef CONFIG_OF
static const struct of_device_id evi_pb_dt_ids[] = {
	{ .compatible = "uniwest,evi-pb", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, evi_pb_dt_ids);
#endif

static struct spi_driver powerboard_spi_driver = {
	.driver = {
		   .name = driver_name,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(evi_pb_dt_ids),
		   },
	.probe = pb_probe,
	.remove = pb_remove,
};
module_spi_driver(powerboard_spi_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Uniwest evi power driver");
