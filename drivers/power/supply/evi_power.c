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

#define PB_READ		(0x03 << 24)
#define PB_WRITE	(0x02 << 24)

int powerboard_major = 44;
int powerboard_minor = 0;

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
	int (*write_reg) (struct evi_pb *, int, u32);
	int (*read_reg) (struct evi_pb *, int, u32 *);
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

static unsigned int pb_irq;

static struct evi_pb pb_dev = {
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

	int ret = pb->read_reg(pb, reg, &pb->status.raw);

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

static ssize_t show_motor_alarm(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sensor_device_attribute *da = to_sensor_dev_attr(attr);
	uint8_t channel = da->index;

	return sprintf(buf, "%d\n",
			pb_dev.last_motor_status[channel].reg.alert);
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

static SENSOR_DEVICE_ATTR(power1_alarm, S_IRUGO, show_motor_alarm, NULL, 0);
static SENSOR_DEVICE_ATTR(power2_alarm, S_IRUGO, show_motor_alarm, NULL, 1);
static SENSOR_DEVICE_ATTR(power3_alarm, S_IRUGO, show_motor_alarm, NULL, 2);

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

static struct attribute *motors_attr[] = {
	&sensor_dev_attr_power1_alarm.dev_attr.attr,
	&sensor_dev_attr_power2_alarm.dev_attr.attr,
	&sensor_dev_attr_power3_alarm.dev_attr.attr,
	NULL
};
static const struct attribute_group powerboard_group = {
	.attrs = powerboard_attr,
};
static const struct attribute_group motors_group = {
	.attrs = motors_attr,
};

static const struct attribute_group *powerboard_groups[] = {
	&powerboard_group,
	&motors_group,
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
	int ret = pb->read_reg(pb, reg, &pb->motor_status[motor].raw);

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

	ret = pb->read_reg(pb, 2, &status.raw);

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
static void pb_free_char_devices(struct evi_pb *pb)
{
	device_destroy(pb->powerboard_class, pb->data_dev_num);
	cdev_del(&pb->data_cdev);
	unregister_chrdev_region(pb->data_dev_num, 1);
	class_destroy(pb->powerboard_class);
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

static int read_reg(struct evi_pb *pb, int addr, u32 *value)
{
	u32 tx;
	int ret;

	if (down_interruptible(&pb->lock))
		return -ERESTARTSYS;

	tx = cpu_to_be32(addr | PB_READ);
	ret = spi_write_then_read(pb->spi, &tx, sizeof(tx), value, 4);
	up(&pb->lock);
	return ret;
}

static int write_reg(struct evi_pb *pb, int addr, u32 value)
{
	u32 tx[2];
	int ret;

	if (down_interruptible(&pb->lock))
		return -ERESTARTSYS;

	tx[0] = cpu_to_be32(addr | PB_WRITE);
	tx[1] = value;

	ret = spi_write(pb->spi, &tx, sizeof(tx));
	up(&pb->lock);
	return ret;
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

static int legacy_read_reg(struct evi_pb *pb, int addr, u32 *value)
{
	void *buf = value;
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
	ret = pb->read_reg(pb, request.reg_addr, (u32 *)request.data);
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

static int legacy_write_reg(struct evi_pb *pb, int addr, u32 value)
{
	const void *buf = &value;
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
	ret = pb->write_reg(pb, request.reg_addr, *(u32 *)request.data);

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

const struct file_operations dev_fops = {
	.owner = THIS_MODULE,
	.read = NULL,
	.write = NULL,
	.open = pb_open,
	.unlocked_ioctl = pb_ioctl,
	.llseek = no_llseek,
};

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

/**
 * pb_poweroff - shut off system power from powerboard
 */
static void pb_power_off(void)
{
	long ret;

	pr_alert("evi_pb_poweroff called\n");
	ret = pb_dev.write_reg(&pb_dev, 3, 0x00000001);
	if (ret)
		pr_alert("evi_pb_poweroff failed to send: %ld\n", ret);
	msleep(5000);
	pr_alert("evi_pb_poweroff: power not off after 5 seconds!\n");
}

static int pb_fw_detect(struct evi_pb *pb)
{
	int ret;
	u32 ver;

	ret = read_reg(pb, 1, (u32 *)&ver);
	if (ret) {
		dev_err(&pb->spi->dev, "failed to read version: %d\n", ret);
		return ret;
	}

	if (ver == 0 || ver == 0xffffffff) {
		dev_err(&pb->spi->dev, "bad firmware ver: 0x%8.8x\n", ver);
		return -ENODEV;
	}

	memcpy(&pb->device_ver, &ver, sizeof(pb->device_ver));
	pb->read_reg = read_reg;
	pb->write_reg = write_reg;

	return 0;
}

static int pb_legacy_detect(struct evi_pb *pb)
{
	int ret;
	u32 ver;

	pb->spi->mode &= ~SPI_CPHA;
	ret = pb_op_init(pb, 1, EVI_POWER_SPI_READ_REGISTER);
	if (ret)
		return ret;

	ret = pb_read_bytes(pb, (char *)&ver, 4) - 4;
	if (ret)
		return ret;

	if (ver == 0 || ver == 0xffffffff) {
		dev_err(&pb->spi->dev, "bad firmware ver: 0x%8.8x\n", ver);
		return -ENODEV;
	}

	memcpy(&pb->device_ver, &ver, sizeof(pb->device_ver));
	pb->read_reg = legacy_read_reg;
	pb->write_reg = legacy_write_reg;

	return 0;
}

static int pb_detect(struct evi_pb *pb)
{
	int ret;

	ret = pb_fw_detect(pb);
	if (ret) {
		u32 padding;

		dev_info(&pb->spi->dev, "attempt legacy detect\n");
		/*
		 * new protocol uses a 4 byte header plus at least 4 bytes of
		 * data, where old protocol uses exactly 5 bytes.
		 * Pad communication out to 10, a multiple of 5
		 */
		ret = spi_read(pb->spi, &padding, 2);
		if (ret) {
			dev_err(&pb->spi->dev, "failed to read bytes\n");
			return ret;
		}

		ret = pb_legacy_detect(pb);
	}

	if (ret)
		dev_err(&pb->spi->dev, "no powerboard detected: %d\n", ret);
	else
		dev_info(&pb->spi->dev, "found firmware version %d.%d.%d.%d\n",
			 pb->device_ver.major, pb->device_ver.minor,
			 pb->device_ver.rev, pb->device_ver.subrev);

	return ret;
}

static int pb_probe(struct spi_device *spi)
{
	int ret_val = 0;

	memset(&pb_dev, 0, sizeof(struct evi_pb));
	sema_init(&pb_dev.lock, 1);

	pb_dev.spi = spi;
	ret_val = pb_detect(&pb_dev);
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

#ifdef CONFIG_OF
static const struct of_device_id evi_pb_dt_ids[] = {
	{ .compatible = "uniwest,evi-pb", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, evi_pb_dt_ids);
#endif

static const struct spi_device_id evi_pb_spi_ids[] = {
	{DEVICE_NAME, 0},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(spi, evi_pb_spi_ids);

static struct spi_driver powerboard_spi_driver = {
	.driver = {
		   .name = DEVICE_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(evi_pb_dt_ids),
		   },
	.id_table = evi_pb_spi_ids,
	.probe = pb_probe,
	.remove = pb_remove,
};
module_spi_driver(powerboard_spi_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Uniwest evi power driver");
