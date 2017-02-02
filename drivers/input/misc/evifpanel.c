/*
 * Uniwest EVI Front Panel driver
 *
 * Copyright 2015 United Western Technologies
 *
 * Joshua Clayton <stillcompiling@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 */
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/serio.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/wait.h>

#define DRIVER_DESC "Uniwest EVI Frontpanel input driver"
MODULE_AUTHOR("Joshua Clayton <stillcompiling@gmail.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

#define FP_NR_LEDS (9)
#define FP_LED_MASK (GENMASK(FP_NR_LEDS - 1, 0))

struct fp_slider {
	s32 sum;
	s32 count;
};

struct evifpanel {
	struct input_dev *dev;
	struct serio *serio;
	struct gpio_desc *reset;
	wait_queue_head_t request_wq;
	unsigned int bytes;
	char name[64];
	char phys[32];
	unsigned char buf[8];
	s32 h_slider;
	s32 v_slider;
	u32 leds;
};

struct key_map {
	u16 type;
	u16 code;
	s32 value;
	unsigned int byte;
	unsigned int offset;
};

static struct key_map btns[] = {
	{ EV_ABS, ABS_X, 0, 3, 4 },
	{ EV_ABS, ABS_X, 1, 3, 0 },
	{ EV_ABS, ABS_X, 2, 4, 6 },
	{ EV_ABS, ABS_X, 3, 4, 5 },
	{ EV_ABS, ABS_X, 4, 4, 4 },
	{ EV_ABS, ABS_X, 5, 4, 3 },
	{ EV_ABS, ABS_X, 6, 4, 2 },
	{ EV_ABS, ABS_X, 7, 4, 1 },
	{ EV_ABS, ABS_X, 8, 4, 0 },
	{ EV_ABS, ABS_X, 9, 5, 6 },
	{ EV_ABS, ABS_X, 10, 5, 5 },
	{ EV_ABS, ABS_X, 11, 3, 3 },
	{ EV_ABS, ABS_Y, 0, 3, 2 },
	{ EV_ABS, ABS_Y, 1, 6, 2 },
	{ EV_ABS, ABS_Y, 2, 6, 3 },
	{ EV_ABS, ABS_Y, 3, 6, 4 },
	{ EV_ABS, ABS_Y, 4, 6, 5 },
	{ EV_ABS, ABS_Y, 5, 6, 6 },
	{ EV_ABS, ABS_Y, 6, 5, 0 },
	{ EV_ABS, ABS_Y, 7, 5, 1 },
	{ EV_ABS, ABS_Y, 8, 5, 2 },
	{ EV_ABS, ABS_Y, 9, 5, 3 },
	{ EV_ABS, ABS_Y, 10, 5, 4 },
	{ EV_ABS, ABS_Y, 11, 3, 1 },
	{ EV_KEY, KEY_F1, 1, 6, 0 },
	{ EV_KEY, KEY_D, 1, 6, 1 },
	{ EV_KEY, KEY_N, 1, 7, 0 },
	{ EV_KEY, KEY_BACKSPACE, 1, 7, 1 },
	{ EV_KEY, KEY_ENTER, 1, 7, 2 },
	{ EV_KEY, KEY_ESC, 1, 7, 3 },
	{ EV_KEY, KEY_F4, 1, 7, 4 },
	{ EV_KEY, KEY_F3, 1, 7, 5 },
	{ EV_KEY, KEY_F2, 1, 7, 6 },
	{ },
};

static void fp_check_key(struct evifpanel *fp, struct key_map *key)
{
	s32 value = fp->buf[key->byte] & BIT(key->offset);

	input_report_key(fp->dev, key->code, value);
}

static void fp_slider_accumulate(struct evifpanel *fp,
		struct fp_slider *slider, struct key_map *key)
{
	s32 value = !!(fp->buf[key->byte] & BIT(key->offset));

	slider->sum += value * key->value;
	slider->count += value;
}

static s32 fp_slider_value(struct evifpanel *fp, struct fp_slider *slider)
{
	s32 value;

	if (slider->count)
		value = (slider->sum * 0xffff / (11 * slider->count)) - 0x8000;
	else
		value = 0;

	if (value == -1)
		value = 0;

	return value;
}

/*
 * Check buttons against array of key_map
 */
static void fp_check_btns(struct evifpanel *fp, struct key_map *key)
{
	struct fp_slider h_axis, v_axis;

	h_axis.sum = 0;
	h_axis.count = 0;
	v_axis.sum = 0;
	v_axis.count = 0;

	while (key->type) {
		switch (key->type) {
		case EV_KEY:
			fp_check_key(fp, key);
			break;
		case EV_ABS:
			if (key->code == ABS_X)
				fp_slider_accumulate(fp, &h_axis, key);
			else
				fp_slider_accumulate(fp, &v_axis, key);

			break;
		default:
			break; /* ignore unknown types */
		}
		key++;
	}

	fp->h_slider = fp_slider_value(fp, &h_axis);
	fp->v_slider = fp_slider_value(fp, &v_axis);

	input_sync(fp->dev);
}

/*
 * Set the firmware version coming from the fp in an ascii file
 */
static void fp_set_fw_ver(struct evifpanel *fp)
{
	scnprintf(fp->serio->firmware_id, sizeof(fp->serio->firmware_id),
			"evifpanel v%3.3u.%3.3u.%3.3u.%3.3u", fp->buf[4],
			fp->buf[5], fp->buf[6], fp->buf[7]);

	dev_info(&fp->serio->dev, "firmware found: %s\n",
			fp->serio->firmware_id);

	wake_up(&fp->request_wq);
}

/*
 * Request firmware version info from the device
 */
static int fp_request_fw_ver(struct evifpanel *fp)
{
	int ret;

	fp->serio->firmware_id[0] = '\0';

	serio_write(fp->serio, '\xC0');
	serio_write(fp->serio, '\x00');
	serio_write(fp->serio, '\x00');
	serio_write(fp->serio, '\x09');
	serio_write(fp->serio, '\x00');
	serio_write(fp->serio, '\x01');
	serio_write(fp->serio, '\x00');
	serio_write(fp->serio, '\x00');
	ret = wait_event_interruptible_timeout(fp->request_wq,
			fp->serio->firmware_id[0], HZ / 50);

	if (!ret)
		return -EAGAIN;
	else if (ret < 0)
		return ret;

	return 0;
}

/*
 * Send zero or more input events based on the state of the fp
 * Call this when you have a full packet.
 */
static int fp_parse_buf(struct evifpanel *fp)
{
	switch (fp->buf[1]) {
	case '\x03':
		fp_check_btns(fp, btns);
		break;
	case '\x09':
		if (fp->buf[4] || fp->buf[5])
			fp_set_fw_ver(fp);
		break;
	default:
		dev_err(&fp->dev->dev, "Bad cmd id %X in sequence %llX\n",
				fp->buf[1], *(u64 *)(fp->buf));

		return -EIO;
	}

	return 0;
}

static void fp_add_byte(struct evifpanel *fp, unsigned char c)
{
	if (c != '\xC0' && !fp->bytes) {
		dev_err(&fp->dev->dev, "drop %X. looking for check byte\n", c);
		return;
	}

	if (c == '\xC0' && fp->bytes) {
		/* msg check byte should not be found in the middle of a set */
		dev_warn(&fp->dev->dev, "discarding %d bytes from %llX\n",
			 fp->bytes, *(u64 *)(fp->buf));
		fp->bytes = 0;
	}

	fp->buf[fp->bytes] = c;
	++fp->bytes;
}


static irqreturn_t fp_interrupt(struct serio *serio, unsigned char data,
		unsigned int flags)
{
	struct evifpanel *fp = serio_get_drvdata(serio);

	fp_add_byte(fp, data);
	if (fp->bytes == 8) {
		fp_parse_buf(fp);
		fp->bytes = 0;
	}

	return IRQ_HANDLED;
}

static void fp_set_leds(struct evifpanel *fp)
{
	/*
	 * byte 7 contains led 0 through 6 in bits 0 through 6
	 * byte 6 contains led 7 and led 8 in bits 0 and 1
	 * No data byte ever sets the high bit in order
	 * to ensure that 0xc0 is unique to the check byte
	 */
	unsigned char led0 = (unsigned char)((fp->leds & GENMASK(8, 7)) >> 7);
	unsigned char led1 = (unsigned char)(fp->leds & GENMASK(6, 0));

	serio_write(fp->serio, '\xC0');
	serio_write(fp->serio, '\x00');
	serio_write(fp->serio, '\x00');
	serio_write(fp->serio, '\x09');
	serio_write(fp->serio, '\x00');
	serio_write(fp->serio, '\x02');
	serio_write(fp->serio, led0);
	serio_write(fp->serio, led1);
}

static ssize_t v_slider_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct serio *serio = to_serio_port(dev);
	struct evifpanel *fp = serio_get_drvdata(serio);

	return scnprintf(buf, PAGE_SIZE, "%d\n", fp->v_slider);
}

static ssize_t h_slider_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct serio *serio = to_serio_port(dev);
	struct evifpanel *fp = serio_get_drvdata(serio);

	return scnprintf(buf, PAGE_SIZE, "%d\n", fp->h_slider);
}

static ssize_t leds_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct serio *serio = to_serio_port(dev);
	struct evifpanel *fp = serio_get_drvdata(serio);
	int error;
	u32 leds;

	error = kstrtou32(buf, 0, &leds);
	if (error)
		return error;

	if (leds > FP_LED_MASK)
		return -EFAULT;

	fp->leds = leds;
	fp_set_leds(fp);

	return count;
}

static int fp_reset(struct evifpanel *fp)
{
	int ret;

	gpiod_set_value_cansleep(fp->reset, 0);
	msleep(1);
	gpiod_set_value_cansleep(fp->reset, 1);
	ret = fp_request_fw_ver(fp);
	if (ret) {
		dev_err(&fp->serio->dev, "failed to reset: %d \n", ret);
		return ret;
	}

	fp_set_leds(fp);

	return 0;
}

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct serio *serio = to_serio_port(dev);
	struct evifpanel *fp = serio_get_drvdata(serio);
	int error;
	long value;

	error = kstrtol(buf, 0, &value);
	if (error)
		return error;

	if (value != 1)
		return -EFAULT;

	error = fp_reset(fp);
	if (error)
		return error;

	return count;
}

static DEVICE_ATTR_RO(v_slider);
static DEVICE_ATTR_RO(h_slider);
static DEVICE_ATTR_WO(leds);
static DEVICE_ATTR_WO(reset);

static struct attribute *attrs[] = {
	&dev_attr_v_slider.attr,
	&dev_attr_h_slider.attr,
	&dev_attr_leds.attr,
	&dev_attr_reset.attr,
	NULL,
};

struct attribute_group fp_attrs = {
	.attrs = attrs,
};

static void fp_set_device_attrs(struct evifpanel *fp)
{
	snprintf(fp->name, sizeof(fp->name),
			"EVI Frontpanel keypad and sliders");
	snprintf(fp->phys, sizeof(fp->phys),
			"%s/input0", fp->serio->phys);

	fp->dev->name = fp->name;
	fp->dev->phys = fp->phys;
	fp->dev->id.bustype = BUS_RS232;
	fp->dev->dev.parent = &fp->serio->dev;

	fp->dev->evbit[0] = BIT_MASK(EV_KEY);
	__set_bit(KEY_D, fp->dev->keybit);
	__set_bit(KEY_N, fp->dev->keybit);
	__set_bit(KEY_BACKSPACE, fp->dev->keybit);
	__set_bit(KEY_ENTER, fp->dev->keybit);
	__set_bit(KEY_F1, fp->dev->keybit);
	__set_bit(KEY_F2, fp->dev->keybit);
	__set_bit(KEY_F3, fp->dev->keybit);
	__set_bit(KEY_F4, fp->dev->keybit);
	__set_bit(KEY_ESC, fp->dev->keybit);
}

static int fp_connect(struct serio *serio, struct serio_driver *drv)
{
	struct evifpanel *fp;
	struct input_dev *input_dev;

	int error = -ENOMEM;

	fp = kzalloc(sizeof(struct evifpanel), GFP_KERNEL);

	input_dev = input_allocate_device();
	if (!input_dev || !fp) {
		pr_err("evifpanel: failed to get memory\n");
		goto fail1;
	}

	init_waitqueue_head(&fp->request_wq);
	fp->dev = input_dev;
	fp->serio = serio;
	fp_set_device_attrs(fp);
	serio_set_drvdata(serio, fp);

	error = serio_open(serio, drv);
	if (error) {
		pr_err("evifpanel: failed to open serio\n");
		goto fail2;
	}

	serio->dev.of_node = of_find_node_by_name(NULL, "evi_fpanel");
	if (!serio->dev.of_node) {
		pr_err("evifpanel: failed to get devicetree node\n");
		goto dts_fail;
	}

	fp->reset = gpiod_get(&serio->dev, "reset", GPIOD_OUT_HIGH);

	error = fp_reset(fp);
	if (error) {
		dev_err(&serio->dev, "reset again\n");
		error = fp_reset(fp);
	}

	if (error)
		goto fail2;

	error = sysfs_create_group(&serio->dev.kobj, &fp_attrs);
	if (error) {
		dev_err(&serio->dev, "failed to add sysfs group\n");
		goto sysfs_fail;
	}

	error = input_register_device(input_dev);
	if (error) {
		pr_err("evifpanel: failed to register input device\n");
		goto fail3;
	}

	return 0;

fail3:
	serio_close(serio);
sysfs_fail:
	sysfs_remove_group(&serio->dev.kobj, &fp_attrs);
fail2:
	gpiod_put(fp->reset);
dts_fail:
	serio_set_drvdata(serio, NULL);
fail1:
	input_free_device(input_dev);
	kfree(fp);
	pr_err("fp_connect: FAILED\n");

	return error;
}

static void fp_disconnect(struct serio *serio)
{
	struct evifpanel *fp = serio_get_drvdata(serio);

	input_unregister_device(fp->dev);
	sysfs_remove_group(&serio->dev.kobj, &fp_attrs);
	gpiod_put(fp->reset);
	serio_close(serio);
	serio_set_drvdata(serio, NULL);
	kfree(fp);
};

static struct serio_device_id fp_ids[] = {
	{
		.type  = SERIO_RS232,
		.proto = SERIO_ANY,
		.id    = SERIO_ANY,
		.extra = SERIO_ANY,
	},
	{ 0 }
};
MODULE_DEVICE_TABLE(serio, fp_ids);

static struct serio_driver fp_drv = {
	.driver = {
		.name = "evifpanel",
	},
	.description = DRIVER_DESC,
	.id_table    = fp_ids,
	.connect     = fp_connect,
	.interrupt   = fp_interrupt,
	.disconnect  = fp_disconnect,
};

module_serio_driver(fp_drv);
