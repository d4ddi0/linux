/*
 *  Driver for evi top io lines as keyboard buttons
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define DRV_NAME	"evi-top-io"
#define ETIO_BUTTON_VAL(field, offset) (!(field & BIT(offset + 28)))

struct evi_top_io_dev {
	struct input_polled_dev *poll_dev;
	struct device *dev;
	void __iomem *reg;
	unsigned int keys[4];
	unsigned long state;
};

static void evi_io_poll(struct input_polled_dev *dev)
{
	struct evi_top_io_dev *etio = dev->private;
	struct input_dev *input = dev->input;
	unsigned long state = readl_relaxed(etio->reg);
	int i;

	for (i = 0; i < 4; i++) {
		bool val = ETIO_BUTTON_VAL(state, i);

		if (val != ETIO_BUTTON_VAL(etio->state, i))
			input_event(input, EV_KEY, etio->keys[i], val);
	}

	etio->state = state;
	input_sync(input);
}

static const struct of_device_id evi_io_of_match[] = {
	{ .compatible = "uniwest,evi-top-io", },
	{ },
};
MODULE_DEVICE_TABLE(of, evi_io_of_match);

static int evi_io_probe(struct platform_device *pdev)
{
	struct input_polled_dev *poll_dev;
	struct evi_top_io_dev *etio;
	struct resource res;
	int ret;
	int i;

	poll_dev = devm_input_allocate_polled_device(&pdev->dev);
	if (!poll_dev) {
		dev_err(&pdev->dev, "no memory for polled device\n");
		return -ENOMEM;
	}

	poll_dev->poll = evi_io_poll;
	poll_dev->poll_interval = 50;
	poll_dev->input->name = "evi_top_io";
	poll_dev->input->phys = "evi_top_io/input0";
	poll_dev->input->id.bustype = BUS_HOST;
	poll_dev->input->id.vendor = 0x0001;
	poll_dev->input->id.product = 0x0001;
	poll_dev->input->id.version = 0x0100;

	__set_bit(EV_KEY, poll_dev->input->evbit);

	etio = devm_kzalloc(&pdev->dev, sizeof(*etio), GFP_KERNEL);
	if (of_address_to_resource(pdev->dev.of_node, 0, &res))
		return -EINVAL;

	etio->reg = devm_ioremap_resource(&pdev->dev, &res);
	if (!etio->reg)
		return -ENOMEM;

	etio->keys[0] = KEY_F5;
	etio->keys[1] = KEY_F6;
	etio->keys[2] = KEY_F7;
	etio->keys[3] = KEY_F8;
	etio->state = readl_relaxed(etio->reg);

	for (i = 0; i < 4; i++) {
		input_set_capability(poll_dev->input, EV_KEY, etio->keys[i]);
	}

	poll_dev->private = etio;
	ret = input_register_polled_device(poll_dev);
	if (ret) {
		dev_err(&pdev->dev, "unable to register: %d", ret);
	}

	input_sync(poll_dev->input);
	dev_info(&pdev->dev, "initialized EVI_TOP_IO\n");

	return 0;
}

static struct platform_driver evi_io_driver = {
	.probe	= evi_io_probe,
	.driver	= {
		.name	= DRV_NAME,
		.of_match_table = evi_io_of_match,
	},
};
module_platform_driver(evi_io_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Joshua Clayton <stillcompiling@gmail.com>");
MODULE_DESCRIPTION("evi top io buttons driver");
MODULE_ALIAS("platform:" DRV_NAME);
