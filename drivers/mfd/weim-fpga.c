/**
 * Copyright (c) 2016 United Western Technologies, Corporation
 *
 * weim-fpga is a container device for several devices connected
 * to an FPGA which is itself accessible through the imx-weim bus.
 *
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/types.h>
#include <linux/workqueue.h>

struct eeim {
	struct device *dev;
	struct fpga_manager *fmgr;
	const char *fw_name;
};

static const struct of_device_id of_eeim_match[] = {
	{ .compatible = "fsl,weim-fpga", },
	{}
};
MODULE_DEVICE_TABLE(of, of_eeim_match);

static int eeim_probe(struct platform_device *pdev)
{
	int ret;
	const struct of_device_id *match;
	struct eeim *ee;
	const char * const of_fw_name = "uniwest,firmware-img";
	const char * const of_fpga_mgr = "uniwest,fpga-mgr";
	struct device_node *mgr_node;
	struct fpga_image_info info = {
		.flags = 0,
	};

	match = of_match_device(of_eeim_match, &pdev->dev);
	if (!match)
		return -EINVAL;

	ee = devm_kzalloc(&pdev->dev, sizeof(*ee), GFP_KERNEL);
	if (!ee)
		return -ENOMEM;

	ee->dev = &pdev->dev;
	platform_set_drvdata(pdev, ee);
	ee->fw_name = of_get_property(ee->dev->of_node, of_fw_name, NULL);
	if (!ee->fw_name) {
		dev_err(ee->dev, "property %s not found\n", of_fw_name);
		return -EINVAL;
	}

	mgr_node = of_parse_phandle(ee->dev->of_node, of_fpga_mgr, 0);
	if (!mgr_node) {
		dev_err(ee->dev, "phandle %s not resolved\n", of_fpga_mgr);
		return -EINVAL;
	}

	ee->fmgr = of_fpga_mgr_get(mgr_node);
	of_node_put(mgr_node);
	if (IS_ERR(ee->fmgr)) {
		dev_err(ee->dev, "unable to find fpga mgr\n");
		return -EPROBE_DEFER;
	}

	ret = fpga_mgr_firmware_load(ee->fmgr, &info, ee->fw_name);
	fpga_mgr_put(ee->fmgr);
	if (ret == -ENOENT) {
		dev_warn(ee->dev, "Firmware image not found\n");
		return -EPROBE_DEFER;
	} else if (ret) {
		dev_err(ee->dev, "Failed to load firmware: %d\n", ret);
		return ret;
	}

	dev_info(ee->dev, "Loaded %s firmware\n", ee->fw_name);

	of_platform_default_populate(ee->dev->of_node, NULL, ee->dev);

	return 0;
}

static int eeim_remove(struct platform_device *pdev)
{
	dev_notice(&pdev->dev, "removed\n");
	return 0;
}

static struct platform_driver eeim_driver = {
	.driver = {
		.name = "weim-fpga",
		.owner = THIS_MODULE,
		.of_match_table = of_eeim_match,
	},
	.probe = eeim_probe,
	.remove = eeim_remove,
};

module_platform_driver(eeim_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("weim fpga multifunction device");
