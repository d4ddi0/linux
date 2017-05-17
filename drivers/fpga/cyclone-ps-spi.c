/**
 * Altera Cyclone Passive Serial SPI Driver
 *
 *  Copyright (c) 2017 United Western Technologies, Corporation
 *
 *  Joshua Clayton <stillcompiling@gmail.com>
 *
 * Manage Altera FPGA firmware that is loaded over spi using the passive
 * serial configuration method.
 * Firmware must be in binary "rbf" format.
 * Works on Cyclone V. Should work on cyclone series.
 * May work on other Altera FPGAs.
 *
 */

#include <linux/bitrev.h>
#include <linux/delay.h>
#include <linux/fpga/fpga-mgr.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/sizes.h>

#define FPGA_RESET_TIME		50   /* time in usecs to trigger FPGA config */
#define FPGA_MIN_DELAY		50   /* min usecs to wait for config status */
#define FPGA_MAX_DELAY		1000 /* max usecs to wait for config status */

struct cyclonespi_conf {
	struct gpio_desc *config;
	struct gpio_desc *status;
	struct spi_device *spi;
	u32 info_flags;
};

static const struct of_device_id of_ef_match[] = {
	{ .compatible = "altr,fpga-passive-serial", },
	{}
};
MODULE_DEVICE_TABLE(of, of_ef_match);

static enum fpga_mgr_states cyclonespi_state(struct fpga_manager *mgr)
{
	struct cyclonespi_conf *conf = (struct cyclonespi_conf *)mgr->priv;

	if (gpiod_get_value(conf->status))
		return FPGA_MGR_STATE_RESET;

	return FPGA_MGR_STATE_UNKNOWN;
}

static int cyclonespi_write_init(struct fpga_manager *mgr,
				 struct fpga_image_info *info,
				 const char *buf, size_t count)
{
	struct cyclonespi_conf *conf = (struct cyclonespi_conf *)mgr->priv;
	int i;

	conf->info_flags = info->flags;

	if (info->flags & FPGA_MGR_PARTIAL_RECONFIG) {
		dev_err(&mgr->dev, "Partial reconfiguration not supported.\n");
		return -EINVAL;
	}

	gpiod_set_value(conf->config, 1);
	usleep_range(FPGA_RESET_TIME, FPGA_RESET_TIME + 20);
	if (!gpiod_get_value(conf->status)) {
		dev_err(&mgr->dev, "Status pin failed to show a reset\n");
		return -EIO;
	}

	gpiod_set_value(conf->config, 0);
	for (i = 0; i < (FPGA_MAX_DELAY / FPGA_MIN_DELAY); i++) {
		usleep_range(FPGA_MIN_DELAY, FPGA_MIN_DELAY + 20);
		if (!gpiod_get_value(conf->status))
			return 0;
	}

	dev_err(&mgr->dev, "Status pin not ready.\n");
	return -EIO;
}

static void rev_buf(char *buf, size_t len)
{
	u32 *fw32 = (u32 *)buf;
	size_t extra_bytes = (len & 0x03);
	const u32 *fw_end = (u32 *)(buf + len - extra_bytes);

	/* set buffer to lsb first */
	while (fw32 < fw_end) {
		*fw32 = bitrev8x4(*fw32);
		fw32++;
	}

	if (extra_bytes) {
		buf = (char *)fw_end;
		while (extra_bytes) {
			*buf = bitrev8(*buf);
			buf++;
			extra_bytes--;
		}
	}
}

static int cyclonespi_write(struct fpga_manager *mgr, const char *buf,
			    size_t count)
{
	struct cyclonespi_conf *conf = (struct cyclonespi_conf *)mgr->priv;
	const char *fw_data = buf;
	const char *fw_data_end = fw_data + count;

	while (fw_data < fw_data_end) {
		int ret;
		size_t stride = min_t(size_t, fw_data_end - fw_data, SZ_4K);

		if (conf->info_flags & FPGA_MGR_BITSTREAM_LSB_FIRST)
			rev_buf((char *)fw_data, stride);

		ret = spi_write(conf->spi, fw_data, stride);
		if (ret) {
			dev_err(&mgr->dev, "spi error in firmware write: %d\n",
				ret);
			return ret;
		}
		fw_data += stride;
	}

	return 0;
}

static int cyclonespi_write_complete(struct fpga_manager *mgr,
				     struct fpga_image_info *info)
{
	struct cyclonespi_conf *conf = (struct cyclonespi_conf *)mgr->priv;

	if (gpiod_get_value(conf->status)) {
		dev_err(&mgr->dev, "Error during configuration.\n");
		return -EIO;
	}

	return 0;
}

static const struct fpga_manager_ops cyclonespi_ops = {
	.state = cyclonespi_state,
	.write_init = cyclonespi_write_init,
	.write = cyclonespi_write,
	.write_complete = cyclonespi_write_complete,
};

static int cyclonespi_probe(struct spi_device *spi)
{
	struct cyclonespi_conf *conf = devm_kzalloc(&spi->dev, sizeof(*conf),
						    GFP_KERNEL);

	if (!conf)
		return -ENOMEM;

	conf->spi = spi;
	conf->config = devm_gpiod_get(&spi->dev, "nconfig", GPIOD_OUT_HIGH);
	if (IS_ERR(conf->config)) {
		dev_err(&spi->dev, "Failed to get config gpio: %ld\n",
			PTR_ERR(conf->config));
		return PTR_ERR(conf->config);
	}

	conf->status = devm_gpiod_get(&spi->dev, "nstat", GPIOD_IN);
	if (IS_ERR(conf->status)) {
		dev_err(&spi->dev, "Failed to get status gpio: %ld\n",
			PTR_ERR(conf->status));
		return PTR_ERR(conf->status);
	}

	return fpga_mgr_register(&spi->dev,
				 "Altera Cyclone PS SPI FPGA Manager",
				 &cyclonespi_ops, conf);
}

static int cyclonespi_remove(struct spi_device *spi)
{
	fpga_mgr_unregister(&spi->dev);

	return 0;
}

static const struct spi_device_id ef_spi_ids[] = {
	{"cyclone-ps-spi", 0},
	{}
};

MODULE_DEVICE_TABLE(spi, ef_spi_ids);

static struct spi_driver cyclonespi_driver = {
	.driver = {
		.name = "cyclone-ps-spi",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(of_ef_match),
	},
	.id_table = ef_spi_ids,
	.probe = cyclonespi_probe,
	.remove = cyclonespi_remove,
};

module_spi_driver(cyclonespi_driver)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joshua Clayton <stillcompiling@gmail.com>");
MODULE_DESCRIPTION("Module to load Altera FPGA firmware over spi");
