/**
 * Copyright (c) 2015 United Western Technologies, Corporation
 *
 * smartscanner module handles uniwest smartscanner devices
 *
 */
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/sched.h>
#include <linux/uaccess.h>

#include "evi_registers.h"
#include "smartscanner.h"
#include "ss_user.h"

static const u32 SS_SEND = 0x000;
static const u32 SS_RECV = 0x100;
static const u32 SS_FIFO = 0x104;

/*
 * read_scanner
 *
 * read a value from the scanner fifo
 * returns true if a complete message was retrieved
 */
static bool read_scanner(struct smartscanner *ss)
{
	uint32_t msg, msg_type;

	msg = readl_relaxed(ss->base + SS_RECV);
	dev_dbg(ss->dev, "recvd from scanner: 0x%8.8x\n", msg);

	if (!(ss->flags & SCANNER_CONNECTED)) {
		dev_dbg(ss->dev, "scanner not connected. discard 0x%8.8x\n",
				msg);
		return false;
	}

	msg_type = msg >> 24;

	switch (msg_type) {
	case 0x00:
		if (!msg && ((ss->msg & 0x00ff0000) != 0x00ff0000))
			break;
		/* fall through */
	case 0x01:
	case 0x02:
	case 0x03:
		if (msg == ss->msg)
			break;

		ss->last_msg = ss->msg;
		ss->msg = msg;
		if (msg_type == 0x02)
			break;  /* first half of msg. Do not wake the ioctl */

		return true;
	case 0x80:
	case 0xC0:
		ss->flags &= 0xffff0000;
		ss->flags |= (msg & 0xffff);
		break;
	default:
		dev_warn(ss->dev, "unhandled scanner msg: 0x%8.8x\n", msg);
	}

	return false;
}

static irqreturn_t ef_handle_scannerirq(int irq, void *dev)
{
	struct smartscanner *ss = (struct smartscanner *)dev;
	union ef_status status;
	uint32_t nr_msgs;

	status.raw_data = readl_relaxed(ss->status);
	dev_dbg(ss->dev, "scanner preread status 0x%8.8x\n", status.raw_data);

	for (nr_msgs = status.nr_scanner_fifo; nr_msgs > 0; nr_msgs--) {
		if (read_scanner(ss)) {
			ss->data_ready = true;
			wake_up(&ss->wq);
		}
	}

	return IRQ_HANDLED;
}

int ef_scannerirq_probe(struct smartscanner *ss)
{
	int ret;

	init_waitqueue_head(&ss->wq);

	ss->irq = irq_of_parse_and_map(ss->dev->of_node, 0);
	if (!ss->irq) {
		dev_err(ss->dev, "Could not find scanner irq\n");
		return -ENODEV;
	}

	ret = request_irq(ss->irq, ef_handle_scannerirq, 0, "evi-scanner", ss);
	if (ret)
		dev_err(ss->dev, "Error requesting scannerirq: %d\n", ret);

	return ret;
}

void ef_scannerirq_remove(struct smartscanner *ss)
{
	free_irq(ss->irq, &ss->irq);
}

static bool ef_seq_num_ok(uint32_t last_msg, uint32_t msg)
{
	uint32_t exected_seq_num = (last_msg + 0x00010000) & 0x00ff0000;

	return ((msg & 0x00ff0000) == exected_seq_num);
}

static long scanner_data(struct smartscanner *ss, struct scanner_command *cmd)
{
	cmd->response = (ss->msg & 0xffff);

	if ((ss->msg & 0xff000000) == 0x03000000)
		cmd->response |= (ss->last_msg & 0xffff) << 16;

	/* magic number indicating data returned */
	return 0x10000;
}

static void _scanner_seq_reset(struct smartscanner *ss,
				struct scanner_command *cmd)
{
	uint32_t seq = ((cmd->command - 0x00000100) & 0x0000ff00) << 8;

	ss->msg &= 0xff00ffff;
	ss->msg |= seq;
}

static long _scanner_cmd(struct smartscanner *ss, struct scanner_command *cmd)
{
	long ret = 0;
	uint32_t msg_type;

	dev_dbg(ss->dev, "sending to scanner: 0x%8.8x\n", cmd->command);
	ss->data_ready = false;
	if ((cmd->command & 0xffff0000) == 0x81240000)
		_scanner_seq_reset(ss, cmd);

	writel_relaxed(cmd->command, ss->base + SS_SEND);
	ret = wait_event_interruptible_timeout(ss->wq, ss->data_ready, HZ/5);

	if (ret < 0)
		return ret;

	if (!ret) {
		uint32_t prev_msg = ss->msg;

		dev_warn(ss->dev, "scanner cmd 0x%8.8x timed out.\n",
			 cmd->command);
		read_scanner(ss);
		if (prev_msg == ss->msg)
			return -EAGAIN;
	}

	if (!ef_seq_num_ok(ss->last_msg, ss->msg)) {
		dev_warn(ss->dev,
			 "invalid scanner seq: 0x%x to 0x%x\n",
			 (ss->last_msg & 0x00ff0000) >> 16,
			 (ss->msg & 0x00ff0000) >> 16);
		return -EIO;
	}

	msg_type = ss->msg >> 24;
	switch (msg_type) {
	case 0x00:
		ret = ss->msg & 0xffff;
		break;
	case 0x01:
	case 0x03:
		ret = scanner_data(ss, cmd);
		break;
	default:
		ret = -EFAULT;
		break;
	}

	if (ret < 0)
		dev_err(ss->dev, "scanner_cmd 0x%8.8x failed: %ld\n",
			cmd->command, ret);

	return ret;
}

static void scanner_seq_reset(struct smartscanner *ss)
{
	struct scanner_command tmp = {.command = 0x8124aa00,};
	long ret;

	ret = _scanner_cmd(ss, &tmp);
	if (ret == -EIO) {
		/* last ditch effort to fix the sequence number */
		tmp.command = 0x8124ab00;
		_scanner_seq_reset(ss, &tmp);
	}
}

static long scanner_cmd(struct smartscanner *ss, void __user *arg)
{
	long ret = 0;
	struct scanner_command cmd;

	if (!(ss->flags & SCANNER_CONNECTED))
		return -ENODEV;

	if (copy_from_user(&cmd, arg, sizeof(cmd))) {
		dev_err(ss->dev, "copying scanner cmd\n");
		return -EINVAL;
	}

	ret = _scanner_cmd(ss, &cmd);
	if (ret < 0) {
		dev_err(ss->dev, "scanner_cmd error: %ld\n", ret);
		scanner_seq_reset(ss);
		ret = _scanner_cmd(ss, &cmd);
	} else if ((ret & 0xFF0000FF) == 0x0000000a) {
		dev_err(ss->dev, "scanner_cmd error: only %ld bytes recvd\n",
			(ret & 0xff00) >> 8);
		ret = _scanner_cmd(ss, &cmd);
	}

	if (copy_to_user(arg, &cmd, sizeof(cmd))) {
		dev_err(ss->dev, "copy_to_user failed in scanner_data()\n");
		ret = -EFAULT;
	}

	return ret;
}

#define NUM_SCANNER_WORDS 11
static int ef_write_scanner_fw(struct smartscanner *ss,
				const struct firmware *fw)
{
	int ret = 0;
	enum state {
		IN_LINE,
		NEED_TO_SEND,
		OUT_OF_LINE,
		END_OF_LINE,
	};
	enum state fw_state = OUT_OF_LINE;
	const u8 *fw_data = fw->data;
	const u8 *fw_data_end = fw->data + fw->size;
	u32 fw_buf;
	u8 *fw_ptr = (u8 *)&fw_buf;
	u8 *fw_end = fw_ptr + sizeof(fw_buf);
	struct scanner_command cmd = {.command = 0x82200000,};

	dev_info(ss->dev, "Writing firmware to scanner\n");

	ret = _scanner_cmd(ss, &cmd);
	if (ret) {
		dev_err(ss->dev, "scanner_fw block init error: %d\n", ret);
		return ret;
	}

	while (fw_data < fw_data_end) {
		switch (*fw_data) {
		case ':':
			fw_state = IN_LINE;
			break;
		case '\n':
		case '\r':
			if (fw_state != OUT_OF_LINE)
				fw_state = END_OF_LINE;
			break;
		default:
			break;
		}

		if (fw_state != OUT_OF_LINE) {
			*fw_ptr = *fw_data;
			fw_ptr++;
		}

		switch (fw_state) {
		case IN_LINE:
			if (fw_ptr >= fw_end) {
				mdelay(1);
				writel_relaxed(cpu_to_be32(fw_buf),
					       ss->base + SS_SEND);
				fw_buf = 0;
				fw_ptr = (u8 *)&fw_buf;
			}
			break;
		case END_OF_LINE:
			mdelay(1);
			cmd.command = cpu_to_be32(fw_buf);
			ret = _scanner_cmd(ss, &cmd);
			if (ret) {
				dev_err(ss->dev, "scanner_fw eol error: %d\n",
					ret);
				if (ret != -EAGAIN)
					return ret;
			}
			fw_buf = 0;
			fw_ptr = (u8 *)&fw_buf;
			fw_state = OUT_OF_LINE;
			break;
		case OUT_OF_LINE:
		default:
			break;
		}

		fw_data++;
	}

	dev_info(ss->dev, "Finished loading firmware\n");
	return 0;
}

static int scanner_load_firmware(struct smartscanner *ss, __user char *path)
{
	int ret;
	int i;
	const struct firmware *fw;
	char pathbuf[512];

	ret = strncpy_from_user(pathbuf, path, sizeof(pathbuf) - 1);
	if (ret < 0) {
		dev_err(ss->dev, "scanner_fw path address invalid\n");
		return -EFAULT;
	}

	pathbuf[sizeof(pathbuf) - 1] = '\0';

	dev_info(ss->dev, "Loading scanner firmware\n");
	ret = request_firmware_direct(&fw, pathbuf, ss->dev);
	if (ret) {
		dev_err(ss->dev, "scanner_fw request error: %d\n", ret);
		return ret;
	}

	for (i = 0; i < 5; i++) {
		ret = ef_write_scanner_fw(ss, fw);
		if (!ret)
			break;
	}

	release_firmware(fw);

	return ret;
}

long scanner_ioctl(struct file *filp, unsigned int command, unsigned long arg,
		   struct smartscanner *ss)
{
	long ret;

	switch (command) {
	case SCANNER_STATUS:
		ret = copy_to_user((char *)arg, &ss->flags, sizeof(ss->flags));
		break;
	case SCANNER_CMD:
		ret = scanner_cmd(ss, (void *)arg);
		break;
	case SCANNER_FIRMWARE:
		ret = scanner_load_firmware(ss, (char *)arg);
		break;
	default:
		dev_warn(ss->dev, "Invalid IOCTL called: %d\n", command);
		ret = -ENOTTY;
	}

	return ret;
}
