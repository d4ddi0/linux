/**
 * Copyright (c) 2015 United Western Technologies, Corporation
 *
 * smartscanner module handles uniwest smartscanner devices
 *
 */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/uaccess.h>

#include "evi_registers.h"
#include "smartscanner.h"
#include "ss_user.h"

static const u32 SS_SEND = 0x00;
static const u32 SS_RECV = 0x100;
static const u32 SS_FIFO = 0x104;
static const u32 SS_FLAG = 0x204;

#define SS_FLAG_CONNECTED BIT(31)
#define SS_BUTTON_OFFSET (8)
#define SS_NUM_BUTTONS (4)
#define SS_BUTTONMASK (0xf << SS_BUTTON_OFFSET)
#define SS_BUTTONS(msg) (((msg) & SS_BUTTONMASK) >> SS_BUTTON_OFFSET)

#define SS_MSGTYPE_OFFSET (24)
#define SS_MSGTYPE_MASK (0xff << SS_MSGTYPE_OFFSET)

#define SS_MSGTYPE(msg) (msg >> SS_MSGTYPE_OFFSET)
#define SS_MSG_RESPONSE_ERROR (0x00)
#define SS_MSG_RESPONSE_16BIT (0x01)
#define SS_MSG_RESPONSE_32BIT_A (0x02)
#define SS_MSG_RESPONSE_32BIT_B (0x03)
#define SS_MSG_EVENT_DEPRECATED (0x80)
#define SS_MSG_EVENT (0xc0)

#define SS_SEQNO_OFFSET (16)
#define SS_SEQNO_MASK (0xff << SS_SEQNO_OFFSET)
#define SS_SEQNO_INCREMENT BIT(SS_SEQNO_OFFSET)
#define SS_MSG_SEQNO(msg) ((msg & SS_SEQNO_MASK) >> SS_SEQNO_OFFSET)

static void gen_events(struct smartscanner *ss, u32 msg)
{
	unsigned long changed_buttons = SS_BUTTONS(msg ^ ss->flags);
	unsigned button;

	for_each_set_bit(button, &changed_buttons, SS_NUM_BUTTONS) {
		bool value = !!(SS_BUTTONS(msg) & BIT(button));

		input_event(ss->input, EV_KEY, ss->keys[button], value);
		input_sync(ss->input);
	}
}

/*
 * read_scanner
 *
 * read a value from the scanner fifo
 * returns true if a complete message was retrieved
 */
static bool read_scanner(struct smartscanner *ss)
{
	u32 msg, msg_type;

	msg = readl_relaxed(ss->base + SS_RECV);
	dev_dbg(ss->dev, "recvd from scanner: 0x%8.8x\n", msg);

	if (!(ss->flags & SCANNER_CONNECTED)) {
		dev_dbg(ss->dev, "scanner not connected. discard 0x%8.8x\n",
			msg);
		return false;
	}

	msg_type = SS_MSGTYPE(msg);

	switch (msg_type) {
	case SS_MSG_RESPONSE_ERROR:
		/*
		 * All zeros can mean we read from an empty fifo
		 * ignore any all zero msg unless the SEQNO is legit
		 * i.e. the last SEQNO was 0xff
		 */
		if (msg == 0x00000000 && (SS_MSG_SEQNO(ss->msg) != 0xff))
			break;
		/* fall through */
	case SS_MSG_RESPONSE_16BIT:
	case SS_MSG_RESPONSE_32BIT_A:
	case SS_MSG_RESPONSE_32BIT_B:
		if (msg == ss->msg)
			break;

		ss->last_msg = ss->msg;
		ss->msg = msg;
		if (msg_type == SS_MSG_RESPONSE_32BIT_A)
			break;  /* first half of msg. Do not wake the ioctl */

		return true;
	case SS_MSG_EVENT_DEPRECATED:
	case SS_MSG_EVENT:
		gen_events(ss, msg);
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
	u32 nr_msgs;

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

static void ss_connect(struct smartscanner *ss)
{
	ss->msg = (0 - SS_SEQNO_INCREMENT) & SS_SEQNO_MASK;
	ss->flags |= SCANNER_CONNECTED;
}

static void ss_disconnect(struct smartscanner *ss)
{
	int i;

	ss->flags = 0;
	/* flush the scanner fifo */
	for (i = 0; i < 8; i++)
		readl_relaxed(ss->base + SS_RECV);
}

static void scanner_update_status(struct work_struct *work)
{
	struct smartscanner *ss = container_of(work, struct smartscanner,
					       status_work);
	u32 status, changes;

	status = readl_relaxed(ss->base + SS_FLAG);
	changes = status ^ ss->last_status;
	ss->last_status = status;

	if (!(changes & SS_FLAG_CONNECTED))
		return;

	if (status & SS_FLAG_CONNECTED) {
		/* debounce scanner connect */
		msleep(100);
		status = readl_relaxed(ss->base + SS_FLAG);
		if (status & SS_FLAG_CONNECTED)
			ss_connect(ss);
	} else {
		ss_disconnect(ss);
	}
}

static irqreturn_t ef_handle_statusirq(int irq, void *dev)
{
	struct smartscanner *ss = (struct smartscanner *)dev;

	schedule_work(&ss->status_work);
	return IRQ_HANDLED;
}

static int ss_input_init(struct smartscanner *ss)
{
	int i;
	int ret;

	ss->input = devm_input_allocate_device(ss->dev);
	if (!ss->input) {
		dev_err(ss->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	ss->input->name = "evi_smartscanner";
	ss->input->phys = "evi_smartscanner/input0";
	ss->input->id.bustype = BUS_HOST;
	ss->input->id.vendor = 0x0001;
	ss->input->id.product = 0x0002;
	ss->input->id.version = 0x0100;

	__set_bit(EV_KEY, ss->input->evbit);

	ss->keys[0] = KEY_F9;
	ss->keys[1] = KEY_F10;
	ss->keys[2] = KEY_F11;
	ss->keys[3] = KEY_F12;

	for (i = 0; i < SS_NUM_BUTTONS; i++) {
		input_set_capability(ss->input, EV_KEY, ss->keys[i]);
	}

	ret = input_register_device(ss->input);
	if (ret) {
		dev_err(ss->dev, "Unable to register input device: %d\n", ret);
		return ret;
	}

	return 0;
}

int ef_scannerirq_probe(struct smartscanner *ss)
{
	int ret;

	init_waitqueue_head(&ss->wq);

	if (!ss->irq) {
		dev_err(ss->dev, "Could not find scanner irq\n");
		return -ENODEV;
	}

	ret = request_irq(ss->irq, ef_handle_scannerirq, 0, "evi-scanner", ss);
	if (ret)
		dev_err(ss->dev, "Error requesting scannerirq: %d\n", ret);

	if (!ss->statusirq) {
		dev_err(ss->dev, "Could not find status irq\n");
		return -ENODEV;
	}

	ret = request_irq(ss->statusirq, ef_handle_statusirq, 0,
			  "scanner-status", ss);
	if (ret) {
		dev_err(ss->dev, "Error requesting statusirq: %d\n", ret);
		free_irq(ss->irq, &ss->irq);
		return ret;
	}

	ret = ss_input_init(ss);
	if (ret)
		return ret;

	INIT_WORK(&ss->status_work, scanner_update_status);

	schedule_work(&ss->status_work);

	return ret;
}

void ef_scannerirq_remove(struct smartscanner *ss)
{
	free_irq(ss->irq, &ss->irq);
	disable_irq(ss->statusirq);
	free_irq(ss->statusirq, &ss->statusirq);
	input_unregister_device(ss->input);
}

static bool ef_seq_num_ok(u32 last_msg, uint32_t msg)
{
	u32 exected_seq_num = (last_msg + SS_SEQNO_INCREMENT) & SS_SEQNO_MASK;

	return ((msg & SS_SEQNO_MASK) == exected_seq_num);
}

static long scanner_data(struct smartscanner *ss, struct scanner_command *cmd)
{
	cmd->response = (ss->msg & 0xffff);
	if (SS_MSGTYPE(ss->msg) == SS_MSG_RESPONSE_32BIT_B)
		cmd->response |= (ss->last_msg & 0xffff) << 16;

	/* magic number indicating data returned */
	return 0x10000;
}

static void _scanner_seq_reset(struct smartscanner *ss,
			       struct scanner_command *cmd)
{
	u32 seq = ((cmd->command << 8) - SS_SEQNO_INCREMENT) & SS_SEQNO_MASK;

	ss->msg &= ~SS_SEQNO_MASK;
	ss->msg |= seq;
}

static long _scanner_cmd(struct smartscanner *ss, struct scanner_command *cmd)
{
	long ret = 0;

	dev_dbg(ss->dev, "sending to scanner: 0x%8.8x\n", cmd->command);
	ss->data_ready = false;
	if ((cmd->command & 0xffff0000) == 0x81240000)
		_scanner_seq_reset(ss, cmd);

	writel_relaxed(cmd->command, ss->base + SS_SEND);
	ret = wait_event_interruptible_timeout(ss->wq, ss->data_ready, HZ / 5);

	if (ret < 0)
		return ret;

	if (!ret) {
		u32 prev_msg = ss->msg;

		dev_warn(ss->dev, "scanner cmd 0x%8.8x timed out.\n",
			 cmd->command);
		read_scanner(ss);
		if (prev_msg == ss->msg)
			return -EAGAIN;
	}

	if (!ef_seq_num_ok(ss->last_msg, ss->msg)) {
		dev_warn(ss->dev,
			 "invalid scanner seq: 0x%x to 0x%x\n",
			 SS_MSG_SEQNO(ss->last_msg), SS_MSG_SEQNO(ss->msg));
		return -EIO;
	}

	switch (SS_MSGTYPE(ss->msg)) {
	case SS_MSG_RESPONSE_ERROR:
		ret = ss->msg & 0xffff;
		break;
	case SS_MSG_RESPONSE_16BIT:
	case SS_MSG_RESPONSE_32BIT_B:
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
