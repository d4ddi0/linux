#ifndef _UNIWEST_SMARTSCANNER_H
#define _UNIWEST_SMARTSCANNER_H

/**
 * Copyright (c) 2015 United Western Technologies, Corporation
 *
 */

#include <linux/io.h>
#include <linux/compiler.h>
#include <linux/compiler.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

struct smartscanner {
   struct device *dev;
	void __iomem *status;
	void __iomem *base;
   volatile uint32_t *user_flags;
	struct gpio_desc *irq_gpiod;
	unsigned int irq;
	u32 msg;
	u32 last_msg;
	wait_queue_head_t wq;
	bool data_ready;
	bool connected;
};

int ef_scannerirq_probe(struct smartscanner *ss);
void ef_scannerirq_remove(struct smartscanner *ss);
long scanner_cmd(struct smartscanner *ss, void __user *arg);
int ef_load_scanner_firmware(struct smartscanner *ss, __user char *path);

#endif /* _UNIWEST_SMARTSCANNER_H */
