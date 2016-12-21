#ifndef _UNIWEST_SMARTSCANNER_H
#define _UNIWEST_SMARTSCANNER_H

/**
 * Copyright (c) 2015 United Western Technologies, Corporation
 *
 */

#define SCANNER_CONNECTED   (0x00010000)

#include <linux/io.h>
#include <linux/compiler.h>
#include <linux/compiler.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

struct smartscanner {
   struct device *dev;
	void __iomem *status;
	void __iomem *base;
	unsigned int irq;
	u32 flags;
	u32 msg;
	u32 last_msg;
	wait_queue_head_t wq;
	bool data_ready;
};

int ef_scannerirq_probe(struct smartscanner *ss);
void ef_scannerirq_remove(struct smartscanner *ss);
long scanner_ioctl(struct file *filp, unsigned int command, unsigned long arg,
		   struct smartscanner *ss);

#endif /* _UNIWEST_SMARTSCANNER_H */
