#ifndef SS_USER_H
#define SS_USER_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
#endif

struct scanner_command {
	__u32 command;
	__u32 response;
};

#define SCANNER_IOC_MAGIC 'E'

#define SCANNER_STATUS   _IOR(SCANNER_IOC_MAGIC, 7, __u32)
#define SCANNER_CMD      _IOWR(SCANNER_IOC_MAGIC, 12, struct scanner_command)
#define SCANNER_FIRMWARE _IOW(SCANNER_IOC_MAGIC, 14, char *)

#endif /* SS_USER_H */
