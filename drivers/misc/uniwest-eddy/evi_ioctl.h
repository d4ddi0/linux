#ifndef EVI_IOCTL_H
#define EVI_IOCTL_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
#include <stdint.h>
#endif

struct sample {
	uint32_t flags;
	uint32_t timestamp;
	int32_t encoders[4];
	uint32_t x0;
	uint32_t y0;
	uint32_t x1;
	uint32_t y1;
};

struct scanner_command {
	uint32_t command;
	uint32_t response;
};

/* ioctls */
#define EVI_EDDY_IOC_MAGIC 'E'
#define EVI_EDDY_VERSION _IOR(EVI_EDDY_IOC_MAGIC, 0, char *)
#define EVI_EDDY_START_DATA_FLOW _IO(EVI_EDDY_IOC_MAGIC, 3)
#define EVI_EDDY_STOP_DATA_FLOW _IO(EVI_EDDY_IOC_MAGIC, 4)
#define EVI_EDDY_START_HW _IO(EVI_EDDY_IOC_MAGIC, 5)
#define EVI_EDDY_STOP_HW _IO(EVI_EDDY_IOC_MAGIC, 6)
#define EVI_EDDY_SCANNER_STATUS _IOR(EVI_EDDY_IOC_MAGIC, 7, uint32_t)
#define EVI_EDDY_SCANNER_CMD \
	_IOWR(EVI_EDDY_IOC_MAGIC, 12, struct scanner_command)
#define EVI_EDDY_SCANNER_FIRMWARE _IOW(EVI_EDDY_IOC_MAGIC, 14, char *)

#endif /* EVI_IOCTL_H */
