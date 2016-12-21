#ifndef EVI_IOCTL_H
#define EVI_IOCTL_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
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

/* ioctls */
#define EVI_EDDY_IOC_MAGIC 'E'
#define EVI_EDDY_VERSION _IOR(EVI_EDDY_IOC_MAGIC, 0, char *)
#define EVI_EDDY_START_DATA_FLOW _IO(EVI_EDDY_IOC_MAGIC, 3)
#define EVI_EDDY_STOP_DATA_FLOW _IO(EVI_EDDY_IOC_MAGIC, 4)
#define EVI_EDDY_START_HW _IO(EVI_EDDY_IOC_MAGIC, 5)
#define EVI_EDDY_STOP_HW _IO(EVI_EDDY_IOC_MAGIC, 6)

#endif /* EVI_IOCTL_H */
