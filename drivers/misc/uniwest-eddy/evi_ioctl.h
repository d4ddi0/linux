#ifndef EVI_IOCTL_H
#define EVI_IOCTL_H

#include <linux/types.h>
#include <linux/ioctl.h>

struct sample {
	__u32 flags;
	__u32 timestamp;
	__s32 encoders[4];
	__u32 x0;
	__u32 y0;
	__u32 x1;
	__u32 y1;
};

/* ioctls */
#define EVI_EDDY_IOC_MAGIC 'E'
#define EVI_EDDY_VERSION _IOR(EVI_EDDY_IOC_MAGIC, 0, char *)
#define EVI_EDDY_START_DATA_FLOW _IO(EVI_EDDY_IOC_MAGIC, 3)
#define EVI_EDDY_STOP_DATA_FLOW _IO(EVI_EDDY_IOC_MAGIC, 4)
#define EVI_EDDY_START_HW _IO(EVI_EDDY_IOC_MAGIC, 5)
#define EVI_EDDY_STOP_HW _IO(EVI_EDDY_IOC_MAGIC, 6)

#endif /* EVI_IOCTL_H */
