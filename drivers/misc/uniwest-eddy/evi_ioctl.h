#ifndef EVI_IOCTL_H
#define EVI_IOCTL_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
#include <stdint.h>
#endif

#define EVIFPGA_SAMPLE_QUEUE_NUM 1024
#define EVIFPGA_SAMPLE_QUEUE_MASK (EVIFPGA_SAMPLE_QUEUE_NUM - 1)

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
#define EVI_FPGA_IOC_MAGIC 'E'
#define EVI_FPGA_VERSION _IOR(EVI_FPGA_IOC_MAGIC, 0, char *)
#define EVI_FPGA_START_DATA_FLOW _IO(EVI_FPGA_IOC_MAGIC, 3)
#define EVI_FPGA_STOP_DATA_FLOW _IO(EVI_FPGA_IOC_MAGIC, 4)
#define EVI_FPGA_START_FPGA _IO(EVI_FPGA_IOC_MAGIC, 5)
#define EVI_FPGA_STOP_FPGA _IO(EVI_FPGA_IOC_MAGIC, 6)
#define EVI_FPGA_SCANNER_STATUS _IOR(EVI_FPGA_IOC_MAGIC, 7, uint32_t)
#define EVI_FPGA_SCANNER_CMD \
	_IOWR(EVI_FPGA_IOC_MAGIC, 12, struct scanner_command)
#define EVI_FPGA_SCANNER_FIRMWARE _IOW(EVI_FPGA_IOC_MAGIC, 14, char *)

/* mmap sizes and offsets */
#define VM_PAGE_SIZE_CONVERT(a) ((((a) / 4096UL) + 1) * 4096)

#define FPGA_VM_DRIVER_SETTINGS_OFFSET 0UL
#define FPGA_VM_DRIVER_SETTINGS_SIZE (12288UL)

#define FPGA_VM_EIM_OFFSET \
	(FPGA_VM_DRIVER_SETTINGS_OFFSET + FPGA_VM_DRIVER_SETTINGS_SIZE)
#define FPGA_VM_EIM_SIZE (VM_PAGE_SIZE_CONVERT(sizeof(EVI_FPGA_EIM)))

#endif /* EVI_IOCTL_H */
