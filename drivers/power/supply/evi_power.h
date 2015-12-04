/**
 * @file evi_power.h
 *
 * Copyright 2014 United Western Technologies
 *
 */

#ifndef EVI_POWER_H
#define EVI_POWER_H

struct evi_power_version {
	uint8_t subrev;
	uint8_t rev;
	uint8_t minor;
	uint8_t major;
};

struct evi_power_status {
	uint32_t shutting_down: 1;
	uint32_t pmic0_fault: 1;
	uint32_t pmic1_fault: 1;
	uint32_t motor0_fault: 1;
	uint32_t motor1_fault: 1;
	uint32_t motor2_fault: 1;
	uint32_t unused2: 26;
};

struct power_data {
	union {
		struct evi_power_status reg;
		uint32_t raw;
	};
};

struct evi_master_status {
	uint32_t shutdown_ok: 1;
	uint32_t pmic0_clr_fault: 1;
	uint32_t pmic1_clr_fault: 1;
	uint32_t motor0_clr_fault: 1;
	uint32_t motor1_clr_fault: 1;
	uint32_t motor2_clr_fault: 1;
	uint32_t read_reg_clr: 1;
	uint32_t unused1: 8;
	uint32_t sw_freq_sel: 1;
	uint32_t shutdown_not_used: 1;
	uint32_t pmic0_mask_fault: 1;
	uint32_t pmic1_mask_fault: 1;
	uint32_t motor0_mask_fault: 1;
	uint32_t motor1_mask_fault: 1;
	uint32_t motor2_mask_fault: 1;
	uint32_t read_reg_mask: 1;
	uint32_t unused2: 8;
	uint32_t reset: 1;
};

struct master_data {
	union {
		struct evi_master_status reg;
		uint32_t raw;
	};
};

struct power_button {
	uint8_t soft_shutdown_time;
	uint8_t shutdown_irq_repeat_time;
	uint8_t hard_shutdown_time;
	uint8_t unused;
};

struct motor_power {
	uint32_t enable: 1; /* registar 1 */
	uint32_t pwr_good: 1;
	uint32_t alert: 1;
	uint32_t pwr_monitor_err: 1;
	uint32_t unused: 24;
	uint32_t set_voltage; /* register 2 */
	uint32_t current_lvl; /* register 3 */
	uint32_t pwr_lvl; /* register 4 */
	uint32_t bus_volts; /* register 5 */
	uint32_t shunt_volts; /* register 6 */
};

struct motor_data {
	union {
		struct motor_power reg;
		uint32_t raw;
	};
};

struct pmic_registers {
	uint32_t PWRON: 1;
	uint32_t LOW_Vin: 1;
	uint32_t Therm110: 1;
	uint32_t Therm120: 1;
	uint32_t Therm125: 1;
	uint32_t Therm130: 1;
	uint32_t unused: 2;
	uint32_t SW1Afault: 1;
	uint32_t SW1Bfault: 1;
	uint32_t SW1Cfault: 1;
	uint32_t SW2fault: 1;
	uint32_t SW3Afault: 1;
	uint32_t SW3Bfault: 1;
	uint32_t SW4fault: 1;
	uint32_t unused2: 1;
	uint32_t SWBSTfault: 1;
	uint32_t unused3: 6;
	uint32_t OTP_ECC: 1;
	uint32_t VGEN1fault: 1;
	uint32_t VGEN2fault: 1;
	uint32_t VGEN3fault: 1;
	uint32_t VGEN4fault: 1;
	uint32_t VGEN5fault: 1;
	uint32_t VGEN6fault: 1;
	uint32_t unused4: 2;
};

struct pmic_data {
	union {
		struct pmic_registers reg;
		struct {
			uint8_t int_0;
			uint8_t int_1;
			uint8_t int_3;
			uint8_t int_4;
		};
		uint32_t raw;
	};
};

struct pmic_status {
	struct pmic_data sense;
	struct pmic_data mask;
	struct pmic_data irq_status;
	struct pmic_data irq_status_clear;
};

struct evi_spi_registers {
	const struct evi_power_version version_bytes;
	const struct evi_power_status slave_status;
	struct master_data evi_status;
	struct power_button pwr_button;
	struct motor_power motor[3];
	struct pmic_status pmic[2];
};

#define REG_SIZE	(sizeof(uint32_t))
#define NUM_REGISTERS	(sizeof(struct evi_spi_registers) / sizeof(uint32_t))
#define REG_OFFSET(x)	\
	(((size_t)(&((struct evi_spi_registers *)0)->x)) / REG_SIZE)

struct evi_spi_data {
	union {
		struct evi_spi_registers reg;
		uint32_t raw[NUM_REGISTERS];
	};
};

enum evi_power_spi_mode {
	EVI_POWER_SPI_START = 0xAA,
	EVI_POWER_SPI_READ_REGISTER,
	EVI_POWER_SPI_WRITE_REGISTER,
};

#endif /* EVI_POWER_H */
