/***************************************************************************
 * evi_fpga_registers
 *
 * Offsets of fpga registers via the evi EIM bus interface
 *
 * Base hardware address is 0x08000000
 *
 * Example to read EVI_FPGA_EIM_BATTBOXRECV:
 * /unit_tests/memtool 08010704 1
 *
 * Example to write 0x10 EVI_FPGA_EIM_BATTBOXSEND:
 * /unit_tests/memtool 08010700=10
 *
 ***************************************************************************/
#ifndef EVI_FPGA_REGISTERS_H
#define EVI_FPGA_REGISTERS_H

/***************************************************************************
 * EVI_FPGA_EIM address offsets
 ***************************************************************************/

/*
 * EVI_FPGA_EIM_TIMESLOTS
 *
 * There are 128 Timeslots
 * Each FPGA_TIMESLOT_DEF has a diff and abs channel 0x100 bytes each
 * Total size 0x10000 bytes
 */
#define EVI_FPGA_EIM_TIMESLOTS 0x00000

/*
 * EVI_FPGA_EIM_CONTROLREG
 *
 * Bits  6: 0  - Index into table of frequencies, or max frequency to use
 * Bit      7  - 1 = use specified index only,
 *               0 = cycle through indexes from 0 to index number
 * Bits  9: 8  - DAC Test Mode
 *               00 - Normal Operation
 *               01 - Ramp
 *               10 - All 1s
 *               11 - Half Scale
 * Bits 13:10  - encoder enable bits for encoder 3 to 0
 * Bit     14  - Disable all internal LEDs:
 *               0 - LEDs Controlled as defined below
 *               1 - All LEDs off to save power
 * Bit     15  - 0 = normal
 *               1 = no ping-pong in multi-channel mode
 * Bits 17:16  - index of encoder for control (0-3)
 * Bits 19:18  - DAC Test Mode Select
 *               00 - Normal Operation
 *               01 - Probe Drive DAC only, other set to 0
 *               10 - Differential NULL DACs only, others set to 0
 *               11 - Absolute NULL DACs only, others set to 0
 * Bits 23:20  - LEDs (probably unused)
 *               LED 0 ==> 1-sec heart beat (LCD clock)
 *               LED 1 ==> 102.4 system PLL Lock
 *               LED 2 ==> VGA PLL Lock
 *               LED 3 ==> Any saturation (diff sats high during DFT time)
 * Bit     20  - LED 4 (reserved for Reset?)
 * Bit     21  - LED 5 I2C okay
 * Bit     22  - LED 6 Ethernet other communications okay
 * Bit     23  - LED 7
 * Bit     24  - Mux Out Mode,
 *               0 = normal mux probe address output per channel,
 *               1 = static outputs for STROBE, D1-D5
 * Bit     25  - Null Dac outputs,
 *               0 = outputs swapped (newer)
 *               1 = not swapped (older)
 * Bit     26  - Flush all filter channels, Set to 1 then to 0.
 * Bit     27  - Probe_trouble_SD 0 = normal, 1 = shutdown
 * Bit     28  - CMP_Shutdown
 *               0 = normal
 *               1 = shutdown
 * Bit     29  - 0 = normal
 *               1 = enable DSP Skip mode
 * Bit     30  - FilterBypass,
 *               0 = use filter
 *               1 = don't use filter
 * Bit     31  - 1 to Use the data FIFO interrupt; defaults to 0
 */
#define EVI_FPGA_EIM_CONTROLREG 0x10000
/* define unused  0x10004 */
#define EVI_FPGA_EIM_STARTPROCESSING 0x10008
/* Write anything here to start processing */
#define EVI_FPGA_EIM_STOPPROCESSING 0x1000C
/* Write anything here to stop processing */

/*
 * EVI_FPGA_EIM_STATUS:  read here to determine status
 *
 * Bits  3: 0 - UART RX error count. (Rolls from 15 to 0)
 * Bits  6: 4 - Scanner fifo count
 * Bit      7 - Scanner fifo empty
 * Bit      8 - Scanner data all 1's
 * Bit      9 - Scanner data all 0's
 * Bits 11:10 - Not Used
 * Bit     12 - Scanner UART RX error. (uses RTS line to Scanner)
 * Bit     13 - Scanner fifo full
 * Bit     14 - Alarm enable input      From Multi_IO input
 * Bits 31:15 - Not used
 */
union ef_status {
	uint32_t raw_data;
	struct {
		uint32_t nr_scanner_err :4;
		uint32_t nr_scanner_fifo :3;
		uint32_t scanner_empty :1;
		uint32_t scanner_ones :1;
		uint32_t scanner_zeroes :1;
		uint32_t unused0 :2;
		uint32_t scanner_rx_err :1;
		uint32_t scanner_fifo_full :1;
		uint32_t alarm_enable :1;
		uint32_t unused1 :17;
	};
};
#define EVI_FPGA_EIM_STATUS 0x10010

/*
 * EVI_FPGA_EIM_ALARMAUDIOCONTROL
 *
 * Buzzer alarm Frequency and PWM (level) settings
 * Bits 31:16 - Frequency Control.  Sets Frequency as follows:
 *              Freq = 1/((Setting+1)*39.0625 ns) :: 0.000000039
 *              (Note: the 39.0625 ns period comes from the 102.4 MHz clock
 *              divided by four)
 *              Example: a setting of 5119 will generate a 5 KHz tone
 * Bits 15: 0 - Level (PWM) Control.  Setting determines how long the
 *              Alarm_PWM signal remains high.
 *              Current operation: the signal starts high and goes low
 *              when this setting is the count value.
 */
#define EVI_FPGA_EIM_ALARMAUDIOCONTROL 0x10014

/*
 * EVI_FPGA_EIM_BuildDate
 *
 * Time in seconds since the beginning of the POSIX epoch
 * 64 bits wide
 */
#define EVI_FPGA_EIM_BUILDDATE 0x10018

/*
 * EVI_FPGA_EIM_VERSION
 *
 * when displayed in hex, this is 4 parts of the version, 2 digits per part
 */
#define EVI_FPGA_EIM_VERSION 0x10020

/*
 * EVI_FPGA_EIM_ANALOGOUTCONTROL
 *
 * AnalogOutControl format
 * Bits  6: 0 - Primary Timeslot index
 * Bits  9: 7 - X Output
 *                000: Absolute X
 *                001: Absolute Y
 *                010: Differential X
 *                011: Differential Y
 *                100: Mixed (Math) X
 *                101: Mixed (Math) Y
 *                else: None (all zeros)
 * Bits 12:10 - Y Output
 *                000: Absolute X
 *                001: Absolute Y
 *                010: Differential X
 *                011: Differential Y
 *                100: Mixed (Math) X
 *                101: Mixed (Math) Y
 *                else: None (all zeros)
 * Bits 15:13 - unused
 * Bits 17:16 - 00: No XY DAC output for this block
 *              otherwise:XY Output sent for this block (0-3)
 * Bits 31:18 - unused
 */
#define EVI_FPGA_EIM_ANALOGOUTCONTROL 0x10024

/*
 * EVI_FPGA_EIM_NULLDACOFFSET
 * This value is added to the NullDac signal for all time-slots
 * It is a 24-bit number.  Only the most significant 14 bits of the
 * 24-bit number are actually sent to the DAC
 */
#define EVI_FPGA_EIM_NULLDACOFFSET 0x10034


/*
 * EVI_FPGA_EIM_ALARMOUT
 *
 * AlarmOut definitions
 * Bit     0 = Alarm Common Out
 *             1 = alarm, 0 = no alarm (OR of all enabled alarms)
 * Bits  1: 7 - unused
 * Bit      8 = Alarm 1 Out, 1 = alarm, 0 = no alarm
 * Bit      9 = Alarm 2 Out, 1 = alarm, 0 = no alarm
 * Bit     10 = Alarm 3 Out, 1 = alarm, 0 = no alarm
 * Bit     11 = Alarm 4 Out, 1 = alarm, 0 = no alarm
 * Bit     12 = Alarm 5 Out, 1 = alarm, 0 = no alarm
 * Bit     13 = Alarm 6 Out, 1 = alarm, 0 = no alarm
 * Bit     14 = Alarm 7 Out, 1 = alarm, 0 = no alarm
 * Bit     15 = Alarm 8 Out, 1 = alarm, 0 = no alarm
 */
#define EVI_FPGA_EIM_ALARMOUT 0x10044

#define FPGA_REG_ALARM_OUT_OR_BIT   (1 << 0)
#define FPGA_REG_ALARM_OUT_SND_BIT  (1 << 1)
#define FPGA_REG_ALARM_OUT_SHIFT    (8)
#define FPGA_REG_ALARM_OUT_ALARM_1  (1 << FPGA_REG_ALARM_OUT_SHIFT)
#define FPGA_REG_ALARM_OUT_ALARM_2  (1 << (FPGA_REG_ALARM_OUT_SHIFT + 1))
#define FPGA_REG_ALARM_OUT_ALARM_3  (1 << (FPGA_REG_ALARM_OUT_SHIFT + 2))
#define FPGA_REG_ALARM_OUT_ALARM_4  (1 << (FPGA_REG_ALARM_OUT_SHIFT + 3))
#define FPGA_REG_ALARM_OUT_ALARM_5  (1 << (FPGA_REG_ALARM_OUT_SHIFT + 4))
#define FPGA_REG_ALARM_OUT_ALARM_6  (1 << (FPGA_REG_ALARM_OUT_SHIFT + 5))
#define FPGA_REG_ALARM_OUT_ALARM_7  (1 << (FPGA_REG_ALARM_OUT_SHIFT + 6))
#define FPGA_REG_ALARM_OUT_ALARM_8  (1 << (FPGA_REG_ALARM_OUT_SHIFT + 7))
#define FPGA_REG_ALARM_OUT_BIT(bit) (1 << (FPGA_REG_ALARM_OUT_SHIFT + (bit)))

/*
 * EVI_FPGA_EIM_ENCODERCONTROL
 *
 * LSByte is Encoder 0, MSByte is Encoder 3
 *
 * For each byte:
 * Bit 1:0 - mode
 *            0 = linear encoder, command sets to zero at data 0
 *            1 = linear encoder, resets at each data set
 *            2 = rotational encoder, 0-N
 *                (loading a 0 gives 1 count)  period is N+1
 *            3 = rotational encoder, reset to zero on index
 *                (doesn't use the period)

 * Bit   2 - Pulse on position input operates counter in up mode
 * Bit   3 - Clear Encoder register (write only), write unused bits to zero.
 */
#define EVI_FPGA_EIM_ENCODERCONTROL 0x10048

/*
 * EVI_FPGA_EIM_ENCODERTERMINALCOUNT
 *
 * Terminal Count is 1 less than number of counts per revolution
 * or encoders  in rotational mode
 */
#define EVI_FPGA_EIM_ENCODERTERMINALCOUNT 0x1004C

/*
 * EVI_FPGA_EIM_SYNCDEBOUNCE
 *
 * number of 100MHz clocks
 */
#define EVI_FPGA_EIM_SYNCDEBOUNCE 0x1005C

/*
 * EVI_FPGA_EIM_STATICAUXOUT
 *
 * static outputs,
 * Bits 0-4 = data bits(D1-D5),
 * Bit 5 = STROBE
 * StaticAuxOut stores bits to send out to STROBE and D1-D5 if the
 * control register has the MuxOutMode bit is set to 1
 */
#define EVI_FPGA_EIM_STATICAUXOUT 0x10060

/*
 * EVI_FPGA_EIM_FIFO_STATUS
 *
 * Status information regarding the FPGA Data FIFO
 *
 * Bits  11:0 fifo_count:  number of 32 bit data in the fifo
 * Bit     12 empty:       fifo is empty
 * Bit     13 almost_full: fifo has reached almost_full
 * Bit     14 full:        fifo is completely full
 * Bit     15 word_zeroes: an all zeroes word encountered
 * Bit     16 stopped:     fpga cores stopped. 0 if running.
 * Bit     17 int0:        reflects the value of the data interrupt
 * Bit     18 int1:        reflects the value of the scanner interrupt
 * Bits 19:31 unused
 *
 */
union ef_fifo_status {
	uint32_t raw_data;
	struct {
		uint32_t fifo_count :12;
		uint32_t empty :1;
		uint32_t almost_full :1;
		uint32_t full :1;
		uint32_t word_zeroes :1;
		uint32_t stopped :1;
		uint32_t int0 :1;
		uint32_t int1 :1;
		uint32_t unused :13;
	};
};
#define EVI_FPGA_EIM_FIFO_STATUS 0x10064

/*
 * EVI_FPGA_EIM_FPGA_MIXED_CHANNELS
 *
 * 16 mixed locations were set aside. not all are expected to be used
 * Each FPGA_MIXED_DEF is 32 bytes, 8 (4-byte) longs or 0x20 bytes
 */
#define EVI_FPGA_EIM_FPGA_MIXED_CHANNELS 0x10200

/*
 * EVI_FPGA_EIM_MULTI_IO_INPUTCONTROL
 *
 * Need to define. For Alpha, keep it simple
 * Bit 0: Encoder select-- 0: Encoders 2,3 from Multi_IO inputs 3:0
 *                         1: Encoders 2,3 FPGA pins
 */
#define EVI_FPGA_EIM_MULTI_IO_INPUTCONTROL 0x10400

/*
 * EVI_FPGA_EIM_MULTI_IO_OUTPUTCONTROL
 * Uses bits 3:0 ignores high bits
 *      0x0:       Multi_IO out = Software controlled register (slow alarm)
 *      0x1:       Multi_IO out = 128 Mux selection
 *      0x2:       Multi_IO out = Slow Mux selection
 *      0x3:       Multi_IO out = Test Pattern - 0xAA
 *      0x4:       Multi_IO out = Test Pattern - 0x55
 *      0x5:       Multi_IO out = FPGA Alarm Output
 *      otherwise: Multi_IO out = Software controlled register (slow alarm)
 */
#define EVI_FPGA_EIM_MULTI_IO_OUTPUTCONTROL 0x10404

/*
 * EVI_FPGA_EIM_MULTI_IO_REFERENCE
 *
 * Default set by FPGA on power-on reset: 16'h2A90 to set both outputs at +3.3V
 * Bits 15: 0 - Multi_IO voltage reference DAC Control. Used to set
 *              Multi_IO output voltage reference (Ref DAC A)
 *              Multi_IO input voltage reference  (Ref DAC B)
 * Bit     15 - A1- Address 1: Normally 0, unless utilizing
 *              power-down modes (see DAC documentation)
 * Bit     14 - A0- Address 0: 0- DAC A, 1- DAC B
 * Bits 13:12 - OP1:OP0- Operational modes. (see DAC documentation)
 *              Normally 2b'01 to write to the specified register and
 *              update outputs Also may want to use 2b'10 to write to
 *              both registers and update outputs
 * Bits 11: 4 - Eight bit data word to set DAC
 *              Vout = (Vref * Setting)/256,  Vref = 5V
 * Bits  3: 0 - Not used
 *
 * Examples:    Set DAC A to 5 Volts: 0x5FF0
 *              Set DAC B to 0 Volts: 0x1000
 *              Set both DACS to 3.3 Volts: 0x2A90
 */
#define EVI_FPGA_EIM_MULTI_IO_REFERENCE 0x10408

/*
 * EVI_FPGA_EIM_ADC_CONTROL
 *
 * Default set by FPGA on power-on reset: 16'h0080 to reset the ADC
 * Bits 15: 0 - Control Address/Data for ADC setup. (see ADC documentation)
 * Bits 15: 8 - Address
 * Bits  7: 0 - Data
 */
#define EVI_FPGA_EIM_ADC_CONTROL 0x1040C

/*
 * EVI_FPGA_EIM_XY_DAC_CNTL
 *
 * Default set by FPGA on power-on reset
 * Bits 31:20   Not used
 * Bit     19-  DAC rate control (BCLK and LRCLK frequencies)
 *              0         50 kHz (default)
 *              1         25 kHz
 * Bits 18:16-  000       Normal operation (not test mode)
 *              001       Test mode ramp. Test word sets step size
 *                        and number of steps
 *              011       Set X and Y outputs to all ones: 24'hFFFFFF
 *              101       Set X and Y outputs to 24'h7FFFFF
 *              111       Set X and Y outputs to lower 24 bits of test word
 * Bits 15: 8 - Address
 * Bits  7: 0 - Data
 */
#define EVI_FPGA_EIM_XY_DAC_CNTL 0x10410


/*
 * Bits  23:0   Either ramp mode step size or DAC output value
 *              depending on control setting
 * Bits 31:24   In ramp mode, the number of steps before
 *              complementing the add/subtract mode
 */
#define EVI_FPGA_TEST_WORD 0x10414

#define EVI_FPGA_EIM_STORE_SAMPLES 0x10418
/* Bits  12:0   Stop address for raw sample storage (size - 1).
 *              Max value 8191 (0x1FFF) for 8192 sample size
 * Bit     16   1: Turn on storage mode  0: Disable storage mode
 * Bits 22:20   000: normal operation
 *              001: Chan0: Differential: Raw Samples on X, Y off
 *                   Chan1: Absolute:     Raw Samples on X, Y off
 *              010: Chan0: Differential: Raw Samples on Y, X off
 *                   Chan1: Absolute:     Raw Samples on Y, X off
 *              011: Chan0: Differential: Raw Samples on X, Y off
 *                   Chan1: Differential: Normal operation
 *              100: Chan0: Absolute:     Raw Samples on X, Y off
 *                   Chan1: Absolute:     Normal operation
 *              101: Chan0: Differential: Raw Samples on X, Y off
 *                   Chan1: Absolute:     Normal operation
 *              110: Chan0: Differential: Normal operation
 *                   Chan1: Absolute:     Raw Samples on X, normal on Y
 *
 *              Raw Sample word format:
 *              31   27   23   19   15   11   7    3
 *              0000 00|0 |||| |||| |||| ||00 0000 0000
 *                     |  \--------------/
 *                     |          |---------> 14 bit raw data sample
 *                     |--------------------> 0verflow bit
 */

/* align next field on 0x10500 boundary */

/*
 * Scanner communication registers
 *
 * EVI_FPGA_EIM_SCANNERSEND sends 4 bytes to the Scanner UART
 * EVI_FPGA_EIM_SCANNERRECV holds messages from the Scanner UART
 * This register is buffered by an 8 stage FIFO.
 * EVI_FPGA_EIM_SCANNER_FIFO_IN holds the message most recently
 * inserted into the FIFO for debugging
 */
#define EVI_FPGA_EIM_SCANNERSEND 0x10500
#define EVI_FPGA_EIM_SCANNERRECV 0x10600
#define EVI_FPGA_EIM_SCANNER_FIFO_IN 0x10604

/*
 * EVI_FPGA_EIM_BATTBOXSEND
 *
 * Send bits to battery box interface
 *
 * Bit 31 sets control for bit 7(high: write enabled, low: tri-state)
 * Bit 30 sets control for bit 6 (high: write enabled, low: tri-state)
 * Bit 29 sets control for bit 5 (high: write enabled, low: tri-state)
 * Bit 28 sets control for bit 4 (high: write enabled, low: tri-state)
 * Bit 27 sets control for bit 3 (high: write enabled, low: tri-state)
 * Bit 26 sets control for bit 2 (high: write enabled, low: tri-state)
 * Bit 25 sets control for bit 1 (high: write enabled, low: tri-state)
 * Bit 24 sets control for bit 0 (high: write enabled, low: tri-state)
 * Bits 23:08 - Unused
 * Bits  7: 0 - Battery box output
 */
#define EVI_FPGA_EIM_BATTBOXSEND 0x10700

/*
 * EVI_FPGA_EIM_BATTBOXRECV
 *
 * 32 bit word from the Battery Box interface
 *
 * Bit 31 is the CTS line of the scanner UART
 * Bits 7:0 are the Battery Box signals.
 * These may be from the external pins or looped back from the Battery Box
 * write lines that are enabled Reading this word clears the associated
 * interrupt with a CTS status change
 */
#define EVI_FPGA_EIM_BATTBOXRECV 0x10704

/*
 * EVI_FPGA_EIM_DATA_FIFO
 *
 * 32 bit fifo containing up to 25 data points
 * each data point consists of 10
 * 32 bit word in th efollowing order:
 *
 * FLAGS
 * TIMESTAMP_LO
 * ENCODER 0
 * ENCODER 0
 * ENCODER 0
 * ENCODER 0
 * CHANNEL 0 X
 * CHANNEL 0 Y
 * CHANNEL 1 X
 * CHANNEL 1 Y
 */
#define EVI_FPGA_EIM_FIFO 0x11000

/* align next field on 0x12000 boundary */
#define EVI_FPGA_EIM_DATABUF 0x12000

/* Fast Alarm registers */
/*
 * EVI_FPGA_EIM_ALARM_STATE
 *
 * Configure Multi-io alarm interface
 *
 * Bits 31:24 - Unused
 * Bits 23:16 - Output Delay Enable (Removes Delay circuit from output)
 * Bits 15:08 - Output Enable (Enables Individual Alarm Output)
 * Bits 07:00 - Pin Polarity (Sets output to active High or active Low)
 */
union ef_alarm_status {
	uint32_t raw_data;
	struct {
		uint8_t pin_polarity;
		uint8_t output_enable;
		uint8_t delay_enable;
		uint8_t unused;
	};
};
#define EVI_FPGA_EIM_ALARM_STATE   0x10430

/*
 * EVI_FPGA_EIM_ALARM_DURATION
 *
 * Configure Hold Duration for Multi-IO outputs
 *
 * 32-bit register, sets time in 1/4 ms increments
 */

#define EVI_FPGA_EIM_ALARM_DURATION_0 0x10440
#define EVI_FPGA_EIM_ALARM_DURATION_1 0x10444
#define EVI_FPGA_EIM_ALARM_DURATION_2 0x10448
#define EVI_FPGA_EIM_ALARM_DURATION_3 0x1044C
#define EVI_FPGA_EIM_ALARM_DURATION_4 0x10450
#define EVI_FPGA_EIM_ALARM_DURATION_5 0x10454
#define EVI_FPGA_EIM_ALARM_DURATION_6 0x10458
#define EVI_FPGA_EIM_ALARM_DURATION_7 0x1045C

/*
 * EVI_FPGA_EIM_ALARM_DELAY
 *
 * Configure Hold Delay for Multi-IO outputs
 *
 * 32-bit register, sets time in 1/4 ms increments
 */

#define EVI_FPGA_EIM_ALARM_DELAY_0    0x10460
#define EVI_FPGA_EIM_ALARM_DELAY_1    0x10464
#define EVI_FPGA_EIM_ALARM_DELAY_2    0x10468
#define EVI_FPGA_EIM_ALARM_DELAY_3    0x1046C
#define EVI_FPGA_EIM_ALARM_DELAY_4    0x10470
#define EVI_FPGA_EIM_ALARM_DELAY_5    0x10474
#define EVI_FPGA_EIM_ALARM_DELAY_6    0x10478
#define EVI_FPGA_EIM_ALARM_DELAY_7    0x1047C

/*
 * Alarm Register Addressing
 *
 * Addressing
 * Bits 25:16 - 0x002 - Select Alarm Bank
 * Bits 15:09 - Timeslot Select (128 Timeslots)
 * Bits 08:02 - Register Select
 *
 */
#define EVI_FPGA_EIM_ALARM_BASE    0x20000

/*
 * Circular alarm limit and center registers
 *
 * LIM (limit) registers are 32 bit
 * CTR (center) registers are 16 bit
 */
#define EVI_FPGA_EIM_AL_CIR_X_LIM  0x0
#define EVI_FPGA_EIM_AL_CIR_Y_LIM  0x4
#define EVI_FPGA_EIM_AL_CIR_X_CTR  0x8
#define EVI_FPGA_EIM_AL_CIR_Y_CTR  0xC

#define EVI_FPGA_EIM_AL_CIR_0_BASE 0x20010
#define EVI_FPGA_EIM_AL_CIR_1_BASE 0x20020
#define EVI_FPGA_EIM_AL_CIR_2_BASE 0x20030
#define EVI_FPGA_EIM_AL_CIR_3_BASE 0x20040
#define EVI_FPGA_EIM_AL_CIR_4_BASE 0x20050
#define EVI_FPGA_EIM_AL_CIR_5_BASE 0x20060
#define EVI_FPGA_EIM_AL_CIR_6_BASE 0x20070
#define EVI_FPGA_EIM_AL_CIR_7_BASE 0x20080

/*
 * Circular alarm Control Registers
 *
 * Bit     0 - Enable Alarm for this timeslot
 * Bit     1 - Enable Negation for this alarm
 * Bits 8: 2 - Unused
 * Bit     9 - Differental / Absolute channel input
 *             0 = Differental
 *             1 = Absolute
 * Bits 31:10- Unused
 */

#define EVI_FPGA_EIM_AL_CIR_CTL_0  0x20090
#define EVI_FPGA_EIM_AL_CIR_CTL_1  0x20094
#define EVI_FPGA_EIM_AL_CIR_CTL_2  0x20098
#define EVI_FPGA_EIM_AL_CIR_CTL_3  0x2009C
#define EVI_FPGA_EIM_AL_CIR_CTL_4  0x200A0
#define EVI_FPGA_EIM_AL_CIR_CTL_5  0x200A4
#define EVI_FPGA_EIM_AL_CIR_CTL_6  0x200A8
#define EVI_FPGA_EIM_AL_CIR_CTL_7  0x200AC

/*
 * Rectangular alarm limit registers
 *
 * Registers are 32 bit, Comparing against 24 bit value
 *
 */
#define EVI_FPGA_EIM_AL_RECT_X_POS  0x0
#define EVI_FPGA_EIM_AL_RECT_X_NEG  0x4
#define EVI_FPGA_EIM_AL_RECT_Y_POS  0x8
#define EVI_FPGA_EIM_AL_RECT_Y_NEG  0xC

#define EVI_FPGA_EIM_AL_RECT_0_BASE 0x20110
#define EVI_FPGA_EIM_AL_RECT_1_BASE 0x20120
#define EVI_FPGA_EIM_AL_RECT_2_BASE 0x20130
#define EVI_FPGA_EIM_AL_RECT_3_BASE 0x20140
#define EVI_FPGA_EIM_AL_RECT_4_BASE 0x20150
#define EVI_FPGA_EIM_AL_RECT_5_BASE 0x20160
#define EVI_FPGA_EIM_AL_RECT_6_BASE 0x20170
#define EVI_FPGA_EIM_AL_RECT_7_BASE 0x20180

/*
 * Rectangular alarm Control Registers
 *
 * Bit     0 - Enable Alarm X Positive Limit
 * Bit     1 - Enable Alarm X Negitive Limit
 * Bit     2 - Enable Alarm Y Positive Limit
 * Bit     3 - Enable Alarm Y Negitive Limit
 * Bit     4 - Enable Negation X Positive
 * Bit     5 - Enable Negation X Negitive
 * Bit     6 - Enable Negation Y Positive
 * Bit     7 - Enable Negation Y Negitive
 * Bit     8 - Unused
 * Bit     9 - Differental / Absolute channel input
 *             0 = Differental
 *             1 = Absolute
 * Bits 31:10- Unused
 */

#define EVI_FPGA_EIM_AL_RECT_CTL_0  0x20190
#define EVI_FPGA_EIM_AL_RECT_CTL_1  0x20194
#define EVI_FPGA_EIM_AL_RECT_CTL_2  0x20198
#define EVI_FPGA_EIM_AL_RECT_CTL_3  0x2019C
#define EVI_FPGA_EIM_AL_RECT_CTL_4  0x201A0
#define EVI_FPGA_EIM_AL_RECT_CTL_5  0x201A4
#define EVI_FPGA_EIM_AL_RECT_CTL_6  0x201A8
#define EVI_FPGA_EIM_AL_RECT_CTL_7  0x201AC

/*
 * Multi-IO Output Selector/Combinational Logic
 *
 * Bank0
 * Bits  7: 0 Channel 0/ Output 0
 * Bits 15: 8 Channel 1/ Output 1
 * Bits 23:16 Channel 2/ Output 2
 * Bits 31:24 Channel 3/ Output 3
 *
 * Bank1
 * Bits  7: 0 Channel 4/ Output 4
 * Bits 15: 8 Channel 5/ Output 5
 * Bits 23:16 Channel 6/ Output 6
 * Bits 31:24 Channel 7/ Output 7
 *
 * Selections - 8-bit selections
 * 0x00       - Disabled alarm (Outputs a 0)
 * 0x80-0x87  - Rectangular Alarm 0-7
 * 0x88-0x8F  - Circular Alarm 0-7
 * 0x90       - And Gate (For combinational logic)
 * 0x98       - Or Gate (For combinational logic)
 * 0xFF       - Always enabled (Outputs a 1)
 */

#define EVI_FPGA_EIM_AL_OUTPUT_BANK0  0x20000
#define EVI_FPGA_EIM_AL_OUTPUT_BANK1  0x20004
#define EVI_FPGA_EIM_AL_AND_BANK0     0x20100
#define EVI_FPGA_EIM_AL_AND_BANK1     0x20104
#define EVI_FPGA_EIM_AL_OR_BANK0      0x20108
#define EVI_FPGA_EIM_AL_OR_BANK1      0x2010C


/* EIM Bus Test Data */
#define EVI_FPGA_EIM_TEST_0 0x30000
#define EVI_FPGA_EIM_TEST_F 0x30004
#define EVI_FPGA_EIM_TEST_5 0x30008
#define EVI_FPGA_EIM_TEST_A 0x3000C

#endif /* EVI_FPGA_REGISTERS_H */
