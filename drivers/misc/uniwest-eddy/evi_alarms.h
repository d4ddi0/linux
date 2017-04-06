/***************************************************************************
 * evi_alarms.h
 *
 * Offsets of memory mapped eddy current alarm registers
 * in the EVI_EDDY hardware
 *
 ***************************************************************************/
#ifndef EVI_ALARMS_H
#define EVI_ALARMS_H

/*
 * Alarm Register Addressing
 *
 * Addressing
 * Bits 25:16 - 0x002 - Select Alarm Bank
 * Bits 15:09 - Timeslot Select (128 Timeslots)
 * Bits 08:02 - Register Select
 *
 */
#define EVI_ALARM_BASE    0x20000

/*
 * Circular alarm limit and center registers
 *
 * LIM (limit) registers are 32 bit
 * CTR (center) registers are 16 bit
 */
#define EVI_ALARM_CIR_X_LIM  0x0
#define EVI_ALARM_CIR_Y_LIM  0x4
#define EVI_ALARM_CIR_X_CTR  0x8
#define EVI_ALARM_CIR_Y_CTR  0xC

#define EVI_ALARM_CIR_0_BASE 0x20010
#define EVI_ALARM_CIR_1_BASE 0x20020
#define EVI_ALARM_CIR_2_BASE 0x20030
#define EVI_ALARM_CIR_3_BASE 0x20040
#define EVI_ALARM_CIR_4_BASE 0x20050
#define EVI_ALARM_CIR_5_BASE 0x20060
#define EVI_ALARM_CIR_6_BASE 0x20070
#define EVI_ALARM_CIR_7_BASE 0x20080

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

#define EVI_ALARM_CIR_CTL_0  0x20090
#define EVI_ALARM_CIR_CTL_1  0x20094
#define EVI_ALARM_CIR_CTL_2  0x20098
#define EVI_ALARM_CIR_CTL_3  0x2009C
#define EVI_ALARM_CIR_CTL_4  0x200A0
#define EVI_ALARM_CIR_CTL_5  0x200A4
#define EVI_ALARM_CIR_CTL_6  0x200A8
#define EVI_ALARM_CIR_CTL_7  0x200AC

/*
 * Rectangular alarm limit registers
 *
 * Registers are 32 bit, Comparing against 24 bit value
 *
 */
#define EVI_ALARM_RECT_X_POS  0x0
#define EVI_ALARM_RECT_X_NEG  0x4
#define EVI_ALARM_RECT_Y_POS  0x8
#define EVI_ALARM_RECT_Y_NEG  0xC

#define EVI_ALARM_RECT_0_BASE 0x20110
#define EVI_ALARM_RECT_1_BASE 0x20120
#define EVI_ALARM_RECT_2_BASE 0x20130
#define EVI_ALARM_RECT_3_BASE 0x20140
#define EVI_ALARM_RECT_4_BASE 0x20150
#define EVI_ALARM_RECT_5_BASE 0x20160
#define EVI_ALARM_RECT_6_BASE 0x20170
#define EVI_ALARM_RECT_7_BASE 0x20180

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

#define EVI_ALARM_RECT_CTL_0  0x20190
#define EVI_ALARM_RECT_CTL_1  0x20194
#define EVI_ALARM_RECT_CTL_2  0x20198
#define EVI_ALARM_RECT_CTL_3  0x2019C
#define EVI_ALARM_RECT_CTL_4  0x201A0
#define EVI_ALARM_RECT_CTL_5  0x201A4
#define EVI_ALARM_RECT_CTL_6  0x201A8
#define EVI_ALARM_RECT_CTL_7  0x201AC

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

#define EVI_ALARM_OUTPUT_BANK0  0x20000
#define EVI_ALARM_OUTPUT_BANK1  0x20004
#define EVI_ALARM_AND_BANK0     0x20100
#define EVI_ALARM_AND_BANK1     0x20104
#define EVI_ALARM_OR_BANK0      0x20108
#define EVI_ALARM_OR_BANK1      0x2010C

/* Test Data */
#define EVI_TEST_0 0x30000
#define EVI_TEST_F 0x30004
#define EVI_TEST_5 0x30008
#define EVI_TEST_A 0x3000C

#endif /* EVI_ALARMS_H */
