/***************************************************************************
 * EVI_EDDY_CHANNEL_DEF register offsets
 *
 * Offsets from within the diff or abs channel
 * in any one of 128 TIMESLOT definitions
 ***************************************************************************/
#ifndef EVI_EDDY_CHANNEL_H
#define EVI_EDDY_CHANNEL_H

/*
 * EVI_EDDY_MAGSIN
 * EVI_EDDY_MAGCOS
 * EVI_EDDY_XY_SIZE
 *
 * Macros for accessing XY register pairs
 */
#define EVI_EDDY_MAGSIN 0
#define EVI_EDDY_MAGCOS 4
#define EVI_EDDY_XY_SIZE 8

/*
 * EVI_EDDY_CHANNEL_CONTROL
 * Bits 4: 0 - mux output for this channel
 * Bit     5 - Unused
 * Bits    6 - Probe Driver On/Off
 * Bit     7 - unused
 * Bit     8 - Enable differential channel
 * Bit     9 - Enable Absolute channel
 * Bit    10 - Enable two frequency mode (unused)
 * Bit    31 - Bypass filter for this channel,
 *             0 = use filter
 *             1 = bypass filter
 */
#define CHANNEL_CONTROL_MUX_MASK            (0x1F << 0 )
#define CHANNEL_CONTROL_PROBE_DRIVE_ON      (1    << 6 )
#define CHANNEL_CONTROL_ENABLE_DIFF         (1    << 8 )
#define CHANNEL_CONTROL_ENABLE_ABS          (1    << 9 )
#define CHANNEL_CONTROL_ENABLE_TWO_FREQ     (1    << 10)
#define CHANNEL_CONTROL_BYPASS_FILTER       (1    << 31)

#define EVI_EDDY_CHANNEL_CONTROL              0x0000

/*
 * EVI_EDDY_CHANNEL_ANGLE_INCREMENT
 *
 * This used to be TableStep.
 * The value here indicates the angle of the output frequency.
 * 2^32 = 2pi radians = 360*, yes this is 33 bits
 */
#define EVI_EDDY_CHANNEL_ANGLE_INCREMENT      0x0004

/*
 * EVI_EDDY_CHANNEL_PROBE_MAG_SIN
 * EVI_EDDY_CHANNEL_PROBEMAGCOS
 *
 * Probe Drive, 14-bit signed numbers, 0x1fff = 0.9999.., 0x2000 = -1
 *
 * In single frequency mode then the probe drive comes from
 * the differential path.  We shift the 14-bit signed number to the left
 * 4-bits since that's what the hardware looks at
 *
 * In two frequency mode, the probe signal is made up of the sum of the
 * probe drive signals from both the differential and absolute path
 * structures even though only one path can be specified at a time.
 *
 * MAX_DAC_OUT is the maximum value that this can have
 * Care needs to be taken that the combination of ProbeMag X and Y
 * does not have amplitude greater that MAX_DAC_OUT
 */
#define EVI_EDDY_CHANNEL_PROBE_MAG            0x0008
#define EVI_EDDY_CHANNEL_PROBE_MAG_SIN        0x0008
#define EVI_EDDY_CHANNEL_PROBEMAGCOS          0x000C

/*
 * EVI_EDDY_CHANNEL_PREDRIVES
 *
 * bits  7: 0 - PreDrive 3
 * bits 15: 8 - PreDrive 2
 * bits 23:16 - PreDrive 1
 * bits 31:24 - RECV Probe
 */
#define EVI_EDDY_CHANNEL_PREDRIVES            0x0010

#define EVI_EDDY_CHANNEL_DFTSIZE              0x0014

/*
 * EVI_EDDY_CHANNEL_MACMULTIPLIER
 *
 * number to multiply result by to scale to 2^24 DFT
 */
#define EVI_EDDY_CHANNEL_MACMULTIPLIER        0x0018

/*
 * EVI_EDDY_CHANNEL_PROBEDITHERAMPLITUDE
 *
 * amplitude of dither signal for probe
 */
#define EVI_EDDY_CHANNEL_PROBEDITHERAMPLITUDE 0x001C

/*
 * EVI_EDDY_CHANNEL_NUMDFTSUBSETS
 *
 * number of short DFTs to use in calculation of total DFT minus one
 */
#define EVI_EDDY_CHANNEL_NUMDFTSUBSETS        0x0020

/*
 * EVI_EDDY_CHANNEL_DFTSUBSETSIZE
 *
 * number of points in DFT subset minus one
 */
#define EVI_EDDY_CHANNEL_DFTSUBSETSIZE        0x0024

/*
 * EVI_EDDY_CHANNEL_DFTSKIP
 *
 * (This used to be 0x0050 for Pete's MUX128)
 */
#define EVI_EDDY_CHANNEL_DFTSKIP              0x0028

/*
 * EVI_EDDY_CHANNEL_INTERPOLATIONFACTOR
 *
 * This number depends on channel number.
 * The interpolated value is:
 *  Data[n-1] + (Data[n] - Data[n-1]) * InterpolationFactor / 2^30i
 */
#define EVI_EDDY_CHANNEL_INTERPOLATIONFACTOR  0x002C

/* 0x0030 - 0x0076 padding */

/*
 * EVI_EDDY_CHANNEL_IIR_COEF1
 * EVI_EDDY_CHANNEL_IIR_COEF2
 *
 * coefficients for the two-stage bi-quad IIR filter
 * 0x20 each
 */
#define EVI_EDDY_CHANNEL_IIR_COEF1            0x0080
#define EVI_EDDY_CHANNEL_IIR_COEF2            0x00A0

/*
 * EVI_EDDY_CHANNEL_GAIN
 *
 * Bit 3:0 1st stage gain
 *            0=-12db
 *            7 = 30db
 * Bit 7:4 2nd stage gain same units
 */
#define EVI_EDDY_CHANNEL_GAIN                 0x00C0

/*
 * EVI_EDDY_CHANNEL_NULL
 */
#define EVI_EDDY_CHANNEL_NULL                 0x00C4

/*
 * EVI_EDDY_CHANNEL_NULLMAGSIN
 * EVI_EDDY_CHANNEL_NULLMAGCOS
 *
 * Null Dac 1 (1st stage)
 * 18-bit signed numbers, 0x1ffff = 0.9999, 0x20000 = -1
 * Same rules as ProbeMag
 */
#define EVI_EDDY_CHANNEL_NULLMAGSIN           0x00C4
#define EVI_EDDY_CHANNEL_NULLMAGCOS           0x00C8

/*
 * EVI_EDDY_CHANNEL_NULLMAGSIN2
 * EVI_EDDY_CHANNEL_NULLMAGCOS2
 *
 * Null Dac 2 (second stage)
 */
#define EVI_EDDY_CHANNEL_NULLMAGSIN2          0x00CC
#define EVI_EDDY_CHANNEL_NULLMAGCOS2          0x00D0

/*
 * EVI_EDDY_CHANNEL_ROTMAGSIN
 * EVI_EDDY_CHANNEL_ROTMAGCOS
 *
 * Rotation, 32-bit number
 * 0x7fff:ffff = 1.9999..., 0x8000:0000 = -2
 * this includes the digital gain to account for values between
 * the analog gain values
 */
#define EVI_EDDY_CHANNEL_ROT                  0x00D4
#define EVI_EDDY_CHANNEL_ROTMAGSIN            0x00D4
#define EVI_EDDY_CHANNEL_ROTMAGCOS            0x00D8

/*
 * EVI_EDDY_CHANNEL_XSPREAD
 * EVI_EDDY_CHANNEL_YSPREAD
 *
 * spread numbers are 1-128 as 32-bit signed number
 */
#define EVI_EDDY_CHANNEL_XSPREAD              0x00DC
#define EVI_EDDY_CHANNEL_YSPREAD              0x00E0

#define EVI_EDDY_CHANNEL_XNULLOFFSET          0x00E4
#define EVI_EDDY_CHANNEL_YNULLOFFSET          0x00E8

/*
 * EVI_EDDY_CHANNEL_DITHERAMPLITUDE
 *
 * amplitude of dither signal , same units as NullStage
 */
#define EVI_EDDY_CHANNEL_DITHERAMPLITUDE 0x00EC

/*
 * EVI_EDDY_CHANNEL_DITHERAMPLITUDE1
 *
 * amplitude of dither signal for Null Dac 1
 */
#define EVI_EDDY_CHANNEL_DITHERAMPLITUDE1     0x00F0

/* padding 0x00F4 to the end are unused to make length 0x100*/
#define EVI_EDDY_CHANNEL_SIZE                 0x0100
#define EVI_EDDY_CHAN(ts, chan) (EVI_EDDY_CHANNEL_SIZE * ((ts * 2) + chan))


/***************************************************************************
 * EVI_EDDY_IIR_COEF register offsets
 *
 * Offsets from
 ***************************************************************************/
#define EVI_EDDY_IIR_SCALE_B        0x0000 /* 4 bits (0-15) */
#define EVI_EDDY_IIR_SCALE_L        0x0004 /* 4 bits (0-15) */
#define EVI_EDDY_IIR_B              0x0008 /* iir b 3 registers @ 32 bits */
#define EVI_EDDY_IIR_LR             0x0014 /* iir Lr 32 bits */
#define EVI_EDDY_IIR_LI             0x0014 /* iir Li 32 bits */
#define EVI_EDDY_IIR_MLI            0x001C /* iir MLi 32 bits ... unused */
#define EVI_EDDY_IIR_SIZE           0x0020 /* iir MLi 32 bits ... unused */

/***************************************************************************
 * EVI_EDDY_MIXED_DEF
 *
 * EVI_MIXED register offsets
 *
 ***************************************************************************/

/*
 * EVI_EDDY_MIXED_CONTROL
 *
 * MixedTimeslot Control format
 * Bits  6: 0 - Primary Timeslot index
 * Bit      7 - Primary Channel index
 * Bits 14: 8 - Secondary Timeslot index
 * Bit     15 - Secondary Channel Index
 * Bits 17:16 - Mode
 *               0 = Send Primary - Secondary (saturate at 24 bits)
 *               1 = Send Primary + Secondary (saturate at 24 bits)
 *               2 = reserved
 *               3 = reserved
 * Bits 31:18 - unused
 */

#define MIXED_TIMESLOT_MASK              (0x7f)
#define MIXED_CHANNEL_MASK               (0x1)

#define MIXED_PRIMARY_TIMESLOT_SHIFT     (0)
#define MIXED_PRIMARY_CHANNEL_SHIFT      (7)
#define MIXED_SECONDARY_TIMESLOT_SHIFT   (8)
#define MIXED_SECONDARY_CHANNEL_SHIFT    (15)
#define MIXED_OPERATION_MODE_SHIFT       (16)
#define MIXED_ENABLE_SHIFT               (18)

#define EVI_EDDY_MIXED_CONTROL 0x0000

#define EVI_EDDY_MIXED_ROTMAGSIN 0x0004
#define EVI_EDDY_MIXED_ROTMAGCOS 0x0008
#define EVI_EDDY_MIXED_XSPREAD   0x000C
#define EVI_EDDY_MIXED_YSPREAD   0x0010
/* padding 0x0014 - 0x001C */
#define EVI_EDDY_MIXED_SIZE 0x0020

#endif /* EVI_EDDY_CHANNEL_H */
