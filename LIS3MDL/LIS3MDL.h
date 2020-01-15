#ifndef LIS3MDL_H
#define LIS3MDL_H

/* Includes ---------------------------------------------------------------- */
#include "stm32l4xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* Definitions ------------------------------------------------------------- */

/* Allows for a MAXIMUM of 2 LIS3MDL IC's to be used on the same I2C line */
/* DEFAULT is HIGH ADDR */
#define LIS3MDL_SA0_HIGH 0b0011110
#define LIS3MDL_SA0_LOW  0b0011100

/* The LSB is the read/write bit */
#define LIS3MDL_WRITE_BIT 0
#define LIS3MDL_READ_BIT  1

/* External variables ------------------------------------------------------ */

extern I2C_HandleTypeDef hi2c1;

/* Variable definitions ---------------------------------------------------- */

enum LIS3MDLResultStatus {
	LIS3MDL_SUCCESS,
	LIS3MDL_ERROR
};

// Represents a LIS3MDL device
typedef struct {
	enum LIS3MDLResultStatus 	status;

	uint8_t 					address;

	int16_t						magX; // Last set of magnetometer readings
	int16_t						magY;
	int16_t						magZ;

	int16_t						temp;

	I2C_HandleTypeDef*			hi2c;  // Handle to I2C for STM

	int8_t 						counter; // Used in the DCM algorithm
} LIS3MDL;

typedef struct {
	uint8_t 					reg; // Register to write to
	uint8_t						rwBit; // read/write bit,
	uint8_t*					pData; // Pointer to data to write
	uint8_t						numData; // Number of bytes of data
} LIS3MDLMsg;

enum LIS3MDLRegAddr {
	LIS3MDL_WHO_AM_I = 		0x0F,

	LIS3MDL_CTRL_REG1 =		0x20,
	LIS3MDL_CTRL_REG2 = 	0x21,
	LIS3MDL_CTRL_REG3 =		0x22,
	LIS3MDL_CTRL_REG4 =		0x23,
	LIS3MDL_CTRL_REG5 =		0x24,

	LIS3MDL_STATUS_REG =	0x27,

	LIS3MDL_OUT_X_L =		0x28,
	LIS3MDL_OUT_X_H =		0x29,
	LIS3MDL_OUT_Y_L = 		0x2A,
	LIS3MDL_OUT_Y_H = 		0x2B,
	LIS3MDL_OUT_Z_L =		0x2C,
	LIS3MDL_OUT_Z_H = 		0x2D,

	LIS3MDL_TEMP_OUT_L =	0x2E,
	LIS3MDL_TEMP_OUT_H =	0x2F,

	LIS3MDL_INT_CFG = 		0x30,
	LIS3MDL_INT_SRC =		0x31,
	LIS3MDL_INT_THS_L =		0x32,
	LIS3MDL_INT_THS_H =		0x33
};

/* Control register 1 ------------------------------------------------------ */

enum LIS3MDL_CTRL_REG1_TEMP_EN {
	LIS3MDL_TEMP_SENSE_DISABLE =			0x00,
	LIS3MDL_TEMP_SENSE_ENABLE =				0x01
};

enum LIS3MDL_CTRL_REG1_OM {
	LIS3MDL_XY_AXES_LOW_POWER_MODE =		0x00,
	LIS3MDL_XY_AXES_MEDIUM_PERF_MODE =		0x01,
	LIS3MDL_XY_AXES_HIGH_PERF_MODE =		0x02,
	LIS3MDL_XY_AXES_ULTRA_PERF_MODE =		0x03
};

enum LIS3MDL_CTRL_REG1_DO {
	LIS3MDL_OUT_DATA_RATE_0_625_HZ =		0x00,
	LIS3MDL_OUT_DATA_RATE_1_25_HZ =			0x01,
	LIS3MDL_OUT_DATA_RATE_2_5_HZ =			0x02,
	LIS3MDL_OUT_DATA_RATE_5_HZ =			0x03,
	LIS3MDL_OUT_DATA_RATE_10_HZ =			0x04,
	LIS3MDL_OUT_DATA_RATE_20_HZ =			0x05,
	LIS3MDL_OUT_DATA_RATE_40_HZ =			0x06,
	LIS3MDL_OUT_DATA_RATE_80_HZ =			0x07
};

enum LIS3MDL_CTRL_REG1_FAST_ODR {
	// See data sheet for ODR and OM configurations under FAST_ODR
	LIS3MDL_FAST_ODR_DISABLE =				0x00,
	LIS3MDL_FAST_ODR_ENABLE =				0x01
};

enum LIS3MDL_CTRL_REG1_ST {
	LIS3MDL_SELF_TEST_DISABLE =				0x00,
	LIS3MDL_SELF_TEST_ENABLE =				0x01
};

typedef struct {
	enum LIS3MDL_CTRL_REG1_TEMP_EN 			temperatureEnable;
	enum LIS3MDL_CTRL_REG1_OM				operativeMode;
	enum LIS3MDL_CTRL_REG1_DO				outputDataRate;
	enum LIS3MDL_CTRL_REG1_FAST_ODR			fastDataOutEnable; // enables >80Hz
	enum LIS3MDL_CTRL_REG1_ST				selfTestEnable;
} LIS3MDLCTRLReg1Config;

/* Control register 2 ------------------------------------------------------ */

enum LIS3MDL_CTRL_REG2_FS {
	// Gauss is a measure of magnetic field density
	// (the 'B' field in maxwell's equations)
	LIS3MDL_FS_4_GAUSS =					0x00,
	LIS3MDL_FS_8_GAUSS =					0x01,
	LIS3MDL_FS_12_GAUSS =					0x02,
	LIS3MDL_FS_16_GAUSS =					0x03
};

enum LIS3MDL_CTRL_REG2_REBOOT {
	LIS3MDL_NORMAL_MODE =					0x00,
	LIS3MDL_REBOOT_MEMORY_CONTENT =			0x01
};

enum LIS3MDL_CTRL_REG2_SOFT_RST {
	LIS3MDL_SOFT_RESET_DISABLED =			0x00,
	LIS3MDL_SOFT_RESET_ENABLED =			0x01
};

typedef struct {
	enum LIS3MDL_CTRL_REG2_FS 				fullScaleConfig;
	enum LIS3MDL_CTRL_REG2_REBOOT 			rebootMode;
	enum LIS3MDL_CTRL_REG2_SOFT_RST			softResetMode;
} LIS3MDLCTRLReg2Config;

/* Control register 3 ------------------------------------------------------ */

enum LIS3MDL_CTRL_REG3_LP {
	// If set, to 1, CTRL_REG1_DO = 0.625Hz.
	LIS3MDL_LOW_POWER_MODE_DISABLE = 		0x00,
	LIS3MDL_LOW_POWER_MODE_ENABLE =			0x01
};

enum LIS3MDL_CTRL_REG3_SIM {
	LIS3MDL_SPI_4WIRE_IM =					0x00,
	LIS3MDL_SPI_3WIRE_IM =					0x01
};

enum LIS3MDL_CTRL_REG3_MD {
	LIS3MDL_SYS_MODE_CONTINUOUS_CONV_MODE =	0x00,
	LIS3MDL_SYS_MODE_SINGLE_CONV_MODE =		0x01,
	LIS3MDL_SYS_MODE_POWER_DOWN =			0x02,
	LIS3MDL_SYS_MODE_ALSO_POWER_DOWN =		0x03
};

typedef struct {
	enum LIS3MDL_CTRL_REG3_LP 				lowPowerModeConfig;
	enum LIS3MDL_CTRL_REG3_SIM 				SPIInterfaceMode;
	enum LIS3MDL_CTRL_REG3_MD 				systemOperatingMode;
} LIS3MDLCTRLReg3Config;

/* Control register 4 ------------------------------------------------------ */

enum LIS3MDL_CTRL_REG4_OMZ {
	LIS3MDL_Z_AXIS_LOW_PERF_MODE =			0x00,
	LIS3MDL_Z_AXIS_MEDIUM_PERF_MODE =		0x01,
	LIS3MDL_Z_AXIS_HIGH_PERF_MODE =			0x02,
	LIS3MDL_Z_AXIS_ULTRA_PERF_MODE =		0x03
};

enum LIS3MDL_CTRL_REG4_BLE {
	LIS3MDL_DATA_LSB_LOWER_ADDR =			0x00,
	LIS3MDL_DATA_MSB_LOWER_ADDR =			0x01
};

typedef struct {
	enum LIS3MDL_CTRL_REG4_OMZ 				zAxisOperatingMode;
	enum LIS3MDL_CTRL_REG4_BLE 				addressEndianness;
} LIS3MDLCTRLReg4Config;

/* Control register 5 ------------------------------------------------------ */

enum LIS3MDL_CTRL_REG5_FAST_READ {
	LIS3MDL_FAST_READ_DISABLE =				0x00,
	LIS3MDL_FAST_READ_ENABLE =				0x01
};

enum LIS3MDL_CTRL_REG5_BDU {
	LIS3MDL_BLOCK_DATA_CONT_UPDATE =		0x00,
	LIS3MDL_BLOCK_DATA_UPDATE_ON_READ =		0x01
};

typedef struct {
	enum LIS3MDL_CTRL_REG5_FAST_READ 		fastReadEnable;
	enum LIS3MDL_CTRL_REG5_BDU 				blockUpdateMode;
} LIS3MDLCTRLReg5Config;

/* Status Register --------------------------------------------------------- */

enum LIS3MDL_STATUS_REG_ZYXOR {
	LIS3MDL_XYZ_AXIS_NO_DATA_OVERRUN =		0x00,
	LIS3MDL_XYZ_AXIS_DATA_OVERRUN =			0x01
};

enum LIS3MDL_STATUS_REG_ZOR {
	LIS3MDL_Z_AXIS_NO_DATA_OVERRUN =		0x00,
	LIS3MDL_Z_AXIS_DATA_OVERRUN =			0x01
};

enum LIS3MDL_STATUS_REG_YOR {
	LIS3MDL_Y_AXIS_NO_DATA_OVERRUN =		0x00,
	LIS3MDL_Y_AXIS_DATA_OVERRUN =			0x01,
};

enum LIS3MDL_STATUS_REG_XOR {
	LIS3MDL_X_AXIS_NO_DATA_OVERRUN =		0x00,
	LIS3MDL_X_AXIS_DATA_OVERRUN =			0x01
};

enum LIS3MDL_STATUS_REG_ZYXDA {
	LIS3MDL_XYZ_AXIS_NO_DATA_AVAILABLE =	0x00,
	LIS3MDL_XYZ_AXIS_DATA_AVAILABLE =		0x01
};

enum LIS3MDL_STATUS_REG_ZDA {
	LIS3MDL_Z_AXIS_NO_DATA_AVAILABLE = 		0x00,
	LIS3MDL_Z_AXIS_DATA_AVAILABLE =			0x01
};

enum LIS3MDL_STATUS_REG_YDA {
	LIS3MDL_Y_AXIS_NO_DATA_AVAILABLE = 		0x00,
	LIS3MDL_Y_AXIS_DATA_AVAILABLE =			0x01
};

enum LIS3MDL_STATUS_REG_XDA {
	LIS3MDL_X_AXIS_NO_DATA_AVAILABLE = 		0x00,
	LIS3MDL_X_AXIS_DATA_AVAILABLE =			0x01
};

typedef struct {
	enum LIS3MDL_STATUS_REG_ZYXOR 			xyzAxesDataOverrun;
	enum LIS3MDL_STATUS_REG_ZOR				zAxisDataOverrun;
	enum LIS3MDL_STATUS_REG_YOR				yAxisDataOverrun;
	enum LIS3MDL_STATUS_REG_XOR				xAxisDataOverrun;
	enum LIS3MDL_STATUS_REG_ZYXDA 			xyzAxesDataAvailable;
	enum LIS3MDL_STATUS_REG_ZDA				zAxisDataAvailable;
	enum LIS3MDL_STATUS_REG_YDA				yAxisDataAvailable;
	enum LIS3MDL_STATUS_REG_XDA				xAxisDataAvailable;
} LIS3MDLSRegConfig;

/* Magnetometer data output registers -------------------------------------- */

typedef struct {
	int8_t 									OUT_X_L;
	int8_t 									OUT_X_H;

	int8_t									OUT_Y_L;
	int8_t									OUT_Y_H;

	int8_t									OUT_Z_L;
	int8_t									OUT_Z_H;
} magData;

/* Temperature data output registers --------------------------------------- */

typedef struct {
	int8_t 									LIS3MDL_TEMP_OUT_L;
	int8_t									LIS3MDL_TEMP_OUT_H;
} LIS3MDLtempData;

/* Interrupt configuration ------------------------------------------------- */

enum LIS3MDL_INT_CFG_XIEN {
	LIS3MDL_X_AXIS_INT_DISABLE_REQ =		0x00,
	LIS3MDL_X_AXIS_INT_ENABLE_REQ =			0x01
};

enum LIS3MDL_INT_CFG_YIEN {
	LIS3MDL_Y_AXIS_INT_DISABLE_REQ =		0x00,
	LIS3MDL_Y_AXIS_INT_ENABLE_REQ =			0x01
};

enum LIS3MDL_INT_CFG_ZIEN {
	LIS3MDL_Z_AXIS_INT_DISABLE_REQ =		0x00,
	LIS3MDL_Z_AXIS_INT_ENABLE_REQ =			0x01
};

enum LIS3MDL_INT_CFG_IEA {
	LIS3MDL_SET_INT_ACTIVE_LOW =			0x00,
	LIS3MDL_SET_INT_ACTIVE_HIGH =			0x01
};

enum LIS3MDL_INT_CFG_LIR {
	LIS3MDL_LATCH_INT_REQ_LATCHED =			0x00,
	LIS3MDL_LATCH_INT_REQ_NOT_LATCHED =		0x01
};

enum LIS3MDL_INT_CFG_IEN {
	LIS3MDL_INT_DISABLED =					0x00,
	LIS3MDL_INT_ENABLED =					0x01
};

/* Interrupt source configuration ------------------------------------------ */

enum LIS3MDL_INT_SRC_PTH_X { // X-axis value exceeded
	LIS3MDL_X_AXIS_POS_THS_NOT_EXCEEDED =	0x00,
	LIS3MDL_X_AXIS_POS_THS_EXCEEDED =		0x01
};

enum LIS3MDL_INT_SRC_PTH_Y {
	LIS3MDL_Y_AXIS_POS_THS_NOT_EXCEEDED = 	0x00,
	LIS3MDL_Y_AXIS_POS_THS_EXCEEDED =		0x01
};

enum LIS3MDL_INT_SRC_PTH_Z {
	LIS3MDL_Z_AXIS_POS_THS_NOT_EXCEEDED =	0x00,
	LIS3MDL_Z_AXIS_POS_THS_EXCEEDED =		0x01
};

enum LIS3MDL_INT_SRC_NTH_X {
	LIS3MDL_X_AXIS_NEG_THS_NOT_EXCEEDED =	0x00,
	LIS3MDL_X_AXIS_NEG_THS_EXCEEDED =		0x01
};

enum LIS3MDL_INT_SRC_NTH_Y {
	LIS3MDL_Y_AXIS_NEG_THS_NOT_EXCEEDED =	0x00,
	LIS3MDL_Y_AXIS_NEG_THS_EXCEEDED =		0x01
};

enum LIS3MDL_INT_SRC_NTH_Z {
	LIS3MDL_Z_AXIS_NEG_THS_NOT_EXCEEDED =	0x00,
	LIS3MDL_Z_AXIS_NEG_THS_EXCEEDED =		0x01
};

enum LIS3MDL_INT_SRC_MROI { // Meaning the reading is 'on/off-the-scale'
	LIS3MDL_MEAS_RANGE_ADEQUATE =			0x00,
	LIS3MDL_MEAS_RANGE_OVERFLOW =			0x01
};

enum LIS3MDL_INT_SRC_INT {	// Configure with the IEA bit in INT_CFG
	LIS3MDL_INT_BIT_SIGNAL_TRIGGERED =		0x00,
	LIS3MDL_INT_BIT_SIGNAL_NOT_TRIGGERED =	0x01
};

/* Interrupt threshold configuration --------------------------------------- */

typedef struct {
	uint8_t 								INT_THS_L;
	uint8_t									INT_THS_H;
} intThreshConfig;

/* External function definitions ------------------------------------------- */

void LIS3MDL_init_reg(LIS3MDL* device);
int8_t LIS3MDL_read_reg(LIS3MDL* device, uint8_t reg);
void LIS3MDL_get_status_reg(LIS3MDL* device, LIS3MDLSRegConfig* sreg);
enum LIS3MDLResultStatus LIS3MDL_read_mag(LIS3MDL* device);
#endif
