#ifndef LSM6DS33_H
#define LSM6DS33_H

/* Includes ---------------------------------------------------------------- */
#include "stm32l4xx_hal.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/* Definitions ------------------------------------------------------------- */

#define GYRO_GAIN 1
#define ACCL_GAIN 1

/* Allows for a MAXIMUM of 2 LSM6DS33 IC's to be used on the same I2C line */
/* DEFAULT IS HIGH ADDR */
#define LSM6DS33_SA0_HIGH_ADDR 0b1101011 // device addresses depend on state of
#define LSM6DS33_SA0_LOW_ADDR  0b1101010 //  SA0 pin.

#define LSM6DS33_WRITE_BIT 	   0
#define LSM6DS33_READ_BIT 	   1

// DEBUG VARIABLE
#define LSM6DS33_DEBUG

/* External variables ------------------------------------------------------ */

extern I2C_HandleTypeDef hi2c1;

/* Variable definitions ---------------------------------------------------- */

enum LSM6DS33ResultStatus {
	LSM6DS33_SUCCESS,
	LSM6DS33_ERROR
};

// What does this do?
enum LSM6DS33DeviceType {
	LSM6DS33_DEVICE_TYPE_AUTO,
	LSM6DS33_DEVICE_TYPE_DS33
};


// Represents a LSM6DS33 device
typedef struct {
	enum LSM6DS33DeviceType 	device;
	uint8_t 					address;

	enum LSM6DS33ResultStatus 	status; // Indicates whether the last read succeeded or not

	int16_t 					acclX; // Last set of accelerometer readings
	int16_t 					acclY;
	int16_t						acclZ;

	int16_t 					gyroX; // Last set of gyroscope readings
	int16_t						gyroY;
	int16_t						gyroZ;

	int16_t 					temp; // Last temperature reading

	I2C_HandleTypeDef*			hi2c;  // Handle to I2C for STM

} LSM6DS33;



typedef struct {
	uint8_t 					reg; // Register to write to
	uint8_t						rwBit; // read/write bit,
	uint8_t*					pData; // Pointer to data to write
	uint8_t						numData; // Number of bytes of data
} LSM6DS33Msg;


enum LSM6DS33SA0State {
	LSM6DS33_SA0_AUTO,				  // Used when the MCU controls the SA0 pin
	LSM6DS33_SA0_LOW,
	LSM6DS33_SA0_HIGH
};

enum LSM6DS33RegAddr {
	LSM6DS33_FUNC_CFG_ACCESS = 0x01, // Embedded functions configuration register

	LSM6DS33_FIFO_CTRL1 = 	   0x06, // FIFO Configuration registers
	LSM6DS33_FIFO_CTRL2 = 	   0x07,
	LSM6DS33_FIFO_CTRL3 = 	   0x08,
	LSM6DS33_FIFO_CTRL4 = 	   0x09,
	LSM6DS33_FIFO_CTRL5 = 	   0x0A,
	LSM6DS33_ORIENT_CFG_G =    0x0B,

	LSM6DS33_INT1_CTRL =       0x0D, // INT1 pin control
	LSM6DS33_INT2_CTRL = 	   0x0E, // INT2 pin control
	LSM6DS33_WHO_AM_I =        0x0F, // Who I am ID

	LSM6DS33_CTRL1_XL =        0x10, // Accl and gyro control registers
	LSM6DS33_CTRL2_G = 	       0x11,
	LSM6DS33_CTRL3_C =         0x12,
	LSM6DS33_CTRL4_C =         0x13,
	LSM6DS33_CTRL5_C =         0x14,
	LSM6DS33_CTRL6_C =         0x15,
	LSM6DS33_CTRL7_G =         0x16,
	LSM6DS33_CTRL8_XL =        0x17,
	LSM6DS33_CTRL9_XL =        0x18,
	LSM6DS33_CTRL10_C =        0x19,

	LSM6DS33_WAKE_UP_SRC =     0x1B, // Interrupt registers
	LSM6DS33_TAP_SRC =         0x1C,
	LSM6DS33_D6D_SRC =         0x1D,

	LSM6DS33_STATUS_REG =      0x1E, // Status data register

	LSM6DS33_OUT_TEMP_L =      0x20, // Temperature output data register
	LSM6DS33_OUT_TEMP_H =      0x21,

	LSM6DS33_OUTX_L_G =        0x22, // Gyro output register
	LSM6DS33_OUTX_H_G =        0x23,
	LSM6DS33_OUTY_L_G =        0x24,
	LSM6DS33_OUTY_H_G =        0x25,
	LSM6DS33_OUTZ_L_G =        0x26,
	LSM6DS33_OUTZ_H_G =        0x27,

	LSM6DS33_OUTX_L_XL =       0x28, // Accl output register
	LSM6DS33_OUTX_H_XL =       0x29,
	LSM6DS33_OUTY_L_XL =       0x2A,
	LSM6DS33_OUTY_H_XL =       0x2B,
	LSM6DS33_OUTZ_L_XL =       0x2C,
	LSM6DS33_OUTZ_H_XL =       0x2D,

	LSM6DS33_FIFO_STATUS1 =    0x3A, // FIFO status registers
	LSM6DS33_FIFO_STATUS2 =    0x3B,
	LSM6DS33_FIFO_STATUS3 =    0x3C,
	LSM6DS33_FIFO_STATUS4 =    0x3D,

	LSM6DS33_FIFO_DATA_OUT_L = 0x3E, // FIFO data output registers
	LSM6DS33_FIFO_DATA_OUT_H = 0x3F,

	LSM6DS33_TIMESTAMP0_REG =  0x40, // Time-stamp output registers
	LSM6DS33_TIMESTAMP1_REG =  0x41,
	LSM6DS33_TIMESTAMP2_REG =  0x42,

	LSM6DS33_STEP_TIMESTAMP_L= 0x49, // Step counter timestamp registers
	LSM6DS33_STEP_TIMESTAMP_H= 0x4A,

	LSM6DS33_STEP_COUNTER_L =  0x4B, // Step counter output registers
	LSM6DS33_STEP_COUNTER_H =  0x4C,

	LSM6DS33_FUNC_SRC =        0x53, // Interrupt register

	LSM6DS33_TAP_CFG =         0x58, // Interrupt registers
	LSM6DS33_TAP_THS_6D =      0x59,
	LSM6DS33_INT_DUR2 =        0x5A,
	LSM6DS33_WAKE_UP_THS =     0x5B,
	LSM6DS33_WAKE_UP_DUR =     0x5C,
	LSM6DS33_FREE_FALL =       0x5D,
	LSM6DS33_MD1_CFG =         0x5E,
	LSM6DS33_MD2_CFG =         0x5F
};

/* FIFO Control register 1 (r/w) ------------------------------------------- */

typedef struct {
	uint8_t						FIFOThreshold;
} FIFOCTRL1Config;

/* FIFO Control register 2 (r/w) ------------------------------------------- */

enum FIFO_CTRL2_TIMER_PEDO_FIFO_EN {
	FIFO_CTRL2_DISABLE_STEP_AND_TIME = 		0x00,
	FIFO_CTRL2_ENABLE_STEP_AND_TIME = 		0x01
};

enum FIFO_CTRL2_PEDO_FIFO_DRDY {
	FIFO_CTRL2_ENABLE_WRITE_G_XL_DRDY = 	0x00,
	FIFO_CTRL2_ENABLE_WRITE_STEP_DET = 		0x01
};

typedef struct {
	enum FIFO_CTRL2_TIMER_PEDO_FIFO_EN			pedoStepAndTimeEnable;
	enum FIFO_CTRL2_PEDO_FIFO_DRDY				pedoFIFOWriteConfig;
	uint8_t										FIFOThresholdLevel; // 4-bit
} FIFOCTRL2Config;

/* FIFO Control register 3 (r/w) ------------------------------------------- */

enum FIFO_CTRL3_DEC_FIFO_GYRO {
	GYRO_NOT_IN_FIFO =						0x00,
	GYRO_NO_DEC =							0x01,
	GYRO_DEC_2 =							0x02,
	GYRO_DEC_3 = 							0x03,
	GYRO_DEC_4 =							0x04,
	GYRO_DEC_8 =							0x05,
	GYRO_DEC_16 = 							0x06,
	GYRO_DEC_32 = 							0x07
};

enum FIFO_CTRL3_DEC_FIFO_XL {
	ACCL_NOT_IN_FIFO =						0x00,
	ACCL_NO_DEC =							0x01,
	ACCL_DEC_2 =							0x02,
	ACCL_DEC_3 =							0x03,
	ACCL_DEC_4 =							0x04,
	ACCL_DEC_8 =							0x05,
	ACCL_DEC_16 = 							0x06,
	ACCL_DEC_32 =							0x07
};

typedef struct {
	enum FIFO_CTRL3_DEC_FIFO_GYRO				gyroDecimateConfig;
	enum FIFO_CTRL3_DEC_FIFO_XL					acclDecimateConfig;
} FIFOCTRL3Config;

/* FIFO Control register 4 (r/w) ------------------------------------------- */

enum FIFO_CTRL4_ONLY_HIGH_DATA {
	DISABLE_MSB_MEMORIZE_XL_G =				0x00,
	ENABLE_MSB_MEMORIZE_XL_G =				0x01
};

enum FIFO_CTRL4_TIMER_PEDO_DEC_FIFO {
	PEDO_NOT_IN_FIFO = 						0x00,
	PEDO_NO_DEC =							0x01,
	PEDO_DEC_2 =							0x02,
	PEDO_DEC_3 =							0x03,
	PEDO_DEC_4 =							0x04,
	PEDO_DEC_8 =							0x05,
	PEDO_DEC_16 =							0x06,
	PEDO_DEC_32 =							0x07
};

typedef struct {
	enum FIFO_CTRL4_ONLY_HIGH_DATA			FIFODataStorageConfig;
	enum FIFO_CTRL4_TIMER_PEDO_DEC_FIFO    	pedoDecimateConfig;
} FIFOCTRL4Config;

/* FIFO Control register 5 (r/w) ------------------------------------------- */

enum FIFO_CTRL5_ODR_FIFO {
	FIFO_ODR_DISABLED =						0x00,
	FIFO_ODR_13_HZ =						0x01,
	FIFO_ODR_26_HZ =						0x02,
	FIFO_ODR_52_HZ =						0x03,
	FIFO_ODR_104_HZ =						0x04,
	FIFO_ODR_208_HZ =						0x05,
	FIFO_ODR_416_HZ =						0x06,
	FIFO_ODR_833_HZ = 						0x07,
	FIFO_ODR_1660_HZ =						0x08,
	FIFO_ODR_3330_HZ =						0x09,
	FIFO_ODR_6660_HZ =						0x0A
};

enum FIFO_CTRL5_FIFO_MODE {
	FIFO_MODE_DISABLED =					0x00,
	FIFO_MODE_STOP_ON_FULL =				0x01,
	FIFO_MODE_CONT_UNTIL_TRIG_DEASSERT =	0x03,
	FIFO_MODE_BYPASS_UNTIL_TRIG_DEASSERT =	0x04,
	FIFO_MODE_CONT_OVERWRITE =				0x06
};

typedef struct {
	enum FIFO_CTRL5_ODR_FIFO				FIFOODRConfig;
	enum FIFO_CTRL5_FIFO_MODE				FIFOModeConfig;
} FIFOCTRL5Config;

/* Accelerometer configuration register (r/w) ------------------------------ */
enum CTRL1_XL_ODR_Mode {
	LSM6DS33_ACCL_OFF = 		0x00, // OFF
	LSM6DS33_ACCL_13HZ = 		0x01, // Low Power Mode
	LSM6DS33_ACCL_26HZ =		0x02,
	LSM6DS33_ACCL_52HZ = 		0x03,
	LSM6DS33_ACCL_104HZ =		0x04, // Normal Mode
	LSM6DS33_ACCL_208HZ =		0x05,
	LSM6DS33_ACCL_416HZ =		0x06, // High performance mode
	LSM6DS33_ACCL_833HZ =		0x07,
	LSM6DS33_ACCL_1660HZ =		0x08,
	LSM6DS33_ACCL_3330HZ =		0x09,
	LSM6DS33_ACCL_6660HZ = 		0x0A
};

enum CTRL1_XL_FS_XL_Mode {
	LSM6DS33_ACCL_2G =			0x00,
	LSM6DS33_ACCL_16G = 		0x01,
	LSM6DS33_ACCL_4G =			0x02,
	LSM6DS33_ACCL_8G =			0x03
};

enum CTRL1_XL_BW_XL_Mode {
	LSM6DS33_ACCL_BW_400HZ =	0x00,
	LSM6DS33_ACCL_BW_200HZ =	0x01,
	LSM6DS33_ACCL_BW_100HZ =	0x02,
	LSM6DS33_ACCL_BW_50HZ =		0x03
};

typedef struct {
	enum CTRL1_XL_ODR_Mode		ODRXLMode; // Output data rate + power mode
	enum CTRL1_XL_FS_XL_Mode	FSXLMode; // Accelerometer full scale selection
	enum CTRL1_XL_BW_XL_Mode 	BWXLMode; // Anti-aliasing filter bandwidth
} CTRL1Config;

/* Gyroscope configuration register (r/w) --------------------------------- */

enum CTRL2_G_ODR_Mode {
	LSM6DS33_GYRO_OFF =			0x00, // Low power
	LSM6DS33_GYRO_13HZ =		0x01,
	LSM6DS33_GYRO_26HZ =		0x02,
	LSM6DS33_GYRO_52HZ =		0x03,
	LSM6DS33_GYRO_104HZ =		0x04, // Normal mode
	LSM6DS33_GYRO_208HZ =		0x05,
	LSM6DS33_GYRO_416HZ =		0x06, // High performance
	LSM6DS33_GYRO_833HZ =		0x07,
	LSM6DS33_GYRO_1660HZ =		0x08
};

enum CTRL2_G_FS_G_Mode {
	LSM6DS33_GYRO_245DPS =		0x00,
	LSM6DS33_GYRO_500DPS =		0x01,
	LSM6DS33_GYRO_1000DPS =		0x02,
	LSM6DS33_GYRO_2000DPS =		0x03
};

enum CTRL2_G_FS_125_Mode {
	LSM6DS33_GYRO_125_DISABLE =	0x00,
	LSM6DS33_GYRO_125_ENABLE =	0x01
};

typedef struct {
	enum CTRL2_G_ODR_Mode		ODRGMode; // Output data rate
	enum CTRL2_G_FS_G_Mode		FSGMode; // Gyro full-scale selection
	enum CTRL2_G_FS_125_Mode	FSG125Mode; // 125dps full-scale EN
} CTRL2Config;

/* Control register 3 (r/w) ------------------------------------------------ */

enum CTRL3_C_BOOT {
	NORMAL_MODE = 				0x00,
	REBOOT_MEM 	= 				0x01
};

enum CTRL3_C_BDU {
	CONTINUOUS_UPDATE = 		0x00,
	WAIT_FOR_READ =				0x01
};

enum CTRL3_C_H_LACTIVE {
	INT_OUT_PADS_HIGH =			0x00,
	INT_OUT_PADS_LOW =			0x01
};

enum CTRL3_C_PP_OD {
	PUSH_PULL_MODE =			0x00,
	OPEN_DRAIN_MODE = 			0x01
};

enum CTRL3_C_SIM {
	FOUR_WIRE_IM =				0x00,
	THREE_WIRE_IM =				0x01
};

enum CTRL3_C_IF_INC {
	REG_ADDR_AUTO_INC_DISABLED= 0x00,
	REG_ADDR_AUTO_INC_ENABLED=	0x01
};

enum CTRL3_C_BLE {
	DATA_LSB =					0x00,
	DATA_MSB =					0x01
};

enum CTRL3_C_SW_RESET {
	SW_NORMAL_MODE = 			0x00,
	SW_RESET_DEVICE =			0x01
};

typedef struct {
	enum CTRL3_C_BOOT			bootMode; // Reboot memory content
	enum CTRL3_C_BDU			bduMode; // block data update
	enum CTRL3_C_H_LACTIVE		intActiveMode; // Interrupt activation level
	enum CTRL3_C_PP_OD			intPadMode;	// push-pull/open-drain select on
											// int1 and int2 pads
	enum CTRL3_C_SIM			simMode; // SPI Interface mode
	enum CTRL3_C_IF_INC			regIncMode; // register auto increment during
											// multi-byte access with I2C/SPI
	enum CTRL3_C_BLE			endianMode; // Big-Endian/little-endian
	enum CTRL3_C_SW_RESET		swResetMode; // Software reset
} CTRL3Config;

/* Control register 4 (r/w) ------------------------------------------------ */

enum CTRL4_C_XL_BW_SCAL_ODR {
	ACCL_ANALOG_FILTER_AUTO = 		0x00, // CTRL1_XL Analog filter Auto
	ACCL_ANALOG_FILTER_MANUAL =		0x01  // CTRL1_XL_BW_XL select
};

enum CTRL4_C_SLEEP_G {
	GYRO_SLEEP_DISABLED = 			0x00,
	GYRO_SLEEP_ENABLED = 			0x01
};

enum CTRL4_C_INT2_on_INT1 {
	INT_SIG_SHARE =					0x00, // Interrupt signals shared between
										  // INT1 and INT2 pads
	INT_SIG_ALL_INT1 =				0x01 // all interrupts in logic or INT1
};

enum CTRL4_C_FIFO_TEMP_EN {
	TEMP_DATA_DISABLE =				0x00, // Enable temperature data as 4th
										  // FIFO set
	TEMP_DATA_ENABLE =				0x01
};

enum CTRL4_C_DRDY_MASK {
	DA_TMR_DISABLE = 				0x00,
	DA_TMR_ENABLE = 				0x01
};

enum CTRL4_C_I2C_DISABLE {
	ENABLE_I2C_AND_SPI =			0x00,
	DISABLE_I2C =					0x01
};

enum CTRL4_C_STOP_ON_FTH {
	FIFO_DEPTH_UNLIMITED =			0x00,
	FIFO_DEPTH_LIMITED =			0x01
};

typedef struct {
	enum CTRL4_C_XL_BW_SCAL_ODR		accelBandwidthSelect;
	enum CTRL4_C_SLEEP_G			gyroSleepEN;
	enum CTRL4_C_INT2_on_INT1		IntSigSelect;
	enum CTRL4_C_FIFO_TEMP_EN		tempFIFOEnable;
	enum CTRL4_C_DRDY_MASK			dataAvailableEN;
	enum CTRL4_C_I2C_DISABLE		i2cEnable;
	enum CTRL4_C_STOP_ON_FTH		FIFOThreshold;
} CTRL4Config;

/* Control register 5 (r/w) ------------------------------------------------ */

enum CTRL5_C_ROUNDING {
	NONE =							0x00,
	ACCL_ONLY =						0x01,
	GYRO_ONLY =						0x02,
	GYRO_AND_ACCL = 				0x03
};

enum CTRL5_C_ST_G {
	GYRO_NORMAL_MODE =				0x00, // Self test disabled
	GYRO_POSITIVE_SIGN_TEST =		0x01,
	GYRO_NOT_ALLOWED =				0x02,
	GYRO_NEGATIVE_SIGN_TEST =		0x03
};

enum CTRL5_C_ST_XL {
	ACCL_NORMAL_MODE =				0x00, // Self test disabled
	ACCL_POSITIVE_SIGN_TEST =		0x01,
	ACCL_NEGATIVE_SIGN_TEST =		0x02,
	ACCL_NOT_ALLOWED =				0x03
};

typedef struct {
	enum CTRL5_C_ROUNDING 			circularBMRoundMode;
	enum CTRL5_C_ST_G				gyroSelfTest;
	enum CTRL5_C_ST_XL				acclSelfTest;
} CTRL5Config;

/* Control register 6 (r/w) ------------------------------------------------ */

enum CTRL6_C_TRIG_EN {
	EXTERN_TRIG_DISABLE = 			0x00,
	EXTERN_TRIG_ENABLE =			0x01
};

enum CTRL6_C_LVLen {
	LVL_SENSE_TRIG_DISABLE =		0x00,
	LVL_SENSE_TRIG_ENABLE =			0x01
};

enum CTRL6_C_LVL2_EN {
	LVL_SENSE_LATCH_DISABLE =		0x00,
	LVL_SENSE_LATCH_ENABLE =		0x01
};

enum CTRL6_C_XL_HM_MODE {
	ACCL_HP_MODE_ENABLE =			0x00,
	ACCL_HP_MODE_DISABLE =			0x01
};

typedef struct {
	enum CTRL6_C_TRIG_EN			gyroExternEdgeSenseTriggerEnable;
	enum CTRL6_C_LVLen				gyroLevelSenseTriggerEnable;
	enum CTRL6_C_LVL2_EN			gyroLevelSenseLatchEnable;
	enum CTRL6_C_XL_HM_MODE			acclHighPerformanceMode;
}CTRL6Config;

/* Gyro sensor control register 7 (r/w) ------------------------------------ */

enum CTRL7_G_G_HM_MODE {
	GYRO_HP_MODE_ENABLE =			0x00,
	GYRO_HP_MODE_DISABLE = 			0x01
};

enum CTRL7_G_HP_G_EN {
	HPF_DISABLE =					0x00, // High pass filter disable
	HPF_ENABLE =					0x01
};

enum CTRL7_G_HP_G_RST {
	HPF_RESET_OFF = 				0x00,
	HPF_RESET_ON =					0x01
};

enum CTRL7_G_ROUNDING_STATUS {
	SOURCE_REG_ROUNDING_DISABLE = 	0x00,
	SOURCE_REG_ROUNDING_ENABLE =	0x01
};

enum CTRL7_G_HPCF_G {
	HPF_CUTOFF_8_1mHZ =				0x00,
	HPF_CUTOFF_32_4mHZ =			0x01,
	HPF_CUTOFF_2_07HZ =				0x02,
	HPF_CUTOFF_16_32HZ =			0x03
};

typedef struct {
	enum CTRL7_G_G_HM_MODE			gyroHighPerformanceMode;
	enum CTRL7_G_HP_G_EN			gyroHPFEnable;
	enum CTRL7_G_HP_G_RST			gyroHPFReset;
	enum CTRL7_G_ROUNDING_STATUS	srcRegisterRoundingEnable;
	enum CTRL7_G_HPCF_G				gyroHPFCutoffFreq;
} CTRL7Config;

/* Accelerometer sensor control register 8 (r/w) --------------------------- */

enum CTRL8_XL_LPF2_XL_EN {
	LPF_DISABLE = 					0x00, // Note, depends on HP_SLOPE_XL_EN
	LPF_ENABLE =					0x01
};

enum CTRL8_XL_HPCF_XL {
	HPF_SLOPE_ODR_XL_DIV_50 = 		0x00, // Slope filter with cutoff at ODR_XL/50
	HPF_HP_ODR_XL_DIV_100 =			0x01,
	HPF_HP_ODR_XL_DIV_9 =			0x02,
	HPF_HP_ODR_XL_DIV_400 =			0x03
};

enum CTRL8_XL_HP_SLOPE_XL_EN {
	HP_SLOPE_DISABLE =				0x00, // Refer to figure 5
	HP_SLOPE_ENABLE =				0x01
};

enum CTRL8_XL_LOW_PASS_ON_6D {
	LP_ON_6D_DISABLE = 				0x00, // RAW data fed into 6d func
	LP_ON_6D_ENABLE =				0x01 // Low pass filter fed into 6D func
};

typedef struct {
	enum CTRL8_XL_LPF2_XL_EN		acclLPF2Selec;
	enum CTRL8_XL_HPCF_XL			acclHPFAndSFConfig;
	enum CTRL8_XL_HP_SLOPE_XL_EN	acclHPFSelec;
	enum CTRL8_XL_LOW_PASS_ON_6D	acclLPFOn6D;
} CTRL8Config;

/* Accelerometer sensor control register 9 (r/w) --------------------------- */

enum CTRL9_XL_Zen_XL {
	ACCL_Z_AXIS_OUTPUT_DISABLE = 	0x00,
	ACCL_Z_AXIS_OUTPUT_ENABLE =		0x01
};

enum CTRL9_XL_Yen_XL {
	ACCL_Y_AXIS_OUTPUT_DISABLE = 	0x00,
	ACCL_Y_AXIS_OUTPUT_ENABLE =		0x01
};

enum CTRL9_XL_Xen_XL {
	ACCL_X_AXIS_OUTPUT_DISABLE =	0x00,
	ACCL_X_AXIS_OUTPUT_ENABLE =		0x01
};

typedef struct {
	enum CTRL9_XL_Zen_XL			acclZAxisEnable;
	enum CTRL9_XL_Yen_XL			acclYAxisEnable;
	enum CTRL9_XL_Xen_XL			acclXAxisEnable;
} CTRL9Config ;

/* Control register 10 (r/w) ----------------------------------------------- */

enum CTRL10_C_Zen_G {
	GYRO_Z_AXIS_OUTPUT_DISABLE = 	0x00,
	GYRO_Z_AXIS_OUTPUT_ENABLE =		0x01
};

enum CTRL10_C_Yen_G {
	GYRO_Y_AXIS_OUTPUT_DISABLE = 	0x00,
	GYRO_Y_AXIS_OUTPUT_ENABLE = 	0x01
};

enum CTRL10_C_Xen_G {
	GYRO_X_AXIS_OUTPUT_DISABLE = 	0x00,
	GYRO_X_AXIS_OUTPUT_ENABLE =		0x01
};

enum CTRL10_C_FUNC_EN { // Enables embedded functionalities (pedometer, tilt,
						// free fall, etc)
	EMBED_FUNCT_DISABLE = 			0x00,
	EMBED_FUNCT_ENABLE =			0x01
};

enum CTRL10_C_PEDO_RST_STEP {
	RST_STEP_COUNT_DISABLE =		0x00,
	RST_SET_COUNT_ENABLE =			0x01
};

enum CTRL10_C_SIGN_MOTION_EN {
	SIGN_MOTION_DETECT_DISABLE =	0x00, // Significant motion detect disable
	SIGN_MOTION_DETECT_ENABLE =		0x01
};

typedef struct {
	enum CTRL10_C_Zen_G				gyroZAxisEnable;
	enum CTRL10_C_Yen_G				gyroYAxisEnable;
	enum CTRL10_C_Xen_G				gyroXAxisEnable;
	enum CTRL10_C_FUNC_EN			embedFunctEnable;
	enum CTRL10_C_PEDO_RST_STEP		stepCounterReset;
	enum CTRL10_C_SIGN_MOTION_EN	signMotionDetEnable;
} CTRL10Config;

/* CTRL Config registers (r/w) --------------------------------------------- */

typedef struct {
	CTRL1Config 				CTRL1Reg;
	CTRL2Config					CTRL2Reg;
	CTRL3Config					CTRL3Reg;
	CTRL4Config					CTRL4Reg;
	CTRL5Config 				CTRL5Reg;
	CTRL6Config					CTRL6Reg;
	CTRL7Config					CTRL7Reg;
	CTRL8Config					CTRL8Reg;
	CTRL9Config					CTRL9Reg;
	CTRL10Config				CTRL10Reg;
} LSM6DS33CTRLConfig;

/* Wake-up internal source register (r) ------------------------------------ */

enum WAKE_UP_SRC_FF_IA {
	FREE_FALL_NOT_DETECTED =		0x00,
	FREE_FALL_DETECTED =			0x01
};

enum WAKE_UP_SRC_SLEEP_STATE_IA {
	SLEEP_EVENT_NOT_DETECTED =		0x00,
	SLEEP_EVENT_DETECTED =			0x01
};

enum WAKE_UP_SRC_WU_IA {
	WAKE_UP_EVENT_NOT_DETECTED =	0x00,
	WAKE_UP_EVENT_DETECTED =		0x01
};

enum WAKE_UP_SRC_X_WU {
	WAKE_UP_ON_X_AXIS_NOT_DET =		0x00,
	WAKE_UP_ON_X_AXIS_DET =			0x01
};

enum WAKE_UP_SRC_Y_WU {
	WAKE_UP_ON_Y_AXIS_NOT_DET =		0x00,
	WAKE_UP_ON_Y_AXIS_DET =			0x01
};

enum WAKE_UP_SRC_Z_WU {
	WAKE_UP_ON_Z_AXIS_NOT_DET =		0x00,
	WAKE_UP_ON_Z_AXIS_DET =			0x01
};

typedef struct {
	enum WAKE_UP_SRC_FF_IA			freeFallDetectStatus;
	enum WAKE_UP_SRC_SLEEP_STATE_IA	sleepEventStatus;
	enum WAKE_UP_SRC_WU_IA			wakeUpEventDetectStatus;
	enum WAKE_UP_SRC_X_WU			xAxisWakeUpDetectStatus;
	enum WAKE_UP_SRC_Y_WU			yAxisWakeUpDetectStatus;
	enum WAKE_UP_SRC_Z_WU			zAxisWakeUpDetectStatus;
} WakeUpSrcRegister;

/* Tap source register (r) ------------------------------------------------- */

enum TAP_SRC_TAP_IA {
	TAP_EVENT_NOT_DETECTED =		0x00,
	TAP_EVENT_DETECTED =			0x01
};

enum TAP_SRC_SINGLE_TAP {
	SINGLE_TAP_EVENT_NOT_DETECTED =	0x00,
	SINGLE_TAP_EVENT_DETECTED =		0x01
};

enum TAP_SRC_DOUBLE_TAP {
	DOUBLE_TAP_EVENT_NOT_DETECTED =	0x00,
	DOUBLE_TAP_EVENT_DETECTED =		0x01
};

enum TAP_SRC_TAP_SIGN {
	ACCL_POSITIVE_SIGN_TAP_DETECT =	0x00,
	ACCL_NEGATIVE_SIGN_TAP_DETECT =	0x01
};

enum TAP_SRC_X_TAP {
	TAP_EVENT_ON_X_NOT_DETECTED =	0x00,
	TAP_EVENT_ON_X_DETECTED =		0x01
};

enum TAP_SRC_Y_TAP {
	TAP_EVENT_ON_Y_NOT_DETECTED =	0x00,
	TAP_EVENT_ON_Y_DETECTED =		0x01
};

enum TAP_SRC_Z_TAP {
	TAP_EVENT_ON_Z_NOT_DETECTED =	0x00,
	TAP_EVENT_ON_Z_DETECTED =		0x01
};

typedef struct {
	enum TAP_SRC_TAP_IA				tapEventDetectStatus;
	enum TAP_SRC_SINGLE_TAP			singleTapEventStatus;
	enum TAP_SRC_DOUBLE_TAP			doubleTapEventStatus;
	enum TAP_SRC_TAP_SIGN			acclSignTapDetect;
	enum TAP_SRC_X_TAP				acclXAxisTapDetectStatus;
	enum TAP_SRC_Y_TAP				acclYAxisTapDetectStatus;
	enum TAP_SRC_Z_TAP				acclZAxisTapDetectStatus;
} TapSrcRegister;

/* Portrait, landscape, face-up and face-down source register (r) ---------- */

enum D6D_SRC_D6D_IA { // Used for smart phone orientation change
	CHANGE_POSITION_NOT_DETECTED =	0x00,
	CHANGE_POSITION_DETECTED =		0x01
};

enum D6D_SRC_ZH {
	Z_AXIS_HIGH_NOT_DETECTED =		0x00,
	Z_AXIS_HIGH_DETECTED =			0x01
};

enum D6D_SRC_ZL {
	Z_AXIS_LOW_NOT_DETECTED =		0x00,
	Z_AXIS_LOW_DETECTED =			0x01
};

enum D6D_SRC_YH {
	Y_AXIS_HIGH_NOT_DETECTED =		0x00,
	Y_AXIS_HIGH_DETECTED =			0x01
};

enum D6D_SRC_YL {
	Y_AXIS_LOW_NOT_DETECTED =		0x00,
	Y_AXIS_LOW_DETECTED =			0x01
};

enum D6D_SRC_XH {
	X_AXIS_HIGH_NOT_DETECTED =		0x00,
	X_AXIS_HIGH_DETECTED =			0x01
};

enum D6D_SRC_XL {
	X_AXIS_LOW_NOT_DETECTED =		0x00,
	X_AXIS_LOW_DETECTED =			0x01
};

typedef struct {
	enum D6D_SRC_D6D_IA				positionChangeInt;
	enum D6D_SRC_ZH					zAxisHighEvent;
	enum D6D_SRC_ZL					zAxisLowEvent;
	enum D6D_SRC_YH					yAxisHighEvent;
	enum D6D_SRC_YL					yAxisLowEvent;
	enum D6D_SRC_XH					xAxisHighEvent;
	enum D6D_SRC_XL					xAxisLowEvent;
} D6DSrcRegister;

/* Status Register (r) ------------------------------------------------------ */

enum STATUS_REG_EV_BOOT {
	NO_BOOT_RUNNING =				0x00,
	BOOT_RUNNING =					0x01
};

enum STATUS_REG_TDA {
	NO_NEW_TEMP_DATA_RDY =			0x00,
	NEW_TEMP_DATA_RDY =				0x01
};

enum STATUS_REG_GDA {
	NO_NEW_GYRO_DATA_RDY = 			0x00,
	NEW_GYRO_DATA_RDY =				0x01
};

enum STATUS_REG_XLDA {
	NO_NEW_ACCL_DATA_RDY =			0x00,
	NEW_ACCL_DATA_RDY =				0x01
};

typedef struct {
	enum STATUS_REG_EV_BOOT			bootRunningFlag;
	enum STATUS_REG_TDA				tempDataAvailable;
	enum STATUS_REG_GDA				gyroDataAvailable;
	enum STATUS_REG_XLDA			acclDataAvailable;
} StatusRegister;

/* Temperature data output registers (r) ----------------------------------- */

typedef struct { // Register is expressed as a two's compliment, 16-bit word
	int8_t							OUT_TEMP_L;
	int8_t							OUT_TEMP_H;
} tempData;

/* Gyro data output registers (r) ------------------------------------------ */

typedef struct { // Each axis expressed as a two's compliment, 16-bit word
	int8_t 							OUTX_L_G;
	int8_t							OUTX_H_G;
	int8_t							OUTY_L_G;
	int8_t							OUTY_H_G;
	int8_t							OUTZ_L_G;
	int8_t							OUTZ_H_G;
} gyroData;

/* Accelerometer data output registers (r) --------------------------------- */

typedef struct { // Same format as gyroData
	int8_t 							OUTX_L_XL;
	int8_t							OUTX_H_XL;
	int8_t							OUTY_L_XL;
	int8_t							OUTY_H_XL;
	int8_t							OUTZ_L_XL;
	int8_t							OUTZ_H_XL;
} acclData;


/* FIFO status control registers (r) --------------------------------------- */

typedef struct {
	uint8_t 						numUnreadWords; // 0 -> 7 bytes, the rest
													// are in FIFOStatus2
} FIFOStatus1;

enum FIFO_STATUS2_FTH {
	FIFO_BELOW_WATERLVL =			0x00,
	FIFO_EQUAL_OR_ABOVE_WATERLVL = 	0x01
};

enum FIFO_STATUS2_FIFO_OVER_RUN {
	FIFO_NOT_FILLED =				0x00,
	FIFO_COMPLETELY_FILLED =		0x01
};

enum FIFO_STATUS2_FIFO_FULL {
	FIFO_NOT_FULL =					0x00,
	FIFO_FULL_AT_NEXT_ODR =			0x01
};

enum FIFO_STATUS2_FIFO_EMPTY {
	FIFO_CONTAINS_DATA =			0x00,
	FIFO_IS_EMPTY =					0x01
};

typedef struct {
	enum FIFO_STATUS2_FTH			FIFOWatermarkStatus;
	enum FIFO_STATUS2_FIFO_OVER_RUN	FIFOOverrunStatus;
	enum FIFO_STATUS2_FIFO_FULL		FIFOFullStatus;
	enum FIFO_STATUS2_FIFO_EMPTY	FIFOEmptyBit;
	uint8_t							numUnreadWords2; // 8 -> 11 bytes
} FIFOStatus2;


typedef struct {
	uint8_t							FIFOPatternL; // bits 0 -> 7
} FIFOStatus3;

typedef struct {
	uint8_t							FIFOPatternH; // Bits 8 -> 9
} FIFOStatus4;

typedef struct {
	int8_t							FIFO_DATA_OUT_L;
	int8_t							FIFO_DATA_OUT_H;
} FIFODataOutRegister;

/* Timestamp Register ------------------------------------------------------ */

typedef struct {
	int8_t							TIMESTAMP0_REG; // r
	int8_t							TIMESTAMP1_REG; // r
	int8_t							TIMESTAMP2_REG; // r/w, reset by writing
													// AAh value
} TimestampRegister;

typedef struct {
	int8_t							STEP_TIMESTAMP_L; // Step counter timestamp
													  // info register
	int8_t							STEP_TIMESTAMP_H;
} StepTimestampRegister;

typedef struct {
	int8_t							STEP_COUNTER_L;
	int8_t							STEP_COUNTER_H;
} StepCounterOutputRegister;

/* Significant motion interrupt source register ---------------------------- */

enum FUNC_SRC_STEP_COUNTER_DELTA_IA {
	NO_STEP_DURING_TIME_STEP =		0x00,
	STEP_DET_DURING_TIME_STEP =		0x01
};

enum FUNC_SRC_SIGN_MOTION_IA {
	NO_SIGN_MOTION_DET =			0x00,
	SIGN_MOTION_DET =				0x01
};

enum FUNC_SRC_TILT_IA {
	NO_TILT_DET =					0x00,
	TILE_EVENT_DET =				0x01
};

enum FUNC_SRC_STEP_DETECTED {
	STEP_DETECT_EVENT_NOT_DET =		0x00,
	STEP_DETECT_EVENT_DET =			0x01
};

enum FUNC_SRC_STEP_OVERFLOW {
	STEP_COUNTER_NOT_OVERFLOW =		0x00,
	STEP_COUNTER_OVERFLOW =			0x01
};

typedef struct {
	enum FUNC_SRC_STEP_COUNTER_DELTA_IA		pedometerTimeStepRecogniton;
	enum FUNC_SRC_SIGN_MOTION_IA			signMotionEventDetStatus;
	enum FUNC_SRC_TILT_IA					tiltEventDetStatus;
	enum FUNC_SRC_STEP_DETECTED				stepDetEventStatus;
	enum FUNC_SRC_STEP_OVERFLOW				stepCounterOverflowStatus;
} FuncSrcInterruptSourceRegister;

/* Significant motion recognition config register (r/w) --------------------- */

enum TAP_CFG_TIMER_EN {
	TIME_STAMP_COUNT_DISABLED =		0x00,
	TIME_STAMP_COUNT_ENABLED =		0x01
};

enum TAP_CGF_PEDO_EN {
	PEDOMETER_ALGO_DISABLED =		0x00,
	PEDOMETER_ALGO_ENABLED =		0x01
};

enum TAP_CFG_TILT_EN {
	TIME_CALC_DISABLE =				0x00,
	TIME_CALC_ENABLE =				0x01
};

enum TAP_CFG_SLOPE_FDS {
	ACCL_HP_LPF2_DISABLE =			0x00,
	ACCL_HP_LPF2_ENABLE =			0x01
};

enum TAP_CFG_TAP_X_EN {
	TAP_DET_X_AXIS_DISABLE =		0x00,
	TAP_DET_X_AXIS_ENABLE =			0x01
};

enum TAP_CFG_TAP_Y_EN {
	TAP_DET_Y_AXIS_DISABLE =		0x00,
	TAP_DET_Y_AXIS_ENABLE =			0x01
};

enum TAP_CFG_TAP_Z_AXIS {
	TAP_DET_Z_AXIS_DISABLE =		0x00,
	TAP_DET_Z_AXIS_ENABLE =			0x01
};

enum TAP_CFG_LIR {
	INT_REQ_NOT_LATCHED =			0x00,
	INT_REQ_LATCHED =				0x01
};

typedef struct {
	enum TAP_CFG_TIMER_EN					timeStampCountEnable;
	enum TAP_CGF_PEDO_EN					pedometerAlgoEnable;
	enum TAP_CFG_TILT_EN					tiltCalcEnable;
	enum TAP_CFG_SLOPE_FDS					acclFilterTypeEnable;
	enum TAP_CFG_TAP_X_EN					xAxisTapRecognition;
	enum TAP_CFG_TAP_Y_EN					yAxisTapRecognition;
	enum TAP_CFG_TAP_Z_AXIS					zAxisTapRecognition;
	enum TAP_CFG_LIR						latchInterrupt;
} TAPCFGConfigurationRegister;

/* Portrait/Landscape position and tap func threshold register (r/w) ------- */

enum TAP_THS_6D_D4D_EN {
	ORIENTATION_4D_DET_DISABLE =	0x00,
	ORIENTATION_4D_DET_ENABLE =		0x01
};

enum TAP_THS_6D_SIXD_THS {
	D6D_FUNC_THRESH_80_DEG =		0x00,
	D6D_FUNC_THRESH_70_DEG =		0x01,
	D6D_FUNC_THRESH_60_DEG =		0x02,
	D6D_FUNC_THRESH_50_DEG =		0x03
};

typedef struct {
	enum TAP_THS_6D_D4D_EN			orientationDetEnable;
	enum TAP_THS_6D_SIXD_THS		D6DThreshFunction;
	uint8_t							tapThresh; // 6-bit register.
} PositionAndTapFuncThreshRegister; // TAP_THS_6D

/* Tap recognition function setting register (r/w) ------------------------- */

typedef struct {
	/* default value is 0000b, which corresponds to 16*ODR_XL time.
	 * If any other value is set, the LSB corresponds to 32*ODR_XL time
	 */
	uint8_t							DURDoubleTapMaxTimeGap; // 4-bits

	/* default value is 00b, which corresponds to be 2*ODR_time.
	 * Similar to DUR, 1LSB corresponds to 4*ODR_time
	 */
	uint8_t							QUIETDebouncingPeriod;	 // 2-bits

	/* Default value is 00b, corresponds to 4*ODR_time, 1LSB corresponds
	 * to 8*ODR_time
	 */
	uint8_t 						SHOCKMaxDurationOverthreshold; // 2-bits
} TapRecognitionFunctionSettingRegister; // INT_DUR2

/* Free-fall function duration setting register (r/w) ---------------------- */

enum FREE_FALL_FF_THS {
	FF_THS_156_mg =					0x00,
	FF_THS_219_mg =					0x01,
	FF_THS_250_mg =					0x02,
	FF_THS_312_mg =					0x03,
	FF_THS_344_mg =					0x04,
	FF_THS_406_mg =					0x05,
	FF_THS_469_mg =					0x06,
	FF_THS_500_mg =					0x07
};

typedef struct {
	uint8_t							FF_DUR; // Free fall duration event 5-bits
	enum FREE_FALL_FF_THS			FF_THS;
} freeFallDurationConfigRegister;

/* INT1 register functions routing (r/w) ----------------------------------  */

enum MD1_CFG_INT1_INACT_STATE {
	ROUTING_ON_INT1_ON_INACT_DISABLED =	0x00,
	ROUTING_ON_INT1_ON_INACT_ENABLED =	0x01
};

enum MD1_CFG_INT1_SINGLE_TAP {
	SINGLE_TAP_ROUTING_INT1_DISABLE =	0x00,
	SINGLE_TAP_ROUTING_INT1_ENABLE =	0x01
};

enum MD1_CFG_INT1_WU {
	WAKEUP_EVENT_ROUTING_INT1_DISABLE =	0x00,
	WAKEUP_EVENT_ROUTING_INT1_ENABLE =	0x01
};

enum MD1_CFG_INT1_FF {
	FREE_FALL_EVENT_ROUTE_INT1_DISABLE=	0x00,
	FREE_FALL_EVENT_ROUTE_INT1_ENABLE = 0x01
};

enum MD1_CFG_INT1_DOUBLE_TAP {
	DOUBLE_TAP_ROUTING_INT1_DISABLE =	0x00,
	DOUBLE_TAP_ROUTING_INT1_ENABLE =	0x01
};

enum MD1_CFG_INT1_6D {
	EVENT_6D_ROUTE_INT1_DISABLE =		0x00,
	EVENT_6D_ROUTE_INT1_ENABLE =		0x01
};

enum MD1_CFG_INT1_TILT {
	TILT_EVENT_ROUTE_INT1_DISABLE =		0x00,
	TILT_EVENT_ROUTE_INT1_ENABLE =		0x01
};

enum MD1_CFG_INT1_TIMER {
	TIMER_END_COUNTER_INT1_DISABLE =	0x00,
	TIMER_END_COUNTER_INT1_ENABLE =		0x01
};

typedef struct {
	enum MD1_CFG_INT1_INACT_STATE		inactivityRouting;
	enum MD1_CFG_INT1_SINGLE_TAP		singleTapRecogRouting;
	enum MD1_CFG_INT1_WU				wakeupEventRouting;
	enum MD1_CFG_INT1_FF				freefallEventRouting;
	enum MD1_CFG_INT1_DOUBLE_TAP		doubleTapRecogRouting;
	enum MD1_CFG_INT1_6D				event6DRouting;
	enum MD1_CFG_INT1_TILT				tiltEventRouting;
	enum MD1_CFG_INT1_TIMER				timerEndOfCounterEventRouting;
} FuncRoutingINT1Register; // MD1_CFG

/* INT2 register functions routing (r/w) ----------------------------------- */

enum MD2_CFG_INT2_INACT_STATE {
	ROUTING_ON_INT2_ON_INACT_DISABLED =	0x00,
	ROUTING_ON_INT2_ON_INACT_ENABLED =	0x01
};

enum MD2_CFG_INT2_SINGLE_TAP {
	SINGLE_TAP_ROUTE_INT2_DISABLE =		0x00,
	SINGLE_TAP_ROUTE_INT2_ENABLE =		0x01
};

enum MD2_CFG_INT2_WU {
	WAKEUP_EVENT_ROUTING_INT2_DISABLE =	0x00,
	WAKEUP_EVENT_ROUTING_INT2_ENABLE =	0x01
};

enum MD2_CFG_INT2_FF {
	FREE_FALL_EVENT_ROUTE_INT2_DISABLE=	0x00,
	FREE_FALL_EVENT_ROUTE_INT2_ENABLE = 0x01
};

enum MD2_CFG_INT2_DOUBLE_TAP {
	DOUBLE_TAP_ROUTING_INT2_DISABLE =	0x00,
	DOUBLE_TAP_ROUTING_INT2_ENABLE =	0x01
};

enum MD2_CFG_INT2_6D {
	EVENT_6D_ROUTE_INT2_DISABLE =		0x00,
	EVENT_6D_ROUTE_INT2_ENABLE =		0x01
};

enum MD2_CFG_INT2_TILT {
	TILT_EVENT_ROUTE_INT2_DISABLE =		0x00,
	TILT_EVENT_ROUTE_INT2_ENABLE =		0x01
};

typedef struct {
	enum MD2_CFG_INT2_INACT_STATE		inactivityRouting;
	enum MD2_CFG_INT2_SINGLE_TAP		singleTapRecogRouting;
	enum MD2_CFG_INT2_WU				wakeupEventRouting;
	enum MD2_CFG_INT2_FF				freefallEventRouting;
	enum MD2_CFG_INT2_DOUBLE_TAP		doubleTapRecogRouting;
	enum MD2_CFG_INT2_6D				event6DRouting;
	enum MD2_CFG_INT2_TILT				tiltEventRouting;
} FuncRoutingINT2Register; // MD2_CFG

/* Pedometer threshold register (r/w) -------------------------------------- */

enum PEDO_THS_REG_PEDO_4G {
	PEDO_FULL_SCALE_2G =				0x00,
	PEDO_FULL_SCALE_4G =				0x01 // Unless CTRL1_XL < 4g
};

typedef struct {
	enum PEDO_THS_REG_PEDO_4G			internalFullScale;

	/* @PEDO_4G = 0, 1LSB = 16mg.
	 * @PEDO_4G = 1, 1LSB = 32mg
	 */
	uint8_t								minThreshConfig; // 5-bits
} pedometerThresholdRegister;

/* Significant motion configuration register ------------------------------- */

typedef struct {
	uint8_t 							signMotionValue; // Default is '6'
} signMotionConfReg;

/* Pedometer debouncing register (r/w) ------------------------------------- */

typedef struct {
	uint8_t								DEM_TIME; // DEM_TIME*80ms, 5-bits
	uint8_t 							DEB_STEP; // Min steps to incr counter,
												  // 3-bits
} pedometerDebounceReg;

/* Step counter step detect delta time register (r/w) ---------------------- */

typedef struct {
	uint8_t 							SC_DELTA; // Time period value
												  // (1LSB = 1.6384s)
} stepCounterIntervalConfigReg;

/* External functions ------------------------------------------------------ */

void LSM6DS33_init(LSM6DS33* device, enum LSM6DS33DeviceType type,
		enum LSM6DS33SA0State sa0);
void LSM6DS33_read(LSM6DS33* device);
int8_t LSM6DS33_read_reg(LSM6DS33* device, uint8_t reg);
void LSM6DS33_calcAcclRes(LSM6DS33* device);
void LSM6DS33_calcGyroRes(LSM6DS33* device);
#endif
