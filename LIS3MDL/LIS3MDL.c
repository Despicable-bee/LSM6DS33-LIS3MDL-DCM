/* Includes ----------------------------------------------------------------- */
#include "LIS3MDL.h"

/* Definitions -------------------------------------------------------------- */
#define LIS3MDL_WHO_AM_I_VALUE		0x3D

/* Static function prototypes ----------------------------------------------- */

static void LIS3MDL_CTRL_REG1_config(LIS3MDL* device);
static void LIS3MDL_CTRL_REG2_config(LIS3MDL* device);
static void LIS3MDL_CTRL_REG3_config(LIS3MDL* device);
static void LIS3MDL_CTRL_REG4_config(LIS3MDL* device);
static void LIS3MDL_CTRL_REG5_config(LIS3MDL* device);

static void LIS3MDL_test_reg(LIS3MDL* device);

static void LIS3MDL_write_reg(LIS3MDL* device, LIS3MDLMsg* pMsg);


/* Function definitions ----------------------------------------------------- */

/**
 * @brief:  Responsible for enabling the temperature sensor, selecting the
 * X and Y operative modes, selecting the output data rate, enabling the
 * fast output data rate option, and enabling self test.
 * @param:  device - Contains information relevant for configuring the device
 */
static void LIS3MDL_CTRL_REG1_config(LIS3MDL* device) {
	/* TEMP_EN | OM1 | OM0 | DO2 | DO1 | DO0 | FAST_ODR | ST */

	// Local variable init
	LIS3MDLCTRLReg1Config config;
	uint8_t data = 0;
	LIS3MDLMsg msg;

	// Configure each of the bits
	config.temperatureEnable = LIS3MDL_TEMP_SENSE_ENABLE;
	config.operativeMode = LIS3MDL_XY_AXES_ULTRA_PERF_MODE;
	config.outputDataRate = LIS3MDL_OUT_DATA_RATE_10_HZ;
	config.fastDataOutEnable = LIS3MDL_FAST_ODR_DISABLE;
	config.selfTestEnable = LIS3MDL_SELF_TEST_DISABLE;


	// Combine each data bit into the byte to be sent
	data = (config.temperatureEnable << 7) |\
			(config.operativeMode << 5) |\
			(config.outputDataRate << 2) |\
			(config.fastDataOutEnable << 1) | (config.selfTestEnable);


	msg.pData = &data;
	msg.rwBit = LIS3MDL_WRITE_BIT; // Write
	msg.reg = LIS3MDL_CTRL_REG1;
	msg.numData = 1;

	// Send that data to the device
	LIS3MDL_write_reg(device, &msg);
}

/**
 * @brief:  Responsible for configuring the full-scale selection (gauss),
 * 	the reboot mode, and the software reset of the registers back to their
 * 	default values.
 */
static void LIS3MDL_CTRL_REG2_config(LIS3MDL* device) {
	/* 0 | FS1 | FS2 | 0 | REBOOT | 0 | SOFT_RST | 0 | 0 */

	LIS3MDLCTRLReg2Config config;
	uint8_t data = 0;
	LIS3MDLMsg msg;

	// configure each of the bits
	config.fullScaleConfig = LIS3MDL_FS_4_GAUSS;
	config.rebootMode = LIS3MDL_NORMAL_MODE;
	config.softResetMode = LIS3MDL_SOFT_RESET_DISABLED;

	data = (config.fullScaleConfig << 6) | (config.rebootMode << 4) |
			(config.softResetMode << 2);

	msg.pData = &data;
	msg.rwBit = LIS3MDL_WRITE_BIT;
	msg.reg = LIS3MDL_CTRL_REG2;
	msg.numData = 1;

	LIS3MDL_write_reg(device, &msg);
}

/**
 * @brief:  Responsible for configuring the power mode, the SPI wire interface,
 * 	and the operating mode.
 */
static void LIS3MDL_CTRL_REG3_config(LIS3MDL* device) {
	/* 0 | 0 | LP | 0 | 0 | SIM | MD1 | MD0 */

	LIS3MDLCTRLReg3Config config;
	uint8_t data = 0;
	LIS3MDLMsg msg;

	// configure each of the bits
	config.lowPowerModeConfig = LIS3MDL_LOW_POWER_MODE_DISABLE;
	config.SPIInterfaceMode = LIS3MDL_SPI_4WIRE_IM;
	config.systemOperatingMode = LIS3MDL_SYS_MODE_CONTINUOUS_CONV_MODE;

	data = (0 << 7) | (0 << 6) | (config.lowPowerModeConfig << 5) | \
		   (0 << 4) | (0 << 3) | (config.SPIInterfaceMode << 2) |\
			(config.systemOperatingMode);

	msg.pData = &data;
	msg.rwBit = LIS3MDL_WRITE_BIT;
	msg.reg = LIS3MDL_CTRL_REG3;
	msg.numData = 1;

	LIS3MDL_write_reg(device, &msg);
}

/**
 * @brief:  Responsible for configuring the z-axis operative mode and
 * 	the endianness for the output data.
 */
static void LIS3MDL_CTRL_REG4_config(LIS3MDL* device) {
	/* 0 | 0 | 0 | 0 | OMZ1 | OMZ0 | BLE | 0 */

	LIS3MDLCTRLReg4Config config;
	uint8_t data = 0;
	LIS3MDLMsg msg;

	// Configure each of the bits
	config.zAxisOperatingMode = LIS3MDL_Z_AXIS_ULTRA_PERF_MODE;
	config.addressEndianness = LIS3MDL_DATA_LSB_LOWER_ADDR;

	data = (0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | \
			(config.zAxisOperatingMode << 2) | (config.addressEndianness << 1) |\
			0;

	msg.pData = &data;
	msg.rwBit = LIS3MDL_WRITE_BIT;
	msg.reg = LIS3MDL_CTRL_REG4;
	msg.numData = 1;

	LIS3MDL_write_reg(device, &msg);
}

/**
 * @brief:  Responsible for configuring the fast read (increase reading
 * 	efficiency), and the block update mode for the magnetic data.
 */
static void LIS3MDL_CTRL_REG5_config(LIS3MDL* device) {
	/* FAST_READ | BDU | 0 | 0 | 0 | 0 | 0 | 0  */

	LIS3MDLCTRLReg5Config config;
	uint8_t data = 0;
	LIS3MDLMsg msg;

	// Configure each of the... yeah you get it.
	config.fastReadEnable = LIS3MDL_FAST_READ_DISABLE;
	config.blockUpdateMode = LIS3MDL_BLOCK_DATA_CONT_UPDATE;

	data = (config.fastReadEnable << 7) | (config.blockUpdateMode << 6) | 0;

	msg.pData = &data;
	msg.rwBit = LIS3MDL_WRITE_BIT;
	msg.reg = LIS3MDL_CTRL_REG5;
	msg.numData = 1;

	LIS3MDL_write_reg(device, &msg);
}

/**
 * @brief:  Gets the current status register
 */
void LIS3MDL_get_status_reg(LIS3MDL* device, LIS3MDLSRegConfig* sreg) {
	/* ZYXOR | ZOR | YOR | XOR | ZYXDA | ZDA | YDA | XDA */
	uint8_t reg = LIS3MDL_read_reg(device, LIS3MDL_STATUS_REG);
	sreg->xyzAxesDataOverrun = ((reg & 0x80) >> 7);
	sreg->zAxisDataOverrun = ((reg & 0x40) >> 6);
	sreg->yAxisDataOverrun = ((reg & 0x20) >> 5);
	sreg->xAxisDataOverrun = ((reg & 0x10) >> 4);
	sreg->xyzAxesDataAvailable = ((reg & 0x08) >> 3);
	sreg->zAxisDataAvailable = ((reg & 0x04) >> 2);
	sreg->yAxisDataAvailable = ((reg & 0x02) >> 1);
	sreg->xAxisDataAvailable = ((reg & 0x01));
}

/**
 * @brief:  Reads the magnetometer data from the device and organises it
 * 	into the device struct
 */
enum LIS3MDLResultStatus LIS3MDL_read_mag(LIS3MDL* device) {
	// testing
	int8_t value;
	uint8_t dataSet[8];
	// Assert multiple reads
	uint8_t reg = LIS3MDL_OUT_X_L | 0x80;
	// Tell device that we want to read the value of reg
	HAL_StatusTypeDef stat1 = HAL_I2C_Master_Transmit(device->hi2c,
			(device->address << 1) | LIS3MDL_READ_BIT, &reg, 1,
			HAL_MAX_DELAY);

	if(stat1 != HAL_OK) {
		// todo error handling
		__NOP();
	}

	// Receive the byte
	HAL_StatusTypeDef stat = HAL_I2C_Master_Receive(device->hi2c,
			(device->address << 1) | LIS3MDL_READ_BIT, &dataSet, 8,
					HAL_MAX_DELAY);

	if(stat != HAL_OK) {
		// todo error handling
		__NOP();
	}

	device->magX = (dataSet[1] << 8) | (dataSet[0]);
	device->magY = (dataSet[3] << 8) | (dataSet[2]);
	device->magZ = (dataSet[5] << 8) | (dataSet[4]);
	device->temp = (dataSet[7] << 8) | (dataSet[6]);

	// Todo: error handling

	device->status = LIS3MDL_SUCCESS;

	return LIS3MDL_SUCCESS;
}

/**
 * @brief:  Sends a number of bytes to the LIS3MDL via the I2C interface
 * @param:  device - Contains information relevant to the device,
 * @param:  pMsg - pointer to the message to send
 */
static void LIS3MDL_write_reg(LIS3MDL* device, LIS3MDLMsg* pMsg) {
	// Create the buffer to contain the data to send
	// Register | data
	uint8_t dataToSend[pMsg->numData + 1];

	// Set the message to write
	dataToSend[0] = pMsg->reg;

	// Copy the remaining data bytes to the buffer
	for(int i = 1; i < pMsg->numData + 1; i++) {
		dataToSend[i] = pMsg->pData[i-1];
	}

	HAL_StatusTypeDef stat1 = HAL_I2C_Master_Transmit(device->hi2c,
			(device->address << 1) | LIS3MDL_WRITE_BIT,
			dataToSend, pMsg->numData + 1, HAL_MAX_DELAY);
	if(stat1 != HAL_OK) {
		// todo error handling
		__NOP();
	}
}

/**
 * @brief:  Reads a single specified register from a LIS3MDL device via the
 * 	I2C interface.
 * @param:  device - Contains information relevant for communicating with
 * 	the device
 * @param:  reg - Register we want to read from.
 * @retval: Upon success, function will return the value of the register,
 * 	otherwise will return -1.
 */
int8_t LIS3MDL_read_reg(LIS3MDL* device, uint8_t reg) {
	int8_t value;
	// Tell device that we want to read the value of reg
	HAL_StatusTypeDef stat1 = HAL_I2C_Master_Transmit(device->hi2c,
			(device->address << 1) | LIS3MDL_READ_BIT, &reg, 1,
			HAL_MAX_DELAY);

	if(stat1 != HAL_OK) {
		// todo error handling
		__NOP();
	}

	// Receive the byte
	HAL_StatusTypeDef stat = HAL_I2C_Master_Receive(device->hi2c,
			(device->address << 1) | LIS3MDL_READ_BIT, &value, 1,
					HAL_MAX_DELAY);

	if(stat != HAL_OK) {
		// todo error handling
		__NOP();
	}

	return value;
}


static void LIS3MDL_test_reg(LIS3MDL* device) {
	if(LIS3MDL_read_reg(device, LIS3MDL_WHO_AM_I) != LIS3MDL_WHO_AM_I_VALUE) {
		// Todo error handling
		__NOP();
	}
	// Good this is the correct value
	// Todo, add some logging
}

/**
 * @brief:
 */
void LIS3MDL_init_reg(LIS3MDL* device) {
	LIS3MDL_test_reg(device);
	// Todo some config stuff
	LIS3MDL_CTRL_REG1_config(device);
	LIS3MDL_CTRL_REG2_config(device);
	LIS3MDL_CTRL_REG3_config(device);
	LIS3MDL_CTRL_REG4_config(device);
	LIS3MDL_CTRL_REG5_config(device);

	device->magX = 0;
	device->magY = 0;
	device->magZ = 0;
	device->temp = 0;
	device->counter = 0;
}

