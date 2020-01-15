/* Includes ---------------------------------------------------------------- */
#include "LSM6DS33.h"

/* Definitions ------------------------------------------------------------- */
#define LSM6DS33_WHO_AM_I_VALUE 	0x69 // Legend

/* Static function prototypes ---------------------------------------------- */

static void LSM6DS33_CTRL_config(LSM6DS33* device);

/* CTRL register config functions ------------------------------------------ */
static void LSM6DS33_CTRL1_config(LSM6DS33* device);
static void LSM6DS33_CTRL2_config(LSM6DS33* device);
static void LSM6DS33_CTRL3_config(LSM6DS33* device);
static void LSM6DS33_CTRL4_config(LSM6DS33* device);
static void LSM6DS33_CTRL5_config(LSM6DS33* device);
static void LSM6DS33_CTRL6_config(LSM6DS33* device);
static void LSM6DS33_CTRL7_config(LSM6DS33* device);
static void LSM6DS33_CTRL8_config(LSM6DS33* device);
static void LSM6DS33_CTRL9_config(LSM6DS33* device);
static void LSM6DS33_CTRL10_config(LSM6DS33* device);


static void LSM6DS33_FUNC_CFG_ACCESS_Config(LSM6DS33* device);

static void LSM6DS33_FIFO_CTRL_Config(LSM6DS33* device);
static void LSM6DS33_FIFO_CTRL1_Config(LSM6DS33* device);
static void LSM6DS33_FIFO_CTRL2_Config(LSM6DS33* device);
static void LSM6DS33_FIFO_CTRL3_Config(LSM6DS33* device);
static void LSM6DS33_FIFO_CTRL4_Config(LSM6DS33* device);
static void LSM6DS33_FIFO_CTRL5_Config(LSM6DS33* device);

/* TODO - Implement - */
static void LSM6DS33_ORIENT_CFG_C_Config(LSM6DS33* device);

static void LSM6DS33_INT1_CTRL_Config(LSM6DS33* device);
static void LSM6DS33_INT2_CTRL_Config(LSM6DS33* device);

static void LSM6DS33_WAKE_UP_SRC_Config(LSM6DS33* device);
static void LSM6DS33_TAP_CFG_Config(LSM6DS33* device);
static void LSM6DS33_D6D_SRC_Config(LSM6DS33* device);

/* End - TODO - */

static void LSM6DS33_read_accl(LSM6DS33* device);
static void LSM6DS33_read_FIFO(LSM6DS33* device);
static void LSM6DS33_read_gyro(LSM6DS33* device);
static void LSM6DS33_read_temp(LSM6DS33* device);
static void LSM6DS33_write_reg(LSM6DS33* device, LSM6DS33Msg* msg);
static enum LSM6DS33ResultStatus LSM6DS33_test_reg(uint8_t address,
		uint8_t reg);

/**
 * @brief:  Initialises the device based on its type and the state of the SA0
 * pin (i.e. configures what device address to use (see LSM6DS33.h).
 */
void LSM6DS33_init(LSM6DS33* device, enum LSM6DS33DeviceType type,
		enum LSM6DS33SA0State sa0) {
	switch(sa0) {
	case LSM6DS33_SA0_HIGH:
		if(LSM6DS33_test_reg(LSM6DS33_SA0_HIGH_ADDR,
				LSM6DS33_WHO_AM_I) == LSM6DS33_SUCCESS) {
			// We have confirmation that the I2C works, continue with init
			LSM6DS33_CTRL_config(device); // test
			//LSM6DS33_TAP_CFG_Config(device);
			//LSM6DS33_FIFO_CTRL_Config(device);
		} else {
			// Error
		}
		break;
	case LSM6DS33_SA0_LOW:
		if(LSM6DS33_test_reg(LSM6DS33_SA0_LOW_ADDR,
				LSM6DS33_WHO_AM_I) == LSM6DS33_SUCCESS) {
			// Todo

		} else {
			// Abort
		}
		break;
	case LSM6DS33_SA0_AUTO:
		// Todo
		break;
	default:
		break;
	}
}

/**
 * @brief Reads accelerometer, temperature and gyro data when available
 */
void LSM6DS33_read(LSM6DS33* device) {
	// Test

	uint8_t value;
	uint8_t dataSet[14];
	uint8_t reg = LSM6DS33_OUT_TEMP_L;

	// Tell device that we want to read the value of reg
	HAL_StatusTypeDef stat1 = HAL_I2C_Master_Transmit(device->hi2c,
			(device->address << 1) | LSM6DS33_READ_BIT, &reg, 1, HAL_MAX_DELAY);

	if(stat1 != HAL_OK) {
		// todo error handling
		__NOP();
	}

	// Receive the byte
	HAL_StatusTypeDef stat = HAL_I2C_Master_Receive(device->hi2c,
			(device->address << 1) | 0, dataSet, 14, HAL_MAX_DELAY);

	if(stat != HAL_OK) {
		// todo Error handling
		__NOP();
	}

	// Update temperature data
	device->temp = (dataSet[1] << 8) | (dataSet[0]);

	// Update gyro data
	device->gyroX = (dataSet[3] << 8) | (dataSet[2]);
	device->gyroY = (dataSet[5] << 8) | (dataSet[4]);
	device->gyroZ = (dataSet[7] << 8) | (dataSet[6]);

	// Update accelerometer data
	device->acclX = (dataSet[9] << 8) | (dataSet[8]);
	device->acclY = (dataSet[11] << 8) | (dataSet[10]);
	device->acclZ = (dataSet[13] << 8) | (dataSet[12]);

	// Check to see if the FIFO is configured
//	bool LSM6DS33_FIFO_ENABLED = false;
//	if(LSM6DS33_FIFO_ENABLED) {
//		LSM6DS33_read_FIFO(device);
//
//	} else {
//		// check to see if accelerometer data is available
//		bool XLDA = false;
//		bool GDA = false;
//		bool TDA = false;
//		int8_t sreg;
//		do {
//			sreg = LSM6DS33_read_reg(device, LSM6DS33_STATUS_REG);
//			if((sreg & 1) == 1)
//				XLDA = true;
//		} while( !XLDA );
//		LSM6DS33_read_accl(device);
//
//		// Now do the same for the gyro
//		do {
//			sreg = LSM6DS33_read_reg(device, LSM6DS33_STATUS_REG);
//			if(((sreg >> 1) & 1) == 1)
//				GDA = true;
//		} while( !GDA );
//		LSM6DS33_read_gyro(device);
//
//		// Now check what the temperature is
//		do {
//			sreg = LSM6DS33_read_reg(device, LSM6DS33_STATUS_REG);
//			if ( ( (sreg >> 2) & 1) == 1)
//				TDA = true;
//		} while( !TDA );
//		LSM6DS33_read_temp(device);
//	}
}

/**
 * @brief Calculates the acceleration rate based on the sensitivity setting
 * 	of the device.
 * @param device - Device that holds relevant information for computing the
 * 	result.
 */
void LSM6DS33_calcAcclRes(LSM6DS33* device) {
	// Possible values are 2g, 4g, 8g and 16g.
	// First get the scale (for now this is static)
	int8_t acclScale = LSM6DS33_ACCL_2G;
	switch(acclScale) {
	case LSM6DS33_ACCL_2G:
		break;
	case LSM6DS33_ACCL_4G:
		break;
	case LSM6DS33_ACCL_8G:
		break;
	case LSM6DS33_ACCL_16G:
		break;
	}
}

/**
 * @brief Reads the Gyro, accelerometer and temperature data (in that order)
 * 	from the fifo when there is data available.
 */
static void LSM6DS33_read_FIFO(LSM6DS33* device) {
	uint8_t status1, status2;
	uint16_t numWordsAvailable;
	while(true) {
		// Check number of unread words in FIFO
		status1 = LSM6DS33_read_reg(device, LSM6DS33_FIFO_STATUS1);
		status2 = LSM6DS33_read_reg(device, LSM6DS33_FIFO_STATUS2);

		// 12-bit
		numWordsAvailable = ((0x0F & status2) << 8) | status1;

		// Check to see if there is words available
		if(numWordsAvailable > 2) {
			// Great, read the 2 sets of data (12 bytes)
			uint8_t gyroXL = LSM6DS33_read_reg(device, LSM6DS33_FIFO_DATA_OUT_L);
			uint8_t gyroXH = LSM6DS33_read_reg(device, LSM6DS33_FIFO_DATA_OUT_H);

			uint8_t gyroYL = LSM6DS33_read_reg(device, LSM6DS33_FIFO_DATA_OUT_L);
			uint8_t gyroYH = LSM6DS33_read_reg(device, LSM6DS33_FIFO_DATA_OUT_H);

			uint8_t gyroZL = LSM6DS33_read_reg(device, LSM6DS33_FIFO_DATA_OUT_L);
			uint8_t gyroZH = LSM6DS33_read_reg(device, LSM6DS33_FIFO_DATA_OUT_H);

			uint8_t acclXL = LSM6DS33_read_reg(device, LSM6DS33_FIFO_DATA_OUT_L);
			uint8_t acclXH = LSM6DS33_read_reg(device, LSM6DS33_FIFO_DATA_OUT_H);

			uint8_t acclYL = LSM6DS33_read_reg(device, LSM6DS33_FIFO_DATA_OUT_L);
			uint8_t acclYH = LSM6DS33_read_reg(device, LSM6DS33_FIFO_DATA_OUT_H);

			uint8_t acclZL = LSM6DS33_read_reg(device, LSM6DS33_FIFO_DATA_OUT_L);
			uint8_t acclZH = LSM6DS33_read_reg(device, LSM6DS33_FIFO_DATA_OUT_H);

			// Update gyro data
			device->gyroX = (gyroXH << 8) | (gyroXL);
			device->gyroY = (gyroYH << 8) | (gyroYL);
			device->gyroZ = (gyroZH << 8) | (gyroZL);

			// Update accelerometer data
			device->acclX = (acclXH << 8) | (acclXL);
			device->acclY = (acclYH << 8) | (acclYL);
			device->acclZ = (acclZH << 8) | (acclZL);

			// Update temperature data
			LSM6DS33_read_temp(device);

			// We're done here
			break;
		} else {
			HAL_Delay(10);
		}
	}
}

/**
 * @brief Reads the temperature data off the chip device.
 * 	(required for real time calibration).
 */
static void LSM6DS33_read_temp(LSM6DS33* device) {
	tempData data;
	data.OUT_TEMP_L = LSM6DS33_read_reg(device, LSM6DS33_OUT_TEMP_L);
	data.OUT_TEMP_H = LSM6DS33_read_reg(device, LSM6DS33_OUT_TEMP_H);
	device->temp = (data.OUT_TEMP_H << 8) | (data.OUT_TEMP_L);
}

/**
 * @brief Reads the raw accelerometer data
 */
static void LSM6DS33_read_accl(LSM6DS33* device) {
	acclData data;
	// Get the x-axis accelerometer data
	data.OUTX_L_XL = LSM6DS33_read_reg(device, LSM6DS33_OUTX_L_XL);
	data.OUTX_H_XL = LSM6DS33_read_reg(device, LSM6DS33_OUTX_H_XL);

	// Get the y-axis accelerometer data
	data.OUTY_L_XL = LSM6DS33_read_reg(device, LSM6DS33_OUTY_L_XL);
	data.OUTY_H_XL = LSM6DS33_read_reg(device, LSM6DS33_OUTY_H_XL);

	// Get the z-axis accelerometer data
	data.OUTZ_L_XL = LSM6DS33_read_reg(device, LSM6DS33_OUTZ_L_XL);
	data.OUTZ_H_XL = LSM6DS33_read_reg(device, LSM6DS33_OUTZ_H_XL);

	// Combine the low and high bytes to get the word
	device->acclX = (data.OUTX_H_XL << 8) | (data.OUTX_L_XL);
	device->acclY = (data.OUTY_H_XL << 8) | (data.OUTY_L_XL);
	device->acclZ = (data.OUTZ_H_XL << 8) | (data.OUTZ_L_XL);

	// TODO: error handling

	device->status = LSM6DS33_SUCCESS;
}

/**
 *
 */
static void LSM6DS33_read_gyro(LSM6DS33* device) {
	gyroData data;
	// Get the x-axis gyroscope data
	data.OUTX_L_G = LSM6DS33_read_reg(device, LSM6DS33_OUTX_L_G);
	data.OUTX_H_G = LSM6DS33_read_reg(device, LSM6DS33_OUTX_H_G);

	// Get the y-axis gyroscope data
	data.OUTY_L_G = LSM6DS33_read_reg(device, LSM6DS33_OUTY_L_G);
	data.OUTY_H_G = LSM6DS33_read_reg(device, LSM6DS33_OUTY_H_G);

	// Get the z-axis gyroscope data
	data.OUTZ_L_G = LSM6DS33_read_reg(device, LSM6DS33_OUTZ_L_G);
	data.OUTZ_H_G = LSM6DS33_read_reg(device, LSM6DS33_OUTZ_H_G);

	// Combine the low and high bytes to get the word
	device->gyroX = (data.OUTX_H_G << 8) | data.OUTX_L_G;
	device->gyroY = (data.OUTY_H_G << 8) | data.OUTY_L_G;
	device->gyroZ = (data.OUTZ_H_G << 8) | data.OUTZ_L_G;

	// TODO: error handling

	device->status = LSM6DS33_SUCCESS;
}

/**
 * @brief:  Writes an amount of data to a specified register
 * @param:  device - Contains all the information about the LSM6DS33
 * @param:  msg - contains all the information to send
 */
static void LSM6DS33_write_reg(LSM6DS33* device, LSM6DS33Msg* msg) {

	// Create the buffer to contain the data to send
	// Register | data
	uint8_t dataToSend[msg->numData + 1];

	// Set the message to write
	dataToSend[0] = msg->reg;

	// Copy the remaining data bytes to the buffer
	for(int i = 1; i < msg->numData + 1; i++) {
		dataToSend[i] = msg->pData[i-1];
	}

	HAL_StatusTypeDef stat1 = HAL_I2C_Master_Transmit(device->hi2c,
			(device->address << 1) | LSM6DS33_WRITE_BIT,
			dataToSend, msg->numData + 1, HAL_MAX_DELAY);
	if(stat1 != HAL_OK) {
		// todo error handling
		__NOP();
	}
}

/**
 * @brief:  Reads data from a specified register
 * @param:  device - Contains all the information about the LSM6DS33
 * @param:  reg - The register to read from
 */
int8_t LSM6DS33_read_reg(LSM6DS33* device, uint8_t reg) {

	uint8_t value;
	// Tell device that we want to read the value of reg
	HAL_StatusTypeDef stat1 = HAL_I2C_Master_Transmit(device->hi2c,
			(device->address << 1) | LSM6DS33_READ_BIT, &reg, 1, HAL_MAX_DELAY);

	if(stat1 != HAL_OK) {
		// todo error handling
		__NOP();
	}

	// Receive the byte
	HAL_StatusTypeDef stat = HAL_I2C_Master_Receive(device->hi2c,
			(device->address << 1) | 0, &value, 1, HAL_MAX_DELAY);

	if(stat != HAL_OK) {
		// todo Error handling
		__NOP();
	}

	return value;
}

static void LSM6DS33_FIFO_CTRL_Config(LSM6DS33* device) {
	LSM6DS33_FIFO_CTRL1_Config(device);
	LSM6DS33_FIFO_CTRL2_Config(device);
	LSM6DS33_FIFO_CTRL3_Config(device);
	LSM6DS33_FIFO_CTRL4_Config(device);
	LSM6DS33_FIFO_CTRL5_Config(device);
}

/**
 * @brief Configures the FIFO threshold level. This is used to generate a flag
 * 	whenever the number of bytes in the FIFO will exceed the waterlevel upon
 * 	the next write.
 * 	Minimum resolution for the FIFO is 1 LSB = 2 bytes.
 * 	First 8 bits here, last 4 in FIFO_CTRL2
 */
static void LSM6DS33_FIFO_CTRL1_Config(LSM6DS33* device) {
	FIFOCTRL1Config fifo1;
	// 8kiB available.
	fifo1.FIFOThreshold = 4; // flag raised when # * 2 bytes in FIFO
	uint8_t data = 0 | (fifo1.FIFOThreshold);
	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_FIFO_CTRL1;
	msg.numData = 1;
	msg.rwBit = 0; // Write

	LSM6DS33_write_reg(device, &msg);
}

/**
 * @brief Enables the pedometer step count and time stamp, the pedometer write mode
 *  and the FIFO threshold setting for the pedometer.
 */
static void LSM6DS33_FIFO_CTRL2_Config(LSM6DS33* device) {
	/* TIMER_PEDO_FIFO_EN | TIMER_PEDO_FIFO_DRDY | 0 | 0 | FTH_11 | FTH_10 | FTH_9 | FTH_8 */
	FIFOCTRL2Config fifo2;
	fifo2.FIFOThresholdLevel = 0;
	fifo2.pedoFIFOWriteConfig = FIFO_CTRL2_ENABLE_WRITE_G_XL_DRDY; // Doesn't matter since step and time disabled
	fifo2.pedoStepAndTimeEnable = FIFO_CTRL2_DISABLE_STEP_AND_TIME;
	uint8_t data = 0;

	data = (fifo2.pedoStepAndTimeEnable << 7) | \
		   (fifo2.pedoFIFOWriteConfig << 6) | (0 << 5) | (0 << 4) | \
		   (0x0F & fifo2.FIFOThresholdLevel);

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_FIFO_CTRL2;
	msg.numData = 1;
	msg.rwBit = 0; // Write

	LSM6DS33_write_reg(device, &msg);
}

/**
 * @brief Configures the gyro and accelerometer FIFO decimation setting.
 * NOTE: For those who don't know, decimation is analogous to down-sampling/
 * compression in digital signal processing.
 * Why is decimation important?
 * ----------------------------
 * For those who don't have a degree in electrical engineering, decimation
 * is used to downsample a continuous signal. Coupling this with a low pass
 * filter, one can create a lower resolution interpretation of a signal.
 * Ok, why would I want this?
 * Downsampling is the first step in a typical DSP case. The next step
 * would be time expansion followed by interpolation.
 * By sampling theorem, a signal is unlikey to change very much between samples,
 * thus a DSP method can be used as a form of digital filtering.
 * This is important as your data stream from the gyro or accelerometer will
 * not be completely noise free.
 */
static void LSM6DS33_FIFO_CTRL3_Config(LSM6DS33* device) {
	/* 0 | 0 | DEC_FIFO_GYRO2 | DEC_FIFO_GYRO1 | DEC_FIFO_GYRO0 | DEC_FIFO_XL2 | DEC_FIFO_XL1 | DEC_FIFO_XL0 */
	FIFOCTRL3Config fifo3;
	// Gyro sampling rate must be equal to or greater than 416Hz
	fifo3.gyroDecimateConfig = GYRO_DEC_3;  // Gyro ODR = 1660Hz / 3  = 553.33Hz
	fifo3.acclDecimateConfig = GYRO_DEC_16; // Accl ODR = 6660Hz / 16 = 416.25Hz
	uint8_t data = 0;

	data = (0 << 7) | (0 << 6) | (fifo3.gyroDecimateConfig << 3) | \
			(fifo3.acclDecimateConfig);

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_FIFO_CTRL3;
	msg.numData = 1;
	msg.rwBit = 0;

	LSM6DS33_write_reg(device, &msg);
}

/**
 * @brief enables 8-bit storage in FIFO and configures pedometer decimation.
 */
static void LSM6DS33_FIFO_CTRL4_Config(LSM6DS33* device) {
	/* 0 | ONLY_HIGH_DATA | TIMER_PEDO_DEC_FIFO2 | TIMER_PEDO_DEC_FIFO1 | TIMER_PEDO_DEC_FIFO0 | 0 | 0 | 0 */
	FIFOCTRL4Config fifo4;
	fifo4.FIFODataStorageConfig = DISABLE_MSB_MEMORIZE_XL_G;
	fifo4.pedoDecimateConfig = PEDO_NOT_IN_FIFO;

	uint8_t data = 0;

	data = (0 << 7) | (fifo4.FIFODataStorageConfig << 6) | \
			(fifo4.pedoDecimateConfig << 3) | (0 << 2) | (0 << 1) | 0;

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_FIFO_CTRL4;
	msg.numData = 1;
	msg.rwBit = 0;

	LSM6DS33_write_reg(device, &msg);
}

/**
 * @brief Configures FIFO ODR selection and FIFO collection mode
 */
static void LSM6DS33_FIFO_CTRL5_Config(LSM6DS33* device) {
	/* 0 | ODR_FIFO_3 | ODR_FIFO_2 | ODR_FIFO_1 | ODR_FIFO_0 | FIFO_MODE_2 | FIFO_MODE_1 | FIFO_MODE_0 */
	FIFOCTRL5Config fifo5;
	fifo5.FIFOODRConfig = FIFO_ODR_416_HZ;
	fifo5.FIFOModeConfig = FIFO_MODE_STOP_ON_FULL;

	uint8_t data = 0;

	data = (0 << 7) | (fifo5.FIFOODRConfig << 3) | (fifo5.FIFOModeConfig);

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_FIFO_CTRL5;
	msg.numData = 1;
	msg.rwBit = 0;

	LSM6DS33_write_reg(device, &msg);
}

/**
 * @brief:  Configures the control registers. The idea here is this library
 * puts the device into a sort of 'default' mode. If you need more functionality
 * you can easily modify the library by studying the intuitive sections I've laid
 * out and reading the data sheet.
 * @param:  device - device to configure
 * @retval: none
 */
static void LSM6DS33_CTRL_config(LSM6DS33* device) {
	LSM6DS33_CTRL1_config(device);
	LSM6DS33_CTRL2_config(device);
	LSM6DS33_CTRL3_config(device);
	LSM6DS33_CTRL4_config(device);
	LSM6DS33_CTRL5_config(device);
	LSM6DS33_CTRL6_config(device);
	LSM6DS33_CTRL7_config(device);
	LSM6DS33_CTRL8_config(device);
	LSM6DS33_CTRL9_config(device);
	LSM6DS33_CTRL10_config(device);
}

/* Accelerometer configuration register (r/w) ------------------------------ */

/**
 * @brief:  Accelerometer configuration register. Used to configure the output
 * data rate, the power mode, the scale selection and the filter bandwidth
 */
static void LSM6DS33_CTRL1_config(LSM6DS33* device) {
	/* ODR_XL3 | ODR_XL2 | ODR_XL1 | ODR_XL0 | FS_XL1 | FS_XL0 | BW_XL1 | BW_XL0 */
	enum CTRL1_XL_ODR_Mode powerMode = LSM6DS33_ACCL_52HZ;
	enum CTRL1_XL_FS_XL_Mode scaleSelect = LSM6DS33_ACCL_8G;
	enum CTRL1_XL_BW_XL_Mode bandwidthSelect = LSM6DS33_ACCL_BW_50HZ;
	// 00000001 -> 00010000
	uint8_t data = 0;
	data = (powerMode << 4) | (scaleSelect << 2) | (bandwidthSelect);

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_CTRL1_XL;
	msg.numData = 1;
	msg.rwBit = 0; // Write

	LSM6DS33_write_reg(device, &msg);
}

/* Gyroscope configuration register (r/w) --------------------------------- */

/**
 * @brief:  Gyroscope configuration register. Used to configure the data rate,
 * 	and scale select
 */
static void LSM6DS33_CTRL2_config(LSM6DS33* device) {
	/* ODR_G3 | ODR_G2 | ODR_G1 | ODR_G0 | FS_G1 | FS_G0 | FS_125 | 0 */
	// Last bit in register must be zero in order for correct operation
	CTRL2Config ctrl2Reg;
	ctrl2Reg.ODRGMode = LSM6DS33_GYRO_104HZ;
	ctrl2Reg.FSGMode = LSM6DS33_GYRO_2000DPS;
	ctrl2Reg.FSG125Mode = LSM6DS33_GYRO_125_DISABLE;
	uint8_t data = 0;
	data = (ctrl2Reg.ODRGMode << 4) | \
			(ctrl2Reg.FSGMode << 2) | \
			(ctrl2Reg.FSG125Mode << 1) | 0;

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_CTRL2_G;
	msg.numData = 1;
	msg.rwBit = 0; // Write

	LSM6DS33_write_reg(device, &msg);
}

/* Control register 3 (r/w) ------------------------------------------------ */

/**
 * @brief:  Control register 3. Used to configure boot mode content,
 * 	block data update method. interrupt activation level, pp/open-drain
 * 	select for interrupt pads, SPI config, Endian-ness and reset behaviour.
 */
static void LSM6DS33_CTRL3_config(LSM6DS33* device) {
	/* BOOT | BDU | H_LACTIVE | PP_OD | SIM | IF_INC | BLE | SW_RESET */
	CTRL3Config ctrl3Reg;
	ctrl3Reg.bootMode = NORMAL_MODE;
	ctrl3Reg.bduMode = WAIT_FOR_READ;
	ctrl3Reg.intActiveMode = INT_OUT_PADS_HIGH; // Active high for int pads
	ctrl3Reg.intPadMode = PUSH_PULL_MODE;
	ctrl3Reg.simMode = FOUR_WIRE_IM; // SPI 4-wire interface

	// Register address is automatically incremented when performing
	//	multi-byte access with serial (I2C or SPI). Useful for getting
	// 16-bit words out without having to do two reads.
	ctrl3Reg.regIncMode = REG_ADDR_AUTO_INC_ENABLED; // IMPORTANT!

	// Registers in this system are ordered L, H. Hence why we're using
	// 	this configuration
	ctrl3Reg.endianMode = DATA_LSB; // LSB at lowest address
	ctrl3Reg.swResetMode = SW_NORMAL_MODE; // Software reset

	uint8_t data = 0;
	data = (ctrl3Reg.bootMode << 7) | (ctrl3Reg.bduMode << 6) |\
			(ctrl3Reg.intActiveMode << 5) | (ctrl3Reg.intPadMode << 4) |\
			(ctrl3Reg.simMode << 3) | (ctrl3Reg.regIncMode << 2) |
			(ctrl3Reg.endianMode << 1) | (ctrl3Reg.swResetMode);

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_CTRL3_C;
	msg.numData = 1;
	msg.rwBit = 0; // Write

	LSM6DS33_write_reg(device, &msg);
}

/* Control register 4 (r/w) ------------------------------------------------ */

/**
 * @brief:  Control register 4. Used to configure accl bandwidth, gyro sleep
 * 	mode, interrupt signal enable, temp data FIFO, data available timer,
 * 	serial peripheral select, and FIFO threshold level.
 */
static void LSM6DS33_CTRL4_config(LSM6DS33* device) {
	/* XL_BW_SCAL_ODR | SLEEP_G | INT2_on_INT1 | FIFO_TEMP_EN | DRDY_MASK |
	 * I2C_disable | 0 | STOP_ON_FTH */
	CTRL4Config ctrl4Reg;
	ctrl4Reg.accelBandwidthSelect = ACCL_ANALOG_FILTER_AUTO;
	ctrl4Reg.gyroSleepEN = GYRO_SLEEP_DISABLED;
	ctrl4Reg.IntSigSelect = INT_SIG_SHARE;
	ctrl4Reg.tempFIFOEnable = TEMP_DATA_DISABLE;
	ctrl4Reg.dataAvailableEN = DA_TMR_DISABLE;
	ctrl4Reg.i2cEnable = ENABLE_I2C_AND_SPI;
	ctrl4Reg.FIFOThreshold = FIFO_DEPTH_UNLIMITED;

	uint8_t data = 0;
	data = (ctrl4Reg.accelBandwidthSelect << 7) |\
				   (ctrl4Reg.gyroSleepEN << 6) |\
				   (ctrl4Reg.IntSigSelect << 5) |\
				   (ctrl4Reg.tempFIFOEnable << 4) |\
				   (ctrl4Reg.dataAvailableEN << 3) |\
				   (ctrl4Reg.i2cEnable << 2) |\
				   (0 << 1) | (ctrl4Reg.FIFOThreshold);

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_CTRL4_C;
	msg.numData = 1;
	msg.rwBit = 0; // Write

	LSM6DS33_write_reg(device, &msg);
}

/* Control register 5 (r/w) ------------------------------------------------ */

/**
 * @brief:  Control register 5. Used to configure rounding and self test
 * 	behaviour for the gyroscope and accelerometer.
 */
static void LSM6DS33_CTRL5_config(LSM6DS33* device) {
	/* ROUNDING2 | ROUNDING1 | ROUNDING0 | 0 | ST1_G | ST0_G | ST1_XL | ST0_XL */
	CTRL5Config ctrl5Reg;
	ctrl5Reg.circularBMRoundMode = NONE;
	ctrl5Reg.gyroSelfTest = GYRO_NORMAL_MODE;
	ctrl5Reg.acclSelfTest = ACCL_NORMAL_MODE;

	uint8_t data = 0;
	data = (ctrl5Reg.circularBMRoundMode << 5) | (0 << 4) | \
			(ctrl5Reg.gyroSelfTest << 2) | (ctrl5Reg.acclSelfTest);

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_CTRL5_C;
	msg.numData = 1;
	msg.rwBit = 0; // Write

	LSM6DS33_write_reg(device, &msg);
}

/* Control register 6 (r/w) ------------------------------------------------ */

/**
 * @brief: Control register 6. Used to configure gyro edge-sense trigger enable,
 * gyro data level-sense tigger enable, gyro level-senese latch enable, and accl
 * high-performance operating mode disable
 */
static void LSM6DS33_CTRL6_config(LSM6DS33* device) {
	/* TRIG_EN | LVLen | LVL2_EN | XL_HM_MODE | 0 | 0 | 0 | 0 */
	CTRL6Config ctrl6Reg;
	ctrl6Reg.gyroExternEdgeSenseTriggerEnable = EXTERN_TRIG_DISABLE;
	ctrl6Reg.gyroLevelSenseTriggerEnable = LVL_SENSE_TRIG_DISABLE;
	ctrl6Reg.gyroLevelSenseLatchEnable = LVL_SENSE_LATCH_DISABLE;
	ctrl6Reg.acclHighPerformanceMode = ACCL_HP_MODE_ENABLE;

	uint8_t data = 0;
	data = (ctrl6Reg.gyroExternEdgeSenseTriggerEnable << 7) |\
				   (ctrl6Reg.gyroLevelSenseTriggerEnable << 6) |\
				   (ctrl6Reg.gyroLevelSenseLatchEnable << 5) |\
				   (ctrl6Reg.acclHighPerformanceMode << 4) |\
				   (0 << 3) | (0 << 2) | (0 << 1) | 0;

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_CTRL6_C;
	msg.numData = 1;
	msg.rwBit = 0; // Write;

	LSM6DS33_write_reg(device, &msg);
}

/* Gyro sensor control register 7 (r/w) ------------------------------------ */

/**
 * @brief: Control register 7. Used to configure gyro high-performance mode,
 * 	gyro HPF enable, gyro digital filter reset, source register rounding
 * 	function enable, and HPF cutoff freq select.
 */
static void LSM6DS33_CTRL7_config(LSM6DS33* device) {
	/* G_HM_MODE | HP_G_EN | HPCF_G1 | HPCF_G0 | HP_G_RST | ROUNDING_STATUS | 0 | 0 */
	CTRL7Config ctrl7Reg;
	ctrl7Reg.gyroHighPerformanceMode = GYRO_HP_MODE_ENABLE;
	ctrl7Reg.gyroHPFEnable = HPF_DISABLE; // Disabled because HPF ruins long slow movement accuracy
	ctrl7Reg.gyroHPFCutoffFreq = HPF_CUTOFF_8_1mHZ;
	ctrl7Reg.gyroHPFReset = HPF_RESET_OFF;
	ctrl7Reg.srcRegisterRoundingEnable = SOURCE_REG_ROUNDING_DISABLE;

	uint8_t data = 0;
	data = (ctrl7Reg.gyroHighPerformanceMode << 7) |\
				   (ctrl7Reg.gyroHPFEnable << 6) |\
				   (ctrl7Reg.gyroHPFCutoffFreq << 4) |\
				   (ctrl7Reg.gyroHPFReset << 3) |\
				   (ctrl7Reg.srcRegisterRoundingEnable << 2) |\
				   (0 << 1) | 0;

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_CTRL7_G;
	msg.numData = 1;
	msg.rwBit = 0; // Write

	LSM6DS33_write_reg(device, &msg);
}

/* Accelerometer sensor control register 8 (r/w) --------------------------- */

/**
 * @brief:  Control register 8. Used to configure the accl LPF, the accl
 * 	HPF and Slope filter + corresponding cutoff freq settings, and the
 * 	LPF on the 6D functions.
 * 	REMEMBER: A high pass filter is used to remove the gravity vector, while
 * 	a low pass filter can be used to isolate it
 */
static void LSM6DS33_CTRL8_config(LSM6DS33* device) {
	/* LPF2_XL_EN | HPCF_XL1 | HPCF_XL0 | 0 | 0 | HP_SLOPE_XL_EN | 0 | LOW_PASS_ON_6D */
	CTRL8Config ctrl8Reg;
	ctrl8Reg.acclLPF2Selec = LPF_ENABLE;
	ctrl8Reg.acclHPFAndSFConfig = HPF_HP_ODR_XL_DIV_400; // Not used since LPF enabled
	ctrl8Reg.acclHPFSelec = HP_SLOPE_ENABLE;
	ctrl8Reg.acclLPFOn6D = LP_ON_6D_DISABLE;

	uint8_t data = 0;
	data = (ctrl8Reg.acclLPF2Selec << 7) | \
				   (ctrl8Reg.acclHPFAndSFConfig << 5) | \
				   (0 << 4) | (0 << 3) | (ctrl8Reg.acclHPFSelec << 2) | \
				   (0 << 1) | ctrl8Reg.acclLPFOn6D;

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_CTRL8_XL;
	msg.numData = 1;
	msg.rwBit = 0; // Write

	LSM6DS33_write_reg(device, &msg);
}

/* Accelerometer sensor control register 9 (r/w) --------------------------- */

/**
 * @brief:  Control register 9. Used to enable the different axes
 * 	of the accelerometer for logging data.
 */
static void LSM6DS33_CTRL9_config(LSM6DS33* device) {
	/* 0 | 0 | Zen_XL | Yen_XL | Xen_XL | 0 | 0 | 0 */
	CTRL9Config ctrl9Reg;
	ctrl9Reg.acclZAxisEnable = ACCL_Z_AXIS_OUTPUT_ENABLE;
	ctrl9Reg.acclYAxisEnable = ACCL_Y_AXIS_OUTPUT_ENABLE;
	ctrl9Reg.acclXAxisEnable = ACCL_X_AXIS_OUTPUT_ENABLE;

	uint8_t data = 0;
	data = (0 << 7) | (0 << 6) | (ctrl9Reg.acclZAxisEnable << 5) | \
			(ctrl9Reg.acclYAxisEnable << 4) | (ctrl9Reg.acclXAxisEnable << 3) | \
			(0 << 2) | (0 << 1) | 0;

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_CTRL9_XL;
	msg.numData = 1;
	msg.rwBit = 0; // Write

	LSM6DS33_write_reg(device, &msg);
}

/* Control register 10 (r/w) ----------------------------------------------- */

/**
 * @brief: Control register 10. Used to enable the various axes of the gyroscope,
 * enable embedded functions such as the pedometer, tile and significant motion,
 * the accelerometer HP and LPF2 filters, and reset the pedometer step counter.
 */
static void LSM6DS33_CTRL10_config(LSM6DS33* device) {
	/* 0 | 0 | Zen_G | Yen_G | Xen_G | FUNC_EN | PEDO_RST_STEP | SIGN_MOTION_EN */
	CTRL10Config ctrl10Reg;
	ctrl10Reg.gyroZAxisEnable = GYRO_Z_AXIS_OUTPUT_ENABLE;
	ctrl10Reg.gyroYAxisEnable = GYRO_Y_AXIS_OUTPUT_ENABLE;
	ctrl10Reg.gyroXAxisEnable = GYRO_X_AXIS_OUTPUT_ENABLE;
	ctrl10Reg.embedFunctEnable = EMBED_FUNCT_ENABLE; //
	ctrl10Reg.stepCounterReset = RST_STEP_COUNT_DISABLE;
	ctrl10Reg.signMotionDetEnable = SIGN_MOTION_DETECT_DISABLE;

	uint8_t data = 0;
	data = (0 << 7) | (0 << 6) | \
			(ctrl10Reg.gyroZAxisEnable << 5) | \
			(ctrl10Reg.gyroYAxisEnable << 4) | \
			(ctrl10Reg.gyroXAxisEnable << 3) | \
			(ctrl10Reg.embedFunctEnable << 2) | \
			(ctrl10Reg.stepCounterReset << 1) | \
			(ctrl10Reg.signMotionDetEnable);

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_CTRL10_C;
	msg.numData = 1;
	msg.rwBit = 0; // Write

	LSM6DS33_write_reg(device, &msg);
}

/**
 * @brief Time-stamp, pedometer, tilt and filtering config functions.
 */
static void LSM6DS33_TAP_CFG_Config(LSM6DS33* device) {
	/* TIMER_EN | PEDO_EN | TILT_EN | SLOPE_FDS | TAP_X_EN | TAP_Y_EN | TAP_Z_EN | LIR */
	TAPCFGConfigurationRegister TAPCFGConfig;
	TAPCFGConfig.timeStampCountEnable = TIME_STAMP_COUNT_DISABLED;
	TAPCFGConfig.pedometerAlgoEnable = PEDOMETER_ALGO_DISABLED;
	TAPCFGConfig.tiltCalcEnable = TIME_CALC_DISABLE;
	TAPCFGConfig.acclFilterTypeEnable = ACCL_HP_LPF2_ENABLE;
	TAPCFGConfig.xAxisTapRecognition = TAP_DET_X_AXIS_DISABLE;
	TAPCFGConfig.yAxisTapRecognition = TAP_DET_Y_AXIS_DISABLE;
	TAPCFGConfig.zAxisTapRecognition = TAP_DET_Z_AXIS_DISABLE;
	TAPCFGConfig.latchInterrupt = INT_REQ_NOT_LATCHED;

	uint8_t data = 0;
	data = (TAPCFGConfig.timeStampCountEnable << 7) | \
		   (TAPCFGConfig.pedometerAlgoEnable << 6) |
		   (TAPCFGConfig.tiltCalcEnable << 5) |
		   (TAPCFGConfig.acclFilterTypeEnable << 4) |
		   (TAPCFGConfig.xAxisTapRecognition << 3) |
		   (TAPCFGConfig.yAxisTapRecognition << 2) |
		   (TAPCFGConfig.zAxisTapRecognition << 1) |
		   (TAPCFGConfig.latchInterrupt);

	LSM6DS33Msg msg;
	msg.pData = &data;
	msg.reg = LSM6DS33_TAP_CFG;
	msg.numData = 1;
	msg.reg = 0; // Write

	LSM6DS33_write_reg(device, &msg);
}

/**
 * @brief:  Tests the device to see if we can communicate with the LSM6DS33
 * @param:  address - device address on the I2C bus we wish to communicate with
 * @param:  reg     - register of the device we wish to write to
 * @retval: Upon success, function will return LSM6DS33_SUCCESS, otherwise will
 * 		return LSM6DS33_ERROR
 */
static enum LSM6DS33ResultStatus LSM6DS33_test_reg(uint8_t address,
		uint8_t reg) {
	uint8_t value;
	uint8_t pData = LSM6DS33_WHO_AM_I;
	// Tell the LSM6DS33 that we want to read from its WHO_AM_I register
	HAL_StatusTypeDef stat1 = HAL_I2C_Master_Transmit(&hi2c1, (address << 1) | 1, &pData, 1, HAL_MAX_DELAY);
	if(stat1 != HAL_OK) {
		// todo error handling
		__NOP();
	}

	// receive the byte of data (it should be 0x69, heh)
	HAL_StatusTypeDef stat = HAL_I2C_Master_Receive(&hi2c1, (address << 1) | 0, &value, 1, HAL_MAX_DELAY);
	if(stat == HAL_OK && value == LSM6DS33_WHO_AM_I_VALUE) {
		return LSM6DS33_SUCCESS;
	} else {
		return LSM6DS33_ERROR;
	}
}
