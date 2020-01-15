/* License Information ----------------------------------------------------- */
// TODO
/* Includes ---------------------------------------------------------------- */

#include "DCM.h"

/* Function prototypes ----------------------------------------------------- */
void init_matrix_data(MatrixData* matrix);
void init_correction_data(vectorCorrectionData* data);
void init_euler_angle_data(eulerAngleData* data);
void init_sensor_data(sensorDataStruct* sensorData);

/**
 * @brief normalises the DCM_matrix data
 */
void normalize(MatrixData* matrix) {
	float error = 0;
	float temporary[3][3];
	float renorm = 0;

	// Calculate the error
	vectorDotStruct dotStruct;
	dotStruct.vector1 = &(matrix->DCM_Matrix[0][0]);
	dotStruct.vector2 = &(matrix->DCM_Matrix[1][0]);

	error = -1 * vector_dot_product(&dotStruct) * 0.5;

	// Rescale by the error
	vectorScaleStruct scaleStruct;
	scaleStruct.vectorOut = &(temporary[0][0]);
	scaleStruct.vectorIn = &(matrix->DCM_Matrix[1][0]);
	scaleStruct.scale = error;

	vector_scale(&scaleStruct);

	scaleStruct.vectorOut = &(temporary[1][0]);
	scaleStruct.vectorIn = &(matrix->DCM_Matrix[0][0]);

	vector_scale(&scaleStruct);

	// Add the original and rescaled vectors together
	vectorAddStruct addStruct;
	addStruct.vectorOut = &(temporary[0][0]);
	addStruct.vectorIn1 = &(temporary[0][0]);
	addStruct.vectorIn2 = &(matrix->DCM_Matrix[0][0]);

	vector_add(&addStruct);

	addStruct.vectorOut = &(temporary[1][0]);
	addStruct.vectorIn1 = &(temporary[1][0]);
	addStruct.vectorIn2 = &(matrix->DCM_Matrix[1][0]);

	vector_add(&addStruct);

	// Get a cross product of the two scaled and added vectors (find the orthogonal
	// vector).

	vectorCrossStruct crossStruct;
	crossStruct.vectorOut = &(temporary[2][0]);
	crossStruct.v1 = &(temporary[0][0]);
	crossStruct.v2 = &(temporary[1][0]);

	vector_cross_product(&crossStruct);

	// Now re-normalize everything
	for(int i = 0; i < 3; i++) {
		dotStruct.vector1 = &(temporary[i][0]);
		dotStruct.vector2 = &(temporary[i][0]);
		renorm = 0.5 * (3 - vector_dot_product(&dotStruct));

		scaleStruct.vectorOut = &(matrix->DCM_Matrix[i][0]);
		scaleStruct.vectorIn = &(temporary[i][0]);
		scaleStruct.scale = renorm;
		vector_scale(&scaleStruct);
	}
}

/**
 * @brief Reads the data from the gyro and pushes it to the
 * 	sensorData struct
 */
void dcm_read_gyro(LSM6DS33* deviceData, DCMStruct* dcmData) {
	// Gyro x
	dcmData->sensorData.gyro_x = dcmData->sensorData.sensor_sign[0] * \
			(deviceData->gyroX - dcmData->sensorData.sensor_offset[0]);

	// Gyro y
	dcmData->sensorData.gyro_y = dcmData->sensorData.sensor_sign[1] * \
				(deviceData->gyroY - dcmData->sensorData.sensor_offset[1]);

	// Gyro Z
	dcmData->sensorData.gyro_z = dcmData->sensorData.sensor_sign[2] * \
				(deviceData->gyroZ - dcmData->sensorData.sensor_offset[2]);
}

/**
 * @brief Reads the data from the accelerometer and pushes it to the sensorData
 * 	struct
 * NOTE: we might need to shift the accelerometer data back 4 bits
 *  ( >> 4 ).
 */
void dcm_read_accl(LSM6DS33* deviceData, DCMStruct* dcmData) {
	// Accelerometer X
	dcmData->sensorData.accl_x = dcmData->sensorData.sensor_sign[3] * \
			((deviceData->acclX >> 4) - dcmData->sensorData.sensor_offset[3]);
	// Accelerometer Y
	dcmData->sensorData.accl_y = dcmData->sensorData.sensor_sign[4] * \
			((deviceData->acclY >> 4) - dcmData->sensorData.sensor_offset[4]);
	// Accelerometer Z
	dcmData->sensorData.accl_z = dcmData->sensorData.sensor_sign[5] * \
			((deviceData->acclZ >> 4) - dcmData->sensorData.sensor_offset[5]);
}

/**
 * @brief Reads the data from the magnetometer and pushes it to the
 * sensorData struct
 */
void dcm_read_magn(LIS3MDL* deviceData, DCMStruct* dcmData) {
	// Magnetometer X
	dcmData->sensorData.magn_x = dcmData->sensorData.sensor_sign[6] * \
			deviceData->magX;

	// Magnetometer Y
	dcmData->sensorData.magn_y = dcmData->sensorData.sensor_sign[7] * \
			deviceData->magY;

	// Magnetometer Z
	dcmData->sensorData.magn_z = dcmData->sensorData.sensor_sign[8] * \
			deviceData->magZ;
}

/**
 * @brief A PI controller used for correcting drift.
 */
void drift_correction(vectorCorrectionData* correctionData,
		eulerAngleData* angleData,
		MatrixData* matrixData,
		compassHeadingData* compassData) {
	float magHeadingX;
	float magHeadingY;
	float errorCourse;
	// Compensate the roll, pitch and Yaw drift
	static float scaledOmegaP[3];
	static float scaledOmegaI[3];
	float accelMagnitude;
	float accelWeight;

	/* ------------- Roll and Pitch -------------- */
	// Calculate the magnitude of the accelerometer vector
	accelMagnitude = sqrt(correctionData->accelVector[0] * correctionData->accelVector[0] + \
			correctionData->accelVector[1] * correctionData->accelVector[1] + \
			correctionData->accelVector[2] * correctionData->accelVector[2]);
	accelMagnitude = accelMagnitude / ACCEL_SENSITIVITY; // Scale to get G's
	// Apply dynamic weighting of accelerometer info (PI control gain dynamic gain turning)
	// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0, >1.5G = 0.0)
	// Which in english just means that the if the reading is a standard deviation of 0.5G
	// away from the mean (which in this case is 1), the reading is zero. All this does is
	// eliminate weird spikes and noise (like when the sensor is placed perfectly vertical,
	// you might get some tiny readings that mess with your algorithm.
	accelWeight = constrain(1 - 2 * fabsf(1 - accelMagnitude),0,1); // So it linearly sweeps
									// the weight between 0 and 1G's

	vectorCrossStruct crossStruct;
	crossStruct.vectorOut = &(angleData->errorRollPitch[0]);
	crossStruct.v1 = &(correctionData->accelVector[0]);
	crossStruct.v2 = &(matrixData->DCM_Matrix[2][0]);

	// Here we're getting the perpendicular vector from the new accelVector and the
	// previous matrix vector, which will give us the ground reference vector
	vector_cross_product(&crossStruct); // adjust the ground reference

	vectorScaleStruct scaleStruct;
	scaleStruct.vectorOut = &(correctionData->omegaP[0]);
	scaleStruct.vectorIn = &(angleData->errorRollPitch[0]);
	scaleStruct.scale = Kp_ROLLPITCH * accelWeight;

	// Adjust the scale pf the error vector based on the proportional gain times the weighting.
	// If the change is super large, then we assume there is no error (or we can't quantify it).
	vector_scale(&scaleStruct);

	scaleStruct.vectorOut = &(scaledOmegaI[0]);
	scaleStruct.vectorIn = &(angleData->errorRollPitch[0]);
	scaleStruct.scale = Ki_ROLLPITCH * accelWeight; // No integral if accel is zero.

	vector_scale(&scaleStruct);

	vectorAddStruct addStruct;
	addStruct.vectorOut = correctionData->omegaI;
	addStruct.vectorIn1 = correctionData->omegaI;
	addStruct.vectorIn2 = scaledOmegaI;

	vector_add(&addStruct);

	/* ----------------- Yaw ------------------ */
	// We make the gyro YAW drift correction based on compass and magnetic heading

	magHeadingX = cos(compassData->MAG_heading);
	magHeadingY = sin(compassData->MAG_heading);

	errorCourse = (matrixData->DCM_Matrix[0][0] * magHeadingY) - \
			(matrixData->DCM_Matrix[1][0] * magHeadingX); // Calculating YAW error

	scaleStruct.vectorOut = angleData->errorYaw;
	scaleStruct.vectorIn = &(matrixData->DCM_Matrix[2][0]);
	scaleStruct.scale = errorCourse;
	// Applies the yaw correction to the XYZ rotation of the vehicle, depending on the position
	vector_scale(&scaleStruct);

	scaleStruct.vectorOut = scaledOmegaP;
	scaleStruct.vectorIn = angleData->errorYaw;
	scaleStruct.scale = Kp_YAW;

	// 0.01 proportion of the YAW
	vector_scale(&scaleStruct);

	addStruct.vectorOut = correctionData->omegaP;
	addStruct.vectorIn1 = correctionData->omegaP;
	addStruct.vectorIn2 = scaledOmegaP;

	// Adding proportional constant
	vector_add(&addStruct);

	scaleStruct.vectorOut = scaledOmegaI;
	scaleStruct.vectorIn = angleData->errorYaw;
	scaleStruct.scale = Ki_YAW;

	// 0.0001 Integrator term
	vector_scale(&scaleStruct);

	addStruct.vectorOut = correctionData->omegaI;
	addStruct.vectorIn1 = correctionData->omegaI;
	addStruct.vectorIn2 = scaledOmegaI;

	// adding integrator to the omegaI
	vector_add(&addStruct);
}


void to_euler_angles(MatrixData* matrixData, eulerAngleData* angleData) {
	angleData->pitch = -1 * asin(matrixData->DCM_Matrix[2][0]);
	angleData->roll = atan2(matrixData->DCM_Matrix[2][1],
			matrixData->DCM_Matrix[2][2]);
	angleData->yaw = atan2(matrixData->DCM_Matrix[1][0], matrixData->DCM_Matrix[0][0]);
}

/**
 * @brief Generates the magnetic heading based on the roll and pitch of the
 */
void compass_heading(eulerAngleData* angleData,
		compassHeadingData* compassData,
		sensorDataStruct* sensorData) {
	float MAG_X, MAG_Y, cos_roll, sin_roll, cos_pitch, sin_pitch;

	cos_roll = cos(angleData->roll);
	sin_roll = sin(angleData->roll);
	cos_pitch = cos(angleData->pitch);
	sin_pitch = sin(angleData->pitch);

	// adjust for the LSM6DS33 compass axis offsets/sensitivity differences
	// by scaling to +/- 0.5 range
	compassData->c_magn_x = (float)(sensorData->magn_x - \
			sensorData->sensor_sign[6] * M_X_MIN) / (M_X_MAX - M_X_MIN) - \
			sensorData->sensor_sign[6] * 0.5;

	compassData->c_magn_y = (float)(sensorData->magn_y - \
			sensorData->sensor_sign[7] * M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - \
			sensorData->sensor_sign[7] * 0.5;

	compassData->c_magn_z = (float)(sensorData->magn_z - \
			sensorData->sensor_sign[8] * M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - \
			sensorData->sensor_sign[8] * 0.5;

	// Tilt compensated magnetic field X:
	MAG_X = compassData->c_magn_x * cos_pitch + \
			compassData->c_magn_y * sin_roll * sin_pitch + \
			compassData->c_magn_z * cos_roll * sin_pitch;

	// Tilt compensation magnetic field Y:
	MAG_Y = compassData->c_magn_y * cos_roll - compassData->c_magn_z * sin_roll;

	// Magnetic heading
	compassData->MAG_heading = atan2(-MAG_Y, MAG_X);
}

/**
 * @brief Master initialisation function for the DCM algorithm.
 * NOTE: Device must be initialized with the z axis pointing up
 */
void dcm_init(DCMStruct* DCMData, LSM6DS33* deviceData) {
	init_matrix_data(&(DCMData->matrixData));
	init_correction_data(&(DCMData->correctionData));
	init_euler_angle_data(&(DCMData->angleData));
	init_sensor_data(&(DCMData->sensorData));

	// Compute the offset
	for(int i = 0; i < OFFSET_SAMPLE_NUMBER; i ++) {
		// Read the accelerometer and gyro
		LSM6DS33_read(deviceData);
		DCMData->sensorData.sensor_offset[0] += (int32_t)deviceData->gyroX;
		DCMData->sensorData.sensor_offset[1] += (int32_t)deviceData->gyroY;
		DCMData->sensorData.sensor_offset[2] += (int32_t)deviceData->gyroZ;
		DCMData->sensorData.sensor_offset[3] += (int32_t)deviceData->acclX;
		DCMData->sensorData.sensor_offset[4] += (int32_t)deviceData->acclY;
		DCMData->sensorData.sensor_offset[5] += (int32_t)deviceData->acclZ;
		HAL_Delay(20); // Wait 20ms between samples
	}

	// Average the cumulative values
	for(int i = 0; i < 6; i++) {
		DCMData->sensorData.sensor_offset[i] = \
				DCMData->sensorData.sensor_offset[i]/OFFSET_SAMPLE_NUMBER;
	}

	// Subtract 1G from the z-axis of the accelerometer
	DCMData->sensorData.sensor_offset[5] -= ACCEL_SENSITIVITY * \
			DCMData->sensorData.sensor_sign[5];

	DCMData->counter = 0;
	DCMData->timer_old = 0;
	DCMData->timer = 0;

}

/**
 * @brief Main loop for the DCM algorithm
 */
void dcm_loop(DCMStruct* DCMData, LSM6DS33* gyroAcclData, LIS3MDL* magnData) {

	// Gyro integration time


	// DCM algorithm
	LSM6DS33_read(gyroAcclData); // Get new gyro and accelerometer data
	dcm_read_gyro(gyroAcclData, DCMData); // push to sensor data struct
	dcm_read_accl(gyroAcclData, DCMData);

	if( DCMData->counter > 5) { // read compass data (at 10Hz)
		DCMData->counter = 0;
		LIS3MDL_read_mag(magnData); // Get new magnetometer data
		dcm_read_magn(magnData, DCMData); // push to sensor data struct
		compass_heading(&(DCMData->angleData), &(DCMData->compassData),
				&(DCMData->sensorData)); // calculate magnetic heading
	} else {
		DCMData->counter++;
	}

	// Calculation...
	matrix_update(&(DCMData->matrixData), &(DCMData->correctionData), &(DCMData->sensorData));

	normalize(&(DCMData->matrixData));

	drift_correction(&(DCMData->correctionData), &(DCMData->angleData),
			&(DCMData->matrixData), &(DCMData->compassData));

	to_euler_angles(&(DCMData->matrixData), &(DCMData->angleData));
}

/**
 * @brief initializes the sensor data sign (rest orientation) and the
 * 	sensor readings
 */
void init_sensor_data(sensorDataStruct* sensorData) {
	for(int i = 0; i < 9; i++) {
		if(i >= 3 && i <= 5)
			sensorData->sensor_sign[i] = -1;
		else
			sensorData->sensor_sign[i] = 1;
	}
	sensorData->accl_x = 0;
	sensorData->accl_y = 0;
	sensorData->accl_z = 0;
	sensorData->gyro_x = 0;
	sensorData->gyro_y = 0;
	sensorData->gyro_z = 0;
	sensorData->magn_x = 0;
	sensorData->magn_y = 0;
	sensorData->magn_z = 0;
}

/**
 * @brief
 */
void matrix_update(MatrixData* matrixData,
		vectorCorrectionData* correctionData,
		sensorDataStruct* sensorData) {
	// Push new sensor data into correction vector struct
	correctionData->gyroVector[0] = GYRO_SCALED_X(sensorData->gyro_x); // gyro x roll
	correctionData->gyroVector[1] = GYRO_SCALED_Y(sensorData->gyro_y); // gyro y pitch
	correctionData->gyroVector[2] = GYRO_SCALED_Z(sensorData->gyro_z); // gyro z yaw

	correctionData->accelVector[0] = sensorData->accl_x;
	correctionData->accelVector[1] = sensorData->accl_y;
	correctionData->accelVector[2] = sensorData->accl_z;

	vectorAddStruct addStruct;
	addStruct.vectorOut = &(correctionData->omega[0]);
	addStruct.vectorIn1 = &(correctionData->gyroVector[0]);
	addStruct.vectorIn2 = &(correctionData->omegaI[0]);

	vector_add(&addStruct); // Adding proportional term

	addStruct.vectorOut = &(correctionData->omegaVector[0]);
	addStruct.vectorIn1 = &(correctionData->omega[0]);
	addStruct.vectorIn2 = &(correctionData->omegaP[0]);

	vector_add(&addStruct); // adding integral term

#ifdef DRIFT_CORRECTION_ENABLE
	matrixData->updateMatrix[0][0] = 0;
	matrixData->updateMatrix[0][1] = -1 * matrixData->G_Dt * \
									correctionData->omegaVector[2]; // -z
	matrixData->updateMatrix[0][2] = matrixData->G_Dt * \
									correctionData->omegaVector[1]; // y
	matrixData->updateMatrix[1][0] = matrixData->G_Dt * \
									correctionData->omegaVector[2]; // z
	matrixData->updateMatrix[1][1] = 0;
	matrixData->updateMatrix[1][2] = -1 * matrixData->G_Dt * \
									correctionData->omegaVector[0]; // -x
	matrixData->updateMatrix[2][0] = -1 * matrixData->G_Dt * \
									correctionData->omegaVector[1]; // -y
	matrixData->updateMatrix[2][1] = matrixData->G_Dt * \
									correctionData->omegaVector[0]; // x
	matrixData->updateMatrix[2][2] = 0;
#else
	// TODO
#endif
	matrix_multiply(matrixData);; // a * b = c

	for(int x = 0; x < 3; x++) {
		for(int y = 0; y < 3; y++) {
			matrixData->DCM_Matrix[x][y] += matrixData->temporaryMatrix[x][y];
		}
	}
}

/**
 * @brief A rubbish implementation of the constain function used in the arduino
 * library
 */
float constrain(float x, float a, float b) {
	if(a < x && x < b)
		return x;
	else if(x < a)
		return a;
	else
		return b;
}

/**
 * @brief initialises all the data involved with the drift correction
 * function.
 */
void init_correction_data(vectorCorrectionData* data) {
	for(int i = 0; i < 3; i++) {
		data->accelVector[i] = 0;
		data->gyroVector[i] = 0;
		data->omega[i] = 0;
		data->omegaP[i] = 0;
		data->omegaI[i] = 0;
		data->omegaVector[i] = 0;
	}
}

/**
 * @brief initializes the dcm matrix struct with a normalised diagonal matrix
 */
void init_matrix_data(MatrixData* matrix) {
	float counter = 0;
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			if(i == j)
				matrix->DCM_Matrix[i][j] = 1;
			else
				matrix->DCM_Matrix[i][j] = 0;
			matrix->updateMatrix[i][j] = counter;
			matrix->temporaryMatrix[i][j] = 0;
			counter++;
		}
	}
	matrix->G_Dt = DCM_INTEGRAL_TIME;
}

/**
 * @brief Performs a 3x3 matrix multiplication
 */
void matrix_multiply(MatrixData* matrix) {
	for(int x = 0; x < 3; x++) {
		for(int y = 0; y < 3; y++) {
			matrix->temporaryMatrix[x][y] = 0;
			for(int w = 0; w < 3; w++) {
				matrix->temporaryMatrix[x][y] += (matrix->DCM_Matrix[x][w]) * \
						(matrix->updateMatrix[w][y]);
			}
		}
	}
}

/**
 * @brief initializes the euler angle data.
 */
void init_euler_angle_data(eulerAngleData* data) {
	data->pitch = 0;
	data->roll = 0;
	data->yaw = 0;
	for(int i = 0; i < 3; i++) {
		data->errorRollPitch[i] = 0;
		data->errorYaw[i] = 0;
	}
}

/**
 * @brief Performs a dot product operation between two 3 x 1 vectors.
 */
float vector_dot_product(vectorDotStruct* data) {
	float op = 0;

	for(int i = 0; i < 3; i++) {
		op += data->vector1[i] * data->vector2[i];
	}

	return op;
}

/**
 * @brief Performs a cross product operation between two 3 x 1 vectors.
 */
void vector_cross_product(vectorCrossStruct* data) {
	data->vectorOut[0] = (data->v1[1] * data->v2[2]) - (data->v1[2] * data->v2[1]);
	data->vectorOut[1] = (data->v1[2] * data->v2[0]) - (data->v1[0] * data->v2[2]);
	data->vectorOut[2] = (data->v1[0] * data->v2[1]) - (data->v1[1] * data->v2[0]);
}

/**
 * @brief Multiplies the vector by a scalar
 */
void vector_scale(vectorScaleStruct* data) {
	for(int i = 0; i < 3; i++) {
		data->vectorOut[i] = data->vectorIn[i] * data->scale;
	}
}

/**
 * @brief Adds two vectors
 */
void vector_add(vectorAddStruct* data) {

	for(int i = 0; i < 3; i++) {
		data->vectorOut[i] = data->vectorIn1[i] + data->vectorIn2[i];
	}
}
