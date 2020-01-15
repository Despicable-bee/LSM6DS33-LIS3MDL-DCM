#ifndef DCM_H
#define DCM_H

/* Includes ---------------------------------------------------------------- */

#include "LSM6DS33.h"
#include "LIS3MDL.h"

/* Definitions ------------------------------------------------------------- */

#define LSM6DS33_ACCL_8G_SENSITIVITY 256

#define ACCEL_SENSITIVITY LSM6DS33_ACCL_8G_SENSITIVITY

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

#define GYRO_GAIN_X 0.07 // Sensitivity of the gyro
#define GYRO_GAIN_Y 0.07
#define GYRO_GAIN_Z 0.07

#define ToRad(x) ((x) * 0.01745329252) // * pi / 180
#define ToDeg(x) ((x) * 57.2957795131) // * 180 / pi

#define GYRO_SCALED_X(x) ((x) * ToRad(GYRO_GAIN_X))
#define GYRO_SCALED_Y(x) ((x) * ToRad(GYRO_GAIN_Y))
#define GYRO_SCALED_Z(x) ((x) * ToRad(GYRO_GAIN_Z))

#define DRIFT_CORRECTION_ENABLE

#define DCM_INTEGRAL_TIME 0.02 // 50Hz

#define OFFSET_SAMPLE_NUMBER 32

// LIS3MDL calibration constants
#define M_X_MIN -1000
#define M_Y_MIN -1000
#define M_Z_MIN -1000
#define M_X_MAX +1000
#define M_Y_MAX +1000
#define M_Z_MAX +1000

/* Type definitions -------------------------------------------------------- */
typedef struct {
	float* vectorOut; // 3 x 1 vector
	float* vectorIn; // 3 x 1 vector
	float scale;
} vectorScaleStruct;

typedef struct {
	float* vectorOut; // 3 x 1 vector
	float* v1; // 3 x 1 vector
	float* v2; // 3 x 1 vector
} vectorCrossStruct;

typedef struct {
	float* vector1; // 3 x 1 vector
	float* vector2; // 3 x 1 vector
} vectorDotStruct;

typedef struct {
	float* vectorOut; // 3 x 1 vector
	float* vectorIn1; // 3 x 1 vector
	float* vectorIn2; // 3 x 1 vector
} vectorAddStruct;

typedef struct {
	float DCM_Matrix[3][3];
	float updateMatrix[3][3];
	float temporaryMatrix[3][3];
	float G_Dt; // Integration time (will run at 50Hz if possible)
} MatrixData;

typedef struct {
	float accelVector[3]; // Store acceleration in vector
	float gyroVector[3]; // Store gyro turn rate in vector
	float omegaVector[3]; // Corrected gyroVector data
	float omegaP[3]; // Omega proporational correction
	float omegaI[3]; // Omega integrator
	float omega[3];
} vectorCorrectionData;

typedef struct {
	float roll;
	float pitch;
	float yaw;
	float errorRollPitch[3];
	float errorYaw[3];
} eulerAngleData;

typedef struct {
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

	int16_t accl_x;
	int16_t accl_y;
	int16_t accl_z;

	int16_t magn_x;
	int16_t magn_y;
	int16_t magn_z;

	int8_t sensor_sign[9];

	int32_t sensor_offset[6];
} sensorDataStruct;

typedef struct {
	float c_magn_x;
	float c_magn_y;
	float c_magn_z;
	float MAG_heading;
} compassHeadingData;

typedef struct {
	MatrixData matrixData;
	vectorCorrectionData correctionData;
	eulerAngleData angleData;
	sensorDataStruct sensorData;
	compassHeadingData compassData;
	int timer;
	int timer_old;
	uint8_t counter;
} DCMStruct;

/* Function prototypes ----------------------------------------------------- */

void dcm_init(DCMStruct* DCMData, LSM6DS33* deviceData);

void normalize(MatrixData* matrix);
void drift_correction(vectorCorrectionData* correctionData,
		eulerAngleData* angleData,
		MatrixData* matrixData,
		compassHeadingData* compassData);
void to_euler_angles(MatrixData* matrixData, eulerAngleData* angleData);
void compass_heading(eulerAngleData* angleData,
		compassHeadingData* compassData,
		sensorDataStruct* sensorData);
void matrix_update(MatrixData* matrixData,
		vectorCorrectionData* correctionData,
		sensorDataStruct* sensorData);

float vector_dot_product(vectorDotStruct* data);
void vector_cross_product(vectorCrossStruct* data);
void vector_scale(vectorScaleStruct* data);
void vector_add(vectorAddStruct* data);
void matrix_multiply(MatrixData* matrix);
float constrain(float x, float a, float b);

void dcm_read_gyro(LSM6DS33* deviceData, DCMStruct* dcmData);
void dcm_read_accl(LSM6DS33* deviceData, DCMStruct* dcmData);
void dcm_read_magn(LIS3MDL* deviceData, DCMStruct* dcmData);

void dcm_loop(DCMStruct* DCMData, LSM6DS33* gyroAcclData,
		LIS3MDL * magnData);


#endif
