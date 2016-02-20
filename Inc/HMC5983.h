#pragma once

#define HMC5883_ADDRESS 				0x1E

#define HMC5883_CONFIG_REG_A    0x00
#define HMC5883_CONFIG_REG_B    0x01
#define HMC5883_MODE_REG        0x02
#define HMC5883_DATA_X_MSB_REG  0x03
#define HMC5883_STATUS_REG      0x09

///////////////////////////////////////////////////////////////////////////////

//#define SENSOR_CONFIG 0x18  // 1 Sample average, 75 Hz
//#define SENSOR_CONFIG 0x38  // 2 Sample average, 75 Hz
//#define SENSOR_CONFIG 0x58  // 4 Sample average, 75 Hz
//#define SENSOR_CONFIG 0x78  // 8 Sample average, 75 Hz

//#define SENSOR_CONFIG 0x10  // 1 Sample average, 15 Hz
//#define SENSOR_CONFIG 0x30  // 2 Sample average, 15 Hz
//#define SENSOR_CONFIG 0x50  // 4 Sample average, 15 Hz
#define SENSOR_CONFIG 0x70      // 8 Sample average, 15 Hz

#define NORMAL_MEASUREMENT_CONFIGURATION 0x00
#define POSITIVE_BIAS_CONFIGURATION      0x01

///////////////////////////////////////////////////////////////////////////////

//#define SENSOR_GAIN 0x00  // +/- 0.88 Ga
#define SENSOR_GAIN 0x20        // +/- 1.3  Ga (default)
//#define SENSOR_GAIN 0x40  // +/- 1.9  Ga
//#define SENSOR_GAIN 0x60  // +/- 2.5  Ga
//#define SENSOR_GAIN 0x80  // +/- 4.0  Ga
//#define SENSOR_GAIN 0xA0  // +/- 4.7  Ga
//#define SENSOR_GAIN 0xC0  // +/- 5.6  Ga
//#define SENSOR_GAIN 0xE0  // +/- 8.1  Ga

///////////////////////////////////////////////////////////////////////////////

#define OP_MODE_CONTINUOUS 0x00 // Continuous conversion
#define OP_MODE_SINGLE     0x01 // Single conversion

#define STATUS_RDY         0x01 // Data Ready
