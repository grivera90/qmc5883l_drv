/**
* \file     qmc5883l.h
* \brief    This is the header code for the qmc5883l driver
*
*	Created on: 6 feb. 2022
* 		Author: Gonzalo Rivera
*/
#ifndef COMPONENTS_QMC5883L_INCLUDE_QMC5883L_H_
#define COMPONENTS_QMC5883L_INCLUDE_QMC5883L_H_

#include <stdio.h>
#include <stdint.h>
#include "stdbool.h"
/******************************************************************************
        Constants
 ******************************************************************************/
#define QMC5883L_I2C_ADDR_DEF	 	0x0D 								// I2C address
#define QMC5883L_ADDR_WRITE		( QMC5883L_I2C_ADDR_DEF << 1 )			// 0x3C
#define QMC5883L_ADDR_READ		((QMC5883L_I2C_ADDR_DEF << 1)  | 1 )	// 0x3D

#define MODE_STANDBY    0b00000000
#define MODE_CONTINUOUS	0b00000001

#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100

#define RNG_2G          0b00000000
#define RNG_8G          0b00010000

#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000
/******************************************************************************
        Data types
 ******************************************************************************/
typedef enum
{
	QMC5883L_OK = 0,
	QMC5883L_TIMEOUT,
	QMC5883L_FAIL = -1,
}qmc5883l_sts_t;

/**
 * Output data rate
 */
typedef enum {
    QMC5883L_DR_10	= ODR_10Hz,		//!< 10Hz
    QMC5883L_DR_50	= ODR_50Hz,     //!< 50Hz
    QMC5883L_DR_100 = ODR_100Hz,	//!< 100Hz
    QMC5883L_DR_200 = ODR_200Hz,	//!< 200Hz
} qmc5883l_odr_t;

/**
 * Oversampling rate
 */
typedef enum {
    QMC5883L_OSR_64 	= OSR_64, 	//!< 64 samples
    QMC5883L_OSR_128	= OSR_128,	//!< 128 samples
    QMC5883L_OSR_256	= OSR_256,	//!< 256 samples
    QMC5883L_OSR_512	= OSR_512,	//!< 512 samples
} qmc5883l_osr_t;

/**
 * Field range
 */
typedef enum {
    QMC5883L_RNG_2 = RNG_2G,	//!< -2G..+2G
    QMC5883L_RNG_8 = RNG_8G,   	//!< -8G..+8G
} qmc5883l_range_t;

/**
 * Mode
 */
typedef enum {
    QMC5883L_MODE_STANDBY		= MODE_STANDBY, 	//!< Standby low power mode, no measurements
    QMC5883L_MODE_CONTINUOUS	= MODE_CONTINUOUS,  //!< Continuous measurements
} qmc5883l_mode_t;


typedef struct
{
    qmc5883l_range_t 		range;
    qmc5883l_odr_t			output_data_rate;
    qmc5883l_mode_t			opmode;
    qmc5883l_osr_t			oversampling_rate;
} qmc5883l_config_t;
/**
 * Raw measurement result
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} qmc5883l_raw_data_t;

/**
 * Measurement result, milligauss
 */
typedef struct
{
    float x_mgauss;
    float y_mgauss;
    float z_mgauss;
} qmc5883l_data_mgauss_t;

/**
 * Device descriptor
 */
typedef struct {


	qmc5883l_config_t		config;

    qmc5883l_raw_data_t 	raw_data;

    qmc5883l_data_mgauss_t 	data_mgauss;

    float 					azimuth_value;

	const char 				*fw_version;
	const char 				*TAG_MODULE;

    /* i2c functions */
	int 		(*qmc5883l_i2c_init)		(void);
	int 		(*qmc5883l_write_i2c)		(uint8_t dev_address, uint8_t reg_address, uint8_t reg_value);
	int 		(*qmc5883l_write_set_ptr)	(uint8_t dev_address, uint8_t reg_address);
	int 		(*qmc5883l_read_i2c)		(uint8_t dev_address_rd, uint8_t *data);

	/* timer functions */
	uint64_t 	(*timer_get_useconds) 	(void);
	bool		(*timer_expired)		(uint64_t currently_timer_value, uint64_t timeout);
	void 		(*delay)				(uint32_t miliseconds);

} qmc5883l_t;
/******************************************************************************
        Public function prototypes
 ******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

int qmc5883l_init(qmc5883l_t *qmc5883l, bool default_config);

int qmc5883l_read(qmc5883l_t *qmc5883l);

#ifdef __cplusplus
} // extern "C"
#endif


#endif /* COMPONENTS_QMC5883L_INCLUDE_QMC5883L_H_ */
