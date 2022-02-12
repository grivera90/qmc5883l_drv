/**
* \file     qmc5883l.c
* \brief    This is the source code for the qmc5883l driver
*
*	Created on: 6 feb. 2022
* 		Author: Gonzalo Rivera
*/
/******************************************************************************
        Interfaces
 ******************************************************************************/
#include "qmc5883l.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"
/******************************************************************************
        Defines and constants
 ******************************************************************************/
static const char * MODULE_NAME 	= "[NAME]";
static const char * DRIVER_VERSION 	= "fw_v1.0.0";

// Register names
#define REG_XOUT_L 0x00
#define REG_XOUT_H 0x01
#define REG_YOUT_L 0x02
#define REG_YOUT_H 0x03
#define REG_ZOUT_L 0x04
#define REG_ZOUT_H 0x05
#define REG_STATE  0x06
#define REG_TOUT_L 0x07
#define REG_TOUT_H 0x08
#define REG_CTRL1  0x09
#define REG_CTRL2  0x0a
#define REG_FBR    0x0b
#define REG_ID     0x0d

#define PI 3.1415926535897932384626433832795
/******************************************************************************
        Data types
 ******************************************************************************/

/******************************************************************************
        Local function prototypes
 ******************************************************************************/
static int 		qmc5883l_softreset	(qmc5883l_t *qmc5883l);
static int 		qmc5883l_setmode	(qmc5883l_t *qmc5883l);
static float	get_azimuth			(int a, int b);
/******************************************************************************
        Local variables
 ******************************************************************************/

/******************************************************************************
        Public function definitions
 ******************************************************************************/
int qmc5883l_init(qmc5883l_t *qmc5883l, bool default_config)
{
	int ret = QMC5883L_OK;
	//=========================== Init I2C=====================================
	ret = qmc5883l->qmc5883l_i2c_init();
	if(ret != QMC5883L_OK)
		return ret;

	if(ret == QMC5883L_OK)
	{
		qmc5883l->TAG_MODULE = MODULE_NAME;
		qmc5883l->fw_version = DRIVER_VERSION;
	}

	//========================== Reset ========================================
	if(qmc5883l_softreset(qmc5883l) != QMC5883L_OK)
		return QMC5883L_FAIL;
	//========================== Default config ===============================
	if(default_config == true)
	{
		qmc5883l->config.opmode 			= QMC5883L_MODE_CONTINUOUS;
		qmc5883l->config.output_data_rate 	= QMC5883L_DR_200;
		qmc5883l->config.oversampling_rate 	= QMC5883L_OSR_512;
		qmc5883l->config.range 				= QMC5883L_RNG_8;
		ret = qmc5883l_setmode(qmc5883l);
	}
	else
	{
		//config by user in main.c
		ret = qmc5883l_setmode(qmc5883l);

	}

	return ret;
}

int qmc5883l_read(qmc5883l_t *qmc5883l)
{
	int ret = QMC5883L_OK;
	uint8_t data[6];
	int i = 0;

	// set pointer to the register 0x00
	ret = qmc5883l->qmc5883l_write_set_ptr(QMC5883L_ADDR_WRITE, 0x00);
	if(ret != QMC5883L_OK)
		return ret;

	// read data from 0x00 to 0x05
	for(i = 0; i < 6; i++)
	{
		ret = qmc5883l->qmc5883l_read_i2c(QMC5883L_ADDR_READ, &data[i]);

		if(ret != QMC5883L_OK)
			return ret;
	}

	if((ret != QMC5883L_OK ) && (i < 5))
		return ret;

	// if all right then:
	qmc5883l->raw_data.x = data[1];
	qmc5883l->raw_data.x = qmc5883l->raw_data.x << 8 | data[0];
	qmc5883l->raw_data.y = data[3];
	qmc5883l->raw_data.y = qmc5883l->raw_data.y << 8 | data[2];
	qmc5883l->raw_data.z = data[5];
	qmc5883l->raw_data.z = qmc5883l->raw_data.z << 8 | data[4];

	qmc5883l->azimuth_value = get_azimuth(qmc5883l->raw_data.x, qmc5883l->raw_data.y);

    float f = (qmc5883l->config.range == QMC5883L_RNG_2 ? 2000.0 : 8000.0) / 32768;

    qmc5883l->data_mgauss.x_mgauss = qmc5883l->raw_data.x * f;
    qmc5883l->data_mgauss.y_mgauss = qmc5883l->raw_data.y * f;
    qmc5883l->data_mgauss.z_mgauss = qmc5883l->raw_data.z * f;

	return ret;
}
/******************************************************************************
        Private function definitions
 ******************************************************************************/
static int qmc5883l_softreset(qmc5883l_t *qmc5883l)
{
	return qmc5883l->qmc5883l_write_i2c(QMC5883L_ADDR_WRITE, 0x0B, 0x01);
}

static int qmc5883l_setmode(qmc5883l_t *qmc5883l)
{
	int ret = QMC5883L_OK;



	ret = qmc5883l->qmc5883l_write_i2c(	QMC5883L_ADDR_WRITE,
										REG_CTRL1,
										qmc5883l->config.opmode|
										qmc5883l->config.output_data_rate|
										qmc5883l->config.oversampling_rate|
										qmc5883l->config.range);
	return ret;
}

static float get_azimuth(int a, int b)
{
	float the_azimuth = atan2(a, b ) * 180.0/PI;
	return the_azimuth < 0 ? 360 + the_azimuth : the_azimuth;
}

/* End of file ***************************************************************/

