
/******************************************************************************
* Includes
*******************************************************************************/
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "stdbool.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#include "qmc5883l.h"
/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
#define delay_ms(x) vTaskDelay(x / portTICK_PERIOD_MS);
#define HIGH 1
#define LOW 0
#define GPIO_LED 2

/* i2c definitions */
#define I2C_TIMEOUT 				(1000 / portTICK_RATE_MS)
#define I2C_MASTER_SCL_IO 			22    	/*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 			21     	/*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 				0		/*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 			400000	/*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 	0     	/*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 	0     	/*!< I2C master doesn't need buffer */
/******************************************************************************
* Data types
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static const char *MODULE_NAME = "[main]";
qmc5883l_t 		qmc5883l;
qmc5883l_sts_t 	qmc5883l_sts;
/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void 	gpio_setup				(void);
static int 		i2c_master_init			(void);
static int 		i2c_master_write_slave	(uint8_t dev_address, uint8_t reg_address, uint8_t reg_value);
static int 		i2c_master_set_ptr		(uint8_t dev_address, uint8_t reg_address);
static int 		i2c_master_read_slave	(uint8_t dev_address_rd, uint8_t *data);
static void 	qmc5883l_delay			(uint32_t miliseconds);
/******************************************************************************
* app main
*******************************************************************************/
void app_main(void)
{
	gpio_setup();

	/* functions pointers */
	qmc5883l.qmc5883l_i2c_init 		= i2c_master_init;
	qmc5883l.qmc5883l_read_i2c 		= i2c_master_read_slave;
	qmc5883l.qmc5883l_write_i2c 	= i2c_master_write_slave;
	qmc5883l.qmc5883l_write_set_ptr = i2c_master_set_ptr;
	qmc5883l.delay 					= qmc5883l_delay;
	/* operation mode of qmc5883l, config by user */
	qmc5883l.config.opmode 				= QMC5883L_MODE_CONTINUOUS;
	qmc5883l.config.output_data_rate 	= QMC5883L_DR_200;
	qmc5883l.config.oversampling_rate 	= QMC5883L_OSR_64;
	qmc5883l.config.range 				= QMC5883L_RNG_2;

	qmc5883l_sts = qmc5883l_init(&qmc5883l, true);
	if(qmc5883l_sts != QMC5883L_OK)
	{
		ESP_LOGD(MODULE_NAME, "qmc5883l sensor init error");
		esp_restart();
	}
	else
	{
		ESP_LOGD(MODULE_NAME, "qmc5883l sensor init ok!");
	}


    while (true) {

        gpio_set_level(GPIO_LED, HIGH);
        delay_ms(50);
        gpio_set_level(GPIO_LED, LOW);
        delay_ms(50);

        if(qmc5883l_read(&qmc5883l) == QMC5883L_OK)
        {
        	ESP_LOGD(MODULE_NAME, "Read values: "
        			"x[mGauss]: %.2f, "
        			"y[mGauss]: %.2f, "
        			"z[mGauss]: %.2f, "
        			"a: %.2f",
        			qmc5883l.data_mgauss.x_mgauss,
					qmc5883l.data_mgauss.y_mgauss,
					qmc5883l.data_mgauss.z_mgauss,
					qmc5883l.azimuth_value
					);
        }

    }
}

/******************************************************************************
* functions definitions
*******************************************************************************/
static void gpio_setup(void)
{
    gpio_config_t io_conf;

	/* led */
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << GPIO_LED);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(GPIO_LED, HIGH);
}

static int i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    esp_err_t err = ESP_OK;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };

    err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK)
    	return err;

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 * @note cannot use master write slave on esp32c3 because there is only one i2c controller on esp32c3
 */
static int i2c_master_write_slave(uint8_t dev_address, uint8_t reg_address, uint8_t reg_value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();											// crea el handler

    i2c_master_start(cmd);																	// genera la señal de start
    i2c_master_write_byte(cmd, dev_address, true);
    i2c_master_write_byte(cmd, reg_address, true);
    i2c_master_write_byte(cmd, reg_value, 	true);
    i2c_master_stop(cmd);																	// genera la señal de stop

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT);					// comienza el proceso de comunicacion mas un timeout
    i2c_cmd_link_delete(cmd);																// desvincula el handler

    return ret;
}

static int i2c_master_set_ptr(uint8_t dev_address, uint8_t reg_address)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();											// crea el handler

    i2c_master_start(cmd);																	// genera la señal de start
    i2c_master_write_byte(cmd, dev_address, true);
    i2c_master_write_byte(cmd, reg_address, true);
    i2c_master_stop(cmd);																	// genera la señal de stop

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT);					// comienza el proceso de comunicacion mas un timeout
    i2c_cmd_link_delete(cmd);																// desvincula el handler

    return ret;
}
/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 * @note cannot use master read slave on esp32c3 because there is only one i2c controller on esp32c3
 */
static int i2c_master_read_slave(uint8_t dev_address_rd, uint8_t *data)
{
	esp_err_t ret = ESP_OK;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();					// crea el handler
    i2c_master_start(cmd);											// genera la señal de start
    i2c_master_write_byte(cmd, dev_address_rd, true);				// escribe la direccion del dispositivo + bit de escritura
    i2c_master_read_byte(cmd, data, true);							// lee el ultimo dato indicando que no envie mas.
    i2c_master_stop(cmd);											// genera la señal de stop
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT);	// comienza el proceso de comunicacion mas un timeout
    i2c_cmd_link_delete(cmd);										// desvincula el handler

    return ret;
}

static void qmc5883l_delay(uint32_t miliseconds)
{
	vTaskDelay(miliseconds / portTICK_PERIOD_MS);
}

