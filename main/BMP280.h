#ifndef __BMP280__
#define __BMP280__

/*Basic I2C*/

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define BMP280_SLAVE_ADRESS     (0x76)
#define BMP280_DEVID_REG        (0xD0)
#define BMP280_CTRL_MEAS        (0xF4)


#define BMP280_DEVID_ID         (0x58)
#define BMP280_TEMP_RAW_REG_BRUST_READ     (0xFA)

/*
TEMP Oversampling = 0x01
Forced Mode = 0x01
binary value - 0010 0001
Hex    value      2    1
*/
#define BMP281_ONLY_TEMP        (0x21)
#define BURST_READ_LEN              3       /*!< Number of sequential bytes to read */

#define TAG "BMP280"


#include "esp_log.h"
#include "driver/i2c_master.h"

esp_err_t bmp280_init();

#endif /* __BMP280__*/

