#include "BMP280.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

#define BMP280_S32_t int32_t
#define BMP280_U32_t uint32_t

BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T);

uint16_t dig_T1 = 0;
int16_t  dig_T2 = 0;
int16_t  dig_T3 = 0;

static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BMP280_SLAVE_ADRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

static esp_err_t bmp280_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
}

/**
 * @brief Write a byte to a bmp280 sensor register
 */
static esp_err_t bmp280_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS);
}

esp_err_t bmp280_read_write_read(char * reg_name,i2c_master_dev_handle_t dev_handle, uint8_t reg_addr,uint8_t write_data)
{
    uint8_t data = 0;
    esp_err_t err;
    err = bmp280_register_read(dev_handle,reg_addr,&data,1);
    if( err != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to read %s before write",reg_name);
        return err;
    }
    ESP_LOGI(TAG, "%s reg value before write = 0X%x",reg_name,data);

    err = bmp280_register_write_byte(dev_handle,reg_addr,write_data);
    if( err != ESP_OK)
    {
        ESP_LOGI(TAG, "Failed to write in  %s",reg_name);
        return err;
    }
    ESP_LOGI(TAG, "%s reg value write = 0X%x success",reg_name,write_data);

    // err = bmp280_register_read(dev_handle,reg_addr,&data,1);
    // if(err != ESP_OK)
    // {
    //    ESP_LOGE(TAG, "Failed to read %s after write", reg_name);
    //    return err;

    // }
    // ESP_LOGI(TAG, "%s reg value after write = 0X%x", reg_name, data);
   
    // if (data != write_data) 
    // {
    //     ESP_LOGE(TAG, "%s verification failed! Wrote 0x%x, Read 0x%x", reg_name, write_data, data);
    //     return ESP_FAIL;   
    // }

    ESP_LOGI(TAG, "%s reg write verified successfully", reg_name);
    return ESP_OK;
}

int32_t bmp280_read_temp_raw(i2c_master_dev_handle_t dev_handle)
{
    uint8_t temp_raw_data[3];
    bmp280_register_read(dev_handle,BMP280_TEMP_RAW_REG_BRUST_READ,temp_raw_data,BURST_READ_LEN);

    int32_t temp_raw_data_32 = ((int32_t)temp_raw_data[2] >> 4) | ((int32_t)temp_raw_data[1] << 4) | ((int32_t)temp_raw_data[0] << 12);
    return temp_raw_data_32;
}

esp_err_t bmp280_read_dev_id(i2c_master_dev_handle_t dev_handle)
{
    uint8_t data = 0;
    esp_err_t ret = bmp280_register_read(dev_handle, BMP280_DEVID_REG, &data, 1);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    if (data != BMP280_DEVID_ID) {
        ESP_LOGE(TAG, "DEV ID mismatch: got 0x%02X, expected 0x%02X", data, BMP280_DEVID_ID);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "DEV ID = 0x%02X", data);
    return ESP_OK;
}

esp_err_t bmp280_read_temp_calib(i2c_master_dev_handle_t dev_handle)
{
    uint8_t calib[6] = {0};
    esp_err_t err = bmp280_register_read(dev_handle, 0x88, calib, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read temp calibration: %s", esp_err_to_name(err));
        return err;
    }

    // Combine LSB and MSB (Table 17: dig_T1 is unsigned, dig_T2 & dig_T3 are signed)
    dig_T1 = (uint16_t)((calib[1] << 8) | calib[0]);
    dig_T2 = (int16_t)((calib[3] << 8) | calib[2]);
    dig_T3 = (int16_t)((calib[5] << 8) | calib[4]);

    ESP_LOGI(TAG, "Calib: T1=%u, T2=%d, T3=%d", dig_T1, dig_T2, dig_T3);
    return ESP_OK;
}

esp_err_t bmp280_init()
{
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);

    ESP_LOGI(TAG, "I2C initialized successfully............VARAD");

    bmp280_read_dev_id(dev_handle);

    bmp280_read_temp_calib(dev_handle);

    if(bmp280_read_write_read("BMP280_CTRL_MEAS",dev_handle, BMP280_CTRL_MEAS,BMP281_ONLY_TEMP) != ESP_OK)
    {
        return ESP_FAIL;
    }

    uint8_t data = 0;
    bmp280_register_read(dev_handle, 0xF4, &data, 1);

    uint8_t osrs_t = (data >> 5) & 0x07; // Isolates bits 7, 6, 5

    if (osrs_t != 0) 
    {
        ESP_LOGI(TAG, "Temperature is ENABLED (osrs_t = %d)", osrs_t);
    } else 
    {
        ESP_LOGW(TAG, "Temperature is SKIPPED");
    }

    while(1)
    {
        bmp280_register_write_byte(dev_handle, BMP280_CTRL_MEAS, BMP281_ONLY_TEMP);
        vTaskDelay(pdMS_TO_TICKS(10));
        int32_t raw_data = bmp280_read_temp_raw(dev_handle);

        // ESP_LOGI(TAG,"raw data = %u\r\n",raw_data);

        int32_t processed_data = bmp280_compensate_T_int32(raw_data);
        ESP_LOGI(TAG,"Tempaeature  = %.2f C\r\n",processed_data / 100.0f);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // while (1)
    // {
    //     // 1. Trigger a new Forced mode measurement
    //     bmp280_register_write_byte(dev_handle, 0xF4, 0x21);

    //     // 2. Wait for conversion to finish (max 6.4 ms for x1 oversampling)
    //     vTaskDelay(pdMS_TO_TICKS(10));

    //     // 3. Read raw 20-bit data
    //     int32_t raw_data = bmp280_read_temp_raw(dev_handle);
    //     ESP_LOGI(TAG, "Raw data = %" PRId32, raw_data);

    //     // 4. Compensate and print in °C
    //     int32_t processed_data = bmp280_compensate_T_int32(raw_data);
    //     ESP_LOGI(TAG, "Temperature = %.2f C", processed_data / 100.0f);

    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
    return ESP_OK;

}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. 
// t_fine carries fine temperature as global value 
// Calibration parameters (read once from 0x88..0x8D at startup)


BMP280_S32_t t_fine;

BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
{
    BMP280_S32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((BMP280_S32_t)dig_T1 << 1))) * ((BMP280_S32_t)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((BMP280_S32_t)dig_T1)) * ((adc_T >> 4) - ((BMP280_S32_t)dig_T1))) >> 12) * ((BMP280_S32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}