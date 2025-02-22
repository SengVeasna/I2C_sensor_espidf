#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "bh1750.h"
#include "esp_system.h"

// Define I2C address and pins
#define BH1750_ADDR BH1750_ADDR_LO
#define I2C_MASTER_SDA GPIO_NUM_21
#define I2C_MASTER_SCL GPIO_NUM_22
#define MQ2_GPIO_PIN GPIO_NUM_4
static const char *TAG1 = "AnalogSensor";
static const char *TAG2 = "DigitalSensor";
static const char *TAG3 = "CommunicateSensor";
// ADC Characteristics
static esp_adc_cal_characteristics_t adc2_chars;

// BH1750 Task: Reads light intensity
void BH1750Task(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t)); // Zero descriptor
    if (bh1750_init_desc(&dev, BH1750_ADDR, 0, I2C_MASTER_SDA, I2C_MASTER_SCL) != ESP_OK)
    {
        ESP_LOGE(TAG3, "Failed to initialize BH1750 descriptor");
        vTaskDelete(NULL);
    }
    if (bh1750_setup(&dev, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH) != ESP_OK)
    {
        ESP_LOGE(TAG3, "Failed to configure BH1750");
        vTaskDelete(NULL);
    }
    while (1)
    {
        uint16_t lux;
        if (bh1750_read(&dev, &lux) == ESP_OK)
        {
            ESP_LOGI(TAG3, "Light Intensity (Lux): %d", lux);
        }
        else
        {
            ESP_LOGE(TAG3, "Failed to read data from BH1750");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1-second delay
    }
}
// Analog Sensor Task: Reads water level
void AnalogSensorTask(void *params)
{
    esp_adc_cal_characteristics_t adc2_chars; // ADC calibration characteristics structure
    // Characterize ADC2
    esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc2_chars);
    // Configure ADC2 channel and attenuation
    adc2_config_channel_atten(ADC2_CHANNEL_2, ADC_ATTEN_DB_11);

    while (1)
    {
        int adc_value = 0; // Variable to store raw ADC value
        esp_err_t result = adc2_get_raw(ADC2_CHANNEL_2, ADC_WIDTH_BIT_12, &adc_value);

        if (result == ESP_OK)
        {
            ESP_LOGI(TAG1, "ADC Value: %d", adc_value);
        }
        else
        {
            ESP_LOGE(TAG1, "Error reading ADC2: %s", esp_err_to_name(result));
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds
    }
    vTaskDelete(NULL);
}
void MQ2Task(void *pvParameters)
{
   //Configuration GPIO for digital input
   gpio_config_t io_config = {0};
   io_config.mode = GPIO_MODE_INPUT;
   io_config.pin_bit_mask = 1ULL << MQ2_GPIO_PIN;
   io_config.pull_up_en = GPIO_PULLUP_ENABLE;
   gpio_config(&io_config);
    while (1)
    {
        int mq2_status = gpio_get_level(MQ2_GPIO_PIN);
        if(mq2_status == ESP_OK)
        {
            ESP_LOGI(TAG2, "MQ2 DETECTED GAS!");
        }
        else
        {
            ESP_LOGI(TAG2, "NO GAS DETECTED BY MQ2!");
        }
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
    vTaskDelete(NULL);
}

// Main Application Entry Point
void app_main(void)
{
    ESP_LOGI(TAG3, "Initializing I2C...");
    if (i2cdev_init() != ESP_OK)
    {
        ESP_LOGE(TAG3, "Failed to initialize I2C driver");
        return;
    }
    ESP_LOGI("ESP32_Tasks", "Creating tasks...");
    xTaskCreate(BH1750Task, "BH1750_Task", 2048, NULL, 5, NULL);
    xTaskCreate(AnalogSensorTask, "Analog_Sensor_Task", 2048, NULL, 5, NULL);
    xTaskCreate(MQ2Task, "MQ2_Task", 2048, NULL, 5, NULL);
}