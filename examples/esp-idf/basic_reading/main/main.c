#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "ads1220.h"

static const char *TAG = "ADS1220_EXAMPLE";

// SPI Pin Configuration
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_CLK  18
#define PIN_CS   5
#define PIN_DRDY 34

// Data callback function
bool data_ready_callback(int32_t raw_data) {
    // Convert to voltage (example: 2.048V reference, gain 128)
    float voltage = (raw_data * 2.048f) / (8388608.0f * 128.0f);
    ESP_LOGI(TAG, "Raw: %ld, Voltage: %.6f V", raw_data, voltage);
    return true;
}

void app_main(void) {
    ESP_LOGI(TAG, "ADS1220 Basic Reading Example");
    
    // Initialize SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI bus initialized");
    
    // Create ADS1220 device instance
    ADS1220_init_config_t init_cfg = {
        .spi_host = SPI2_HOST,
        .cs_pin = PIN_CS,
        .drdy_pin = PIN_DRDY,
        .spi_clock_speed_hz = 1000000,  // 1 MHz
    };
    
    ADS1220_t* adc = ads1220_create(&init_cfg);
    if (!adc) {
        ESP_LOGE(TAG, "Failed to create ADS1220 device");
        return;
    }
    ESP_LOGI(TAG, "ADS1220 device created");
    
    // Reset device
    ads1220_reset(adc);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "ADS1220 reset");
    
    // Get preset configuration for load cell
    ADS1220_Config_t config;
    ESP_ERROR_CHECK(ads1220_get_preset_config(ADS1220_PRESET_LOAD_CELL, &config));
    ESP_LOGI(TAG, "Using LOAD_CELL preset configuration");
    
    // Write configuration to device
    ESP_ERROR_CHECK(ads1220_write_config(adc, &config));
    ESP_LOGI(TAG, "Configuration written");
    
    // Verify configuration by reading back
    ADS1220_Config_t read_cfg;
    ESP_ERROR_CHECK(ads1220_read_config(adc, &read_cfg));
    ESP_LOGI(TAG, "Configuration verified - Reg0: 0x%02X, Reg1: 0x%02X, Reg2: 0x%02X, Reg3: 0x%02X",
             read_cfg.reg[0], read_cfg.reg[1], read_cfg.reg[2], read_cfg.reg[3]);
    
    // Start continuous data acquisition
    ESP_ERROR_CHECK(ads1220_start_continuous(adc, data_ready_callback));
    ESP_LOGI(TAG, "Continuous acquisition started");
    
    // Main loop - data is handled in callback
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "Still running...");
    }
    
    // Cleanup (never reached in this example)
    ads1220_stop_continuous(adc);
    ads1220_destroy(adc);
}
