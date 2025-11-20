/*
 * ADS1220 Basic Reading Example for Arduino
 * 
 * This example demonstrates:
 * - Initializing the ADS1220
 * - Using preset configurations
 * - Continuous data acquisition with callbacks
 * 
 * Hardware Connections:
 * - SCLK  -> GPIO 18
 * - MOSI  -> GPIO 23
 * - MISO  -> GPIO 19
 * - CS    -> GPIO 5
 * - DRDY  -> GPIO 34
 */

#include <SPI.h>
#include "ads1220.h"

// Pin definitions
#define PIN_CS   5
#define PIN_DRDY 34

// ADS1220 device handle
ADS1220_t* adc = NULL;

// Data callback function
void dataReadyCallback(int32_t raw_data) {
    // Convert to voltage (example: 2.048V reference, gain 128)
    float voltage = (raw_data * 2.048f) / (8388608.0f * 128.0f);
    
    Serial.print("Raw: ");
    Serial.print(raw_data);
    Serial.print(", Voltage: ");
    Serial.print(voltage, 6);
    Serial.println(" V");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("ADS1220 Basic Reading Example");
    Serial.println("==============================");
    
    // Initialize SPI
    SPI.begin();
    Serial.println("SPI initialized");
    
    // Initialize SPI bus (ESP-IDF style for compatibility)
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    
    // Create ADS1220 device instance
    ADS1220_init_config_t init_cfg = {
        .spi_host = SPI2_HOST,
        .cs_pin = (gpio_num_t)PIN_CS,
        .drdy_pin = (gpio_num_t)PIN_DRDY,
        .spi_clock_speed_hz = 1000000,  // 1 MHz
    };
    
    adc = ADS1220_create(&init_cfg);
    if (!adc) {
        Serial.println("ERROR: Failed to create ADS1220 device");
        while(1) delay(1000);
    }
    Serial.println("ADS1220 device created");
    
    // Reset device
    ADS1220_reset(adc);
    delay(100);
    Serial.println("ADS1220 reset");
    
    // Get preset configuration for load cell
    ADS1220_Config_t config;
    if (ADS1220_get_preset_config(ADS1220_PRESET_LOAD_CELL, &config) != ESP_OK) {
        Serial.println("ERROR: Failed to get preset config");
        while(1) delay(1000);
    }
    Serial.println("Using LOAD_CELL preset configuration");
    
    // Write configuration to device
    if (ADS1220_write_config(adc, &config) != ESP_OK) {
        Serial.println("ERROR: Failed to write config");
        while(1) delay(1000);
    }
    Serial.println("Configuration written");
    
    // Verify configuration by reading back
    ADS1220_Config_t read_cfg;
    if (ADS1220_read_config(adc, &read_cfg) == ESP_OK) {
        Serial.printf("Configuration verified - Reg0: 0x%02X, Reg1: 0x%02X, Reg2: 0x%02X, Reg3: 0x%02X\n",
                     read_cfg.reg[0], read_cfg.reg[1], read_cfg.reg[2], read_cfg.reg[3]);
    }
    
    // Start continuous data acquisition
    if (ADS1220_start_continuous(adc, dataReadyCallback) != ESP_OK) {
        Serial.println("ERROR: Failed to start continuous mode");
        while(1) delay(1000);
    }
    Serial.println("Continuous acquisition started");
    Serial.println("==============================");
}

void loop() {
    // Data is handled in callback
    // Just keep the main loop alive
    delay(5000);
    Serial.println("Still running...");
}
