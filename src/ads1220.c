#include "ads1220.h"
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "ADS1220";

// --- Perfil: célula de carga (AIN0-1, G=128, DR=2000, VREF externa en REFP0/REFN0)
static const ADS1220_Config_t CFG_LOAD_CELL = {
    .mux = ADS1220_MUX_AIN0_AIN1,
    .gain = ADS1220_GAIN_128,
    .pga_bypass = 0,
    .dr = ADS1220_DR_600SPS,
    .mode = ADS1220_OP_MODE_NORMAL,
    .cm = 1,
    .ts = 0,
    .bcs = 0,
    .vref = ADS1220_VREF_REFP0_REFN0,
    .fir = ADS1220_FIR_OFF,
    .psw = 0,
    .idac = ADS1220_IDAC_OFF,
    .i1mux = ADS1220_IDAC_ROUTE_NONE,
    .i2mux = ADS1220_IDAC_ROUTE_NONE,
    .drdym = 0,
    .reserved = 0
};

// --- Perfil: como Fernando (AIN0-1, G=64, DR=2000, VREF interna)
static const ADS1220_Config_t CFG_FERNANDO = {
    .mux = ADS1220_MUX_AIN0_AIN1,
    .gain = ADS1220_GAIN_64,
    .pga_bypass = 0,
    .dr = ADS1220_DR_TURBO_2000SPS,
    .mode = ADS1220_OP_MODE_TURBO,
    .cm = 1,
    .ts = 0,
    .bcs = 0,
    .vref = ADS1220_VREF_INT_2048,
    .fir = ADS1220_FIR_OFF,
    .psw = 0,
    .idac = ADS1220_IDAC_OFF,
    .i1mux = ADS1220_IDAC_ROUTE_NONE,
    .i2mux = ADS1220_IDAC_ROUTE_NONE,
    .drdym = 0,
    .reserved = 0
};

struct ADS1220_t {
    spi_device_handle_t spi_dev;
    spi_host_device_t spi_host;
    gpio_num_t drdy_pin;
    TaskHandle_t driver_task;
    ads1220_data_callback_t callback;
    void* callback_arg;
};

static inline int ads_data_ready(gpio_num_t pin){
    return gpio_get_level(pin)==0;
}

static void IRAM_ATTR ads1220_drdy_isr(void *arg) {
    ADS1220_t* dev = (ADS1220_t*)arg;
    BaseType_t high_task_wakeup = pdFALSE;
    if (dev && dev->driver_task) {
        vTaskNotifyGiveFromISR(dev->driver_task, &high_task_wakeup);
    }
    if (high_task_wakeup) {
        portYIELD_FROM_ISR();
    }
}

static int32_t ads_read_raw_internal(spi_device_handle_t dev) {
    uint8_t tx[4] = { ADS1220_RDATA.u8, 0, 0, 0 };
    uint8_t rx[4];
    spi_transaction_t t = {
        .length = 32, .tx_buffer = tx, .rx_buffer = rx
    };
    spi_device_transmit(dev, &t);
    
    int32_t v = ((int32_t)rx[1]<<16)|((int32_t)rx[2]<<8)|rx[3];
    if (v & 0x800000) v |= 0xFF000000;
    return v;
}

static void ads1220_driver_task(void *arg) {
    ADS1220_t* dev = (ADS1220_t*)arg;
    while (1) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
            int32_t raw = ads_read_raw_internal(dev->spi_dev);
            if (dev->callback) {
                dev->callback(raw, dev->callback_arg);
            }
        }
    }
}

ADS1220_t* ads1220_create(const ADS1220_init_config_t* config) {
    if (!config) {
        ESP_LOGE(TAG, "Config is NULL");
        return NULL;
    }
    
    ADS1220_t* dev = (ADS1220_t*)calloc(1, sizeof(ADS1220_t));
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate ADS1220 instance");
        return NULL;
    }
    
    dev->spi_host = config->spi_host;
    dev->drdy_pin = config->drdy_pin;
    
    // Add device to SPI bus (bus must be already initialized)
    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = config->spi_clock_speed_hz,
        .mode = 1,  // SPI mode 1 (CPOL=0, CPHA=1)
        .spics_io_num = config->cs_pin,
        .queue_size = 2
    };
    
    esp_err_t ret = spi_bus_add_device(config->spi_host, &dev_config, &dev->spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        free(dev);
        return NULL;
    }
    
    // Initialize DRDY GPIO
    gpio_config_t io = { 
        .pin_bit_mask = 1ULL << config->drdy_pin, 
        .mode = GPIO_MODE_INPUT, 
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io);
    
    return dev;
}

void ads1220_destroy(ADS1220_t* dev) {
    if (!dev) return;
    
    // Stop continuous mode if running
    ads1220_stop_continuous(dev);
    
    // Remove device from SPI bus
    spi_bus_remove_device(dev->spi_dev);
    
    free(dev);
}

esp_err_t ads1220_read_data(ADS1220_t* dev, int32_t *data) {
    if (!dev || !data) return ESP_ERR_INVALID_ARG;
    *data = ads_read_raw_internal(dev->spi_dev);
    return ESP_OK;
}

esp_err_t ads1220_read_oneshot(ADS1220_t* dev, int32_t *data, uint32_t timeout_ms) {
    if (!dev || !data) return ESP_ERR_INVALID_ARG;
    
    // Send START command to trigger a single conversion
    esp_err_t ret = ads1220_start(dev);
    if (ret != ESP_OK) return ret;
    
    // Wait for DRDY to go low (data ready) with timeout
    uint32_t elapsed_us = 0;
    uint32_t timeout_us = timeout_ms * 1000;
    const uint32_t poll_interval_us = 100; // Poll every 100us
    
    while (!ads_data_ready(dev->drdy_pin)) {
        if (elapsed_us >= timeout_us) {
            return ESP_ERR_TIMEOUT;
        }
        esp_rom_delay_us(poll_interval_us);
        elapsed_us += poll_interval_us;
    }
    
    // Read the conversion result
    *data = ads_read_raw_internal(dev->spi_dev);
    return ESP_OK;
}

esp_err_t ads1220_get_preset_config(ADS1220_Preset_t preset, ADS1220_Config_t *cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;
    
    switch (preset) {
        case ADS1220_PRESET_LOAD_CELL:
            *cfg = CFG_LOAD_CELL;
            break;
        case ADS1220_PRESET_FERNANDO:
            *cfg = CFG_FERNANDO;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

esp_err_t ads1220_set_preset_config(ADS1220_t* dev, ADS1220_Preset_t preset) {
    ADS1220_Config_t cfg;
    esp_err_t ret = ads1220_get_preset_config(preset, &cfg);
    if (ret != ESP_OK) return ret;
    return ads1220_write_config(dev, &cfg);
}

esp_err_t ads1220_read_config(ADS1220_t* dev, ADS1220_Config_t *out) {
    if (!dev || !out) return ESP_ERR_INVALID_ARG;
    
    uint8_t cmd = ADS1220_RREG(ADS1220_REG0, 4).u8;
    spi_transaction_t t = { 
        .length = 40, 
        .tx_buffer = &cmd,
        .rxlength = 32,
        .rx_buffer = out->reg
    };
    return spi_device_transmit(dev->spi_dev, &t);
}

esp_err_t ads1220_write_config(ADS1220_t* dev, const ADS1220_Config_t *cfg) {
    if (!dev || !cfg) return ESP_ERR_INVALID_ARG;
    
    spi_transaction_t t = { 
        .length = 40, 
        .tx_buffer = (uint8_t[]){
            ADS1220_WREG(ADS1220_REG0, 4).u8,
            cfg->reg[0], cfg->reg[1], cfg->reg[2], cfg->reg[3]
        }
    };
    return spi_device_transmit(dev->spi_dev, &t);
}

esp_err_t ads1220_start_continuous(ADS1220_t* dev, ads1220_data_callback_t callback, void* callback_arg) {
    if (!dev || !callback) return ESP_ERR_INVALID_ARG;
    
    dev->callback = callback;
    dev->callback_arg = callback_arg;

    if (!dev->driver_task) {
        xTaskCreate(ads1220_driver_task, "ads1220", 4096, dev, configMAX_PRIORITIES - 1, &dev->driver_task);
    }

    // Install ISR service if not already installed (might fail if already installed, ignore error)
    gpio_install_isr_service(0);
    gpio_isr_handler_add(dev->drdy_pin, ads1220_drdy_isr, dev);
    
    // Enable interrupt
    gpio_intr_enable(dev->drdy_pin);

    // Send START command
    return ads1220_start(dev);
}

void ads1220_stop_continuous(ADS1220_t* dev) {
    if (!dev) return;
    
    ads1220_powerdown(dev);
    gpio_intr_disable(dev->drdy_pin);
    gpio_isr_handler_remove(dev->drdy_pin);
    
    if (dev->driver_task) {
        vTaskDelete(dev->driver_task);
        dev->driver_task = NULL;
    }
    
    dev->callback = NULL;
    dev->callback_arg = NULL;
}

/**
 * @brief Macro to generate simple command functions.
 * @internal
 */
#define ADS1220_CMD_FUNC(name, cmd) esp_err_t ads1220_##name(ADS1220_t* dev) { \
    const ADS1220Command_t data = cmd; \
    return spi_device_transmit(dev->spi_dev, &(spi_transaction_t){ .length = 8, .tx_buffer = &data }); }

ADS1220_CMD_FUNC(reset, ADS1220_RESET)
ADS1220_CMD_FUNC(powerdown, ADS1220_POWERDOWN)
ADS1220_CMD_FUNC(start, ADS1220_STARTSYNC)
ADS1220_CMD_FUNC(sync, ADS1220_STARTSYNC)
