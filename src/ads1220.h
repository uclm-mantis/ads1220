#pragma once

#include <stdint.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_rom_sys.h"   // ets_delay_us

#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 *  Command Structure
 * =========================
 */
typedef union __attribute__((packed)) {
    uint8_t u8;
    struct {
        uint8_t x       : 1;  /* bit0 (don't care, LSB) */
        uint8_t opcode7 : 7;  /* bits1..7 (opcode) */
    } f7;
    struct {
        uint8_t n       : 2;  /* bits0..1: count-1 */
        uint8_t r       : 2;  /* bits2..3: start reg */
        uint8_t opcode4 : 4;  /* bits4..7 */
    } f4;
} ADS1220Command_t;

/* --- Fixed Commands (F7) --- */
static const ADS1220Command_t ADS1220_RESET     = { .f7 = { .x = 0, .opcode7 = 0b0000011 } };
static const ADS1220Command_t ADS1220_STARTSYNC = { .f7 = { .x = 0, .opcode7 = 0b0000100 } };
static const ADS1220Command_t ADS1220_POWERDOWN = { .f7 = { .x = 0, .opcode7 = 0b0000001 } };
static const ADS1220Command_t ADS1220_RDATA     = { .f7 = { .x = 0, .opcode7 = 0b0001000 } };

/* --- Register Enums --- */
typedef enum {
    ADS1220_REG0 = 0,
    ADS1220_REG1,
    ADS1220_REG2,
    ADS1220_REG3,
} ADS1220_Reg_t;

/* --- Parametric Commands (F4) --- */
static inline ADS1220Command_t ADS1220_RREG(ADS1220_Reg_t reg, uint8_t count) { 
    return (ADS1220Command_t){ .f4 = { .n = (uint8_t)(count - 1), .r = reg, .opcode4 = 0b0010 } };
}

static inline ADS1220Command_t ADS1220_WREG(ADS1220_Reg_t reg, uint8_t count) { 
    return (ADS1220Command_t){ .f4 = { .n = (uint8_t)(count - 1), .r = reg, .opcode4 = 0b0100 } };
}

/* =========================
 *  Register Definitions
 * =========================
 */

/* --- Global Configuration --- */
typedef union {
    uint8_t reg[4];
    struct __attribute__((packed)) {
        /* Register 0 */
        uint8_t pga_bypass : 1; /* 0: PGA enabled, 1: PGA disabled (bypassed) */
        uint8_t gain       : 3; /* Gain setting */
        uint8_t mux        : 4; /* Input multiplexer configuration */
        
        /* Register 1 */
        uint8_t bcs        : 1; /* Bit 0: Burn-out Current Source */
        uint8_t ts         : 1; /* Bit 1: Temp Sensor */
        uint8_t cm         : 1; /* Bit 2: Conversion Mode */
        uint8_t mode       : 2; /* Bits 3-4: Operating Mode */
        uint8_t dr         : 3; /* Bits 5-7: Data Rate */

        /* Register 2 */
        uint8_t idac       : 3; /* Bits 0-2: IDAC current setting */
        uint8_t psw        : 1; /* Bit 3: Low-side power switch */
        uint8_t fir        : 2; /* Bits 4-5: FIR filter configuration */
        uint8_t vref       : 2; /* Bits 6-7: Voltage reference selection */

        /* Register 3 */
        uint8_t reserved   : 1; /* Bit 0: Reserved */
        uint8_t drdym      : 1; /* Bit 1: DRDY mode */
        uint8_t i2mux      : 3; /* Bits 2-4: IDAC2 routing */
        uint8_t i1mux      : 3; /* Bits 5-7: IDAC1 routing */
    };
} ADS1220_Config_t;

/* =========================
 *  Enums
 * =========================
 */
typedef enum {
    ADS1220_MUX_AIN0_AIN1 = 0x0,
    ADS1220_MUX_AIN0_AIN2 = 0x1,
    ADS1220_MUX_AIN0_AIN3 = 0x2,
    ADS1220_MUX_AIN1_AIN2 = 0x3,
    ADS1220_MUX_AIN1_AIN3 = 0x4,
    ADS1220_MUX_AIN2_AIN3 = 0x5,
    ADS1220_MUX_AIN1_AIN0 = 0x6,
    ADS1220_MUX_AIN3_AIN2 = 0x7,
    ADS1220_MUX_AIN0_GND  = 0x8,
    ADS1220_MUX_AIN1_GND  = 0x9,
    ADS1220_MUX_AIN2_GND  = 0xA,
    ADS1220_MUX_AIN3_GND  = 0xB,
    ADS1220_MUX_REFP_REFN_4 = 0xC,
    ADS1220_MUX_AVDD_AVSS_4 = 0xD,
    ADS1220_MUX_AINP_AINN_SHORTED_TO_AVDD_AVSS_2 = 0xE,
} ADS1220_Mux_t;

typedef enum {
    ADS1220_GAIN_1   = 0,
    ADS1220_GAIN_2   = 1,
    ADS1220_GAIN_4   = 2,
    ADS1220_GAIN_8   = 3,
    ADS1220_GAIN_16  = 4,
    ADS1220_GAIN_32  = 5,
    ADS1220_GAIN_64  = 6,
    ADS1220_GAIN_128 = 7,
} ADS1220_Gain_t;

typedef enum {
    ADS1220_DR_20SPS   = 0,
    ADS1220_DR_45SPS   = 1,
    ADS1220_DR_90SPS   = 2,
    ADS1220_DR_175SPS  = 3,
    ADS1220_DR_330SPS  = 4,
    ADS1220_DR_600SPS  = 5,
    ADS1220_DR_1000SPS = 6,

    ADS1220_DR_DUTY_5SPS   = 0,
    ADS1220_DR_DUTY_11_25SPS  = 1,
    ADS1220_DR_DUTY_22_5SPS  = 2,
    ADS1220_DR_DUTY_44SPS  = 3,
    ADS1220_DR_DUTY_82_5SPS  = 4,
    ADS1220_DR_DUTY_150SPS  = 5,
    ADS1220_DR_DUTY_250SPS = 6,
    
    ADS1220_DR_TURBO_40SPS   = 0,
    ADS1220_DR_TURBO_90SPS   = 1,
    ADS1220_DR_TURBO_180SPS   = 2,
    ADS1220_DR_TURBO_350SPS  = 3,
    ADS1220_DR_TURBO_660SPS  = 4,
    ADS1220_DR_TURBO_1200SPS = 5,
    ADS1220_DR_TURBO_2000SPS = 6,
} ADS1220_DR_t;

typedef enum {
    ADS1220_OP_MODE_NORMAL = 0,
    ADS1220_OP_MODE_DUTY   = 1,
    ADS1220_OP_MODE_TURBO  = 2,
} ADS1220_OpMode_t;

typedef enum {
    ADS1220_VREF_INT_2048 = 0,
    ADS1220_VREF_REFP0_REFN0 = 1,
    ADS1220_VREF_REFP1_REFN1 = 2,
    ADS1220_VREF_AVDD_AVSS = 3,
} ADS1220_VRef_t;

typedef enum {
    ADS1220_FIR_OFF = 0,
    ADS1220_FIR_50_60 = 1, // Rejection 50/60Hz
    ADS1220_FIR_50 = 2,
    ADS1220_FIR_60 = 3,
} ADS1220_FIR_t;

typedef enum {
    ADS1220_IDAC_OFF = 0,
    ADS1220_IDAC_10UA = 1,
    ADS1220_IDAC_50UA = 2,
    ADS1220_IDAC_100UA = 3,
    ADS1220_IDAC_250UA = 4,
    ADS1220_IDAC_500UA = 5,
    ADS1220_IDAC_1000UA = 6,
    ADS1220_IDAC_1500UA = 7,
} ADS1220_IDAC_t;

typedef enum {
    ADS1220_IDAC_ROUTE_NONE = 0,
    ADS1220_IDAC_ROUTE_AIN0 = 1,
    ADS1220_IDAC_ROUTE_AIN1 = 2,
    ADS1220_IDAC_ROUTE_AIN2 = 3,
    ADS1220_IDAC_ROUTE_AIN3 = 4,
    ADS1220_IDAC_ROUTE_REFP0 = 5,
    ADS1220_IDAC_ROUTE_REFN0 = 6,
    ADS1220_IDAC_ROUTE_REFP1 = 7,
} ADS1220_IDAC_Route_t;


/* =========================
 *  Initialization Config
 * =========================
 */

/**
 * @brief Configuration structure for ADS1220 initialization
 */
typedef struct {
    spi_host_device_t spi_host;    /**< SPI host (e.g., SPI2_HOST, SPI3_HOST) */
    gpio_num_t cs_pin;              /**< Chip Select pin number (unique per device) */
    gpio_num_t drdy_pin;            /**< Data Ready pin number (unique per device) */
    int spi_clock_speed_hz;         /**< SPI clock speed in Hz (e.g., 1000000 for 1MHz) */
} ADS1220_init_config_t;

/**
 * @brief Preset configuration profiles for common use cases
 */
typedef enum {
    ADS1220_PRESET_LOAD_CELL,  /**< Load cell: AIN0-1, G=128, DR=2000, VREF external (REFP0/REFN0) */
    ADS1220_PRESET_FERNANDO,   /**< Fernando's config: AIN0-1, G=64, DR=2000, VREF internal */
} ADS1220_Preset_t;


/* =========================
 *  API Functions
 * =========================
 */

/**
 * @brief Opaque handle for ADS1220 device instance
 */
typedef struct ADS1220_t ADS1220_t;

/**
 * @brief Callback function type for continuous data acquisition
 * @param raw_data The 24-bit ADC result (sign-extended to 32-bit)
 */
typedef void (*ads1220_data_callback_t)(int32_t raw_data);

/**
 * @brief Creates and initializes an ADS1220 device instance.
 * 
 * This function adds the ADS1220 device to an existing SPI bus. The SPI bus must be
 * initialized separately using spi_bus_initialize() before calling this function.
 * Multiple ADS1220 instances can share the same SPI bus with different CS pins.
 * 
 * @param config Pointer to initialization configuration structure
 * @return ADS1220_t* Pointer to device instance, or NULL on failure
 */
ADS1220_t* ADS1220_create(const ADS1220_init_config_t* config);

/**
 * @brief Destroys an ADS1220 device instance and frees resources.
 * 
 * @param dev Pointer to device instance
 */
void ADS1220_destroy(ADS1220_t* dev);

/**
 * @brief Reads the last conversion result (24-bit signed).
 * 
 * @param dev Pointer to ADS1220 device instance
 * @param[out] data Pointer to store the 24-bit result (sign-extended to 32-bit)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ADS1220_read_data(ADS1220_t* dev, int32_t *data);

/**
 * @brief Starts continuous data acquisition triggered by DRDY interrupt.
 * 
 * @param dev Pointer to ADS1220 device instance
 * @param callback Function to call when new data is ready
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ADS1220_start_continuous(ADS1220_t* dev, ads1220_data_callback_t callback);

/**
 * @brief Stops continuous data acquisition.
 * 
 * @param dev Pointer to ADS1220 device instance
 */
void ADS1220_stop_continuous(ADS1220_t* dev);

/**
 * @brief Reads all 4 configuration registers from the ADS1220.
 * 
 * @param dev Pointer to ADS1220 device instance
 * @param[out] out Pointer to configuration structure to store the read values
 * @return esp_err_t ESP_OK on success, or SPI error code
 */
esp_err_t ADS1220_read_config(ADS1220_t* dev, ADS1220_Config_t *out);

/**
 * @brief Gets a preconfigured profile for common use cases.
 * 
 * @param preset The preset profile to retrieve
 * @param[out] cfg Pointer to configuration structure to store the preset values
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if preset is invalid
 */
esp_err_t ADS1220_get_preset_config(ADS1220_Preset_t preset, ADS1220_Config_t *cfg);

/**
 * @brief Writes all 4 configuration registers to the ADS1220.
 * 
 * @param dev Pointer to ADS1220 device instance
 * @param cfg Pointer to configuration structure with values to write
 * @return esp_err_t ESP_OK on success, or SPI error code
 */
esp_err_t ADS1220_write_config(ADS1220_t* dev, const ADS1220_Config_t *cfg);

/**
 * @brief Macro to generate simple command functions.
 * @internal
 */
#define ADS1220_CMD_FUNC(name, cmd) static inline esp_err_t ADS1220_##name(ADS1220_t* dev) { \
    const ADS1220Command_t data = cmd; \
    return spi_device_transmit(dev->spi_dev, &(spi_transaction_t){ .length = 8, .tx_buffer = &data }); }

/**
 * @brief Resets the ADS1220 to default configuration.
 * @param dev Pointer to ADS1220 device instance
 * @return esp_err_t ESP_OK on success
 */
ADS1220_CMD_FUNC(reset, ADS1220_RESET)

/**
 * @brief Powers down the ADS1220 to save power.
 * @param dev Pointer to ADS1220 device instance
 * @return esp_err_t ESP_OK on success
 */
ADS1220_CMD_FUNC(powerdown, ADS1220_POWERDOWN)

/**
 * @brief Starts or restarts conversions.
 * @param dev Pointer to ADS1220 device instance
 * @return esp_err_t ESP_OK on success
 */
ADS1220_CMD_FUNC(start, ADS1220_STARTSYNC)

/**
 * @brief Synchronizes conversions (same as start).
 * @param dev Pointer to ADS1220 device instance
 * @return esp_err_t ESP_OK on success
 */
ADS1220_CMD_FUNC(sync, ADS1220_STARTSYNC)

#ifdef __cplusplus
} /* extern "C" */
#endif


