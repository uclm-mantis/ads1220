#ifndef ADS1220_H
#define ADS1220_H

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
 *  Estructura de comando
 * =========================
 * Disposición little-endian (toolchains ESP32):
 *   F7:
 *     bit7..1: opcode7
 *     bit0   : x (don't care, LSB)
 *   F4:
 *     bit7..4: opcode4
 *     bit3..2: r
 *     bit1..0: n  (n = count-1)
 */
typedef union __attribute__((packed)) {
    uint8_t u8;   /* byte crudo para SPI */

    /* F7: 1 LSB don't-care + 7 MSBs opcode */
    struct {
        uint8_t x       : 1;  /* bit0 (don't care, LSB) */
        uint8_t opcode7 : 7;  /* bits1..7 (opcode) */
    } f7;

    /* F4: 4-bit opcode + r (2b) + n (2b) */
    struct {
        uint8_t n       : 2;  /* bits0..1: count-1 (0..3 => 1..4 regs) */
        uint8_t r       : 2;  /* bits2..3: reg inicial (0..3) */
        uint8_t opcode4 : 4;  /* bits4..7 */
    } f4;

} ADS1220Command_t;


/* =========================
 *  Comandos según datasheet
 * =========================
 * Fijos (F7):
 *   RESET      = 0000 011x
 *   START/SYNC = 0000 100x
 *   POWERDOWN  = 0000 001x
 *   RDATA      = 0001 000x
 *
 * Paramétricos (F4):
 *   RREG       = 0010 r r n   (lee n+1 registros)
 *   WREG       = 0100 r r n   (escribe n+1 registros)
 */

/* --- Fijos (F7) --- */
static const ADS1220Command_t ADS1220_RESET     = { .f7 = { .x = 0, .opcode7 = 0b0000011 } };
static const ADS1220Command_t ADS1220_STARTSYNC = { .f7 = { .x = 0, .opcode7 = 0b0000100 } };
static const ADS1220Command_t ADS1220_POWERDOWN = { .f7 = { .x = 0, .opcode7 = 0b0000001 } };
static const ADS1220Command_t ADS1220_RDATA     = { .f7 = { .x = 0, .opcode7 = 0b0001000 } };

/* --- Enum de registros --- */
typedef enum {
    ADS1220_REG0 = 0,   /* Config 0 */
    ADS1220_REG1,       /* Config 1 */
    ADS1220_REG2,       /* Config 2 */
    ADS1220_REG3,       /* Config 3 */
} ADS1220_Reg_t;

/* =========================
 *  Constructores paramétricos
 * =========================
 * count: número real de registros (1..4). El bitfield trunca automáticamente.
 */
static inline ADS1220Command_t ADS1220_RREG(ADS1220_Reg_t reg, uint8_t count) { 
    return (ADS1220Command_t){ .f4 = { .n = (uint8_t)(count - 1), .r = reg, .opcode4 = 0b0010 } };
}

static inline ADS1220Command_t ADS1220_WREG(ADS1220_Reg_t reg, uint8_t count) { 
    return (ADS1220Command_t){ .f4 = { .n = (uint8_t)(count - 1), .r = reg, .opcode4 = 0b0100 } };
}


typedef union {
    uint8_t reg[4];  // acceso crudo a los 4 registros
    struct __attribute__((packed)) {
        // REG0
        uint8_t pga_bypass : 1; // 1 = desactiva PGA, 0 = activo
        uint8_t gain       : 3; // ganancia PGA: 1..128
        uint8_t mux        : 4; // selección de entrada diferencial/single-ended
        // REG1
        uint8_t ts         : 1; // 1 = sensor temp interno
        uint8_t vref       : 1; // 0 = interna 2.048 V, 1 = externa REFp/n
        uint8_t cm         : 1; // 0 = normal, 1 = común-modo detector
        uint8_t mode       : 1; // 0 = single-shot, 1 = continuous
        uint8_t dr         : 3; // data rate
        uint8_t drdy_mode  : 1; // 1 = pin DOUT/DRDY solo DRDY, 0 = DOUT/DRDY combinado
        // REG2
        uint8_t idac_mag   : 3; // magnitude 10µA..1500µA
        uint8_t idac2      : 2; // routing de IDAC2
        uint8_t idac1      : 2; // routing de IDAC1
        uint8_t idac       : 1; // corriente de excitación
        // REG3
        uint8_t vref_sel   : 2; // selección de referencia extra
        uint8_t i2mux      : 2; // multiplexor de IDAC routing extra
        uint8_t fir_conf   : 1; // 0=default, 1=50/60Hz rejection
        uint8_t reserved   : 3;
    };
} ADS1220Config_t;

typedef enum : uint8_t {
    ADS1220_MODE_SSHOT   = 0,
    ADS1220_MODE_CONTINUOUS,
} ADS1220_Mode_t;

typedef enum : uint8_t {
    ADS1220_GAIN_1   = 0,
    ADS1220_GAIN_2,
    ADS1220_GAIN_4,
    ADS1220_GAIN_8,
    ADS1220_GAIN_16,
    ADS1220_GAIN_32,
    ADS1220_GAIN_64,
    ADS1220_GAIN_128,
} ADS1220_Gain_t;

/* ===== MUX (REG0.mux) ===== */
typedef enum : uint8_t {
    ADS1220_MUX_AIN0_AIN1  = 0x0,
    ADS1220_MUX_AIN0_AIN2,
    ADS1220_MUX_AIN0_AIN3,
    ADS1220_MUX_AIN1_AIN2,
    ADS1220_MUX_AIN1_AIN3,
    ADS1220_MUX_AIN2_AIN3,
    ADS1220_MUX_AIN1_AIN0,
    ADS1220_MUX_AIN3_AIN2,
    ADS1220_MUX_AIN0_GND,
    ADS1220_MUX_AIN1_GND,
    ADS1220_MUX_AIN2_GND,
    ADS1220_MUX_AIN3_GND,
    ADS1220_MUX_REFPm_REFN,
    ADS1220_MUX_VDD_DIV2,
    ADS1220_MUX_TEMPSENSOR,
    ADS1220_MUX_SHORTED,
} ADS1220_Mux_t;

/* ===== Data Rate (REG1.dr) ===== */
typedef enum : uint8_t {
    ADS1220_DR_20SPS   = 0,
    ADS1220_DR_45SPS,
    ADS1220_DR_90SPS,
    ADS1220_DR_175SPS,
    ADS1220_DR_330SPS,
    ADS1220_DR_600SPS,
    ADS1220_DR_1000SPS,
    ADS1220_DR_2000SPS,
} ADS1220_DR_t;

/* ===== IDAC magnitude (REG2.idac_mag) ===== */
typedef enum : uint8_t {
    ADS1220_IDAC_OFF    = 0,
    ADS1220_IDAC_10uA,
    ADS1220_IDAC_50uA,
    ADS1220_IDAC_100uA,
    ADS1220_IDAC_250uA,
    ADS1220_IDAC_500uA,
    ADS1220_IDAC_1000uA,
    ADS1220_IDAC_1500uA,
} ADS1220_IdacMag_t;

/* ===== IDAC routing (REG2.idac1, idac2) ===== */
typedef enum : uint8_t {
    ADS1220_IDAC_ROUTE_AIN0 = 0,
    ADS1220_IDAC_ROUTE_AIN1,
    ADS1220_IDAC_ROUTE_AIN2,
    ADS1220_IDAC_ROUTE_AIN3,
    ADS1220_IDAC_ROUTE_REFP,
    ADS1220_IDAC_ROUTE_REFN,
    ADS1220_IDAC_ROUTE_OFF,
} ADS1220_IdacRoute_t;

/* ===== VREF select (REG3.vref_sel) ===== */
typedef enum : uint8_t {
    ADS1220_VREF_INT    = 0,  // interna 2.048V
    ADS1220_VREF_REFP0N0= 1,  // REFP0/REFN0
    ADS1220_VREF_REFP1N1= 2,  // REFP1/REFN1 (si disponible)
    ADS1220_VREF_SUPPLY = 3,  // AVDD-AVSS
} ADS1220_VrefSel_t;

/* ===== I2MUX (REG3.i2mux) ===== */
typedef enum : uint8_t {
    ADS1220_I2MUX_OFF  = 0,
    ADS1220_I2MUX_AIN0,
    ADS1220_I2MUX_AIN1,
    ADS1220_I2MUX_AIN2,
} ADS1220_I2mux_t;


#if (defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L) || defined(__cplusplus)
_Static_assert(sizeof(ADS1220Command_t) == 1, "ADS1220Command_t must be 1 byte");
_Static_assert(sizeof(ADS1220Config_t) == 4, "ADS1220Config_t debe ocupar 4 bytes");
#endif


static inline esp_err_t ADS1220_read_config(spi_device_handle_t dev,
                                            ADS1220Config_t *out)
{
    ADS1220Command_t rreg = ADS1220_RREG(ADS1220_REG0, 4);
    esp_err_t err = spi_device_transmit(dev, &(spi_transaction_t){ .length = 8, .tx_buffer = &rreg });
    if (err != ESP_OK) return err;

    spi_transaction_t t2 = {.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA, .length = 32 };
    err = spi_device_transmit(dev, &t2);
    if (err != ESP_OK) return err;

    *out = *(ADS1220Config_t *)t2.rx_data;
    return ESP_OK;
}

typedef struct __attribute__((packed)) {
    ADS1220Command_t cmd;
    ADS1220Config_t  cfg;
} ADS1220ConfigTransactionBuffer_t;

static inline esp_err_t ADS1220_write_config(spi_device_handle_t dev,
                                             const ADS1220Config_t *cfg)
{
    spi_transaction_t t = { .length = 8 * sizeof(ADS1220ConfigTransactionBuffer_t), 
        .tx_buffer = &(ADS1220ConfigTransactionBuffer_t){ .cmd = ADS1220_WREG(ADS1220_REG0, 4), .cfg = *cfg } };
    return spi_device_transmit(dev, &t);
}

#define ADS1220_CMD_FUNC0(name, cmd) static inline esp_err_t ADS1220_##name(spi_device_handle_t dev) { \
    const ADS1220Command_t data = cmd; \ 
    return spi_device_transmit(dev, &(spi_transaction_t){ .length = 8, .tx_buffer = &data }); }

/* 
esp_err_t ADS1220_reset(spi_device_handle_t dev);       // resets ADS1220
esp_err_t ADS1220_powerdown(spi_device_handle_t dev);   // powerdown mode
esp_err_t ADS1220_start(spi_device_handle_t dev);       // starts continuous mode
esp_err_t ADS1220_sync(spi_device_handle_t dev);        // reinicia filtro digital/modulador 
esp_err_t ADS1220_req_data(spi_device_handle_t dev);    // requests single shot read
*/

ADS1220_CMD_FUNC0(reset, ADS1220_RESET)
ADS1220_CMD_FUNC0(powerdown, ADS1220_POWERDOWN)
ADS1220_CMD_FUNC0(start, ADS1220_STARTSYNC)
ADS1220_CMD_FUNC0(sync, ADS1220_STARTSYNC)
ADS1220_CMD_FUNC0(req_data, ADS1220_RDATA)


typedef struct {
    spi_host_device_t host;         // SPI1_HOST, SPI2_HOST
    int miso_io, mosi_io, sclk_io, cs_io;
    int drdy_io;

    int clock_hz;                   // por defecto 1 MHz
    uint8_t spi_mode;               // por defecto 1 (CPOL=0, CPHA=1)
    int queue_size;                 // por defecto 8
    int dma_chan;                   // por defecto SPI_DMA_CH_AUTO

    int32_t *ringbuf;               // provisto por el usuario
    size_t   ringbuf_capacity;

} ADS1220SPIConfig_t;





#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ADS1220_H */
