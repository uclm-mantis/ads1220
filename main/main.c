// main/main.c  (ESP-IDF v5.x)
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS    5
#define PIN_NUM_DRDY  34

#define CMD_RESET 0x06
#define CMD_START 0x08
#define CMD_RDATA 0x10
#define CMD_WREG(a,n) (0x40 | (((a)&0x03)<<2) | ((n)-1))

typedef struct {
    uint8_t r0, r1, r2, r3;
} ads_cfg_t;

// --- Perfil: célula de carga (AIN0-1, G=128, DR=1000, VREF externa en REFP0/REFN0)
static const ads_cfg_t CFG_LOAD = {
    .r0 = 0b00011110, // MUX=000 (AIN0-1), GAIN=111(128), PGA=1
    .r1 = 0b11001000, // DR=110 (1000SPS), MODE=continuous
    .r2 = 0b01000000, // VREF = 01 (REFP0/REFN0), 50/60 off
    .r3 = 0x00
};

// --- Perfil: +-5V atenuado (AIN2-3, G=1, DR=1000, VREF interna)
static const ads_cfg_t CFG_P5V = {
    .r0 = 0b01110001, // MUX=111 (AIN2-3), GAIN=000(1), PGA=1
    .r1 = 0b11001000, // DR=1000SPS, continuous
    .r2 = 0b00000000, // VREF interna 2.048V
    .r3 = 0x00
};

static spi_device_handle_t hspi;

static esp_err_t ads_write_all(const ads_cfg_t *c){
    uint8_t buf[5] = { CMD_WREG(0,4), c->r0, c->r1, c->r2, c->r3 };
    spi_transaction_t t = { .length=8*5, .tx_buffer=buf };
    return spi_device_transmit(hspi, &t);
}

static inline int ads_data_ready(void){
    return gpio_get_level(PIN_NUM_DRDY)==0;
}

static int32_t ads_read_raw(void){
    uint8_t tx = CMD_RDATA, rx[3];
    spi_transaction_t t = {
        .length = 8, .tx_buffer = &tx,
        .rxlength = 24, .rx_buffer = rx
    };
    spi_device_transmit(hspi, &t);
    int32_t v = ((int32_t)rx[0]<<16)|((int32_t)rx[1]<<8)|rx[2];
    if (v & 0x800000) v |= 0xFF000000;
    return v;
}

static void ads_send(uint8_t c){
    spi_transaction_t t = { .length=8, .tx_buffer=&c };
    spi_device_transmit(hspi, &t);
}

static void ads_init(void){
    spi_bus_config_t bus = { PIN_NUM_MOSI, PIN_NUM_MISO, PIN_NUM_CLK, -1, -1, 0, 0 };
    spi_device_interface_config_t dev = { .clock_speed_hz=1*1000*1000, .mode=1, .spics_io_num=PIN_NUM_CS, .queue_size=2 };
    spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &dev, &hspi);
    gpio_config_t io = { .pin_bit_mask=1ULL<<PIN_NUM_DRDY, .mode=GPIO_MODE_INPUT, .pull_up_en=1 };
    gpio_config(&io);

    ads_send(CMD_RESET);
    vTaskDelay(pdMS_TO_TICKS(2));
    ads_write_all(&CFG_LOAD); // arranca en célula (da igual)
    ads_send(CMD_START);
}

// --- utilidades filtros (opcional)
static int32_t median5(int32_t a,int32_t b,int32_t c,int32_t d,int32_t e){
    int32_t v[5]={a,b,c,d,e};
    for(int i=1;i<5;i++){int32_t k=v[i],j=i-1;for(;j>=0&&v[j]>k;j--)v[j+1]=v[j];v[j+1]=k;}
    return v[2];
}
typedef struct{ double y,alpha; } iir1_t;
static void iir1_init(iir1_t *f,double fs,double fc){ f->y=0; f->alpha=1.0-exp(-2.0*M_PI*fc/fs); }
static inline double iir1_apply(iir1_t *f,double x){ f->y+=f->alpha*(x-f->y); return f->y; }

// --- escalado del canal +-5V
// Vin = (Vdiff)*3  (porque atenuaste 1/3). Vdiff [V] = raw/FS * Vref  (FS=2^23-1 aprox., GAIN=1)
static inline double raw_to_volts_p5v(int32_t raw){
    const double Vref = 2.048;       // interna
    const double FS = 8388607.0;     // 2^23-1
    double vdiff = (raw / FS) * Vref; // ±2.048V FSR
    return vdiff * 3.0;               // deshacer atenuación
}

static void app_task(void *arg){
    const int decim=4;               // 1000 -> 250SPS
    const double fs_out = 1000.0/decim;
    iir1_t iir_p5; iir1_init(&iir_p5, fs_out, 20.0); // p.ej. fc=20 Hz para el ±5V
    iir1_t iir_lc; iir1_init(&iir_lc, fs_out, 10.0); // célula

    // acumuladores
    int32_t acc_p5=0, acc_lc=0;
    int cp5=0, clc=0;
    int32_t win_p5[5]={0}, win_lc[5]={0}; int wp5=0, wlc=0;

    // Estado de canal actual
    enum {CHAN_LC, CHAN_P5} ch = CHAN_P5;
    ads_write_all(&CFG_P5V); // arrancamos leyendo el canal ±5V
    ads_send(CMD_START);

    while(1){
        while(!ads_data_ready()) vTaskDelay(pdMS_TO_TICKS(1));
        int32_t r = ads_read_raw();

        if (ch==CHAN_P5){
            acc_p5 += r; if (++cp5==decim){
                int32_t dec=acc_p5/decim; acc_p5=0; cp5=0;
                win_p5[wp5]=dec; wp5=(wp5+1)%5;
                int32_t med = median5(win_p5[0],win_p5[1],win_p5[2],win_p5[3],win_p5[4]);
                double vf = iir1_apply(&iir_p5, (double)med);
                double Vin = raw_to_volts_p5v((int32_t)vf);
                printf("[±5V] raw=%ld filt=%.0f Vin=%.3f V\n", (long)dec, vf, Vin);
            }
            // alterna a célula si también quieres muestrearla (mismo 250SPS aprox)
            ads_write_all(&CFG_LOAD); ch=CHAN_LC;
        } else {
            acc_lc += r; if (++clc==decim){
                int32_t dec=acc_lc/decim; acc_lc=0; clc=0;
                win_lc[wlc]=dec; wlc=(wlc+1)%5;
                int32_t med=median5(win_lc[0],win_lc[1],win_lc[2],win_lc[3],win_lc[4]);
                double yf=iir1_apply(&iir_lc,(double)med);
                printf("[LC ] raw=%ld filt=%.0f\n", (long)dec, yf);
            }
            ads_write_all(&CFG_P5V); ch=CHAN_P5;
        }
    }
}

void app_main(void){
    ads_init();
    xTaskCreate(app_task, "app", 4096, NULL, 5, NULL);
}
