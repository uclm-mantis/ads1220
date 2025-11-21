# ADS1220 Driver for ESP32

Multi-instance driver for the Texas Instruments ADS1220 24-bit ADC. Compatible with both ESP-IDF and Arduino IDE.

## Features

- ✅ **Multi-instance support** - Control multiple ADS1220 devices on the same SPI bus
- ✅ **Continuous data acquisition** - Interrupt-driven reading with callbacks
- ✅ **Preset configurations** - Ready-to-use profiles for common applications
- ✅ **Full register control** - Direct access to all ADS1220 configuration registers
- ✅ **Bit-field unions** - Readable and type-safe register configuration
- ✅ **Dual platform support** - Works with ESP-IDF and Arduino IDE

## Hardware Connections

| ADS1220 Pin | ESP32 Pin | Description |
|-------------|-----------|-------------|
| SCLK        | GPIO 18   | SPI Clock (configurable) |
| DIN (MOSI)  | GPIO 23   | SPI MOSI (configurable) |
| DOUT (MISO) | GPIO 19   | SPI MISO (configurable) |
| CS          | GPIO 5    | Chip Select (configurable, unique per device) |
| DRDY        | GPIO 34   | Data Ready interrupt (configurable, unique per device) |
| DVDD        | 3.3V      | Digital supply voltage |
| AVDD        | 3.3V      | Analog supply voltage |
| DGND        | GND       | Digital ground |
| AGND        | GND       | Analog ground |

## Installation

### ESP-IDF

#### Method 1: Managed Component (Recommended)

Add the component as a dependency using the ESP-IDF Component Manager. This method automatically manages updates and dependencies.

1. Create or edit `main/idf_component.yml` in your project:
   ```yaml
   dependencies:
     ads1220:
       git: https://github.com/yourusername/ADS1220.git
       version: ">=1.0.0"
   ```

2. Build your project:
   ```bash
   idf.py build
   ```

3. The component will be automatically downloaded to `managed_components/ads1220/`

4. Include in your code:
   ```c
   #include "ads1220.h"
   ```

**Alternative: Using a specific branch or commit:**
```yaml
dependencies:
  ads1220:
    git: https://github.com/yourusername/ADS1220.git
    path: "."
    version: main  # or a specific commit hash
```

#### Method 2: Manual Installation

1. Copy this repository to your project's `components/` directory:
   ```
   your_project/
   ├── components/
   │   └── ADS1220/
   ├── main/
   └── CMakeLists.txt
   ```

2. The component will be automatically detected and linked by ESP-IDF

### Arduino IDE

#### Method 1: Install from ZIP (Recommended)

1. Download the repository as a ZIP file:
   - Go to [https://github.com/uclm-mantis/ads1220](https://github.com/uclm-mantis/ads1220)
   - Click the green **Code** button
   - Select **Download ZIP**

2. In Arduino IDE, go to **Sketch → Include Library → Add .ZIP Library...**

3. Select the downloaded `ads1220-main.zip` file

4. The library will be installed and appear in **Sketch → Include Library → ADS1220**

#### Method 2: Manual Installation

1. Copy this repository to your Arduino libraries directory:
   - **Windows**: `Documents\Arduino\libraries\ADS1220\`
   - **macOS**: `~/Documents/Arduino/libraries/ADS1220/`
   - **Linux**: `~/Arduino/libraries/ADS1220/`

2. Restart Arduino IDE

3. The library will appear in **Sketch → Include Library → ADS1220**

## Quick Start

### ESP-IDF Example

```c
#include "ads1220.h"

void data_callback(int32_t raw_data) {
    printf("ADC: %ld\n", raw_data);
}

void app_main(void) {
    // Initialize SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

    // Create ADS1220 device
    ADS1220_init_config_t init_cfg = {
        .spi_host = SPI2_HOST,
        .cs_pin = 5,
        .drdy_pin = 34,
        .spi_clock_speed_hz = 1000000,
    };
    ADS1220_t* adc = ads1220_create(&init_cfg);

    // Get preset configuration
    ADS1220_Config_t config;
    ads1220_get_preset_config(ADS1220_PRESET_LOAD_CELL, &config);
    
    // Write configuration
    ads1220_write_config(adc, &config);
    
    // Start continuous acquisition
    ads1220_start_continuous(adc, data_callback);
}
```

### Arduino Example

```cpp
#include <SPI.h>
#include "ads1220.h"

#define CS_PIN 5
#define DRDY_PIN 34

ADS1220_t* adc;

void dataCallback(int32_t raw_data) {
    Serial.print("ADC: ");
    Serial.println(raw_data);
}

void setup() {
    Serial.begin(115200);
    SPI.begin();
    
    ADS1220_init_config_t init_cfg = {
        .spi_host = SPI2_HOST,
        .cs_pin = CS_PIN,
        .drdy_pin = DRDY_PIN,
        .spi_clock_speed_hz = 1000000,
    };
    adc = ads1220_create(&init_cfg);
    
    ADS1220_Config_t config;
    ads1220_get_preset_config(ADS1220_PRESET_LOAD_CELL, &config);
    ads1220_write_config(adc, &config);
    ads1220_start_continuous(adc, dataCallback);
}

void loop() {
    // Data is handled in callback
    delay(1000);
}
```

## API Reference

### Device Management

#### `ads1220_create()`
Creates and initializes an ADS1220 device instance.

```c
ADS1220_t* ads1220_create(const ADS1220_init_config_t* config);
```

**Parameters:**
- `config` - Initialization configuration (SPI host, CS pin, DRDY pin, clock speed)

**Returns:** Device handle or `NULL` on failure

---

#### `ads1220_destroy()`
Destroys an ADS1220 device instance and frees resources.

```c
void ads1220_destroy(ads1220_t* dev);
```

---

### Configuration

#### `ads1220_get_preset_config()`
Gets a preconfigured profile for common use cases.

```c
esp_err_t ads1220_get_preset_config(ADS1220_Preset_t preset, ADS1220_Config_t *cfg);
```

**Presets:**
- `ADS1220_PRESET_LOAD_CELL` - Load cell: AIN0-1, G=128, DR=600, VREF external (REFP0/REFN0)
- `ADS1220_PRESET_FERNANDO` - Fernando's config: AIN0-1, G=64, DR=2000, VREF internal

---

#### `ads1220_write_config()`
Writes all 4 configuration registers to the ADS1220.

```c
esp_err_t ads1220_write_config(ADS1220_t* dev, const ADS1220_Config_t *cfg);
```

---

#### `ads1220_read_config()`
Reads all 4 configuration registers from the ADS1220.

```c
esp_err_t ads1220_read_config(ADS1220_t* dev, ADS1220_Config_t *out);
```

---

### Data Acquisition

#### `ads1220_read_data()`
Reads the last conversion result (24-bit signed).

```c
esp_err_t ads1220_read_data(ADS1220_t* dev, int32_t *data);
```

---

#### `ads1220_start_continuous()`
Starts continuous data acquisition triggered by DRDY interrupt.

```c
esp_err_t ads1220_start_continuous(ADS1220_t* dev, ads1220_data_callback_t callback);
```

**Parameters:**
- `callback` - Function called when new data is ready: `void callback(int32_t raw_data)`

---

#### `ads1220_stop_continuous()`
Stops continuous data acquisition.

```c
void ads1220_stop_continuous(ADS1220_t* dev);
```

---

### Commands

#### `ads1220_reset()`
Resets the ADS1220 to default configuration.

```c
esp_err_t ads1220_reset(ADS1220_t* dev);
```

---

#### `ads1220_start()`
Starts or restarts conversions.

```c
esp_err_t ads1220_start(ADS1220_t* dev);
```

---

#### `ads1220_powerdown()`
Powers down the ADS1220 to save power.

```c
esp_err_t ads1220_powerdown(ADS1220_t* dev);
```

---

## Preset Configurations

### Load Cell (`ADS1220_PRESET_LOAD_CELL`)
Optimized for load cell measurements with external reference voltage.

- **Input:** AIN0-1 (differential)
- **Gain:** 128
- **Data Rate:** 2000 SPS (Turbo mode)
- **Reference:** External (REFP0/REFN0)
- **Mode:** Continuous conversion

### Fernando (`ADS1220_PRESET_FERNANDO`)
General-purpose configuration with internal reference.

- **Input:** AIN0-1 (differential)
- **Gain:** 64
- **Data Rate:** 2000 SPS (Turbo mode)
- **Reference:** Internal 2.048V
- **Mode:** Continuous conversion

## Advanced Usage

### Custom Configuration

You can create custom configurations by directly setting the bit-fields:

```c
ADS1220_Config_t config = {
    .mux = ADS1220_MUX_AIN0_AIN1,
    .gain = ADS1220_GAIN_128,
    .pga_bypass = 0,
    .dr = ADS1220_DR_TURBO_2000SPS,
    .mode = ADS1220_MODE_TURBO,
    .cm = 1,  // Continuous conversion
    .vref = ADS1220_VREF_REFP0_REFN0,
    .fir = ADS1220_FIR_OFF,
    .idac = ADS1220_IDAC_OFF,
    // ... other fields
};

ads1220_write_config(adc, &config);
```

### Multiple Devices

```c
// Device 1
ADS1220_init_config_t cfg1 = {
    .spi_host = SPI2_HOST,
    .cs_pin = 5,
    .drdy_pin = 34,
    .spi_clock_speed_hz = 1000000,
};
ADS1220_t* adc1 = ads1220_create(&cfg1);

// Device 2 (same SPI bus, different CS and DRDY)
ADS1220_init_config_t cfg2 = {
    .spi_host = SPI2_HOST,
    .cs_pin = 15,
    .drdy_pin = 35,
    .spi_clock_speed_hz = 1000000,
};
ADS1220_t* adc2 = ads1220_create(&cfg2);
```

## Troubleshooting

**Problem:** No data received
- Check SPI connections (MOSI, MISO, SCLK, CS)
- Verify DRDY pin is connected and configured correctly
- Ensure power supply is stable (3.3V)

**Problem:** Noisy readings
- Use proper grounding
- Add decoupling capacitors near ADS1220 (0.1µF + 10µF)
- Keep analog and digital grounds separate, connect at single point
- Use shielded cables for analog inputs

**Problem:** Build errors in Arduino IDE
- Ensure library is in correct folder
- Restart Arduino IDE after installation
- Check that ESP32 board support is installed

## License

MIT License - see [LICENSE](LICENSE) file for details

## Collaborators:

- Francisco Moya Fernández
- Fernando Castillo García
- Antonio González Rodríguez
- David Rodríguez Rosa
- Sergio Juárez Pérez
- Andrea Martín Parra
- Raúl Cuadros Tardío
- Juan Sánchez Medina

## References

- [ADS1220 Datasheet](https://www.ti.com/lit/ds/symlink/ads1220.pdf)
- [ESP-IDF SPI Master Driver](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/spi_master.html)
