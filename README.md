# LM75A Driver
This is a driver for the LM75A temperature sensor written in C.

## Understand LM75A Basics
- **I2C Addressing**: The LM75A supports 7-bit addressing, where the first four bits are fixed as 1001. The last three bits are configured using pins A2, A1, and A0​
- **Registers**:
    - **Temperature**: Address 0x00, 9-bit data.
    - **Configuration**: Address 0x01, for setting operating modes.
    - **$T_{HYST}$ and $T_{OS}$**: Addresses 0x02 and 0x03, for hysteresis and over-temperature settings​
    - **I2C Operations**: It supports read and write commands, with a pointer register selecting the target data​


## `glitch_ignore_cnt`
Filter out noise or glitches on the I2C bus

### Purpose of glitch_ignore_cnt
 - I2C lines (SDA and SCL) are vulnerable to noise and signal glitches, especially in electrically noisy environments.
 - `glitch_ignore_cnt` defines how many short-duration glitches on the SDA and SCL lines the driver should ignore.
 - This filtering improves communication stability and prevents the ESP32 from misinterpreting brief noise as valid signals.
### How It Works
 - The driver will ignore pulses on the I2C lines that are shorter than the filter count (glitch_ignore_cnt).
 - Higher values provide more noise immunity but may slow down the bus.
 - Lower values offer faster communication but are more sensitive to noise.
### Impact on I2C Communication
| Value | Effect |
| ----- | ------ |
|    0    | No glitch filtering (fast but risky) |
| 1–7 (Default) |	Moderate filtering for typical use |
|  8–15  | Strong filtering for noisy environments |


## 🔥 $T_{OS}$ (Overtemperature Shutdown Threshold)
The $T_{OS}$ register defines the maximum temperature limit.\
If the measured temperature exceeds this limit, the LM75A triggers an overtemperature alert by activating the OS (Overtemperature Shutdown) pin. Default value is 80°C.

### Purpose
Used for thermal protection to prevent devices from overheating.

### Default Behavior
- **Comparator Mode (Default)**: OS pin stays active until the temperature drops below $T_{HYST}$.
- **Interrupt Mode**: OS pin deactivates after any read operation.

### Register Address: 0x03

### Example
If $T_{OS}$ is set to 80°C, the LM75A will assert the OS pin when the temperature reaches 80°C.


## ❄️ $T_{HYST}$ (Hysteresis Threshold)
The $T_{HYST}$ register sets a lower temperature limit to reset the overtemperature alert after it has been triggered. Default value is 75°C.

### Purpose
- Prevents the sensor from rapidly toggling the OS pin when the temperature hovers around the $T_{OS}$ threshold.

### Register Address: 0x02

### Example
If $T_{OS}$ is set to 80°C and $T_{HYST}$ is set to 75°C:
- The OS pin will be triggered when the temperature reaches 80°C.
- The OS pin will only deactivate when the temperature drops below 75°C.


## 🔄 How $T_{OS}$ and $T_{HYST}$ Work Together
This hysteresis prevents false triggers caused by small temperature fluctuations.
1. Rising Temperature:
    - If T > $T_{OS}$, the OS pin is activated.

2. Falling Temperature:
    - The OS pin remains active until the temperature drops below $T_{HYST}$.


## ⚙️ Configuration Register
The Configuration Register allows you to configure the operating mode and other settings of the LM75A sensor.

### Purpose
- Configure the sensor's operating mode (Comparator or Interrupt).
- Enable or disable the OS pin.

### Register Address: 0x01

### Configuration Bits
| Bit | Name       | Description                                      |
| --- | ---------- | ------------------------------------------------ |
| 7   | OS_POL     | OS polarity (0: Active low, 1: Active high)      |
| 6   | OS_F_QUE   | OS fault queue (00: 1 fault, 01: 2 faults, 10: 4 faults, 11: 6 faults) |
| 5   | OS_COMP_INT| OS mode (0: Comparator, 1: Interrupt)            |
| 4   | SHUTDOWN   | Shutdown mode (0: Normal, 1: Shutdown)           |
| 3-0 | Reserved   | Reserved bits                                    |

### Example
To set the sensor to Interrupt mode with active high polarity and 4 faults queue:
- Write `0b01001000` to the Configuration Register.


## 🌡️ Temperature Register
The Temperature Register holds the current temperature reading from the LM75A sensor.

### Purpose
- Read the current temperature in Celsius.

### Register Address: 0x00

### Data Format
- The temperature data is a 9-bit two's complement value.
- The MSB is the most significant byte, and the LSB is the least significant byte.
- The temperature is represented in 0.5°C increments.

### Example
To read the temperature:
- Read 2 bytes from the Temperature Register.
- Combine the bytes and shift right by 7 to get the 9-bit value.
- Multiply by 0.5 to get the temperature in Celsius.

## Example of Usage
```c
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>

#include "lm75a.h"

static const char *TAG = "MAIN";

/* ------------------------------------------------------------------
 * Existing or newly introduced #defines in main
 * Adjust these to match your board / wiring
 * ------------------------------------------------------------------ */
#define I2C_MASTER_SDA_IO  21
#define I2C_MASTER_SCL_IO  22
#define I2C_MASTER_FREQ_HZ 400000
#define LM75A_I2C_ADDR     0x48
#define LM75A_OS_PIN       23

static volatile uint8_t interrupt_count = 0;
static volatile uint8_t last_counter = 0;

// OS interrupt handler
static void os_interrupt_handler(void *arg)
{
  // Toggle LED (example on GPIO2)
  gpio_set_level(GPIO_NUM_2, !gpio_get_level(GPIO_NUM_2));
  // Increment interrupt counter
  interrupt_count++;
}

void app_main(void)
{
  // Configure an LED as output on GPIO2 (example)
  gpio_config_t led_conf = {.pin_bit_mask = (1ULL << GPIO_NUM_2),
                            .mode = GPIO_MODE_OUTPUT,
                            .pull_up_en = GPIO_PULLUP_DISABLE,
                            .pull_down_en = GPIO_PULLDOWN_DISABLE,
                            .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&led_conf);

  // 1) Simplified LM75A init if you have a function like this
  lm75a_t *lm75a = lm75a_create((lm75a_config_t){
    .i2c_port = I2C_NUM_0,
    .master_sda_pin = I2C_MASTER_SDA_IO,
    .master_scl_pin = I2C_MASTER_SCL_IO,
    .os_pin = LM75A_OS_PIN,
    .i2c_frequency = I2C_MASTER_FREQ_HZ,
    .address = LM75A_I2C_ADDR,
  });

  lm75a_status_t status;

  status = lm75a->init(lm75a);
  if (status != LM75A_OK)
  {
    ESP_LOGE(TAG, "Failed to initialize LM75A: %s", lm75a_status_to_string(status));
    return;
  }

  // 2) Optional: set an interrupt on the OS pin
  status = lm75a->set_os_interrupt(lm75a, os_interrupt_handler, NULL);
  if (status != LM75A_OK)
  {
    ESP_LOGE(TAG, "Failed to set OS interrupt: %s", lm75a_status_to_string(status));
    return;
  }

  // 3) Set THYST
  status = lm75a->set_thys(lm75a, -26, true);
  if (status != LM75A_OK)
  {
    ESP_LOGE(TAG, "Failed to set THYST: %s", lm75a_status_to_string(status));
    return;
  }

  // 3.5) Read THYST
  float thys;
  status = lm75a->get_thys(lm75a, &thys);
  if (status != LM75A_OK)
  {
    ESP_LOGE(TAG, "Failed to get THYST: %s", lm75a_status_to_string(status));
    return;
  }
  ESP_LOGI(TAG, "THYST: %.1f°C", thys);

  // 4) Set TOS
  status = lm75a->set_tos(lm75a, 14, false);
  if (status != LM75A_OK)
  {
    ESP_LOGE(TAG, "Failed to set TOS: %s", lm75a_status_to_string(status));
    return;
  }

  // 4.5) Read TOS
  float tos;
  status = lm75a->get_tos(lm75a, &tos);
  if (status != LM75A_OK)
  {
    ESP_LOGE(TAG, "Failed to get TOS: %s", lm75a_status_to_string(status));
    return;
  }
  ESP_LOGI(TAG, "TOS: %.1f°C", tos);

  // 5) Set OS mode
  status = lm75a->set_os_mode(lm75a, LM75A_OS_INTERRUPT_MODE);
  if (status != LM75A_OK)
  {
    ESP_LOGE(TAG, "Failed to set OS mode: %s", lm75a_status_to_string(status));
    return;
  }

  // 5) Get and log product ID
  // .
  // .
  // .

  // 5) Main loop reading temperature
  while (1)
  {
    float temperature;
    status = lm75a->read_temperature(lm75a, &temperature, LM75A_SCALE_CELSIUS);
    if (status != LM75A_OK)
    {
      ESP_LOGE(TAG, "Failed to read temperature: %s", lm75a_status_to_string(status));
    }
    else
    {
      ESP_LOGI(TAG, "Temp: %.2f°C | Interrupt Count: %d", temperature, interrupt_count);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (interrupt_count > last_counter)
    {
      last_counter = interrupt_count;
      ESP_LOGW(TAG, "Interrupt detected! Counter: %d", interrupt_count);
      ESP_LOGW(TAG, "GPIO2: %d", gpio_get_level(GPIO_NUM_2));
    }
  }

  // 7) Clean up (unreachable in this loop example)
  lm75a_destroy(lm75a);
}
```


## Future Works
- [ ] Test with negative temperature values, i.e., submit the sensor to a cold environment and verify if the measures (read_temperature) and the triggers ($T_{OS}$, $T_{HYST}$) are working properly.


## References
1. [LM75A open source - 13xiaobang](https://github.com/13xiaobang/esp32_lm75a/tree/master/main)
2. [LM75A open source - UncleRus](https://github.com/UncleRus/esp-idf-lib/tree/master/components/lm75)
