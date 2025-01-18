/**
 * @file lm75a.c
 * @brief ESP32 Driver for LM75A Temperature Sensor (Updated with new I2C API)
 * @version 0.2
 * @date 2024-12-23
 * @author Pedro Luis Dionísio Fraga
 * @copyright Copyright (c) 2024
 */

#include "lm75a.h"

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_intr_alloc.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//==================== CONFIGURATIONS ====================//
#define BASE_LM75A_DEV_ADDR 0b1001                                // Base I2C address for LM75A
#define LM75A_DEV_ADDR      ((BASE_LM75A_DEV_ADDR << 3) | 0b000)  // Final I2C address

#define LM75A_TEMP_REG    0x00  // Temperature register
#define LM75A_CONF_REG    0x01  // Configuration register
#define LM75A_THYST_REG   0x02  // Hysteresis register
#define LM75A_TOS_REG     0x03  // Overtemperature shutdown register
#define LM75A_PROD_ID_REG 0x07  // Manufacturer ID register

// Configuration Bits
#define LM75A_SHUTDOWN_BIT (1 << 0)  // Shutdown Mode
#define LM75A_OS_MODE_BIT  (1 << 1)  // OS Mode (Comparator/Interrupt)
#define LM75A_OS_POL_BIT   (1 << 2)  // OS Polarity (Active Low/High)

#define ESP_INTR_FLAG_DEFAULT 0

#define DEFAULT_GLITCH_FILTER 7
// Helper to detect half-degree bit
#define LM75A_MSB_BIT_IN_BYTE  0x80
#define LM75A_HALF_DEGREE_MASK (1 << 7)
#define LM75A_SIGNAL_BIT       (1 << 8)  // 1 if negative, 0 if positive

#define LM75A_WAIT_READ_FOREVER -1

#define DEBUG 1

// Macros
#define DEBUG_PRINT_ERR(fmt, ...)        \
  do                                     \
  {                                      \
    if (DEBUG)                           \
      ESP_LOGE(TAG, fmt, ##__VA_ARGS__); \
  } while (0)

#define LM75A_CHECK_INSTANCE(evaluated, ret)           \
  if (!evaluated)                                      \
  {                                                    \
    DEBUG_PRINT_ERR("Error: LM75A_ERR_INVALID_PARAM"); \
    return ret;                                        \
  }

#define UNUSED(x) (void)(x)

static const char *TAG = "LM75A";

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_lm75a_device = NULL;

//==================== I2C INITIALIZATION ====================//
static esp_err_t i2c_master_init(lm75a_config_t *cfg)
{
  esp_err_t ret;

  i2c_master_bus_config_t bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,  // Clock source for I2C peripheral
    .i2c_port = cfg->i2c_port,
    .scl_io_num = cfg->master_scl_pin,
    .sda_io_num = cfg->master_sda_pin,
    .glitch_ignore_cnt = DEFAULT_GLITCH_FILTER,  // Filter out short glitches (default 7)
    .flags.enable_internal_pullup = true,        // Enable internal pull-up resistors
  };
  ret = i2c_new_master_bus(&bus_config, &s_i2c_bus);
  if (ret != ESP_OK)
    return ret;

  i2c_device_config_t device_config = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,  // 7-bit I2C address
    .device_address = cfg->address,
    .scl_speed_hz = cfg->i2c_frequency,
  };
  ret = i2c_master_bus_add_device(s_i2c_bus, &device_config, &s_lm75a_device);
  if (ret != ESP_OK)
    return ret;

  return ESP_OK;
}

lm75a_status_t lm75a_init(lm75a_t *self)
{
  LM75A_CHECK_INSTANCE(self, LM75A_ERR_INVALID_PARAM);
  return (i2c_master_init(&self->config) != ESP_OK) ? LM75A_ERR_I2C_INIT : LM75A_OK;
}

lm75a_status_t lm75a_read_temperature(lm75a_t *self, float *out_temp, lm75a_scale_t scale)
{
  LM75A_CHECK_INSTANCE(self, LM75A_ERR_INVALID_PARAM);
  LM75A_CHECK_INSTANCE(out_temp, LM75A_ERR_INVALID_PARAM);

  uint8_t temp_reg = LM75A_TEMP_REG;
  uint8_t data[2] = {0};

  esp_err_t ret = i2c_master_transmit_receive(s_lm75a_device,
                                              &temp_reg,
                                              sizeof(temp_reg),
                                              data,
                                              sizeof(data),
                                              LM75A_WAIT_READ_FOREVER);
  if (ret != ESP_OK)
    return LM75A_ERR_READ_TEMP;

  // Process the temperature data (9-bit two's complement format)
  int16_t raw_temp = (data[0] << 8) | data[1];
  raw_temp >>= 7;

  // Verify if the temperature is negative
  float celsius;
  celsius = raw_temp * 0.5f;
  if (raw_temp & LM75A_SIGNAL_BIT)
    celsius = -celsius;

  switch (scale)
  {
    case LM75A_SCALE_FAHRENHEIT:
      *out_temp = (celsius * 9 / 5) + 32;
      break;
    case LM75A_SCALE_KELVIN:
      *out_temp = celsius + 273.15;
      break;
    case LM75A_SCALE_CELSIUS:
    default:
      *out_temp = celsius;
      break;
  }

  return LM75A_OK;
}

lm75a_status_t lm75a_set_shutdown(lm75a_t *self, bool enable)
{
  LM75A_CHECK_INSTANCE(self, LM75A_ERR_INVALID_PARAM);

  uint8_t buf[2] = {0};
  buf[0] = LM75A_CONF_REG;
  buf[1] = ((uint8_t)enable & 0x01) ? (LM75A_SHUTDOWN_BIT)
                                    : 0;  // (if not works, put the `<< 1` outside the parenthesis)

  LM75A_CHECK_INSTANCE(s_lm75a_device, LM75A_ERR_DEV_HANDLE);
  esp_err_t ret = i2c_master_transmit(s_lm75a_device, buf, sizeof(buf), LM75A_WAIT_READ_FOREVER);

  return (ret == ESP_OK) ? LM75A_OK : LM75A_ERR_I2C_WRITE;
}

lm75a_status_t lm75a_set_os_mode(lm75a_t *self, lm75a_os_mode_t mode)
{
  LM75A_CHECK_INSTANCE(self, LM75A_ERR_INVALID_PARAM);

  uint8_t buf[2] = {0};
  buf[0] = LM75A_CONF_REG;
  buf[1] = ((uint8_t)mode & 0x01) ? LM75A_OS_MODE_BIT
                                  : 0;  // `&` operation OS_MODE is 0 or 1 preventing invalid data.

  LM75A_CHECK_INSTANCE(s_lm75a_device, LM75A_ERR_DEV_HANDLE);
  esp_err_t ret = i2c_master_transmit(s_lm75a_device, buf, sizeof(buf), LM75A_WAIT_READ_FOREVER);
  return (ret == ESP_OK) ? LM75A_OK : LM75A_ERR_I2C_WRITE;
}

lm75a_status_t lm75a_set_polarity(lm75a_t *self, lm75a_os_polarity_t pol)
{
  LM75A_CHECK_INSTANCE(self, LM75A_ERR_INVALID_PARAM);

  uint8_t buf[2] = {0};
  buf[0] = LM75A_CONF_REG;
  buf[1] = ((uint8_t)pol & 0x01) ? LM75A_OS_POL_BIT : 0;

  esp_err_t ret = i2c_master_transmit(s_lm75a_device, buf, sizeof(buf), LM75A_WAIT_READ_FOREVER);
  return (ret != ESP_OK) ? LM75A_OK : LM75A_ERR_I2C_WRITE;
}

lm75a_status_t lm75a_set_fault_queue(lm75a_t *self, lm75a_fault_queue_t fault_queue)
{
  LM75A_CHECK_INSTANCE(self, LM75A_ERR_INVALID_PARAM);

  uint8_t buf[2] = {0};
  buf[0] = LM75A_CONF_REG;
  buf[1] = ((uint8_t)fault_queue & 0x03) << 0x03;  // The second `0x03` is the mask for the 2 LSBs

  esp_err_t ret = i2c_master_transmit(s_lm75a_device, buf, sizeof(buf), LM75A_WAIT_READ_FOREVER);
  return (ret == ESP_OK) ? LM75A_OK : LM75A_ERR_I2C_WRITE;
}

lm75a_status_t lm75a_set_tos(lm75a_t *self, int16_t tos, bool add_half_degree)
{
  LM75A_CHECK_INSTANCE(self, LM75A_ERR_INVALID_PARAM);

  DEBUG_PRINT_ERR("tos: 0x%02X", (uint8_t)tos);

  uint8_t buf[3] = {0};
  buf[0] = LM75A_TOS_REG;
  if (tos < 0)
  {
    tos = -tos;
    buf[1] = (tos & 0xFF) | (1 << 7);
  }
  else
    buf[1] = (tos & 0xFF);

  buf[2] = add_half_degree ? LM75A_HALF_DEGREE_MASK : 0x00;

  esp_err_t ret = i2c_master_transmit(s_lm75a_device, buf, sizeof(buf), LM75A_WAIT_READ_FOREVER);
  return (ret == ESP_OK) ? LM75A_OK : LM75A_ERR_I2C_WRITE;
}

lm75a_status_t lm75a_set_thys(lm75a_t *self, int16_t thys, bool add_half_degree)
{
  LM75A_CHECK_INSTANCE(self, LM75A_ERR_INVALID_PARAM);

  uint8_t buf[3] = {0};
  buf[0] = LM75A_THYST_REG;

  if (thys < 0)
  {
    thys = -thys;
    buf[1] = (thys & 0xFF) | (1 << 7);
  }
  else
    buf[1] = (thys & 0xFF);
  buf[2] = add_half_degree ? LM75A_HALF_DEGREE_MASK : 0x00;

  esp_err_t ret = i2c_master_transmit(s_lm75a_device, buf, sizeof(buf), LM75A_WAIT_READ_FOREVER);
  return (ret == ESP_OK) ? LM75A_OK : LM75A_ERR_I2C_WRITE;
}

lm75a_status_t lm75a_get_tos(lm75a_t *self, float *out_tos)
{
  LM75A_CHECK_INSTANCE(self, LM75A_ERR_INVALID_PARAM);

  uint8_t reg = LM75A_TOS_REG;
  uint8_t buf[2] = {0};

  esp_err_t ret = i2c_master_transmit_receive(s_lm75a_device,
                                              &reg,
                                              sizeof(reg),
                                              buf,
                                              sizeof(buf),
                                              LM75A_WAIT_READ_FOREVER);
  if (ret != ESP_OK)
    return LM75A_ERR_I2C_READ;

  uint16_t raw_tos = (buf[0] << 8) | buf[1];
  raw_tos >>= 7;
  ESP_LOGW(TAG, "raw_tos: 0x%04X", raw_tos);

  // If the least significant bit (LSB) of the temperature data, which represents 0.5°C, around up
  // the number. It uses the 0x80 bit because D0–D6 is undefined.
  float tos;
  tos = raw_tos * 0.5;

  if (raw_tos & LM75A_SIGNAL_BIT)
  {
    raw_tos &= ~LM75A_SIGNAL_BIT;
    tos = -(raw_tos * 0.5);
  }

  *out_tos = tos;

  return LM75A_OK;
}

lm75a_status_t lm75a_get_thys(lm75a_t *self, float *out_thys)
{
  LM75A_CHECK_INSTANCE(self, LM75A_ERR_INVALID_PARAM);

  uint8_t reg = LM75A_THYST_REG;
  uint8_t buf[2] = {0};

  esp_err_t ret = i2c_master_transmit_receive(s_lm75a_device,
                                              &reg,
                                              sizeof(reg),
                                              buf,
                                              sizeof(buf),
                                              LM75A_WAIT_READ_FOREVER);

  if (ret != ESP_OK)
    return LM75A_ERR_I2C_READ;

  uint16_t raw_thys = (buf[0] << 8) | buf[1];
  raw_thys >>= 7;
  // If the least significant bit (LSB) of the temperature data, which represents 0.5°C, around up
  // the number. It uses the 0x80 bit because D0–D6 is undefined.
  float thys;
  thys = raw_thys * 0.5;

  if (raw_thys & LM75A_SIGNAL_BIT)
  {
    raw_thys &= ~LM75A_SIGNAL_BIT;
    thys = -(raw_thys * 0.5);
  }

  *out_thys = thys;

  return LM75A_OK;
}

//! Not working yet
lm75a_status_t lm75a_get_product_id(lm75a_t *self, int *out_product)
{
  // uint8_t product_id = 0;
  // i2c_master_transmit_receive(s_lm75a_device, (uint8_t[]){LM75A_PROD_ID_REG}, 1, &product_id, 1,
  // -1); return product_id;
  return LM75A_ERR_UNKNOWN;
}

void lm75a_set_os_interrupt(lm75a_t *self, lm75a_os_cb_t cb, void *arg)
{
  gpio_num_t pin = self->config.os_pin;

  gpio_config_t os_conf = {.pin_bit_mask = (1ULL << pin),
                           .mode = GPIO_MODE_INPUT,
                           .pull_up_en = GPIO_PULLUP_ENABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_ANYEDGE};
  gpio_config(&os_conf);

  // Install GPIO ISR service
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

  // Attach interrupt handler
  gpio_isr_handler_add(pin, cb, arg);

  ESP_LOGI(TAG, "OS interrupt initialized on GPIO%d", pin);
}

lm75a_t *lm75a_create(lm75a_config_t config)
{
  lm75a_t *lm75a = (lm75a_t *)malloc(sizeof(lm75a_t));
  if (!lm75a)
  {
    ESP_LOGE(TAG, "Failed to allocate memory for LM75A");
    return NULL;
  }

  lm75a->config = config;
  lm75a->os_cb = NULL;
  lm75a->os_cb_arg = NULL;

  lm75a->init = lm75a_init;
  lm75a->read_temperature = lm75a_read_temperature;
  lm75a->set_os_mode = lm75a_set_os_mode;
  lm75a->set_shutdown = lm75a_set_shutdown;
  lm75a->set_polarity = lm75a_set_polarity;
  lm75a->set_fault_queue = lm75a_set_fault_queue;
  lm75a->set_tos = lm75a_set_tos;
  lm75a->set_thys = lm75a_set_thys;
  lm75a->get_tos = lm75a_get_tos;
  lm75a->get_thys = lm75a_get_thys;
  lm75a->get_product_id = lm75a_get_product_id;
  lm75a->set_os_interrupt = lm75a_set_os_interrupt;

  return lm75a;
}

void lm75a_destroy(lm75a_t *self)
{
  if (!self)
    return;

  i2c_master_bus_rm_device(s_lm75a_device);
  i2c_del_master_bus(s_i2c_bus);
  gpio_isr_handler_remove(self->config.os_pin);
  gpio_reset_pin(self->config.os_pin);
  free(self);
  self = NULL;

  ESP_LOGI(TAG, "LM75A deinitialized successfully.");
}
