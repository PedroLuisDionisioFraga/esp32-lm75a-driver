/**
 * @file lm75a.h
 * @brief LM75A Temperature Sensor Driver for ESP32 (I2C)
 * @version 1.0
 * @date 2024-12-23
 * @author Pedro Luis Dionísio Fraga
 */

#ifndef LM75A_H
#define LM75A_H

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>


// TODO: Document all code in doxygen format
// TODO: Remove unnecessary comments, all comments should be added in function documentation
// TODO: Remove unecessary and libraries
// TODO: Put the logs only debug mode
// TODO: Add return types personalized for each function (include/return_code.h)
// TODO: Refactor read_temperature to return a error code,  add a parameter to return the temperature and the scale (Celsius, Fahrenheit or Kelvin)
// TODO: Add a example of usage in README.md

#ifdef __cplusplus
extern "C"
{
#endif
  typedef void (*lm75a_os_cb_t)(void *arg);

  typedef enum
  {
    LM75A_OS_COMPARATOR_MODE = 0,
    LM75A_OS_INTERRUPT_MODE
  } lm75a_os_mode_t;

  // O.S. is an open-drain output under all conditions.
  typedef enum
  {
    LM75A_OS_POLARITY_LOW = 0,
    LM75A_OS_POLARITY_HIGH
  } lm75a_os_polarity_t;

  typedef struct
  {
    i2c_port_t i2c_port;
    gpio_num_t master_sda_pin;
    gpio_num_t master_scl_pin;
    gpio_num_t os_pin;
    uint32_t i2c_frequency;
    uint8_t address;
  } lm75a_config_t;

  typedef enum
  {
    LM75A_FAULT_QUEUE_1 = 0x00,
    LM75A_FAULT_QUEUE_2 = 0x01,
    LM75A_FAULT_QUEUE_4 = 0x02,
    LM75A_FAULT_QUEUE_6 = 0x03
  } lm75a_fault_queue_t;

  typedef struct lm75a
  {
    lm75a_config_t config;
    // Verify the need of this
    lm75a_os_cb_t os_cb;
    void *os_cb_arg;

    esp_err_t (*init)(struct lm75a *self);
    float (*read_temperature)(struct lm75a *self);
    void (*set_shutdown)(struct lm75a *self, bool enable);
    // TODO: Test the two modes and document it
    void (*set_os_mode)(struct lm75a *self, lm75a_os_mode_t mode);
    void (*set_polarity)(struct lm75a *self, lm75a_os_polarity_t pol);
    // TODO: Test it better
    void (*set_fault_queue)(struct lm75a *self, lm75a_fault_queue_t fault_queue);
    void (*set_tos)(struct lm75a *self, int tos);
    void (*set_thys)(struct lm75a *self, int thys);
    float (*get_tos)(struct lm75a *self);
    float (*get_thys)(struct lm75a *self);
    // TODO: Fix this
    int (*get_product_id)(struct lm75a *self);
    void (*set_os_interrupt)(struct lm75a *self, lm75a_os_cb_t cb, void *arg);

  } lm75a_t;

  // ========== Public API ==========
  lm75a_t *lm75a_create(lm75a_config_t config);
  void lm75a_destroy(lm75a_t *sensor);

#ifdef __cplusplus
}
#endif

#endif  // LM75A_H
