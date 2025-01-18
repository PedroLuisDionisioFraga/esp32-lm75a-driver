/**
 * @file lm75a.h
 * @brief LM75A Temperature Sensor Driver for ESP32 (I2C)
 * @version 1.1
 * @date 2024-12-23
 *
 * @author
 *  - Original Author: Pedro Luis Dionísio Fraga
 *
 * This header provides a driver interface to the LM75A temperature sensor.
 * It allows initialization, reading temperature, configuring thresholds,
 * and handling hardware interrupts via the OS pin.
 *
 * @note
 *  For usage examples, see the README.md in this component.
 */

#ifndef LM75A_H
#define LM75A_H

#include <driver/gpio.h>        // Needed for gpio_num_t
#include <driver/i2c_master.h>  // Needed for i2c_port_t
#include <esp_err.h>
#include <stdbool.h>
#include <stdint.h>

#include "return_code.h"  // Personalized return types (lm75a_status_t)

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

  /** @brief Temperature scale used when reading temperature. */
  typedef enum
  {
    LM75A_SCALE_CELSIUS = 0, /**< Celsius scale.    */
    LM75A_SCALE_FAHRENHEIT,  /**< Fahrenheit scale. */
    LM75A_SCALE_KELVIN       /**< Kelvin scale.     */
  } lm75a_scale_t;

  typedef struct lm75a
  {
    lm75a_config_t config; /**< Sensor configuration. */
    lm75a_os_cb_t os_cb;   /**< Callback for OS interrupt (if needed). */
    void *os_cb_arg;       /**< Argument to pass to OS interrupt callback. */

    /**
     * @brief Initialize the LM75A with the provided config.
     *
     * @param[in] self Pointer to driver instance.
     * @return lm75a_status_t Custom driver status.
     */
    lm75a_status_t (*init)(struct lm75a *self);

    /**
     * @brief Refactored function to read temperature.
     *
     * @param[in]  self    Pointer to driver instance.
     * @param[out] out_temp Where the read temperature is stored.
     * @param[in]  scale   Desired temperature scale (Celsius, Fahrenheit, Kelvin).
     *
     * @return lm75a_status_t Custom driver status.
     */
    lm75a_status_t (*read_temperature)(struct lm75a *self, float *out_temp, lm75a_scale_t scale);

    /**
     * @brief Enable or disable the LM75A's shutdown mode.
     *
     * @param[in] self   Pointer to driver instance.
     * @param[in] enable True to enable shutdown, false otherwise.
     */
    lm75a_status_t (*set_shutdown)(struct lm75a *self, bool enable);

    /**
     * @brief Set the operating mode for the OS pin (comparator vs. interrupt).
     *
     * @param[in] self Pointer to driver instance.
     * @param[in] mode OS mode.
     */
    lm75a_status_t (*set_os_mode)(struct lm75a *self, lm75a_os_mode_t mode);

    /**
     * @brief Set the OS pin polarity (active-low or active-high).
     *
     * @param[in] self Pointer to driver instance.
     * @param[in] pol  Polarity mode.
     */
    lm75a_status_t (*set_polarity)(struct lm75a *self, lm75a_os_polarity_t pol);

    /**
     * @brief Configure the LM75A fault queue (1, 2, 4, or 6 faults).
     *
     * @param[in] self        Pointer to driver instance.
     * @param[in] fault_queue One of the enumerated fault queue options.
     */
    lm75a_status_t (*set_fault_queue)(struct lm75a *self, lm75a_fault_queue_t fault_queue);

    /**
     * @brief Set Overtemperature Shutdown threshold (TOS).
     *
     * @param[in] self Pointer to driver instance.
     * @param[in] tos  Threshold in °C.
     */
    lm75a_status_t (*set_tos)(struct lm75a *self, int16_t tos, bool add_half_degree);

    /**
     * @brief Set Overtemperature Shutdown threshold (TOS).
     *
     * @param[in] self Pointer to driver instance.
     * @param[in] tos  Threshold in °C.
     * @param[in] add_half_degree Boolean to add 0.5°C to the threshold.
     * @return lm75a_status_t Custom driver status.
     */
    lm75a_status_t (*set_tos)(struct lm75a *self, int16_t tos, bool add_half_degree);

    /**
     * @brief Set Hysteresis threshold (THYST).
     *
     * @param[in] self Pointer to driver instance.
     * @param[in] thys Hysteresis in °C.
     * @param[in] add_half_degree Boolean to add 0.5°C to the threshold.
     * @return lm75a_status_t Custom driver status.
     */
    lm75a_status_t (*set_thys)(struct lm75a *self, int16_t thys, bool add_half_degree);

    /**
     * @brief Read the current THYST (Hysteresis) threshold.
     *
     * @param[in]  self      Pointer to driver instance.
     * @param[out] out_thys  Where the THYST value (°C) is stored.
     *
     * @return lm75a_status_t Custom driver status.
     */
    lm75a_status_t (*get_thys)(struct lm75a *self, float *out_thys);

    /**
     * @brief Get the product ID from the device register.
     *
     * @param[in]  self         Pointer to driver instance.
     * @param[out] out_product  Where the product ID is stored.
     *
     * @return lm75a_status_t Custom driver status.
     */
    lm75a_status_t (*get_product_id)(struct lm75a *self, int *out_product);

    /**
     * @brief Register/enable OS pin interrupt.
     *
     * @param[in] self Pointer to driver instance.
     * @param[in] cb   Callback function for OS interrupt.
     * @param[in] arg  User argument passed to the callback.
     *
     * @return lm75a_status_t Custom driver status.
     */
    lm75a_status_t (*set_os_interrupt)(struct lm75a *self, lm75a_os_cb_t cb, void *arg);

  } lm75a_t;

  /**
   * @brief Create a new LM75A driver object, using the given configuration.
   *
   * @param[in] config LM75A configuration (I2C pins, address, etc.).
   * @return lm75a_t* Pointer to allocated/initialized driver object; NULL on failure.
   *
   * @note Caller is responsible for calling lm75a_destroy() when done.
   */
  lm75a_t *lm75a_create(lm75a_config_t config);

  /**
   * @brief Destroy the LM75A driver object, freeing resources.
   *
   * @param[in] sensor Pointer to the driver object to destroy.
   */
  lm75a_status_t lm75a_destroy(lm75a_t *self);

#ifdef __cplusplus
}
#endif

#endif  // LM75A_H
