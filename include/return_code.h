/**
 * @file return_code.h
 * @author Pedro Luis Dionísio Fraga (pedrodfraga@hotmail.com)
 *
 * @brief Return codes to LM75A driver functions
 *
 * @version 0.1
 * @date 2025-01-14
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef RETURN_CODE_H
#define RETURN_CODE_H

/**
 * @brief LM75A Driver Return Codes
 */
typedef enum
{
  LM75A_OK = 0,             ///< Operation successful
  LM75A_ERR_I2C_INIT,       ///< I2C initialization failed
  LM75A_ERR_I2C_DEV_ADD,    ///< Failed to add LM75A device to the I2C bus
  LM75A_ERR_I2C_WRITE,      ///< I2C write operation failed
  LM75A_ERR_I2C_READ,       ///< I2C read operation failed
  LM75A_ERR_INVALID_PARAM,  ///< Invalid parameter passed
  LM75A_ERR_GPIO_CONFIG,    ///< GPIO configuration failed
  LM75A_ERR_ISR_INSTALL,    ///< ISR installation failed
  LM75A_ERR_UNKNOWN         ///< Unknown error
} lm75a_status_t;

#endif  // RETURN_CODE_H
