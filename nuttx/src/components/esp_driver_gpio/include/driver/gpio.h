/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdio.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "soc/soc_caps.h"
#include "hal/gpio_types.h"
#include "esp_rom_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_PIN_COUNT                      (SOC_GPIO_PIN_COUNT)
/// Check whether it is a valid GPIO number
#define GPIO_IS_VALID_GPIO(gpio_num)        ((gpio_num >= 0) && \
                                              (((1ULL << (gpio_num)) & SOC_GPIO_VALID_GPIO_MASK) != 0))
/// Check whether it can be a valid GPIO number of output mode
#define GPIO_IS_VALID_OUTPUT_GPIO(gpio_num) ((gpio_num >= 0) && \
                                              (((1ULL << (gpio_num)) & SOC_GPIO_VALID_OUTPUT_GPIO_MASK) != 0))
/// Check whether it can be a valid digital I/O pad
#define GPIO_IS_VALID_DIGITAL_IO_PAD(gpio_num) ((gpio_num >= 0) && \
                                                 (((1ULL << (gpio_num)) & SOC_GPIO_VALID_DIGITAL_IO_PAD_MASK) != 0))

#define GPIO_CHECK(a, str, ret_val) ESP_RETURN_ON_FALSE(a, ret_val, "gpio", "%s", str)

#define GPIO_IS_DEEP_SLEEP_WAKEUP_VALID_GPIO(gpio_num)    ((gpio_num >= 0) && \
                                                          (((1ULL << (gpio_num)) & SOC_GPIO_DEEP_SLEEP_WAKE_VALID_GPIO_MASK) != 0))

esp_err_t gpio_hold_en(gpio_num_t gpio_num);

/**
  * @brief Enable pull-up on GPIO.
  *
  * @param gpio_num GPIO number
  *
  * @return
  *     - ESP_OK Success
  *     - ESP_ERR_INVALID_ARG Parameter error
  */
esp_err_t gpio_pullup_en(gpio_num_t gpio_num);

/**
  * @brief Disable pull-up on GPIO.
  *
  * @param gpio_num GPIO number
  *
  * @return
  *     - ESP_OK Success
  *     - ESP_ERR_INVALID_ARG Parameter error
  */
esp_err_t gpio_pullup_dis(gpio_num_t gpio_num);

/**
  * @brief Enable pull-down on GPIO.
  *
  * @param gpio_num GPIO number
  *
  * @return
  *     - ESP_OK Success
  *     - ESP_ERR_INVALID_ARG Parameter error
  */
esp_err_t gpio_pulldown_en(gpio_num_t gpio_num);

/**
  * @brief Disable pull-down on GPIO.
  *
  * @param gpio_num GPIO number
  *
  * @return
  *     - ESP_OK Success
  *     - ESP_ERR_INVALID_ARG Parameter error
  */
esp_err_t gpio_pulldown_dis(gpio_num_t gpio_num);

/**
 * @brief Enable GPIO wake-up function.
 *
 * @param gpio_num GPIO number.
 *
 * @param intr_type GPIO wake-up type. Only GPIO_INTR_LOW_LEVEL or GPIO_INTR_HIGH_LEVEL can be used.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t gpio_wakeup_enable(gpio_num_t gpio_num, gpio_int_type_t intr_type);

/**
 * @brief Disable GPIO wake-up function.
 *
 * @param gpio_num GPIO number
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t gpio_wakeup_disable(gpio_num_t gpio_num);

/**
  * @brief Enable SLP_SEL to change GPIO status automantically in lightsleep.
  * @param gpio_num GPIO number of the pad.
  *
  * @return
  *     - ESP_OK Success
  *
  */
esp_err_t gpio_sleep_sel_en(gpio_num_t gpio_num);

/**
  * @brief Disable SLP_SEL to change GPIO status automantically in lightsleep.
  * @param gpio_num GPIO number of the pad.
  *
  * @return
  *     - ESP_OK Success
  */
esp_err_t gpio_sleep_sel_dis(gpio_num_t gpio_num);

/**
 * @brief    GPIO set direction at sleep
 *
 * Configure GPIO direction,such as output_only,input_only,output_and_input
 *
 * @param  gpio_num  Configure GPIO pins number, it should be GPIO number. If you want to set direction of e.g. GPIO16, gpio_num should be GPIO_NUM_16 (16);
 * @param  mode GPIO direction
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG GPIO error
 */
esp_err_t gpio_sleep_set_direction(gpio_num_t gpio_num, gpio_mode_t mode);

/**
 * @brief  Configure GPIO pull-up/pull-down resistors at sleep
 *
 * @note ESP32: Only pins that support both input & output have integrated pull-up and pull-down resistors. Input-only GPIOs 34-39 do not.
 *
 * @param  gpio_num GPIO number. If you want to set pull up or down mode for e.g. GPIO16, gpio_num should be GPIO_NUM_16 (16);
 * @param  pull GPIO pull up/down mode.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG : Parameter error
 */
esp_err_t gpio_sleep_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull);

#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP

/**
 * @brief Enable GPIO deep-sleep wake-up function.
 *
 * @param gpio_num GPIO number.
 *
 * @param intr_type GPIO wake-up type. Only GPIO_INTR_LOW_LEVEL or GPIO_INTR_HIGH_LEVEL can be used.
 *
 * @note Called by the SDK. User shouldn't call this directly in the APP.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t gpio_deep_sleep_wakeup_enable(gpio_num_t gpio_num, gpio_int_type_t intr_type);
#endif

/**
 * @brief Configure the pin to be used for analog purpose (such as ADC, touch, etc.)
 *
 * @param gpio_num GPIO number
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG GPIO number error
 */
esp_err_t gpio_config_as_analog(gpio_num_t gpio_num);

/**
  * @brief Set pad input to a peripheral signal through the IOMUX.
  *
  * @param gpio_num GPIO number of the pad.
  * @param func The index number of the IOMUX function to be selected for the pin.
  *        One of the ``FUNC_X_*`` of specified pin (X) in ``soc/io_mux_reg.h``.
  * @param signal_idx Peripheral signal id to input. One of the ``*_IN_IDX`` signals in ``soc/gpio_sig_map.h``.
  *
  * @return
  *     - ESP_OK Success
  *     - ESP_ERR_INVALID_ARG GPIO number error
  */
esp_err_t gpio_iomux_input(gpio_num_t gpio_num, int func, uint32_t signal_idx);

/**
  * @brief Set peripheral output to an GPIO pad through the IOMUX.
  *
  * @param gpio_num GPIO number of the pad.
  * @param func The index number of the IOMUX function to be selected for the pin.
  *        One of the ``FUNC_X_*`` of specified pin (X) in ``soc/io_mux_reg.h``.
  *
  * @return
  *     - ESP_OK Success
  *     - ESP_ERR_INVALID_ARG GPIO number error
  */
esp_err_t gpio_iomux_output(gpio_num_t gpio_num, int func);

#ifdef __cplusplus
}
#endif
