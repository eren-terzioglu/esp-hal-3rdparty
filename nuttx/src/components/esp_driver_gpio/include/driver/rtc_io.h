/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "soc/soc_caps.h"
#include "hal/rtc_io_types.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Determine if the specified IO is a valid RTC GPIO.
 *
 * @param gpio_num GPIO number
 * @return true if the IO is valid for RTC GPIO use. false otherwise.
 */
bool rtc_gpio_is_valid_gpio(gpio_num_t gpio_num);

/**
 * @brief Enable hold function on an RTC IO pad
 *
 * @param gpio_num GPIO number (e.g. GPIO_NUM_12)
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG The IO is not an RTC IO
 */
esp_err_t rtc_gpio_hold_en(gpio_num_t gpio_num);

#define RTC_GPIO_IS_VALID_GPIO(gpio_num) rtc_gpio_is_valid_gpio(gpio_num)

#if SOC_RTCIO_PIN_COUNT > 0
/**
 * @brief Get RTC IO index number by GPIO number.
 *
 * @param gpio_num GPIO number
 * @return
 *        >=0: Index of RTC IO.
 *        -1 : The IO is not an RTC IO.
 */
int rtc_io_number_get(gpio_num_t gpio_num);

#endif // SOC_RTCIO_PIN_COUNT > 0

#ifdef __cplusplus
}
#endif
