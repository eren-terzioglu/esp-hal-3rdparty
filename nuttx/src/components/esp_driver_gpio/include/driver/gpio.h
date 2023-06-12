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

#ifdef __cplusplus
}
#endif
