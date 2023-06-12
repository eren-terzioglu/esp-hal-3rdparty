/*
 * SPDX-FileCopyrightText: 2019-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/io_mux.h"
#include "hal/rtc_io_hal.h"
#include "soc/rtc_io_periph.h"
#include "soc/soc_caps.h"
#include "driver/gpio.h"

#include "spinlock.h"

#define ENTER_CRITICAL_SECTION(lock) do { \
            assert(g_flags == UINT32_MAX); \
            g_flags = spin_lock_irqsave(lock); \
        } while(0)
#define LEAVE_CRITICAL_SECTION(lock) do { \
            spin_unlock_irqrestore((lock), g_flags); \
            g_flags = UINT32_MAX; \
        } while(0)
static irqstate_t g_flags = UINT32_MAX;

extern spinlock_t rtc_spinlock;

#define RTCIO_ENTER_CRITICAL()  ENTER_CRITICAL_SECTION(&rtc_spinlock)
#define RTCIO_EXIT_CRITICAL()  LEAVE_CRITICAL_SECTION(&rtc_spinlock)

#if SOC_RTCIO_PIN_COUNT > 0
/*---------------------------------------------------------------
                        RTC IO
---------------------------------------------------------------*/
int rtc_io_number_get(gpio_num_t gpio_num)
{
    return rtc_io_num_map[gpio_num];
}

#endif // SOC_RTCIO_PIN_COUNT > 0

bool rtc_gpio_is_valid_gpio(gpio_num_t gpio_num)
{
#if SOC_RTCIO_PIN_COUNT > 0
    return (gpio_num < GPIO_PIN_COUNT && rtc_io_num_map[gpio_num] >= 0);
#else
    return false;
#endif
}

esp_err_t rtc_gpio_hold_en(gpio_num_t gpio_num)
{
    ESP_RETURN_ON_FALSE(rtc_gpio_is_valid_gpio(gpio_num), ESP_ERR_INVALID_ARG, "RTCIO", "RTCIO number error");
    RTCIO_ENTER_CRITICAL();
    rtcio_hal_hold_enable(rtc_io_number_get(gpio_num));
    RTCIO_EXIT_CRITICAL();
    return ESP_OK;
}
