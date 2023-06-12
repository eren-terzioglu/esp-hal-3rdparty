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
#include "hal/gpio_hal.h"
#include "soc/rtc_io_periph.h"
#include "soc/gpio_periph.h"
#include "soc/soc_caps.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"

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

typedef struct {
    gpio_hal_context_t *gpio_hal;
    spinlock_t gpio_spinlock;
} gpio_context_t;

static gpio_hal_context_t _gpio_hal = {
    .dev = GPIO_HAL_GET_HW(GPIO_PORT_0)
};

static gpio_context_t gpio_context = {
    .gpio_hal = &_gpio_hal,
    .gpio_spinlock = SP_UNLOCKED,
};

esp_err_t gpio_hold_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "Only output-capable GPIO support this function", ESP_ERR_NOT_SUPPORTED);
    int ret = ESP_OK;

    if (rtc_gpio_is_valid_gpio(gpio_num)) {
#if SOC_RTCIO_HOLD_SUPPORTED
        ret = rtc_gpio_hold_en(gpio_num);
#endif
    } else if (GPIO_HOLD_MASK[gpio_num]) {
        ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
        gpio_hal_hold_en(gpio_context.gpio_hal, gpio_num);
        LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    } else {
        ret = ESP_ERR_NOT_SUPPORTED;
    }

    return ret;
}

#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP && SOC_DEEP_SLEEP_SUPPORTED
esp_err_t gpio_deep_sleep_wakeup_enable(gpio_num_t gpio_num, gpio_int_type_t intr_type)
{
    if (!GPIO_IS_DEEP_SLEEP_WAKEUP_VALID_GPIO(gpio_num)) {
        ESP_LOGE("gpio", "GPIO %d does not support deep sleep wakeup", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    if ((intr_type != GPIO_INTR_LOW_LEVEL) && (intr_type != GPIO_INTR_HIGH_LEVEL)) {
        ESP_LOGE("gpio", "GPIO wakeup only supports level mode, but edge mode set. gpio_num:%u", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
#if SOC_LP_IO_CLOCK_IS_INDEPENDENT
    io_mux_enable_lp_io_clock(gpio_num, true);
#endif
    gpio_hal_deepsleep_wakeup_enable(gpio_context.gpio_hal, gpio_num, intr_type);
#if CONFIG_ESP_SLEEP_GPIO_RESET_WORKAROUND || CONFIG_PM_SLP_DISABLE_GPIO
    gpio_hal_sleep_sel_dis(gpio_context.gpio_hal, gpio_num);
#endif
    LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    return ESP_OK;
}
#endif // SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP && SOC_DEEP_SLEEP_SUPPORTED
