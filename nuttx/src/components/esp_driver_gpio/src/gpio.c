/*
 * SPDX-FileCopyrightText: 2019-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sdkconfig.h"

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

static const char *GPIO_TAG = "gpio";

#ifdef CONFIG_ARCH_CHIP_ESP32C3_GENERIC
#define SOC_GPIO_SUPPORT_RTC_INDEPENDENT 0
#endif

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

esp_err_t gpio_pullup_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "GPIO number error (input-only pad has no internal PU)", ESP_ERR_INVALID_ARG);

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
        ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
        gpio_hal_pullup_en(gpio_context.gpio_hal, gpio_num);
        LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_pullup_en(gpio_num);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occurred
#endif
    }

    return ESP_OK;
}

esp_err_t gpio_pullup_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
        ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
        gpio_hal_pullup_dis(gpio_context.gpio_hal, gpio_num);
        LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_pullup_dis(gpio_num);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occurred
#endif
    }

    return ESP_OK;
}

esp_err_t gpio_pulldown_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "GPIO number error (input-only pad has no internal PD)", ESP_ERR_INVALID_ARG);

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
        ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
        gpio_hal_pulldown_en(gpio_context.gpio_hal, gpio_num);
        LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_pulldown_en(gpio_num);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occurred
#endif
    }

    return ESP_OK;
}

esp_err_t gpio_pulldown_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    if (!rtc_gpio_is_valid_gpio(gpio_num) || SOC_GPIO_SUPPORT_RTC_INDEPENDENT) {
        ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
        gpio_hal_pulldown_dis(gpio_context.gpio_hal, gpio_num);
        LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    } else {
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
        rtc_gpio_pulldown_dis(gpio_num);
#else
        abort(); // This should be eliminated as unreachable, unless a programming error has occurred
#endif
    }

    return ESP_OK;
}

esp_err_t gpio_wakeup_enable(gpio_num_t gpio_num, gpio_int_type_t intr_type)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    if ((intr_type == GPIO_INTR_LOW_LEVEL) || (intr_type == GPIO_INTR_HIGH_LEVEL)) {
#if SOC_RTCIO_WAKE_SUPPORTED
        if (rtc_gpio_is_valid_gpio(gpio_num)) {
#if SOC_LP_IO_CLOCK_IS_INDEPENDENT
            // LP_IO Wake-up function does not depend on LP_IO Matrix, but uses its clock to
            // sample the wake-up signal, we need to enable the LP_IO clock here.
            io_mux_enable_lp_io_clock(gpio_num, true);
#endif
            ret = rtc_gpio_wakeup_enable(gpio_num, intr_type);
        }
#endif
        ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
        gpio_hal_set_intr_type(gpio_context.gpio_hal, gpio_num, intr_type);
        gpio_hal_wakeup_enable(gpio_context.gpio_hal, gpio_num);
#if CONFIG_ESP_SLEEP_GPIO_RESET_WORKAROUND || CONFIG_PM_SLP_DISABLE_GPIO
        gpio_hal_sleep_sel_dis(gpio_context.gpio_hal, gpio_num);
#endif
        LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    } else {
        ESP_LOGE(GPIO_TAG, "GPIO wakeup only supports level mode, but edge mode set. gpio_num:%u", gpio_num);
        ret = ESP_ERR_INVALID_ARG;
    }

    return ret;
}

esp_err_t gpio_wakeup_disable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;
#if SOC_RTCIO_WAKE_SUPPORTED
    if (rtc_gpio_is_valid_gpio(gpio_num)) {
        ret = rtc_gpio_wakeup_disable(gpio_num);
#if SOC_LP_IO_CLOCK_IS_INDEPENDENT
        io_mux_enable_lp_io_clock(gpio_num, false);
#endif
    }
#endif
    ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    gpio_hal_wakeup_disable(gpio_context.gpio_hal, gpio_num);
#if CONFIG_ESP_SLEEP_GPIO_RESET_WORKAROUND || CONFIG_PM_SLP_DISABLE_GPIO
    gpio_hal_sleep_sel_en(gpio_context.gpio_hal, gpio_num);
#endif
    LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    return ret;
}

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

static esp_err_t gpio_sleep_pullup_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    gpio_hal_sleep_pullup_en(gpio_context.gpio_hal, gpio_num);
    LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);

    return ESP_OK;
}

static esp_err_t gpio_sleep_pullup_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    gpio_hal_sleep_pullup_dis(gpio_context.gpio_hal, gpio_num);
    LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);

    return ESP_OK;
}

static esp_err_t gpio_sleep_pulldown_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    gpio_hal_sleep_pulldown_en(gpio_context.gpio_hal, gpio_num);
    LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);

    return ESP_OK;
}

static esp_err_t gpio_sleep_pulldown_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    gpio_hal_sleep_pulldown_dis(gpio_context.gpio_hal, gpio_num);
    LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);

    return ESP_OK;
}

static esp_err_t gpio_sleep_input_disable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_input_disable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}

static esp_err_t gpio_sleep_input_enable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_input_enable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}

static esp_err_t gpio_sleep_output_disable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_output_disable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}

static esp_err_t gpio_sleep_output_enable(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_OUTPUT_GPIO(gpio_num), "GPIO output gpio_num error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_output_enable(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}

esp_err_t gpio_sleep_set_direction(gpio_num_t gpio_num, gpio_mode_t mode)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    if ((GPIO_IS_VALID_OUTPUT_GPIO(gpio_num) != true) && (mode & GPIO_MODE_DEF_OUTPUT)) {
        ESP_LOGE(GPIO_TAG, "io_num=%d can only be input", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ESP_OK;

    if (mode & GPIO_MODE_DEF_INPUT) {
        gpio_sleep_input_enable(gpio_num);
    } else {
        gpio_sleep_input_disable(gpio_num);
    }

    if (mode & GPIO_MODE_DEF_OUTPUT) {
        gpio_sleep_output_enable(gpio_num);
    } else {
        gpio_sleep_output_disable(gpio_num);
    }

    return ret;
}

esp_err_t gpio_sleep_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    GPIO_CHECK(pull <= GPIO_FLOATING, "GPIO pull mode error", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

    switch (pull) {
    case GPIO_PULLUP_ONLY:
        gpio_sleep_pulldown_dis(gpio_num);
        gpio_sleep_pullup_en(gpio_num);
        break;

    case GPIO_PULLDOWN_ONLY:
        gpio_sleep_pulldown_en(gpio_num);
        gpio_sleep_pullup_dis(gpio_num);
        break;

    case GPIO_PULLUP_PULLDOWN:
        gpio_sleep_pulldown_en(gpio_num);
        gpio_sleep_pullup_en(gpio_num);
        break;

    case GPIO_FLOATING:
        gpio_sleep_pulldown_dis(gpio_num);
        gpio_sleep_pullup_dis(gpio_num);
        break;

    default:
        ESP_LOGE(GPIO_TAG, "Unknown pull up/down mode,gpio_num=%u,pull=%u", gpio_num, pull);
        ret = ESP_ERR_INVALID_ARG;
        break;
    }

    return ret;
}

esp_err_t gpio_sleep_sel_en(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    gpio_hal_sleep_sel_en(gpio_context.gpio_hal, gpio_num);
    LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);

    return ESP_OK;
}

esp_err_t gpio_sleep_sel_dis(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);

    ENTER_CRITICAL_SECTION(&gpio_context.gpio_spinlock);
    gpio_hal_sleep_sel_dis(gpio_context.gpio_hal, gpio_num);
    LEAVE_CRITICAL_SECTION(&gpio_context.gpio_spinlock);

    return ESP_OK;
}

#if CONFIG_GPIO_ESP32_SUPPORT_SWITCH_SLP_PULL
esp_err_t gpio_sleep_pupd_config_apply(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_pupd_config_apply(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}

esp_err_t gpio_sleep_pupd_config_unapply(gpio_num_t gpio_num)
{
    GPIO_CHECK(GPIO_IS_VALID_GPIO(gpio_num), "GPIO number error", ESP_ERR_INVALID_ARG);
    gpio_hal_sleep_pupd_config_unapply(gpio_context.gpio_hal, gpio_num);
    return ESP_OK;
}
#endif // CONFIG_GPIO_ESP32_SUPPORT_SWITCH_SLP_PULL

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
