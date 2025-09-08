/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdarg.h>
#include "sdkconfig.h"
#include "esp_flash.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "esp_cpu.h"
#include "rom/cache.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#include "soc/soc_caps.h"

#ifdef __NuttX__
#  include <nuttx/init.h>
#if defined(CONFIG_IDF_TARGET_ESP32)
#  include "esp32_irq.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
#  include "esp32s2_irq.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#  include "esp32s3_irq.h"
#else
#  include "esp_irq.h"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32
#  define esp_intr_noniram_disable  esp32_irq_noniram_disable
#  define esp_intr_noniram_enable   esp32_irq_noniram_enable
#endif
#ifdef CONFIG_IDF_TARGET_ESP32S2
#  define esp_intr_noniram_disable() do {} while(0);
#  define esp_intr_noniram_enable() do {} while(0);
#endif
#ifdef CONFIG_IDF_TARGET_ESP32S3
#  define esp_intr_noniram_disable  esp32s3_irq_noniram_disable
#  define esp_intr_noniram_enable   esp32s3_irq_noniram_enable
#endif

#endif // __NuttX__

static IRAM_ATTR esp_err_t start(void *arg)
{
#if SOC_BRANCH_PREDICTOR_SUPPORTED
    //branch predictor will start cache request as well
    esp_cpu_branch_prediction_disable();
#endif

#ifdef __NuttX__
    if (OSINIT_OS_READY()) {
        esp_intr_noniram_disable();
    }
#endif
#if CONFIG_IDF_TARGET_ESP32
    Cache_Read_Disable(0);
    Cache_Read_Disable(1);
#else
    cache_hal_suspend(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_ALL);
#endif

    return ESP_OK;
}

static IRAM_ATTR esp_err_t end(void *arg)
{
#if CONFIG_IDF_TARGET_ESP32
    Cache_Read_Enable(0);
    Cache_Read_Enable(1);
#else
    cache_hal_resume(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_ALL);
#endif

#if SOC_BRANCH_PREDICTOR_SUPPORTED
    esp_cpu_branch_prediction_enable();
#endif
#ifdef __NuttX__
    if (OSINIT_OS_READY()) {
        esp_intr_noniram_enable();
    }
#endif
    return ESP_OK;
}

static esp_err_t delay_us(void *arg, uint32_t us)
{
    esp_rom_delay_us(us);
    return ESP_OK;
}

// Currently when the os is not up yet, the caller is supposed to call esp_flash APIs with proper
// buffers.
void* get_temp_buffer_not_supported(void* arg, size_t reqest_size, size_t* out_size)
{
    return NULL;
}

const DRAM_ATTR esp_flash_os_functions_t esp_flash_noos_functions = {
    .start = start,
    .end = end,
    .delay_us = delay_us,
    .region_protected = NULL,
    /* the caller is supposed to call esp_flash_read/esp_flash_write APIs with buffers in DRAM */
    .get_temp_buffer = NULL,
    .release_temp_buffer = NULL,
    .yield = NULL,
};

esp_err_t esp_flash_app_disable_os_functions(esp_flash_t* chip)
{
    chip->os_func = &esp_flash_noos_functions;

    return ESP_OK;
}
