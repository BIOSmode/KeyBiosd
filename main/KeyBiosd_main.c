/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "keyBiosd.h"

void app_main(void)
{
    ven_OneTimeInit();
    ESP_LOGI("MAIN", "One time init done.\n");

    while (1) 
    {
        ven_Runner();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
