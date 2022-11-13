/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"

#define EXP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default
#define VEN_BUTTON (GPIO_NUM_35) // Test gpio35
static const char *TAG = "example";

/************* TinyUSB descriptors ****************/

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD) ),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE) )
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 0, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
}

/********* Application ***************/

typedef enum {
    MOUSE_DIR_RIGHT,
    MOUSE_DIR_DOWN,
    MOUSE_DIR_LEFT,
    MOUSE_DIR_UP,
    MOUSE_DIR_MAX,
} mouse_dir_t;

#define DISTANCE_MAX        125
#define DELTA_SCALAR        5

static void mouse_draw_square_next_delta(int8_t *delta_x_ret, int8_t *delta_y_ret)
{
    static mouse_dir_t cur_dir = MOUSE_DIR_RIGHT;
    static uint32_t distance = 0;

    // Calculate next delta
    if (cur_dir == MOUSE_DIR_RIGHT) {
        *delta_x_ret = DELTA_SCALAR;
        *delta_y_ret = 0;
    } else if (cur_dir == MOUSE_DIR_DOWN) {
        *delta_x_ret = 0;
        *delta_y_ret = DELTA_SCALAR;
    } else if (cur_dir == MOUSE_DIR_LEFT) {
        *delta_x_ret = -DELTA_SCALAR;
        *delta_y_ret = 0;
    } else if (cur_dir == MOUSE_DIR_UP) {
        *delta_x_ret = 0;
        *delta_y_ret = -DELTA_SCALAR;
    }

    // Update cumulative distance for current direction
    distance += DELTA_SCALAR;
    // Check if we need to change direction
    if (distance >= DISTANCE_MAX) {
        distance = 0;
        cur_dir++;
        if (cur_dir == MOUSE_DIR_MAX) {
            cur_dir = 0;
        }
    }
}

static void send_keyboard_report(uint8_t keyVal, bool isShift)
{
    uint8_t keycode[6] = {0, 0, 0, 0, 0, 0};  // Need init keycode
    if (isShift)
    {
        keycode[0] = HID_KEY_SHIFT_LEFT;
        keycode[1] = keyVal;
    }
    else
    {
        keycode[0] = keyVal;
    }
    ESP_LOGI(TAG, "Sending Keyboard report %x,%x,%x,%x,%x,%x", keycode[0], keycode[1], keycode[2]
                                                             , keycode[3], keycode[4], keycode[5]);
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
    vTaskDelay(pdMS_TO_TICKS(50));
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void exp_send_hid_demo(void)
{
    // Keyboard output: Send key 'a/A' pressed and released
    send_keyboard_report(HID_KEY_A, false);
    send_keyboard_report(HID_KEY_SPACE, false);

    // Mouse output: Move mouse cursor in square trajectory
    ESP_LOGI(TAG, "Sending Mouse report");
    int8_t delta_x;
    int8_t delta_y;
    for (int i = 0; i < (DISTANCE_MAX / DELTA_SCALAR) * 4; i++) {
        // Get the next x and y delta in the draw square pattern
        mouse_draw_square_next_delta(&delta_x, &delta_y);
        tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, 0x00, delta_x, delta_y, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void ven_send_hid_demo(void)
{
    // Keyboard output: Send key 'bios/BIOS' pressed and released
    send_keyboard_report(HID_KEY_B, false);
    send_keyboard_report(HID_KEY_I, false);
    send_keyboard_report(HID_KEY_O, false);
    send_keyboard_report(HID_KEY_S, false);
    send_keyboard_report(HID_KEY_SPACE, false);

    send_keyboard_report(HID_KEY_B, true);
    send_keyboard_report(HID_KEY_I, true);
    send_keyboard_report(HID_KEY_O, true);
    send_keyboard_report(HID_KEY_S, true);

    send_keyboard_report(HID_KEY_ENTER, false);
}

void app_main(void)
{
    uint64_t enableGpioPin = BIT64(EXP_BUTTON) | BIT64(VEN_BUTTON);

    // Initialize button that will trigger HID reports
    const gpio_config_t boot_button_config = {.pin_bit_mask = enableGpioPin,
                                              .mode = GPIO_MODE_INPUT,
                                              .intr_type = GPIO_INTR_DISABLE,
                                              .pull_up_en = true,
                                              .pull_down_en = false,
                                             };
    ESP_ERROR_CHECK(gpio_config(&boot_button_config));
    ESP_LOGI(TAG, "Enable Gpio 0x%llx", enableGpioPin);

    ESP_LOGI(TAG, "USB initialization Start");
    const tinyusb_config_t tusb_cfg = {.device_descriptor = NULL,
                                       .string_descriptor = NULL,
                                       .external_phy = false,
                                       .configuration_descriptor = hid_configuration_descriptor,
                                      };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    while (1) 
    {
        if (tud_mounted()) {
            static bool send_hid_data[2] = {true, true};
            if (send_hid_data[0]) 
            {
                exp_send_hid_demo();
            }
            send_hid_data[0] = !gpio_get_level(EXP_BUTTON);
            vTaskDelay(pdMS_TO_TICKS(100));

            if (send_hid_data[1])
            {
                ven_send_hid_demo();
            }
            send_hid_data[1] = !gpio_get_level(VEN_BUTTON);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
