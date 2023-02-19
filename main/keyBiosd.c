/*
 *
 * Authon: sunguiyu96@gmail.com
 * Start date: 2022/12/17
 * Desciprtion: 
 * 
 */

#include <stdlib.h>
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"

#include "keyBiosd.h"


/**********************************************/
/************ MACRO AND STUCTURE **************/
/**********************************************/
#define EXP_BUTTON               (GPIO_NUM_0) // Use BOOT signal by default
#define DISTANCE_MAX             125
#define DELTA_SCALAR             5
#define VEN_BUTTON_CNT           4
#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

#define MEMBRANE_KEYBOARD_1      (GPIO_NUM_35)
#define MEMBRANE_KEYBOARD_2      (GPIO_NUM_36)
#define MEMBRANE_KEYBOARD_3      (GPIO_NUM_37)
#define MEMBRANE_KEYBOARD_4      (GPIO_NUM_38)

#define MATRIX_KEYBOARD_C1       (GPIO_NUM_15)
#define MATRIX_KEYBOARD_C2       (GPIO_NUM_16)
#define MATRIX_KEYBOARD_C3       (GPIO_NUM_17)
#define MATRIX_KEYBOARD_C4       (GPIO_NUM_18)
#define MATRIX_KEYBOARD_R1       (GPIO_NUM_7)
#define MATRIX_KEYBOARD_R2       (GPIO_NUM_6)
#define MATRIX_KEYBOARD_R3       (GPIO_NUM_5)
#define MATRIX_KEYBOARD_R4       (GPIO_NUM_4)

typedef enum 
{
    MOUSE_DIR_RIGHT,
    MOUSE_DIR_DOWN,
    MOUSE_DIR_LEFT,
    MOUSE_DIR_UP,
    MOUSE_DIR_MAX,
} mouse_dir_e;


/**********************************************/
/****************** VARIABLE ******************/
/**********************************************/
static const char *TAG = "KeyBiosd";

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = 
{
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD) ),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE) )
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = 
{
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 0, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

biosd_button_info_t buttonInfo[VEN_BUTTON_CNT];

/**********************************************/
/************* PRIVATE FUNCTION ***************/
/**********************************************/
static void mouse_draw_square_next_delta(int8_t *delta_x_ret, int8_t *delta_y_ret)
{
    static mouse_dir_e cur_dir = MOUSE_DIR_RIGHT;
    static uint32_t distance = 0;

    // Calculate next delta
    if (cur_dir == MOUSE_DIR_RIGHT) 
    {
        *delta_x_ret = DELTA_SCALAR;
        *delta_y_ret = 0;
    } else if (cur_dir == MOUSE_DIR_DOWN) 
    {
        *delta_x_ret = 0;
        *delta_y_ret = DELTA_SCALAR;
    } else if (cur_dir == MOUSE_DIR_LEFT) 
    {
        *delta_x_ret = -DELTA_SCALAR;
        *delta_y_ret = 0;
    } else if (cur_dir == MOUSE_DIR_UP) 
    {
        *delta_x_ret = 0;
        *delta_y_ret = -DELTA_SCALAR;
    }

    // Update cumulative distance for current direction
    distance += DELTA_SCALAR;
    // Check if we need to change direction
    if (distance >= DISTANCE_MAX) 
    {
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
    ESP_LOGI(TAG, "Sending Keyboard report %x,%x,%x,%x,%x,%x", keycode[0], keycode[1], keycode[2], keycode[3], keycode[4], keycode[5]);
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
    vTaskDelay(pdMS_TO_TICKS(10));
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void ven_send_number_hid_demo(uint32_t num)
{
    if (num != 0)
    {
        send_keyboard_report(HID_KEY_1 + num - 1, false);
    }
    else
    {
        send_keyboard_report(HID_KEY_0, false);
    }
}

static uint8_t ven_scan_matrix_col(gpio_num_t rowGpio)
{
    ESP_ERROR_CHECK(gpio_set_level(MATRIX_KEYBOARD_C1, 1));
    if (gpio_get_level(rowGpio))
    {
        return 3;
    }

    ESP_ERROR_CHECK(gpio_set_level(MATRIX_KEYBOARD_C1, 0));
    ESP_ERROR_CHECK(gpio_set_level(MATRIX_KEYBOARD_C2, 1));
    if (gpio_get_level(rowGpio))
    {
        return 2;
    }

    ESP_ERROR_CHECK(gpio_set_level(MATRIX_KEYBOARD_C2, 0));
    ESP_ERROR_CHECK(gpio_set_level(MATRIX_KEYBOARD_C3, 1));
    if (gpio_get_level(rowGpio))
    {
        return 1;
    }

    ESP_ERROR_CHECK(gpio_set_level(MATRIX_KEYBOARD_C3, 0));
    ESP_ERROR_CHECK(gpio_set_level(MATRIX_KEYBOARD_C4, 1));
    if (gpio_get_level(rowGpio))
    {
        return 0;
    }

    return 0;
}

void ven_scan_switch_demo(void)
{
    uint8_t row = 0, col = 0;
    ESP_ERROR_CHECK(gpio_set_level(MATRIX_KEYBOARD_C1, 0));
    ESP_ERROR_CHECK(gpio_set_level(MATRIX_KEYBOARD_C2, 0));
    ESP_ERROR_CHECK(gpio_set_level(MATRIX_KEYBOARD_C3, 0));
    ESP_ERROR_CHECK(gpio_set_level(MATRIX_KEYBOARD_C4, 0));

    if (!gpio_get_level(MATRIX_KEYBOARD_R4))
    {
        row = 0;
        col = ven_scan_matrix_col(MATRIX_KEYBOARD_R4);
    }
    else if (!gpio_get_level(MATRIX_KEYBOARD_R3))
    {
        row = 1;
        col = ven_scan_matrix_col(MATRIX_KEYBOARD_R3);
    }
    else if (!gpio_get_level(MATRIX_KEYBOARD_R2))
    {
        row = 2;
        col = ven_scan_matrix_col(MATRIX_KEYBOARD_R2);
    }
    else if (!gpio_get_level(MATRIX_KEYBOARD_R1))
    {
        row = 3;
        col = ven_scan_matrix_col(MATRIX_KEYBOARD_R1);
    }
    else
    {
        return;
    }

    uint32_t buttonVal = row * 4 + col + 1;
    ESP_LOGI(TAG, "Button S%lu", buttonVal);
    send_keyboard_report(HID_KEY_S, true);
    if (buttonVal > 9)
    {
        ven_send_number_hid_demo(buttonVal / 10);
    }
    ven_send_number_hid_demo(buttonVal % 10);
}

static void ven_init_button_info(biosd_button_info_t *pButInfo, uint8_t gpioPin, bool gpioInitSts, uint8_t hidKey, bool isShift)
{
    pButInfo->gpioPin = gpioPin;
    pButInfo->gpioSts = gpioInitSts;
    pButInfo->hidKey = hidKey;
    pButInfo->isShift = isShift;
    const gpio_config_t button_config = {.pin_bit_mask =BIT64(gpioPin),
                                        .mode = GPIO_MODE_INPUT,
                                        .intr_type = GPIO_INTR_DISABLE,
                                        .pull_up_en = gpioInitSts ? true : false,
                                        .pull_down_en = gpioInitSts ? false : true,
                                        };
    ESP_ERROR_CHECK(gpio_config(&button_config));
}

/**********************************************/
/************* PUBLIC FUNCTION ****************/
/**********************************************/
// TinyUSB HID callbacks
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

void exp_send_hid_demo(void)
{
    // Mouse output: Move mouse cursor in square trajectory
    ESP_LOGI(TAG, "Sending Mouse report");
    int8_t delta_x;
    int8_t delta_y;
    for (int i = 0; i < (DISTANCE_MAX / DELTA_SCALAR) * 4; i++) 
    {
        // Get the next x and y delta in the draw square pattern
        mouse_draw_square_next_delta(&delta_x, &delta_y);
        tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, 0x00, delta_x, delta_y, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void ven_send_hid_demo(void)
{
    // Keyboard output: Send key 'bios/BIOS' pressed and released
    for (uint8_t i = 0; i < VEN_BUTTON_CNT; i++)
    {
        if (!buttonInfo[i].gpioSts)
        {
            send_keyboard_report(buttonInfo[i].hidKey, buttonInfo[i].isShift);
        }
        buttonInfo[i].gpioSts = gpio_get_level(buttonInfo[i].gpioPin);
    }
}

void ven_OneTimeInit(void)
{
    ESP_LOGI(TAG, "ESP IDF Version: %s.\n", esp_get_idf_version());
    
    const gpio_config_t boot_button_config = {.pin_bit_mask = BIT64(EXP_BUTTON),
                                              .mode = GPIO_MODE_INPUT,
                                              .intr_type = GPIO_INTR_DISABLE,
                                              .pull_up_en = true,
                                              .pull_down_en = false,
                                             };
    ESP_ERROR_CHECK(gpio_config(&boot_button_config));

    // Initialize button that will trigger HID reports
    ven_init_button_info(&buttonInfo[0], MEMBRANE_KEYBOARD_1, true, HID_KEY_B, true);
    ven_init_button_info(&buttonInfo[1], MEMBRANE_KEYBOARD_2, true, HID_KEY_I, true);
    ven_init_button_info(&buttonInfo[2], MEMBRANE_KEYBOARD_3, true, HID_KEY_O, true);
    ven_init_button_info(&buttonInfo[3], MEMBRANE_KEYBOARD_4, true, HID_KEY_S, true);

    // Initialize matrix keyboard gpio
    const gpio_config_t col_button_config = {.pin_bit_mask = BIT64(MATRIX_KEYBOARD_C1) | BIT64(MATRIX_KEYBOARD_C2) | BIT64(MATRIX_KEYBOARD_C3) | BIT64(MATRIX_KEYBOARD_C4),
                                             .mode = GPIO_MODE_OUTPUT,
                                             .intr_type = GPIO_INTR_DISABLE,
                                             .pull_up_en = false,
                                             .pull_down_en = true};
    ESP_ERROR_CHECK(gpio_config(&col_button_config));
    const gpio_config_t row_button_config = {.pin_bit_mask = BIT64(MATRIX_KEYBOARD_R1) | BIT64(MATRIX_KEYBOARD_R2) | BIT64(MATRIX_KEYBOARD_R3) | BIT64(MATRIX_KEYBOARD_R4),
                                             .mode = GPIO_MODE_INPUT,
                                             .intr_type = GPIO_INTR_DISABLE,
                                             .pull_up_en = true,
                                             .pull_down_en = false};
    ESP_ERROR_CHECK(gpio_config(&row_button_config));

    ESP_LOGI(TAG, "USB initialization Start");
    const tinyusb_config_t tusb_cfg = {.device_descriptor = NULL,
                                       .string_descriptor = NULL,
                                       .external_phy = false,
                                       .configuration_descriptor = hid_configuration_descriptor,
                                      };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");
}

void ven_Runner(void)
{
    if (tud_mounted()) 
    {
        static bool send_hid_data = true;
        if (send_hid_data) 
        {
            exp_send_hid_demo();
        }
        send_hid_data = !gpio_get_level(EXP_BUTTON);

        ven_send_hid_demo();
        ven_scan_switch_demo();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}
