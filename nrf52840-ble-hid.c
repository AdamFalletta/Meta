#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "bsp.h"
#include "app_timer.h"
#include "nrf_ble_gatt.h"
#include "ble_hids.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"

// BLE Parameters
#define DEVICE_NAME "Nordic_HID"      // Name of the device
#define APP_BLE_OBSERVER_PRIO 3       // BLE event observer priority
#define APP_BLE_CONN_CFG_TAG 1        // BLE connection configuration
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(30, UNIT_1_25_MS)
#define SLAVE_LATENCY 6
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)

// HID Keyboard Configuration
#define INPUT_REPORT_KEYS_MAX_LEN 8   // Max length of the input report
#define OUTPUT_REPORT_MAX_LEN 1       // Max length of the output report
#define INPUT_REPORT_ID 1             // ID of the input report
#define OUTPUT_REPORT_ID 1            // ID of the output report

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
static ble_hids_t m_hids;

// Button Pins
#define BUTTON_1_PIN BSP_BUTTON_0
#define BUTTON_2_PIN BSP_BUTTON_1
#define BUTTON_3_PIN BSP_BUTTON_2
#define BUTTON_4_PIN BSP_BUTTON_3
#define BUTTON_5_PIN 28               // Example: Custom GPIO pin for button 5

static void ble_stack_init(void);
static void hids_init(void);
static void services_init(void);
static void advertising_start(void);

// Button Action Logic
void send_key_press(uint8_t keycode)
{
    ret_code_t err_code;
    uint8_t report[INPUT_REPORT_KEYS_MAX_LEN] = {0};
    report[2] = keycode; // Keycode is placed in the third byte of the HID report.

    // Send the input report
    err_code = ble_hids_inp_rep_send(&m_hids, INPUT_REPORT_ID, INPUT_REPORT_KEYS_MAX_LEN, report, m_conn_handle);
    APP_ERROR_CHECK(err_code);
}

void process_button_presses(void)
{
    if (!nrf_gpio_pin_read(BUTTON_1_PIN))
    {
        send_key_press(0x2A); // Backspace
    }
    else if (!nrf_gpio_pin_read(BUTTON_2_PIN))
    {
        send_key_press(0x04); // 'a'
    }
    else if (!nrf_gpio_pin_read(BUTTON_3_PIN))
    {
        send_key_press(0x05); // 'b'
    }
    else if (!nrf_gpio_pin_read(BUTTON_4_PIN))
    {
        send_key_press(0x28); // Enter
    }
    else if (!nrf_gpio_pin_read(BUTTON_5_PIN))
    {
        send_key_press(0x29); // ESC
    }
}

// Main Function
int main(void)
{
    // Initialize
    bsp_board_init(BSP_INIT_BUTTONS);
    ble_stack_init();
    services_init();
    advertising_start();

    while (true)
    {
        process_button_presses();
        __WFE();
    }
}

// BLE Initialization
static void ble_stack_init(void)
{
    ret_code_t err_code;
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}

// HID Service Initialization
static void hids_init(void)
{
    ret_code_t err_code;
    ble_hids_init_t hids_init_obj = {0};

    // Input Report
    ble_hids_inp_rep_init_t input_report_array[1];
    memset(input_report_array, 0, sizeof(input_report_array));

    input_report_array[0].max_len = INPUT_REPORT_KEYS_MAX_LEN;
    input_report_array[0].rep_ref.report_id = INPUT_REPORT_ID;
    input_report_array[0].rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    hids_init_obj.inp_rep_count = 1;
    hids_init_obj.p_inp_rep_array = input_report_array;

    // HID Service Init
    hids_init_obj.rep_map.data_len = sizeof(input_report_array);
    hids_init_obj.rep_map.p_data = NULL; // Set HID descriptor here.
    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}

// Advertising Initialization
static void advertising_start(void)
{
    ret_code_t err_code;
    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled = true;
    options.ble_adv_fast_interval = MSEC_TO_UNITS(40, UNIT_0_625_MS);
    options.ble_adv_fast_timeout = 30;

    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}
