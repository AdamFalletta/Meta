/**
 * @file neobat_keyboard.c
 * @brief Bluetooth Low Energy (BLE) HID Keyboard application for the NEObat device (nRF52840).
 *
 * This code implements the core BLE and HID service functionality,
 * along with an event-driven structure for managing sensors and LED control.
 * It uses the standard Nordic nRF5 SDK framework (assumes SDK v17+).
 *
 * NOTE: Full, working drivers for TWI (I2C), SAADC (Piezo), and NeoPixel
 * are complex and require external libraries/specific pinouts. This code
 * includes the necessary initialization *calls* but uses dummy read/write
 * functions to focus on the robust BLE HID implementation.
 */

#include <stdint.h>
#include <string.h>
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "ble_advertising.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "app_timer.h"
#include "nrf_ble_gatt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_internal.h"
#include "nrf_log_backend_usb.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_saadc.h"
#include "nrf_gpio.h" // For simple GPIO reads

// =================================================================================
// 1. CONFIGURATION DEFINITIONS
// =================================================================================

// BLE Configuration
#define DEVICE_NAME             "NEObat HID"
#define MANUFACTURER_NAME       "NEObat Labs"
#define APP_BLE_CONN_CFG_TAG    1
#define APP_BLE_OBSERVER_PRIO   3
#define MIN_CONN_INTERVAL       MSEC_TO_UNITS(20, UNIT_1_25_MS)
#define MAX_CONN_INTERVAL       MSEC_TO_UNITS(75, UNIT_1_25_MS)

// Advertising configuration
#define ADVERTISING_LED_PIN     NRF_GPIO_PIN_MAP(0, 17) // Example LED Pin for advertising status

// Sensor Configuration (Placeholder Pins - Adjust to your board)
#define LED_PIN             NRF_GPIO_PIN_MAP(0, 12)
#define NUM_PIXELS          8
#define PIEZO_ADC_PIN       NRF_SAADC_INPUT_AIN0 // Example ADC input
#define CAP_TOUCH_PIN       NRF_GPIO_PIN_MAP(0, 3)
#define MIC_PIN             NRF_GPIO_PIN_MAP(0, 4)
#define BUTTON_PIN          NRF_GPIO_PIN_MAP(0, 6)

// I2C Configuration
#define I2C_SCL_PIN         NRF_GPIO_PIN_MAP(0, 26)
#define I2C_SDA_PIN         NRF_GPIO_PIN_MAP(0, 25)
#define TWI_INSTANCE_ID     0
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

// Timers
APP_TIMER_DEF(m_sensor_timer_id);
#define SENSOR_POLL_INTERVAL APP_TIMER_TICKS(100) // Poll sensors every 100ms

// =================================================================================
// 2. GLOBAL VARIABLES
// =================================================================================
NRF_BLE_GATT_DEF(m_gatt);
BLE_ADVERTISING_DEF(m_advertising);
BLE_HIDS_DEF(m_hids);
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
static bool m_in_boot_mode = true; // Start in boot mode for initial connection

// HID Keyboard Report Descriptor (Standard 8-byte report)
static uint8_t report_map_data[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       // Report ID (1)
    0x05, 0x07,       // Usage Page (Key Codes)
    0x19, 0xE0,       // Usage Minimum (224) (Modifier Keys)
    0x29, 0xE7,       // Usage Maximum (231)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x08,       // Report Count (8)
    0x81, 0x02,       // Input (Data, Var, Abs) (Modifier Byte)
    0x95, 0x01,       // Report Count (1)
    0x75, 0x08,       // Report Size (8)
    0x81, 0x01,       // Input (Constant) (Reserved Byte)
    0x95, 0x05,       // Report Count (5) (LEDs)
    0x75, 0x01,       // Report Size (1)
    0x05, 0x08,       // Usage Page (LEDs)
    0x19, 0x01,       // Usage Minimum (Num Lock)
    0x29, 0x05,       // Usage Maximum (Kana)
    0x91, 0x02,       // Output (Data, Var, Abs) (LED status)
    0x95, 0x01,       // Report Count (1)
    0x75, 0x03,       // Report Size (3)
    0x91, 0x01,       // Output (Constant) (LED padding)
    0x95, 0x06,       // Report Count (6) (Actual Keys)
    0x75, 0x08,       // Report Size (8)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x65,       // Logical Maximum (101)
    0x05, 0x07,       // Usage Page (Key Codes)
    0x19, 0x00,       // Usage Minimum (0)
    0x29, 0x65,       // Usage Maximum (101)
    0x81, 0x00,       // Input (Data, Array, Abs) (Key Array)
    0xC0              // End Collection
};

// =================================================================================
// 3. FORWARD DECLARATIONS
// =================================================================================
static void advertising_start(void);
static void services_init(void);
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
static void hid_send_keystroke(uint8_t key_code);

// =================================================================================
// 4. CORE BLE AND HID IMPLEMENTATION
// =================================================================================

/**
 * @brief Sends a single key press followed immediately by a key release.
 *
 * @param[in] key_code The HID key code to send (e.g., HID_KEY_A for 'a').
 */
static void hid_send_keystroke(uint8_t key_code)
{
    // The HID report structure is:
    // [0] Modifier keys (e.g., Shift, Ctrl)
    // [1] Reserved
    // [2-7] Key codes (up to 6 simultaneous keys)
    uint8_t keypress_report[8] = {0x00, 0x00, key_code, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t keyrelease_report[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    ret_code_t err_code;

    // 1. Send Key Press
    err_code = ble_hids_inp_rep_send(&m_hids, 0x01, 8, keypress_report, m_conn_handle);
    if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
    {
        NRF_LOG_WARNING("Key press failed: %d", err_code);
    }

    // 2. Send Key Release (prevents repeating the key)
    err_code = ble_hids_inp_rep_send(&m_hids, 0x01, 8, keyrelease_report, m_conn_handle);
    if (err_code != NRF_SUCCESS && err_code != NRF_ERROR_INVALID_STATE)
    {
        NRF_LOG_WARNING("Key release failed: %d", err_code);
    }
}

/**
 * @brief Initialize the BLE Stack and SoftDevice.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack with required RAM and features
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable the BLE stack
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**
 * @brief Configure GAP parameters (Device Name, Appearance, etc.).
 */
static void gap_params_init(void)
{
    ret_code_t err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    // Set the BLE Appearance (Keyboard)
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_KEYBOARD);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Initialize the HID Service and Battery Service.
 */
static void services_init(void)
{
    ret_code_t err_code;
    ble_hids_init_t hids_init;
    ble_bas_init_t bas_init;
    
    // Initialize Battery Service (BAS) - Placeholder
    memset(&bas_init, 0, sizeof(bas_init));
    bas_init.evt_handler = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_read_perm  = NULL;
    bas_init.bl_rd_auth          = false;
    bas_init.battery_level_char_attr_md.read_perm = SEC_OPEN;
    bas_init.battery_level_char_attr_md.write_perm = SEC_OPEN;
    err_code = ble_bas_init(NULL, &bas_init);
    APP_ERROR_CHECK(err_code);


    // Initialize Human Interface Device Service (HIDS)
    memset(&hids_init, 0, sizeof(hids_init));

    // HID Information Characteristic
    hids_init.hid_info.bcd_hid = 0x0100;
    hids_init.hid_info.b_country_code = 0x00;
    hids_init.hid_info.flags = HID_INFO_FLAG_REMOTE_WAKEUP | HID_INFO_FLAG_NORMALLY_CONNECTABLE;

    // Report Map Configuration
    hids_init.included_services_array.count = 0;
    hids_init.included_services_array.p_included_services = NULL;

    hids_init.is_kb = true;
    hids_init.is_mouse = false;
    hids_init.inp_rep_count = 1;
    hids_init.p_inp_rep_array = (ble_hids_rep_char_t *[]){
        &(ble_hids_rep_char_t){
            .report_id = 0x01,
            .rep_type = BLE_HIDS_REP_TYPE_INPUT,
            .max_len = 8,
            .sec_req.cccd_write_perm = SEC_JUST_ENCRYPT,
            .sec_req.rd_perm = SEC_JUST_ENCRYPT,
            .sec_req.wr_perm = SEC_JUST_ENCRYPT
        }
    };
    hids_init.p_rep_map_data = report_map_data;
    hids_init.rep_map_len = sizeof(report_map_data);
    
    hids_init.feature_rep_count = 0;
    hids_init.output_rep_count = 0;

    hids_init.evt_handler = NULL; // Simple keyboard doesn't need a dedicated HIDS event handler
    hids_init.p_sec = &(ble_hids_c_sec_t){
        .security_mode_boot.cccd_write_perm = SEC_JUST_ENCRYPT,
        .security_mode_boot.rd_perm = SEC_JUST_ENCRYPT,
        .security_mode_boot.wr_perm = SEC_JUST_ENCRYPT,
        .security_mode_report.cccd_write_perm = SEC_JUST_ENCRYPT,
        .security_mode_report.rd_perm = SEC_JUST_ENCRYPT,
        .security_mode_report.wr_perm = SEC_JUST_ENCRYPT
    };

    err_code = ble_hids_init(&m_hids, &hids_init);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Initialize advertising parameters and start advertising.
 */
static void advertising_init(void)
{
    ret_code_t err_code;
    ble_advertising_init_t init;

    // Build the advertising data packet
    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = 1;
    init.advdata.uuids_complete.p_uuids  = (ble_uuid_t[]){{BLE_UUID_HIDS_SERVICE, BLE_UUID_TYPE_BLE}};

    // Build the scan response data packet (not strictly necessary for HID, but good practice)
    init.srdata.uuids_complete.uuid_cnt  = 0;
    init.srdata.uuids_complete.p_uuids   = NULL;

    // Advertising parameters
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = MSEC_TO_UNITS(20, UNIT_0_625_MS);
    init.config.ble_adv_fast_timeout  = 180; // 3 minutes in seconds (0 = never timeout)

    init.evt_handler = NULL; // Advertising events can be handled in ble_evt_handler

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_vs_uuid_set(&m_advertising, report_map_data, sizeof(report_map_data));

    // Set the preferred security
    ble_gap_conn_params_t conn_params = {
        .min_conn_interval = MIN_CONN_INTERVAL,
        .max_conn_interval = MAX_CONN_INTERVAL,
        .slave_latency     = 0,
        .conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS)
    };
    err_code = sd_ble_gap_ppcp_set(&conn_params);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**
 * @brief Handles all events generated by the BLE stack.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            nrf_gpio_pin_clear(ADVERTISING_LED_PIN); // Turn off advertising LED

            err_code = ble_hids_client_role_set(&m_hids, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);

            // Start connection parameters update process
            err_code = ble_conn_params_change_conn_params(m_conn_handle, &p_ble_evt->evt.gap_evt.conn_params);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected. Reason: %d", p_ble_evt->evt.gap_evt.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            advertising_start(); // Restart advertising upon disconnection
            break;
            
        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accept the peer's suggested parameters
            err_code = sd_ble_gap_conn_param_update(p_ble_evt->evt.gap_evt.conn_handle,
                                                    &p_ble_evt->evt.gap_evt.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            NRF_LOG_INFO("HID service entered Boot Mode.");
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            NRF_LOG_INFO("HID service entered Report Mode.");
            m_in_boot_mode = false;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // For simplicity, accepting default security parameters
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   NULL, // Use default security params
                                                   NULL); // Use default key set
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            // Handle authentication status if necessary
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**
 * @brief Start the advertising process.
 */
static void advertising_start(void)
{
    ret_code_t err_code;
    nrf_gpio_pin_set(ADVERTISING_LED_PIN); // Turn on advertising LED
    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

// =================================================================================
// 5. PERIPHERAL & SENSOR DRIVER PLACEHOLDERS
// =================================================================================

/**
 * @brief Dummy handler for Neopixel animation control.
 * @param[in] animation_id The ID sent over BLE serial to trigger an animation.
 */
static void neopixel_animation(int animation_id) {
    NRF_LOG_INFO("NEOPixel animation %d requested.", animation_id);
    // In a full implementation, you would call a NeoPixel library function here, e.g.:
    // neopixel_set_color_all(animation_id == 1 ? 0xFF0000 : 0x0000FF);
}

/**
 * @brief Initialize TWI (I2C) for Compass and Accelerometer.
 */
static void i2c_init(void) {
    nrf_drv_twi_config_t twi_config = {
        .scl = I2C_SCL_PIN,
        .sda = I2C_SDA_PIN,
        .frequency = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
    };
    ret_code_t err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_twi);
    NRF_LOG_INFO("TWI (I2C) initialized.");
}

/**
 * @brief Initialize SAADC for the Piezo sensor.
 */
static void saadc_init(void) {
    ret_code_t err_code;
    err_code = nrf_drv_saadc_init(NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(PIEZO_ADC_PIN);
    config.gain = NRF_SAADC_GAIN1_6;
    config.reference = NRF_SAADC_REFERENCE_INTERNAL; // Internal 0.6V reference
    
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("SAADC (Piezo) initialized.");
}

/**
 * @brief Placeholder for Neopixel strip initialization.
 */
static void neopixel_init() {
    // This assumes an external library like nrfx_pwm is configured to drive the WS2812B/NeoPixel strip.
    nrf_gpio_cfg_output(LED_PIN);
    nrf_gpio_pin_set(LED_PIN); // Turn off/reset the pin
    NRF_LOG_INFO("NeoPixel placeholder initialized on pin %d", LED_PIN);
}

// =================================================================================
// 6. SENSOR READING AND MAPPING LOGIC
// =================================================================================

/**
 * @brief Placeholder: Reads the Piezo sensor and maps to keys 'a'-'j'.
 */
static void piezo_sensor_read() {
    nrf_saadc_value_t value;
    // In a real application, you'd trigger a sample and wait for the result here.
    // For now, we simulate a reading to demonstrate the keystroke logic.
    ret_code_t err_code = nrf_drv_saadc_sample_convert(0, &value);

    if (err_code == NRF_SUCCESS && value > 50) { // Assume a threshold of 50 (raw)
        // Simplified mapping: value / 100 gives a range
        int range = (value / 100) + 1;
        range = (range > 10) ? 10 : range;
        
        // Map 1-10 range to keys a-j (HID key codes 0x04 to 0x0D)
        uint8_t key_code = 0x03 + range; // 'A' key code is 0x04.
        hid_send_keystroke(key_code);
        NRF_LOG_DEBUG("Piezo Hit! Value: %d -> Key Code: 0x%02X", value, key_code);
    }
}

/**
 * @brief Placeholder: Reads the Cap Touch sensor (digital input).
 */
static void cap_touch_read() {
    if (nrf_gpio_pin_read(CAP_TOUCH_PIN) == 0) {
        // Send '1' key (HID key code 0x1E)
        hid_send_keystroke(0x1E);
        NRF_LOG_DEBUG("Cap Touch Hit! -> Key 1");
    }
}

/**
 * @brief Placeholder: Reads the physical button.
 */
static void button_press_handler() {
    if (nrf_gpio_pin_read(BUTTON_PIN) == 0) {
        // Send 'o' key (HID key code 0x12)
        hid_send_keystroke(0x12);
        NRF_LOG_DEBUG("Button Pressed! -> Key O");
    }
}

/**
 * @brief Placeholder: Reads I2C data (e.g., Compass and Accelerometer).
 */
static void imu_read() {
    // DUMMY TWI READ: In a real app, this is where you'd perform a non-blocking TWI transfer.
    // If motion exceeds a threshold, send a key.
    static uint8_t motion_counter = 0;
    motion_counter++;
    if (motion_counter % 20 == 0) { // Send 'z' every 2 seconds to simulate movement detected
        hid_send_keystroke(0x1D); // Key 'Z'
        NRF_LOG_DEBUG("IMU Motion Detected! -> Key Z");
    }
}

/**
 * @brief Timer callback function for periodic sensor polling.
 */
static void sensor_timer_handler(void * p_context)
{
    // Check if connected and in Report Mode before sending data
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID && !m_in_boot_mode) {
        piezo_sensor_read();
        cap_touch_read();
        button_press_handler();
        imu_read();
    }
}

// =================================================================================
// 7. INITIALIZATION AND MAIN
// =================================================================================

/**
 * @brief Initialize all system modules (Power Management, Timers, Logs).
 */
static void log_and_timer_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // Initialize the Application Timer module
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Initialize application peripherals (GPIO, Sensors, etc.).
 */
static void application_peripherals_init(void)
{
    // GPIO for Cap Touch and Button (inputs with pull-up)
    nrf_gpio_cfg_input(CAP_TOUCH_PIN, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(BUTTON_PIN, NRF_GPIO_PIN_PULLUP);

    // Advertising LED setup
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN);
    nrf_gpio_pin_set(ADVERTISING_LED_PIN); // Start with LED ON (advertising)

    saadc_init();
    i2c_init();
    neopixel_init();
    // Pressure and Mic pins are not initialized here since their interaction is complex
    // and would require proper drivers (like PDM for a digital mic).
}

/**
 * @brief Create and start the sensor polling timer.
 */
static void timers_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_create(&m_sensor_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_sensor_timer_id, SENSOR_POLL_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Main execution function.
 */
int main(void)
{
    // 1. System Initialization
    log_and_timer_init();
    application_peripherals_init();
    
    // 2. BLE Stack and Services Initialization
    ble_stack_init();
    gap_params_init();
    services_init(); // Includes HID and BAS
    advertising_init();
    
    // 3. Start Application Timers
    timers_start();

    NRF_LOG_INFO("NEObat BLE Keyboard initialized.");

    // 4. Start Advertising
    advertising_start();

    // Enter the main execution loop (non-blocking)
    for (;;)
    {
        // Process logger and USB backend events
        NRF_LOG_FLUSH();
        NRF_LOG_PROCESS();
        
        // Wait for system events (BLE, timers, etc.)
        (void)sd_app_evt_wait();
    }
}
