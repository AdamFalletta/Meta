#include <nrf.h>
#include <nrf_gpio.h>
#include <nrf_drv_saadc.h>
#include <nrf_drv_twi.h>
#include <nrf_drv_gpiote.h>
#include <nrfx_adc.h>
#include <nrfx_pwm.h>
#include <nrfx_spim.h>
#include <ble_hids.h>
#include <app_timer.h>
#include <nrf_log.h>
#include <nrf_delay.h>
#include <neopixel.h>
#include <app_uart.h>
#include <app_error.h>

// HID Services
static ble_hids_t m_hids;

// Placeholder for BLE Services
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

// NeoPixel and LED Strip Configuration
#define LED_PIN 12
#define NUM_PIXELS 8
static uint32_t pixels[NUM_PIXELS];

// Define sensor input pins
#define PIEZO_PIN 2
#define CAP_TOUCH_PIN 3
#define MIC_PIN 4
#define PRESSURE_PIN 5
#define BUTTON_PIN 6

// Define I2C pins and sensors
#define I2C_SCL_PIN 26
#define I2C_SDA_PIN 25
#define COMPASS_ADDR 0x1E
#define ACCELEROMETER_ADDR 0x1D

// Global Variables for I2C
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);

void hid_send_keystroke(char key);

// Initialize ADC for Piezo Sensor (3.6V max)
void saadc_init(void) {
    nrf_drv_saadc_init(NULL, NULL);
    nrf_saadc_channel_config_t config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(PIN_PIEZO);
    config.gain = NRF_SAADC_GAIN1_6;
    config.reference = NRF_SAADC_REFERENCE_INTERNAL;
    nrf_drv_saadc_channel_init(0, &config);
}

// Capture Piezo Vibration Levels and Send Keystrokes
void piezo_sensor_read() {
    nrf_saadc_value_t value;
    nrf_drv_saadc_sample_convert(0, &value);
    float voltage = (3.6 / 1023.0) * value;
    
    if (voltage > 0.36 && voltage <= 3.6) {
        int range = (voltage / 3.6) * 10;
        char key = 'a' + range - 1;  // map voltage range to a-j keystrokes
        hid_send_keystroke(key);
    }
}

// Initialize Cap Touch Sensor for Ball Bearing Contact
void cap_touch_init() {
    nrf_gpio_cfg_input(CAP_TOUCH_PIN, NRF_GPIO_PIN_PULLUP);
}

// Cap Touch Digital Input
void cap_touch_read() {
    if (nrf_gpio_pin_read(CAP_TOUCH_PIN) == 0) {
        hid_send_keystroke('1');  // Send keystroke when ball bearing hits
    }
}

// Initialize Serial Bluetooth RX
void ble_serial_init() {
    // Setup BLE UART or BLE Serial service
    // Configure UART handler to receive and process data
    // Placeholder for BLE RX ready to receive data from website
}

// Placeholder for NeoPixel Animations
void neopixel_animation(int animation_id) {
    switch (animation_id) {
        case 1:
            // Red Animation
            break;
        case 2:
            // Blue Animation
            break;
        case 3:
            // Purple Animation
            break;
        default:
            break;
    }
}

// Handle Serial Data Received Over BLE
void serial_data_handler(char *data) {
    if (strcmp(data, "1") == 0) {
        neopixel_animation(1);  // Trigger red animation
    } else if (strcmp(data, "2") == 0) {
        neopixel_animation(2);  // Trigger blue animation
    }
}

// Pressure Sensor Read (raw)
void pressure_sensor_read() {
    int pressure_value = nrf_gpio_pin_read(PRESSURE_PIN);
    // No conversion needed, send raw pressure signal
}

// Microphone Sensor Read (Sound Detection)
void mic_sensor_read() {
    int mic_value = nrf_gpio_pin_read(MIC_PIN);
    if (mic_value > 0) {
        hid_send_keystroke('q');  // Send keypress 'q' on sound detection
    }
}

// I2C Initialization for Compass and Accelerometer
void i2c_init(void) {
    nrf_drv_twi_config_t twi_config = {
        .scl = I2C_SCL_PIN,
        .sda = I2C_SDA_PIN,
        .frequency = NRF_TWI_FREQ_400K,
    };
    nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    nrf_drv_twi_enable(&m_twi);
}

// Compass Data Read
void compass_read() {
    uint8_t reg = 0x03;  // Example compass data register
    uint8_t data[6];
    nrf_drv_twi_tx(&m_twi, COMPASS_ADDR, &reg, 1, true);
    nrf_drv_twi_rx(&m_twi, COMPASS_ADDR, data, 6);
    // Process compass data and send direction if needed
}

// Accelerometer Data Read
void accelerometer_read() {
    uint8_t data[6];
    nrf_drv_twi_rx(&m_twi, ACCELEROMETER_ADDR, data, 6);
    int16_t x = (data[0] << 8) | data[1];
    int16_t y = (data[2] << 8) | data[3];
    int16_t z = (data[4] << 8) | data[5];
    // Process x, y, z data and handle accordingly
}

// Button Keypress Handler
void button_press_handler() {
    if (nrf_gpio_pin_read(BUTTON_PIN) == 0) {
        hid_send_keystroke('o');  // Send keystroke 'o' on button press
    }
}

// NFC or Beacon Initialization and Listening
void nfc_init() {
    // Initialize NFC module or Beacon listener
    // Placeholder for listening and handling NFC credentials
}

// Main Loop
int main(void) {
    // Initialize peripherals
    saadc_init();
    i2c_init();
    ble_serial_init();
    nfc_init();
    neopixel_init(LED_PIN, NUM_PIXELS);

    while (true) {
        piezo_sensor_read();
        cap_touch_read();
        pressure_sensor_read();
        mic_sensor_read();
        compass_read();
        accelerometer_read();
        button_press_handler();
        nrf_delay_ms(100);  // Adjust as needed
    }
}

// HID Keystroke Sending Function
void hid_send_keystroke(char key) {
    uint8_t keypress[8] = {0};  // HID report buffer
    keypress[2] = key;
    ble_hids_inp_rep_send(&m_hids, 0, sizeof(keypress), keypress, m_conn_handle);
}
