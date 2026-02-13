#ifndef CONFIG_DATA_H
#define CONFIG_DATA_H

#include <stdint.h>
#include <stdbool.h>
typedef enum {
    DEVICE_TYPE_KEYPAD,
    DEVICE_TYPE_ROTARY
} device_type_t;


#define BAUD_RATE 9600
#define BUF_SIZE 1024
#define MAX_DEVICES 30
// #define MAX_KEYPADS 30
#define BUTTONS_PER_KEYPAD 8
#define AP_SSID "JAQUAR_DEVICE"
#define AP_PASS "Jaquar@device"
#define ZONE_WRITE_REG_VAL 5999
#define ZONE_READ_REG_VAL 6999
#define CH_READ_REG_VAL 1999
#define CH_WRITE_REG_VAL 0
#define MAX_SLAVES   99
#define MAX_ZONES    64
#define BACKLIGHT 1
#define DEFAULT_ADDRESS 0x02
#define MAX_CHANNEL_POLL_RANGES 9
#define FRAME_SIZE 8
#define NUM_BUTTONS 8
#define MAX_CHANNELS_PER_RANGE 125
#define ROTARY_DIM_INDICATOR 0x10
#define MAX_TOTAL_CHANNELS 250  // (2 ranges)
#define MODBUS_ZONES_RESP_LEN 133
#define MODBUS_CHANNEL_RESP_LEN 255
#define ON_OFF_FRAME_SIZE 13
#define ACTIVE_PANEL_TIMEOUT_MS 5000  // Reset after 5 seconds of inactivity
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
//#define RED_LED_GPIO GPIO_NUM_15//Indication for AP mode
//#define GREEN_LED_GPIO GPIO_NUM_16//Indication for Keypads Press

#define UART_NUM1 UART_NUM_1
#define TXD_PIN1 GPIO_NUM_36
#define RXD_PIN1 GPIO_NUM_37
// #define RS485_WRITE_PIN1 GPIO_NUM_3
// #define RS485_READ_PIN1 GPIO_NUM_2
#define RS485_ENABLE_PIN1 GPIO_NUM_1

#define UART_NUM2 UART_NUM_2
#define TXD_PIN2 GPIO_NUM_41
#define RXD_PIN2 GPIO_NUM_42
//#define RS485_ENABLE_PIN2 GPIO_NUM_1
#define RS485_WRITE_PIN2 GPIO_NUM_3
#define RS485_READ_PIN2 GPIO_NUM_2

uint8_t default_slave_id = 0x01;
uint8_t default_panel_address = 0x02;
uint8_t active_panel_address = 0x00;
static uint8_t web_active_keypad_no = 0x00;
static uint32_t last_button_press_time = 0;
uint8_t panel_new_address;
uint8_t keypads_count;

typedef struct {
    uint8_t slaveId;
    uint8_t keypadNumber;
    uint8_t zoneNumber;
    uint8_t keyOnScene[BUTTONS_PER_KEYPAD];   // On scene for each key
    uint8_t keyOffScene[BUTTONS_PER_KEYPAD];  // Off scene for each key
} keypad_config_t;

typedef struct {
    uint8_t slaveId;
    uint8_t rotaryNumber;
    uint16_t keyValue[3];     // Channel numbers
} rotary_config_t;

typedef struct {
    device_type_t deviceType;  // KEYPAD or ROTARY
    union {
        keypad_config_t keypad;
        rotary_config_t rotary;
    } config;
} device_config_t;
#define LED_QUEUE_SIZE MAX_DEVICES

typedef struct {
    uint8_t deviceNumber;
    uint16_t value;
    uint8_t dry_cmd_active_onoff_btn;
    bool is_btn_state_on;
    uint8_t device_keymode;
    uint8_t dry_cmd_active_rotary_btn;
    bool pending;
    uint8_t led_cmd[FRAME_SIZE];
    uint8_t dry_onoff_cmd[FRAME_SIZE];
} led_command_t;

static led_command_t led_command_queue[LED_QUEUE_SIZE];
static uint8_t led_queue_head = 0;
static uint8_t led_queue_tail = 0;
static SemaphoreHandle_t led_queue_mutex = NULL;
// Global arrays and variables
static device_config_t device_configs[MAX_DEVICES];
static int channel_poll_range_ids[9] = {125, 250, 375, 500, 625, 750, 875, 1000, 1024};
uint8_t ch1_125[8] = {0x01, 0x03, 0x07, 0xD0, 0x00, 0x7D, 0x85, 0x66};
uint8_t ch126_250[8] = {0x01, 0x03, 0x08, 0x4D, 0x00, 0x7D, 0x17, 0x9C};
uint8_t ch251_375[8] = {0x01, 0x03, 0x08, 0xCA, 0x00, 0x7D, 0xA7, 0xB5};
uint8_t ch376_500[8] = {0x01, 0x03, 0x09, 0x47, 0x00, 0x7D, 0x36, 0x62};
uint8_t ch501_625[8] = {0x01, 0x03, 0x09, 0xC4, 0x00, 0x7D, 0xC7, 0x8A};
uint8_t ch626_750[8] = {0x01, 0x03, 0x0A, 0x41, 0x00, 0x7D, 0xD6, 0x27};
uint8_t ch751_875[8] = {0x01, 0x03, 0x0A, 0xBE, 0x00, 0x7D, 0xE6, 0x17};
uint8_t ch876_1000[8] = {0x01, 0x03, 0x0B, 0x3B, 0x00, 0x7D, 0xF6, 0x02};
uint8_t ch1001_1024[8] = {0x01, 0x03, 0x0B, 0xB8, 0x00, 0x18, 0xC7, 0xC1};
static uint8_t* channel_frames[9] = {
    ch1_125, ch126_250, ch251_375, ch376_500,
    ch501_625, ch626_750, ch751_875, ch876_1000, ch1001_1024
};

// #define RS485_WRITE_MODE1 gpio_set_level(RS485_READ_PIN1, 1); gpio_set_level(RS485_WRITE_PIN1, 1);
// #define RS485_READ_MODE1 gpio_set_level(RS485_READ_PIN1, 0); gpio_set_level(RS485_WRITE_PIN1, 0);

// #define RS485_WRITE_MODE2 gpio_set_level(RS485_ENABLE_PIN2, 1); //gpio_set_level(RS485_WRITE_PIN2, 1);
// #define RS485_READ_MODE2 gpio_set_level(RS485_ENABLE_PIN2, 0); //gpio_set_level(RS485_WRITE_PIN2, 0);
#define RS485_WRITE_MODE1 gpio_set_level(RS485_ENABLE_PIN1, 1); //gpio_set_level(RS485_WRITE_PIN1, 1);
#define RS485_READ_MODE1 gpio_set_level(RS485_ENABLE_PIN1, 0); //gpio_set_level(RS485_WRITE_PIN1, 0);

#define RS485_WRITE_MODE2 gpio_set_level(RS485_READ_PIN2, 1); gpio_set_level(RS485_WRITE_PIN2, 1);
#define RS485_READ_MODE2 gpio_set_level(RS485_READ_PIN2, 0); gpio_set_level(RS485_WRITE_PIN2, 0);

bool save_keypad_config_to_nvs(uint8_t slaveId, uint8_t keypadNumber, uint8_t zoneNumber, keypad_config_t *cfg);
void read_all_keypad_configs_from_nvs(void);
void configure_panel_address(uint8_t device_type);
void send_led_command_priority(uint8_t keypadNumber, uint16_t value);
void check_and_log_button_changes(uint8_t *clean_data);
void send_scene_to_controller(uint8_t slaveId, uint8_t zone, uint8_t scene_no);
void send_channel_to_controller(uint8_t slaveId, uint16_t channel_no, uint8_t ch_val);
void send_led_group_on(uint8_t keypadNumber, const uint8_t* leds, uint8_t count, uint8_t backlight_on);
void send_led_group_off(uint8_t keypadNumber, const uint8_t* leds, uint8_t count, uint8_t backlight_on);
void check_and_log_zones_changes(uint8_t slaveId, uint8_t *response);
void check_and_log_channels_changes(uint8_t slaveId, uint8_t *response, uint16_t channel_start_no);
bool take_mutex_with_timeout(SemaphoreHandle_t mutex, const char* mutex_name, TickType_t timeout);
void give_mutex(SemaphoreHandle_t mutex, const char* mutex_name);
bool clean_devices_response(uint8_t *rx_data, int len, uint8_t *clean_buf, int org_len);
void kick_watchdog();
//void update_rotary_polling_ranges(uint8_t rotaryNumber, bool is_delete);
bool is_keypad_in_nvs(uint8_t keypadNumber);
esp_err_t ota_upload_handler(httpd_req_t *req);
void update_keypad_leds_for_zone(uint8_t slaveId, uint8_t zone, uint8_t scene);
// void send_led_command_immediately(uint8_t keypadNumber, uint16_t value, uint8_t device_mode);
void update_channel_polling_ranges(uint16_t user_channel, bool is_delete, uint8_t keypadNumber);
#endif // MODBUS_DATA_H
