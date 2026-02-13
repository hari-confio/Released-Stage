#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_http_server.h"
#include "index_html.h"
#include "nvs.h"
#include "config_data.h"
#include <ctype.h>
#include <inttypes.h>
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include <math.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_flash_partitions.h"
#include "esp_app_format.h"

static const char *TAG = "MAIN";
static const char *TAG_RS485_DATA = "RS485_DATA";
static const char *TAG_BTN = "BUTTON STATUS";

// Globals based on new config.h typedefs
static device_config_t device_configs[MAX_DEVICES];
static keypad_config_t keypad_configs[MAX_DEVICES];
static rotary_config_t rotary_configs[MAX_DEVICES]; // max rotaries = MAX_DEVICES for convenience
static keypad_config_t last_submitted_config;       // store last submitted keypad config (if needed)
// Boot gating: block dry-contact frames until app fully started
static bool boot_restoring = false;   // true during LED restore
static bool dry_send_enabled = false; // becomes true after startup banner
static rotary_config_t last_submitted_rotary_config;
static bool last_rotary_config_valid = false;
uint8_t keypads_count = 0;
uint8_t rotaries_count = 0;
uint8_t devices_count = 0;        // total devices in device_configs[]
bool polling_task_ready = false;
uint8_t *global_valid_frames[2] = {0}; // Adjust size as needed
uint8_t global_valid_count = 0;
uint16_t keypad_ids_Buff[MAX_DEVICES] = {0};
uint8_t keypad_id_cnt = 0;
uint16_t device_ids_Buff[MAX_DEVICES] = {0};
uint8_t device_id_cnt = 0;
uint8_t poll_deviceType = 0;
bool zones_poll_eligible = false, channels_poll_eligible = false;
bool polling_task_started = false;
bool last_config_valid = false;
//bool send_onoff_cmd_to_dry_contact = false;
bool is_btn_state_on = false;//, all_leds_off = false;
uint8_t dry_cmd_active_onoff_btn = 0, dry_cmd_active_rotary_btn = 0;
uint8_t dry_onoff_cmd[FRAME_SIZE] = {0};
uint8_t dry_dim_cmd[FRAME_SIZE] = {0};
static bool led_command_ready = false;
static uint8_t led_command_frame[FRAME_SIZE] = {0};

TaskHandle_t polling_task = NULL;
TaskHandle_t zone_polling_task = NULL;

wifi_config_t wifi_config = {0};
httpd_handle_t server = NULL;

uint8_t frame[FRAME_SIZE];
uint8_t keypad_onoff_button_no = 0;

static uint16_t last_button_states[MAX_DEVICES][NUM_BUTTONS] = {0};
static uint16_t led_states[MAX_DEVICES] = {0};

static uint16_t last_scene_values[MAX_SLAVES+1][MAX_ZONES+1] = {0};
static uint16_t last_channel_values[MAX_SLAVES + 1][MAX_TOTAL_CHANNELS + 1] = {0};

uint16_t eligible_channel_poll_values[MAX_CHANNEL_POLL_RANGES];
uint8_t eligible_count = 0;

uint8_t poll_option = 0;
uint16_t ch_range_ID = 0;
//uint8_t active_rotary_btn = 0;
uint8_t device_keymode = 0; // kept for LED queueing (0 = scene, 1 = channel)

typedef struct {
    uint16_t channel1_pollVal;
    uint16_t channel2_pollVal;
    uint16_t channel3_pollVal;
} poll_data_t;
poll_data_t poll_buff;

static SemaphoreHandle_t frame_mutex = NULL;
static uint8_t current_frame[FRAME_SIZE];
static bool command_pending = false;

static SemaphoreHandle_t keypad_frame_mutex = NULL;
static uint8_t keypad_current_frame[FRAME_SIZE];
static bool keypad_command_pending = false;

static SemaphoreHandle_t modbus_mutex_uart1 = NULL;
static SemaphoreHandle_t modbus_mutex_uart2 = NULL;
static SemaphoreHandle_t polling_semaphore = NULL;

bool system_healthy = false;
TimerHandle_t task_watchdog_timer = NULL;
#define DEVICE_LIMIT_REACHED() (devices_count >= MAX_DEVICES)
// Button press lockout mechanism
static uint8_t recently_pressed_keypads[MAX_DEVICES] = {0};
static TickType_t press_lockout_timers[MAX_DEVICES] = {0};
#define PRESS_LOCKOUT_PERIOD_MS 500  // 3 seconds lockout

// Function to check if a keypad is in lockout period
// Function to check if a keypad is in lockout period
bool is_keypad_in_lockout(uint8_t keypadNumber) {
    TickType_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (recently_pressed_keypads[i] == keypadNumber) {
            TickType_t time_since_press = current_time - press_lockout_timers[i];
            if (time_since_press < PRESS_LOCKOUT_PERIOD_MS) {
                ESP_LOGI("LOCKOUT_CHECK", "Keypad %d IN lockout (%lu ms remaining)", 
                         keypadNumber, PRESS_LOCKOUT_PERIOD_MS - time_since_press);
                return true;
            } else {
                // Lockout expired, clear the slot
                ESP_LOGI("LOCKOUT_CHECK", "Keypad %d lockout EXPIRED (%lu ms ago)", 
                         keypadNumber, time_since_press - PRESS_LOCKOUT_PERIOD_MS);
                recently_pressed_keypads[i] = 0;
                return false;
            }
        }
    }
    ESP_LOGI("LOCKOUT_CHECK", "Keypad %d NOT in lockout list", keypadNumber);
    return false;
}

// Function to mark a keypad as recently pressed
// Function to mark a keypad as recently pressed
void mark_keypad_pressed(uint8_t keypadNumber) {
    TickType_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // First, check if keypad is already in the list and update it
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (recently_pressed_keypads[i] == keypadNumber) {
            press_lockout_timers[i] = current_time;
            ESP_LOGI("LOCKOUT", "Keypad %d lockout UPDATED - active for %dms", 
                     keypadNumber, PRESS_LOCKOUT_PERIOD_MS);
            return; // Exit early - we updated existing entry
        }
    }
    
    // If not found, find empty slot or replace oldest
    int empty_slot = -1;
    int oldest_slot = 0;
    TickType_t oldest_time = press_lockout_timers[0];
    
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (recently_pressed_keypads[i] == 0) {
            empty_slot = i;
            break;
        }
        if (press_lockout_timers[i] < oldest_time) {
            oldest_time = press_lockout_timers[i];
            oldest_slot = i;
        }
    }
    
    int slot = (empty_slot != -1) ? empty_slot : oldest_slot;
    recently_pressed_keypads[slot] = keypadNumber;
    press_lockout_timers[slot] = current_time;
    
    ESP_LOGI("LOCKOUT", "Keypad %d marked as pressed - lockout active for %dms", 
             keypadNumber, PRESS_LOCKOUT_PERIOD_MS);
}
// Function to calculate Modbus CRC-16
uint16_t modbus_crc16(uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void init_uart1() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_NUM1, 1024, 1024, 0, NULL, 0);
    uart_param_config(UART_NUM1, &uart_config);
    uart_set_pin(UART_NUM1, TXD_PIN1, RXD_PIN1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    gpio_set_direction(RS485_ENABLE_PIN1, GPIO_MODE_OUTPUT);
    RS485_READ_MODE1
}

void init_uart2() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_driver_install(UART_NUM2, 256, 0, 0, NULL, 0);
    uart_param_config(UART_NUM2, &uart_config);
    uart_set_pin(UART_NUM2, TXD_PIN2, RXD_PIN2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    gpio_set_direction(RS485_WRITE_PIN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(RS485_READ_PIN2, GPIO_MODE_OUTPUT);
    RS485_READ_MODE2
}

void initialize_frame_management(void) {
    frame_mutex = xSemaphoreCreateMutex();
    if (frame_mutex == NULL) {
        ESP_LOGE(TAG_RS485_DATA, "Failed to create frame mutex");
    }
    command_pending = false;
    memset(current_frame, 0, FRAME_SIZE);
}

void initialize_keypad_frame_management(void) {
    keypad_frame_mutex = xSemaphoreCreateMutex();
    if (keypad_frame_mutex == NULL) {
        ESP_LOGE(TAG_RS485_DATA, "Failed to create keypad frame mutex");
    }
    keypad_command_pending = false;
    memset(keypad_current_frame, 0, FRAME_SIZE);
}

// ==================== ROBUST MUTEX HANDLING ====================
bool take_mutex_with_timeout(SemaphoreHandle_t mutex, const char* mutex_name, TickType_t timeout) {
    BaseType_t result = xSemaphoreTake(mutex, timeout);
    if (result != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire %s mutex within timeout", mutex_name);
        return false;
    }
    return true;
}

void give_mutex(SemaphoreHandle_t mutex, const char* mutex_name) {
    if (xSemaphoreGive(mutex) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to release %s mutex", mutex_name);
    }
}

// ==================== WATCHDOG FUNCTIONS ====================
void kick_watchdog() {
    system_healthy = true;
}

void system_watchdog_callback(TimerHandle_t xTimer) {
    if (!system_healthy) {
        ESP_LOGE(TAG, "Watchdog timeout - System appears stuck, restarting...");
        esp_restart();
    }
    system_healthy = false;
}

//int scene_to_trigger = -1;
uint8_t send_cmd_buff[FRAME_SIZE];


void send_advOpt_config_command_immediately(uint8_t *config_buff, uint8_t device_mode) {
    if(device_mode == 2) // Curtain with Rotary Config
    {
        if (!take_mutex_with_timeout(modbus_mutex_uart1, "UART1_CURTAIN_IMMEDIATE", pdMS_TO_TICKS(200))) {
            ESP_LOGE("CURTAIN", "Failed to acquire UART1 mutex for immediate CURTAIN Config command");
            return;
        }
        uint8_t curtain_cmd[FRAME_SIZE];
        curtain_cmd[0] = config_buff[0];
        curtain_cmd[1] = 0x55;
        curtain_cmd[2] = config_buff[1];
        curtain_cmd[3] = config_buff[2];
        curtain_cmd[4] = config_buff[3];
        curtain_cmd[5] = 0xFF;
        uint16_t crc = modbus_crc16(curtain_cmd, 6);
        curtain_cmd[6] = crc & 0xFF;
        curtain_cmd[7] = (crc >> 8) & 0xFF;

        RS485_WRITE_MODE1;
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOG_BUFFER_HEXDUMP("curtain_cmd ->", curtain_cmd, FRAME_SIZE, ESP_LOG_INFO);
        int bytes_written = uart_write_bytes(UART_NUM1, (const char *)curtain_cmd, FRAME_SIZE);
        if (bytes_written != FRAME_SIZE) {
            ESP_LOGW("CURTAIN", "UART write incomplete: %d/%d", bytes_written, FRAME_SIZE);
        }
        uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(50));
        give_mutex(modbus_mutex_uart1, "UART1_CURTAIN_IMMEDIATE");
    }
    else if(device_mode == 1) // On Off with keypad Config
    {
        if (!take_mutex_with_timeout(modbus_mutex_uart1, "UART1_ONOFF_IMMEDIATE", pdMS_TO_TICKS(200))) {
            ESP_LOGE("ONOFF", "Failed to acquire UART1 mutex for immediate ONOFF Config command");
            return;
        }
        uint8_t onoff_cmd[FRAME_SIZE];
        onoff_cmd[0] = config_buff[0];
        onoff_cmd[1] = 0x45;
        onoff_cmd[2] = config_buff[1];
        onoff_cmd[3] = config_buff[2];
        onoff_cmd[4] = config_buff[3];
        onoff_cmd[5] = config_buff[4];
        uint16_t crc = modbus_crc16(onoff_cmd, 6);
        onoff_cmd[6] = crc & 0xFF;
        onoff_cmd[7] = (crc >> 8) & 0xFF;

        RS485_WRITE_MODE1;
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOG_BUFFER_HEXDUMP("onoff_cmd ->", onoff_cmd, FRAME_SIZE, ESP_LOG_INFO);
        int bytes_written = uart_write_bytes(UART_NUM1, (const char *)onoff_cmd, FRAME_SIZE);
        if (bytes_written != FRAME_SIZE) {
            ESP_LOGW("ONOFF", "UART write incomplete: %d/%d", bytes_written, FRAME_SIZE);
        }
        uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(50));
        give_mutex(modbus_mutex_uart1, "UART1_ONOFF_IMMEDIATE");
    }
    kick_watchdog();
}

void send_advOpt_config_command_priority(uint8_t *config_buff, uint8_t device_mode) {
    UBaseType_t original_priority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    send_advOpt_config_command_immediately(config_buff, device_mode);
    vTaskPrioritySet(NULL, original_priority);
}
esp_err_t onoff_config_handler(httpd_req_t *req) {
    char query[128];
    char param[16];
    uint8_t keypad_Id = 0;
    uint8_t load1_Button = 0;
    uint8_t load2_Button = 0;
    uint8_t load3_Button = 0;
    uint8_t load4_Button = 0;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        ESP_LOGI("WEB", "Received onoff config query: %s", query);
        
        if (httpd_query_key_value(query, "keypadId", param, sizeof(param)) == ESP_OK) {
            keypad_Id = atoi(param);
            ESP_LOGI("WEB", "Parsed keypadId: %d", keypad_Id);
        } else {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing keypadId parameter");
            return ESP_OK;
        }

        if (httpd_query_key_value(query, "load1Button", param, sizeof(param)) == ESP_OK)
            load1_Button = atoi(param);

        if (httpd_query_key_value(query, "load2Button", param, sizeof(param)) == ESP_OK)
            load2_Button = atoi(param);

        if (httpd_query_key_value(query, "load3Button", param, sizeof(param)) == ESP_OK)
            load3_Button = atoi(param);

        if (httpd_query_key_value(query, "load4Button", param, sizeof(param)) == ESP_OK)
            load4_Button = atoi(param);

        ESP_LOGI("WEB", "On Off Config Received: Keypad %d (loads: %d,%d,%d,%d)",
                 keypad_Id, load1_Button, load2_Button, load3_Button, load4_Button);
        httpd_resp_sendstr(req, "On Off Configuration Saved Successfully!");
        
        uint8_t cmd_buff[5] = {keypad_Id, load1_Button, load2_Button, load3_Button, load4_Button};
        send_advOpt_config_command_priority(cmd_buff, 1);
    } else {
        ESP_LOGE("WEB", "Failed to get query string for onoff config");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid parameters");
    }
    return ESP_OK;
}

// Curtain configuration handler (rotary)
esp_err_t curtain_config_handler(httpd_req_t *req) {
    char query[128];
    char param[16];
    uint8_t rotary_no = 0;
    int open_close1_time = 0;
    int open_close2_time = 0;
    int open_close3_time = 0;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        ESP_LOGI("WEB", "Received curtain config query: %s", query);
        
        if (httpd_query_key_value(query, "rotary", param, sizeof(param)) == ESP_OK) {
            rotary_no = atoi(param);
            ESP_LOGI("WEB", "Parsed rotary: %d", rotary_no);
        } else {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing rotary parameter");
            return ESP_OK;
        }

        if (httpd_query_key_value(query, "load1", param, sizeof(param)) == ESP_OK)
            open_close1_time = atoi(param);

        if (httpd_query_key_value(query, "load2", param, sizeof(param)) == ESP_OK)
            open_close2_time = atoi(param);

        if (httpd_query_key_value(query, "load3", param, sizeof(param)) == ESP_OK)
            open_close3_time = atoi(param);

        ESP_LOGI("WEB", "Curtain Config Received: Rotary %d times %d,%d,%d",
                 rotary_no, open_close1_time, open_close2_time, open_close3_time);
        httpd_resp_sendstr(req, "Curtain Configuration Saved Successfully!");

        uint8_t cmd_buff[4] = {rotary_no, (uint8_t)open_close1_time, (uint8_t)open_close2_time, (uint8_t)open_close3_time};
        send_advOpt_config_command_priority(cmd_buff, 2);
    } else {
        ESP_LOGE("WEB", "Failed to get query string for curtain config");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid parameters");
    }
    return ESP_OK;
}

void print_leftover_devices_in_nvs(void) {
    ESP_LOGI("NVS_CLEANUP", "=== Leftover Devices in NVS ===");
    
    // Count leftover keypads
    nvs_iterator_t it = NULL;
    esp_err_t err = nvs_entry_find(NVS_DEFAULT_PART_NAME, "keypad_ns", NVS_TYPE_BLOB, &it);
    int keypad_count = 0;
    
    if (err == ESP_OK) {
        while (err == ESP_OK && it != NULL) {
            nvs_entry_info_t info;
            nvs_entry_info(it, &info);
            ESP_LOGI("NVS_CLEANUP", "Keypad: %s", info.key);
            keypad_count++;
            err = nvs_entry_next(&it);
        }
        if (it) nvs_release_iterator(it);
    }
    ESP_LOGI("NVS_CLEANUP", "Total leftover keypads: %d", keypad_count);
    
    // Count leftover rotaries
    err = nvs_entry_find(NVS_DEFAULT_PART_NAME, "rotary_ns", NVS_TYPE_BLOB, &it);
    int rotary_count = 0;
    
    if (err == ESP_OK) {
        while (err == ESP_OK && it != NULL) {
            nvs_entry_info_t info;
            nvs_entry_info(it, &info);
            ESP_LOGI("NVS_CLEANUP", "Rotary: %s", info.key);
            rotary_count++;
            err = nvs_entry_next(&it);
        }
        if (it) nvs_release_iterator(it);
    }
    ESP_LOGI("NVS_CLEANUP", "Total leftover rotaries: %d", rotary_count);
    
    ESP_LOGI("NVS_CLEANUP", "=== Total leftover devices: %d ===", keypad_count + rotary_count);
    if(keypad_count == 0) zones_poll_eligible = false;
    if(rotary_count == 0) channels_poll_eligible = false;
    if(keypad_count == 0 && rotary_count == 0){
        ESP_LOGI("WEB", "Deleting ALL devices - clearing all application namespaces");
        
        // List of all namespaces used by your application
        const char* app_namespaces[] = {
            "keypad_ns",
            "rotary_ns", 
            "led_ns",
            "poll_ns",
            "device_ns"  // if you use this
        };
        
        int cleared_count = 0;
        for (int i = 0; i < sizeof(app_namespaces) / sizeof(app_namespaces[0]); i++) {
            nvs_handle_t nvs;
            if (nvs_open(app_namespaces[i], NVS_READWRITE, &nvs) == ESP_OK) {
                esp_err_t err = nvs_erase_all(nvs);
                if (err == ESP_OK) {
                    nvs_commit(nvs);
                    ESP_LOGI("NVS", "Cleared namespace: %s", app_namespaces[i]);
                    cleared_count++;
                }
                nvs_close(nvs);
            }
        }
        
        ESP_LOGI("NVS", "Cleared %d application namespaces", cleared_count);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart(); // Restart to reload empty state

    }
}

void remove_rotary_from_ram(uint8_t rotaryNumber) {
    // Remove from rotary_configs array
    for (int i = 0; i < rotaries_count; i++) {
        if (rotary_configs[i].rotaryNumber == rotaryNumber) {
            // Shift all subsequent elements left
            for (int j = i; j < rotaries_count - 1; j++) {
                memcpy(&rotary_configs[j], &rotary_configs[j + 1], sizeof(rotary_config_t));
            }
            rotaries_count--;
            ESP_LOGI("RAM_CLEANUP", "Removed rotary %d from RAM (remaining: %d)", rotaryNumber, rotaries_count);
            break;
        }
    }
    
    // Remove from device_configs array
    for (int i = 0; i < devices_count; i++) {
        if (device_configs[i].deviceType == DEVICE_TYPE_ROTARY && 
            device_configs[i].config.rotary.rotaryNumber == rotaryNumber) {
            // Shift all subsequent elements left
            for (int j = i; j < devices_count - 1; j++) {
                memcpy(&device_configs[j], &device_configs[j + 1], sizeof(device_config_t));
            }
            devices_count--;
            ESP_LOGI("RAM_CLEANUP", "Removed rotary %d from device_configs (remaining: %d)", rotaryNumber, devices_count);
            break;
        }
    }
    ESP_LOGI(TAG, "Current device count: %d / %d", devices_count, MAX_DEVICES);
}

void remove_keypad_from_ram(uint8_t keypadNumber) {
    // Remove from keypad_configs array
    for (int i = 0; i < keypads_count; i++) {
        if (keypad_configs[i].keypadNumber == keypadNumber) {
            // Shift all subsequent elements left
            for (int j = i; j < keypads_count - 1; j++) {
                memcpy(&keypad_configs[j], &keypad_configs[j + 1], sizeof(keypad_config_t));
                led_states[j] = led_states[j + 1];
            }
            keypads_count--;
            ESP_LOGI("RAM_CLEANUP", "Removed keypad %d from RAM (remaining: %d)", keypadNumber, keypads_count);
            break;
        }
    }
    
    // Remove from keypad_ids_Buff array
    for (int i = 0; i < keypad_id_cnt; i++) {
        if (keypad_ids_Buff[i] == keypadNumber) {
            // Shift all subsequent elements left
            for (int j = i; j < keypad_id_cnt - 1; j++) {
                keypad_ids_Buff[j] = keypad_ids_Buff[j + 1];
            }
            keypad_id_cnt--;
            ESP_LOGI("RAM_CLEANUP", "Removed keypad %d from IDs buffer (remaining: %d)", keypadNumber, keypad_id_cnt);
            break;
        }
    }
    
    // Remove from device_configs array
    for (int i = 0; i < devices_count; i++) {
        if (device_configs[i].deviceType == DEVICE_TYPE_KEYPAD && 
            device_configs[i].config.keypad.keypadNumber == keypadNumber) {
            // Shift all subsequent elements left
            for (int j = i; j < devices_count - 1; j++) {
                memcpy(&device_configs[j], &device_configs[j + 1], sizeof(device_config_t));
            }
            devices_count--;
            ESP_LOGI("RAM_CLEANUP", "Removed keypad %d from device_configs (remaining: %d)", keypadNumber, devices_count);
            
            break;
        }
    }
    ESP_LOGI(TAG, "Current device count: %d / %d", devices_count, MAX_DEVICES);
}

bool delete_keypad_from_nvs(uint8_t keypadNumber) {
    nvs_handle_t nvs;
    char key[16];
    bool found = false;
    
    // First, read the keypad configuration to get scene mappings
    keypad_config_t keypad_config;
    memset(&keypad_config, 0, sizeof(keypad_config_t));
    
    snprintf(key, sizeof(key), "keypad%d", keypadNumber);
    if (nvs_open("keypad_ns", NVS_READWRITE, &nvs) == ESP_OK) {
        size_t len = sizeof(keypad_config_t);
        
        // Try to read the keypad config first to get scene mappings
        if (nvs_get_blob(nvs, key, &keypad_config, &len) == ESP_OK) {
            found = true;
            
            // Print the keypad configuration before deletion
            ESP_LOGI("KEYPAD_DELETE", "Keypad %d configuration before deletion:", keypadNumber);
            ESP_LOGI("KEYPAD_DELETE", "  Slave ID: %d, Zone: %d", keypad_config.slaveId, keypad_config.zoneNumber);
            
            // Now delete the keypad from NVS
            if (nvs_erase_key(nvs, key) == ESP_OK) {
                nvs_commit(nvs);
                ESP_LOGI("NVS", "Deleted keypad %d from NVS", keypadNumber);
                
                // REMOVE FROM RAM ARRAYS
                remove_keypad_from_ram(keypadNumber);
            }
        } else {
            ESP_LOGI("NVS", "Keypad %d not found in NVS", keypadNumber);
        }
        nvs_close(nvs);
    }
    
    // Delete LED state (always try, even if config wasn't found)
    snprintf(key, sizeof(key), "led_state_%d", keypadNumber);
    if (nvs_open("led_ns", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_key(nvs, key);
        nvs_commit(nvs);
        nvs_close(nvs);
        ESP_LOGI("NVS", "Deleted LED state for keypad %d", keypadNumber);
    }
    
    return found;
}

bool delete_rotary_from_nvs(uint8_t rotaryNumber) {
    nvs_handle_t nvs;
    char key[16];
    bool found = false;
    
    // First, read the rotary configuration to get channel values
    rotary_config_t rotary_config;
    memset(&rotary_config, 0, sizeof(rotary_config_t));
    
    snprintf(key, sizeof(key), "rotary%d", rotaryNumber);
    if (nvs_open("rotary_ns", NVS_READWRITE, &nvs) == ESP_OK) {
        size_t len = sizeof(rotary_config_t);
        
        // Try to read the rotary config first to get channel values
        if (nvs_get_blob(nvs, key, &rotary_config, &len) == ESP_OK) {
            found = true;
            
            // Print the rotary configuration before deletion
            ESP_LOGI("ROTARY_DELETE", "Rotary %d configuration before deletion:", rotaryNumber);
            ESP_LOGI("ROTARY_DELETE", "  Slave ID: %d", rotary_config.slaveId);
            
            for (int i = 0; i < 3; i++) {
                if (rotary_config.keyValue[i] != 0) {
                    ESP_LOGI("ROTARY_DELETE", "  Button %d: Channel %d", i + 1, rotary_config.keyValue[i]);
                }
            }
            
            // Now delete the rotary from NVS
            if (nvs_erase_key(nvs, key) == ESP_OK) {
                nvs_commit(nvs);
                ESP_LOGI("NVS", "Deleted rotary %d from NVS", rotaryNumber);
                
                // Remove associated channels from polling ranges ONLY if no other rotary uses them
                for (int i = 0; i < 3; i++) {
                    if (rotary_config.keyValue[i] != 0) {
                        update_channel_polling_ranges(rotary_config.keyValue[i], true, rotaryNumber);
                        ESP_LOGI("ROTARY_DELETE", "Processed channel %d for polling range cleanup", rotary_config.keyValue[i]);
                    }
                }
                
                // REMOVE FROM RAM ARRAYS
                remove_rotary_from_ram(rotaryNumber);
            }
        } else {
            ESP_LOGI("NVS", "Rotary %d not found in NVS", rotaryNumber);
        }
        nvs_close(nvs);
    }
    
    return found;
}

esp_err_t delete_device_handler(httpd_req_t *req) {
    char query[32];
    char id_str[8];
    int device_id = -1;

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "id", id_str, sizeof(id_str)) == ESP_OK) {
            device_id = atoi(id_str);
        }
    }

    if (device_id == 0) {
        httpd_resp_send(req, "Invalid device ID", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }

    if (device_id == -1) {
        ESP_LOGI("WEB", "Deleting ALL devices - clearing all application namespaces");
        
        // List of all namespaces used by your application
        const char* app_namespaces[] = {
            "keypad_ns",
            "rotary_ns", 
            "led_ns",
            "poll_ns",
            "device_ns"  // if you use this
        };
        
        int cleared_count = 0;
        for (int i = 0; i < sizeof(app_namespaces) / sizeof(app_namespaces[0]); i++) {
            nvs_handle_t nvs;
            if (nvs_open(app_namespaces[i], NVS_READWRITE, &nvs) == ESP_OK) {
                esp_err_t err = nvs_erase_all(nvs);
                if (err == ESP_OK) {
                    nvs_commit(nvs);
                    ESP_LOGI("NVS", "Cleared namespace: %s", app_namespaces[i]);
                    cleared_count++;
                }
                nvs_close(nvs);
            }
        }
        
        ESP_LOGI("NVS", "Cleared %d application namespaces", cleared_count);
        httpd_resp_send(req, "All Devices Deleted Successfully - Application Data Cleared", HTTPD_RESP_USE_STRLEN);
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart(); // Restart to reload empty state
        return ESP_OK;
    }

    ESP_LOGI("WEB", "Deleting device %d from NVS", device_id);
    
    // Try to delete as keypad
    bool deleted = delete_keypad_from_nvs(device_id);
    
    // If not a keypad, try as rotary
    if (!deleted) {
        deleted = delete_rotary_from_nvs(device_id);
    }

    if (deleted) {
        httpd_resp_send(req, "Device Deleted Successfully", HTTPD_RESP_USE_STRLEN);
        // Print leftover devices after deletion
        print_leftover_devices_in_nvs();

    } else {
        httpd_resp_send(req, "Device Not Found", HTTPD_RESP_USE_STRLEN);
    }
    
    return ESP_OK;
}


// Save LED state to NVS
// Save LED state to NVS
void save_led_state_to_nvs(uint8_t keypadNumber, uint16_t led_state) {
    // Update the LED state in RAM first
    bool found_in_ram = false;
    for (int i = 0; i < keypads_count; i++) {
        if (keypad_configs[i].keypadNumber == keypadNumber) {
            led_states[i] = led_state;
            found_in_ram = true;
            ESP_LOGI("NVS_LED", "Updated LED state 0x%04X for keypad %d in RAM", led_state, keypadNumber);
            break;
        }
    }
    
    if (!found_in_ram) {
        ESP_LOGW("NVS_LED", "Keypad %d not found in RAM while saving LED state", keypadNumber);
    }
    
    // Save to dedicated LED state storage
    nvs_handle_t led_nvs;
    char led_key[20];
    snprintf(led_key, sizeof(led_key), "led_state_%d", keypadNumber);
    
    esp_err_t err = nvs_open("led_ns", NVS_READWRITE, &led_nvs);
    if (err == ESP_OK) {
        err = nvs_set_u16(led_nvs, led_key, led_state);
        if (err == ESP_OK) {
            nvs_commit(led_nvs);
            ESP_LOGI("NVS_LED", "Saved LED state 0x%04X for keypad %d to NVS", led_state, keypadNumber);
        } else {
            ESP_LOGE("NVS_LED", "Failed to set LED state for keypad %d: %s", keypadNumber, esp_err_to_name(err));
        }
        nvs_close(led_nvs);
    } else {
        ESP_LOGE("NVS_LED", "Failed to open LED namespace for keypad %d: %s", keypadNumber, esp_err_to_name(err));
    }
}

// Enhanced get_led_state_from_nvs that checks dedicated storage first
uint16_t get_led_state_from_nvs_enhanced(uint8_t keypadNumber) {
    // First check if keypad exists in RAM and return its current state
    for (int i = 0; i < keypads_count; i++) {
        if (keypad_configs[i].keypadNumber == keypadNumber) {
            // Keypad exists in RAM, return current LED state
            ESP_LOGI("NVS_LED", "Retrieved LED state 0x%04X for keypad %d from RAM", led_states[i], keypadNumber);
            return led_states[i];
        }
    }
    
    // If not in RAM, try dedicated LED storage
    nvs_handle_t nvs;
    char key[20];
    uint16_t led_state = 0;
    
    snprintf(key, sizeof(key), "led_state_%d", keypadNumber);
    if (nvs_open("led_ns", NVS_READONLY, &nvs) == ESP_OK) {
        if (nvs_get_u16(nvs, key, &led_state) == ESP_OK) {
            ESP_LOGI("NVS_LED", "Retrieved LED state 0x%04X for keypad %d from dedicated storage", led_state, keypadNumber);
            nvs_close(nvs);
            return led_state;
        }
        nvs_close(nvs);
    }
    
    // Default state
    ESP_LOGI("NVS_LED", "Using default LED state 0x0000 for keypad %d", keypadNumber);
    return 0x0000;
}

// void check_and_log_button_changes(uint8_t *clean_data) {
//     uint8_t panel_id = clean_data[0];
//     int active_btn = -1;
//     int Rotary_val = -1;

//     // Rotary frame detection
//     if (clean_data[3] == 0x10) { // Rotary
//         if (clean_data[4] == 0x01) active_btn = 1;
//         else if (clean_data[4] == 0x02) active_btn = 2;
//         else if (clean_data[4] == 0x03) active_btn = 3;
//         Rotary_val = clean_data[5];
//     } else { // Keypads
//         uint8_t val = clean_data[5];
//         switch (val) {
//             case 0x01: active_btn = 1; break;
//             case 0x02: active_btn = 2; break;
//             case 0x04: active_btn = 3; break;
//             case 0x08: active_btn = 4; break;
//             case 0x10: active_btn = 5; break;
//             case 0x20: active_btn = 6; break;
//             case 0x40: active_btn = 7; break;
//             case 0x80: active_btn = 8; break;
//             default: active_btn = 0; break;
//         }
//     }

//     if (active_btn == 0) return;

//     // First try to find a keypad with this panel_id
//     for (int i = 0; i < keypads_count; i++) {
//         keypad_config_t *cfg = &keypad_configs[i];
//         if (cfg->keypadNumber == panel_id) {
//             uint8_t scene_to_trigger = cfg->keyOnScene[active_btn - 1];
//             int zone = cfg->zoneNumber;
//             int slaveId = cfg->slaveId;

//             web_active_keypad_no = panel_id;
//             last_button_press_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
//             // Get previous LED state from NVS
//             uint16_t previous_led_state = get_led_state_from_nvs_enhanced(panel_id);
//             uint16_t btn_mask = (1 << (active_btn - 1));
//             bool was_led_on = (previous_led_state & btn_mask) != 0;
            
//             ESP_LOGI(TAG_BTN, "Keypad %d, Button %d PRESSED | Slave: %d, Zone: %d | Previous LED: %s", 
//                      panel_id, active_btn, slaveId, zone, was_led_on ? "ON" : "OFF");
//             poll_deviceType = 1;
            
//             // === CHANGED SECTION START ===
//             // Determine which scene to send based on previous LED state
//             uint8_t scene_to_send;
//             if (was_led_on) {
//                 // If LED was ON, send OFF scene and turn LED OFF
//                 scene_to_send = cfg->keyOffScene[active_btn - 1];
//                 ESP_LOGI(TAG_BTN, "LED was ON → Sending OFF scene: %d", scene_to_send);
//             } else {
//                 // If LED was OFF, send ON scene and turn LED ON  
//                 scene_to_send = cfg->keyOnScene[active_btn - 1];
//                 ESP_LOGI(TAG_BTN, "LED was OFF → Sending ON scene: %d", scene_to_send);
//             }

//             // Only send scene if it's assigned (not -1)
//             if (scene_to_send != (uint8_t)-1) {  // Compare with -1 cast to uint8_t
//                 ESP_LOGI(TAG_BTN, "Sending Scene %d to Slave %d, Zone %d", scene_to_send, slaveId, zone);
//                 send_scene_to_controller(slaveId, zone, scene_to_send);
//             } else {
//                 ESP_LOGW(TAG_BTN, "No scene configured for this button state (unassigned)");
//             }
//             // === CHANGED SECTION END ===

//             // Update LED state based on previous state (toggle)
//             uint16_t *led_state = &led_states[i];
//             if (was_led_on) {
//                 // Turn LED OFF
//                 *led_state &= ~btn_mask;
//                 ESP_LOGI("LED_TOGGLE", "Keypad %d: Button %d → LED OFF (0x%04X)", panel_id, active_btn, *led_state);
//                 dry_cmd_active_onoff_btn = active_btn;
//                 is_btn_state_on = false;
//             } else {
//                 // Turn LED ON
//                 *led_state |= btn_mask;
//                 ESP_LOGI("LED_TOGGLE", "Keypad %d: Button %d → LED ON (0x%04X)", panel_id, active_btn, *led_state);
//                 dry_cmd_active_onoff_btn = active_btn;
//                 is_btn_state_on = true;
//             }
            
//             // Always set backlight if configured
//             if (BACKLIGHT) *led_state |= (1 << 8);
            
//             // Send LED command
//             send_led_command_priority(panel_id, *led_state);
            
//             // Save LED state to NVS immediately
//             save_led_state_to_nvs(panel_id, *led_state);
            
//             return;
//         }
//     }

//     // If not a keypad, maybe it's a rotary
//     if (clean_data[3] == 0x10) {
//         // Find the rotary config by rotaryNumber == panel_id
//         for (int r = 0; r < rotaries_count; r++) {
//             rotary_config_t *rc = &rotary_configs[r];
//             if (rc->rotaryNumber == panel_id) {
//                 web_active_keypad_no = panel_id;
//                 last_button_press_time = xTaskGetTickCount() * portTICK_PERIOD_MS;                
//                 int btn_index = active_btn - 1;
//                 if (btn_index >= 0 && btn_index < 3 && rc->keyValue[btn_index] > 0) {
//                     uint16_t channel_no = rc->keyValue[btn_index];
//                     uint8_t hexValue = (uint8_t)round(Rotary_val / 2.55);
//                     ESP_LOGI(TAG_BTN, "Rotary %d: Button %d -> Channel %d -> Percent %d (Slave %d)", 
//                              panel_id, active_btn, channel_no, hexValue, rc->slaveId);
//                     poll_deviceType = 2;
//                     send_channel_to_controller(rc->slaveId, channel_no, Rotary_val);
//                 } else {
//                     ESP_LOGW(TAG_BTN, "Rotary %d Button %d has no channel configured", panel_id, active_btn);
//                 }
//                 return;
//             }
//         }
//     }

//     ESP_LOGW(TAG_BTN, "Unknown panel ID: %d", panel_id);
// }
void check_and_log_button_changes(uint8_t *clean_data) {
    uint8_t panel_id = clean_data[0];
    int active_btn = -1;
    int Rotary_val = -1;

    // Rotary frame detection
    if (clean_data[3] == 0x10) { // Rotary
        if (clean_data[4] == 0x01) active_btn = 1;
        else if (clean_data[4] == 0x02) active_btn = 2;
        else if (clean_data[4] == 0x03) active_btn = 3;
        Rotary_val = clean_data[5];
    } else { // Keypads
        uint8_t val = clean_data[5];
        switch (val) {
            case 0x01: active_btn = 1; break;
            case 0x02: active_btn = 2; break;
            case 0x04: active_btn = 3; break;
            case 0x08: active_btn = 4; break;
            case 0x10: active_btn = 5; break;
            case 0x20: active_btn = 6; break;
            case 0x40: active_btn = 7; break;
            case 0x80: active_btn = 8; break;
            default: active_btn = 0; break;
        }
    }

    if (active_btn == 0) return;

    // First try to find a keypad with this panel_id
    for (int i = 0; i < keypads_count; i++) {
        keypad_config_t *cfg = &keypad_configs[i];
        if (cfg->keypadNumber == panel_id) {
            uint8_t scene_to_trigger = cfg->keyOnScene[active_btn - 1];
            int zone = cfg->zoneNumber;
            int slaveId = cfg->slaveId;

            web_active_keypad_no = panel_id;
            last_button_press_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // === MARK KEYPAD AS PRESSED ===
            mark_keypad_pressed(panel_id);
            
            // Get previous LED state from NVS
            uint16_t previous_led_state = get_led_state_from_nvs_enhanced(panel_id);
            uint16_t btn_mask = (1 << (active_btn - 1));
            bool was_led_on = (previous_led_state & btn_mask) != 0;
            
            ESP_LOGI(TAG_BTN, "Keypad %d, Button %d PRESSED | Slave: %d, Zone: %d | Previous LED: %s", 
                     panel_id, active_btn, slaveId, zone, was_led_on ? "ON" : "OFF");
            poll_deviceType = 1;
            
            // Determine which scene to send based on previous LED state
            uint8_t scene_to_send;
            if (was_led_on) {
                // If LED was ON, send OFF scene and turn LED OFF
                scene_to_send = cfg->keyOffScene[active_btn - 1];
                ESP_LOGI(TAG_BTN, "LED was ON → Sending OFF scene: %d", scene_to_send);
            } else {
                // If LED was OFF, send ON scene and turn LED ON  
                scene_to_send = cfg->keyOnScene[active_btn - 1];
                ESP_LOGI(TAG_BTN, "LED was OFF → Sending ON scene: %d", scene_to_send);
            }

            // Only send scene if it's assigned (not -1)
            if (scene_to_send != (uint8_t)-1) {
                ESP_LOGI(TAG_BTN, "Sending Scene %d to Slave %d, Zone %d", scene_to_send, slaveId, zone);
                send_scene_to_controller(slaveId, zone, scene_to_send);
            } else {
                ESP_LOGW(TAG_BTN, "No scene configured for this button state (unassigned)");
            }

            // Update LED state based on previous state (toggle)
            uint16_t *led_state = &led_states[i];
            if (was_led_on) {
                // Turn LED OFF
                *led_state &= ~btn_mask;
                ESP_LOGI("LED_TOGGLE", "Keypad %d: Button %d → LED OFF (0x%04X)", panel_id, active_btn, *led_state);
                dry_cmd_active_onoff_btn = active_btn;
                is_btn_state_on = false;
            } else {
                // Turn LED ON
                *led_state |= btn_mask;
                ESP_LOGI("LED_TOGGLE", "Keypad %d: Button %d → LED ON (0x%04X)", panel_id, active_btn, *led_state);
                dry_cmd_active_onoff_btn = active_btn;
                is_btn_state_on = true;
            }
            
            // Always set backlight if configured
            if (BACKLIGHT) *led_state |= (1 << 8);
            
            // Send LED command immediately for button press
            send_led_command_priority(panel_id, *led_state);
            
            // Save LED state to NVS immediately
            save_led_state_to_nvs(panel_id, *led_state);
            
            return;
        }
    }

    // If not a keypad, maybe it's a rotary
    if (clean_data[3] == 0x10) {
        // Find the rotary config by rotaryNumber == panel_id
        for (int r = 0; r < rotaries_count; r++) {
            rotary_config_t *rc = &rotary_configs[r];
            if (rc->rotaryNumber == panel_id) {
                web_active_keypad_no = panel_id;
                last_button_press_time = xTaskGetTickCount() * portTICK_PERIOD_MS;                
                int btn_index = active_btn - 1;
                if (btn_index >= 0 && btn_index < 3 && rc->keyValue[btn_index] > 0) {
                    uint16_t channel_no = rc->keyValue[btn_index];
                    uint8_t hexValue = (uint8_t)round(Rotary_val / 2.55);
                    ESP_LOGI(TAG_BTN, "Rotary %d: Button %d -> Channel %d -> Percent %d (Slave %d)", 
                             panel_id, active_btn, channel_no, hexValue, rc->slaveId);
                    poll_deviceType = 2;
                    send_channel_to_controller(rc->slaveId, channel_no, Rotary_val);
                } else {
                    ESP_LOGW(TAG_BTN, "Rotary %d Button %d has no channel configured", panel_id, active_btn);
                }
                return;
            }
        }
    }

    ESP_LOGW(TAG_BTN, "Unknown panel ID: %d", panel_id);
}

static TickType_t last_send_time = 0;
const TickType_t send_interval = pdMS_TO_TICKS(500);

// static void dynamic_keypad_polling_handler(void) {
//     uint8_t rx_data[256];
//     uint8_t frame_to_send[FRAME_SIZE];
//     bool processing_led_command = false;

//     if (!take_mutex_with_timeout(modbus_mutex_uart1, "UART1", pdMS_TO_TICKS(100))) {
//         ESP_LOGW(TAG_RS485_DATA, "Failed to take UART1 mutex in dynamic_keypad_polling_handler");
//         return;
//     }

//     // Process LED commands from queue
//     if (led_command_ready) {
//         if (xSemaphoreTake(led_queue_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
//             if (led_queue_head != led_queue_tail) {
//                 led_command_t *cmd = &led_command_queue[led_queue_head];
//                 processing_led_command = true;
                
//                 if (cmd->device_keymode == 0) { // Scene Based Device
//                     ESP_LOGI("LED_POLL", "Sending keypad %d LED/DIM command via polling handler", cmd->deviceNumber);
                    
//                     // Send LED command
//                     RS485_WRITE_MODE1;
//                     vTaskDelay(pdMS_TO_TICKS(8));
//                     uart_write_bytes(UART_NUM1, (const char *)cmd->led_cmd, FRAME_SIZE);
//                     uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(30));
//                     ESP_LOG_BUFFER_HEXDUMP("led cmd ->", cmd->led_cmd, FRAME_SIZE, ESP_LOG_INFO);
//                     for(int i=0;i<FRAME_SIZE;i++){
//                         printf("%02X ", cmd->led_cmd[i]);
//                     }
//                     printf("\n");
//                     vTaskDelay(pdMS_TO_TICKS(100));
                    
//                     // Send dry command
//                     ESP_LOGI("DRY-CMD", "Sending dry command via polling handler");
//                     ESP_LOG_BUFFER_HEXDUMP("dry cmd ->", cmd->dry_onoff_cmd, FRAME_SIZE, ESP_LOG_INFO);
//                     uart_write_bytes(UART_NUM1, (const char *)cmd->dry_onoff_cmd, FRAME_SIZE);
//                     for(int i=0;i<FRAME_SIZE;i++){
//                         printf("%02X ", cmd->dry_onoff_cmd[i]);
//                     }
//                     printf("\n");
//                     uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(30));
//                     RS485_READ_MODE1;
                    
//                 } else if (cmd->device_keymode == 1) { // Channel Based Device (rotary)
//                     ESP_LOGI("LED_POLL", "Sending rotary %d DIM command via polling handler", cmd->deviceNumber);
                    
//                     // Send rotary command (only LED command, no dry command for rotary)
//                     RS485_WRITE_MODE1;
//                     vTaskDelay(pdMS_TO_TICKS(8));
//                     uart_write_bytes(UART_NUM1, (const char *)cmd->led_cmd, FRAME_SIZE);
//                     uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(30));
//                     for(int i=0;i<FRAME_SIZE;i++){
//                         printf("%02X ", cmd->led_cmd[i]);
//                     }
//                     printf("\n");
//                     ESP_LOG_BUFFER_HEXDUMP("rotary cmd ->", cmd->led_cmd, FRAME_SIZE, ESP_LOG_INFO);
//                     RS485_READ_MODE1;
//                 }
                
//                 // Remove command from queue
//                 cmd->pending = false;
//                 led_queue_head = (led_queue_head + 1) % LED_QUEUE_SIZE;
                
//                 // Check if more commands pending
//                 if (led_queue_head == led_queue_tail) {
//                     led_command_ready = false; // No more commands
//                     ESP_LOGI("LED_QUEUE", "All commands processed, queue empty");
//                 } else {
//                     ESP_LOGI("LED_QUEUE", "Command processed, %d remaining in queue", 
//                             (led_queue_tail - led_queue_head + LED_QUEUE_SIZE) % LED_QUEUE_SIZE);
//                 }
//             } else {
//                 led_command_ready = false; // Queue empty
//                 ESP_LOGI("LED_QUEUE", "Queue empty, no commands to process");
//             }
//             xSemaphoreGive(led_queue_mutex);
//         } else {
//             ESP_LOGW("LED_POLL", "Failed to acquire LED queue mutex, skipping LED command processing");
//         }
        
//         if (processing_led_command) {
//             // Skip regular polling when processing LED commands
//             give_mutex(modbus_mutex_uart1, "UART1");
//             kick_watchdog();
//             return;
//         }
//     }

//     // Regular polling code (your existing code)
//     frame_to_send[0] = 0xFD;
//     frame_to_send[1] = 0x03;
//     frame_to_send[2] = 0x10;
//     frame_to_send[3] = 0x0B;
//     frame_to_send[4] = 0x00;
//     frame_to_send[5] = 0x01;
//     frame_to_send[6] = 0xE5;
//     frame_to_send[7] = 0x34;

//     RS485_WRITE_MODE1;
//     vTaskDelay(pdMS_TO_TICKS(8));
//     uart_write_bytes(UART_NUM1, (const char *)frame_to_send, FRAME_SIZE);
//     uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(30));
//     RS485_READ_MODE1;
//     vTaskDelay(pdMS_TO_TICKS(5));

//     int len = uart_read_bytes(UART_NUM1, rx_data, sizeof(rx_data), pdMS_TO_TICKS(80));
//     if (len > 0) {
//         ESP_LOGI(TAG_RS485_DATA, "Keypad Response (%d bytes):", len);
//         ESP_LOG_BUFFER_HEXDUMP("Keypad Response ->", rx_data, FRAME_SIZE, ESP_LOG_INFO);
//         if (!led_command_ready) {
//             uint8_t clean_data[FRAME_SIZE];
//             if (clean_devices_response(rx_data, len, clean_data, FRAME_SIZE)) {
//                 if (clean_data[1] == 0x03) {
//                     check_and_log_button_changes(clean_data);
//                 }
//                 if (clean_data[1] == 0x03 && clean_data[3] == 0x10) {
//                     TickType_t current_time = xTaskGetTickCount();
//                     if (current_time - last_send_time >= send_interval) {
//                         last_send_time = current_time;
//                         check_and_log_button_changes(clean_data);
//                     } else {
//                         ESP_LOGI("LED_POLL", "Send throttled - too soon since last send");
//                     }
//                 }
//             }
//         } else {
//             ESP_LOGI("LED_POLL", "LED command response received, ignoring button processing");
//         }
//     }
//     give_mutex(modbus_mutex_uart1, "UART1");
//     kick_watchdog();
// }


// static void dynamic_keypad_polling_handler(void) {
//     uint8_t rx_data[64];
//     uint8_t frame_to_send[FRAME_SIZE];
//     bool processing_led_command = false;

//     if (!take_mutex_with_timeout(modbus_mutex_uart1, "UART1", pdMS_TO_TICKS(100))) {
//         ESP_LOGW(TAG_RS485_DATA, "Failed to take UART1 mutex in dynamic_keypad_polling_handler");
//         return;
//     }

//     // === ALWAYS SEND POLLING FRAME FIRST (PRIORITIZE BUTTON DETECTION) ===
//     frame_to_send[0] = 0xFD;
//     frame_to_send[1] = 0x03;
//     frame_to_send[2] = 0x10;
//     frame_to_send[3] = 0x0B;
//     frame_to_send[4] = 0x00;
//     frame_to_send[5] = 0x01;
//     frame_to_send[6] = 0xE5;
//     frame_to_send[7] = 0x34;

//     RS485_WRITE_MODE1;
//     vTaskDelay(pdMS_TO_TICKS(8));
//     uart_write_bytes(UART_NUM1, (const char *)frame_to_send, FRAME_SIZE);
//     uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(10));
//     RS485_READ_MODE1;
//     vTaskDelay(pdMS_TO_TICKS(3));
//    // ESP_LOG_BUFFER_HEXDUMP("Keypad poll ->", frame_to_send, FRAME_SIZE, ESP_LOG_INFO);
//     // === READ RESPONSE IMMEDIATELY ===
//     int len = uart_read_bytes(UART_NUM1, rx_data, sizeof(rx_data), pdMS_TO_TICKS(50));
    
//     // === PROCESS BUTTON PRESSES FIRST ===
//     if (len > 0) {
//         ESP_LOGI(TAG_RS485_DATA, "Keypad Response (%d bytes):", len);
//         ESP_LOG_BUFFER_HEXDUMP("Keypad Response ->", rx_data, FRAME_SIZE, ESP_LOG_INFO);
        
//         uint8_t clean_data[FRAME_SIZE];
//         if (clean_devices_response(rx_data, len, clean_data, FRAME_SIZE)) {
//             if (clean_data[1] == 0x03) {
//                 // Process button presses immediately (don't wait for LED commands)
//                 check_and_log_button_changes(clean_data);
//             }
//         }
//     }

//     // === ONLY THEN PROCESS LED COMMANDS ===
//     if (led_command_ready) {
//         if (xSemaphoreTake(led_queue_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
//             if (led_queue_head != led_queue_tail) {
//                 led_command_t *cmd = &led_command_queue[led_queue_head];
//                 processing_led_command = true;
                
//                 if (cmd->device_keymode == 0) { // Scene Based Device
//                     ESP_LOGI("LED_POLL", "Sending keypad %d LED/DIM command", cmd->deviceNumber);
                    
//                     // Send LED command
//                     RS485_WRITE_MODE1;
//                     vTaskDelay(pdMS_TO_TICKS(8));
//                     uart_write_bytes(UART_NUM1, (const char *)cmd->led_cmd, FRAME_SIZE);
//                     uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(30));
//                     ESP_LOG_BUFFER_HEXDUMP("led cmd ->", cmd->led_cmd, FRAME_SIZE, ESP_LOG_INFO);
                    
//                     vTaskDelay(pdMS_TO_TICKS(50)); // Reduced delay
                    
//                     // Send dry command
//                     ESP_LOGI("DRY-CMD", "Sending dry command");
//                     ESP_LOG_BUFFER_HEXDUMP("dry cmd ->", cmd->dry_onoff_cmd, FRAME_SIZE, ESP_LOG_INFO);
//                     uart_write_bytes(UART_NUM1, (const char *)cmd->dry_onoff_cmd, FRAME_SIZE);
//                     uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(30));
//                     RS485_READ_MODE1;
                    
//                 } else if (cmd->device_keymode == 1) { // Channel Based Device (rotary)
//                     ESP_LOGI("LED_POLL", "Sending rotary %d DIM command", cmd->deviceNumber);
                    
//                     // Send rotary command
//                     RS485_WRITE_MODE1;
//                     vTaskDelay(pdMS_TO_TICKS(8));
//                     uart_write_bytes(UART_NUM1, (const char *)cmd->led_cmd, FRAME_SIZE);
//                     uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(30));
//                     ESP_LOG_BUFFER_HEXDUMP("rotary cmd ->", cmd->led_cmd, FRAME_SIZE, ESP_LOG_INFO);
//                     RS485_READ_MODE1;
//                 }
                
//                 // Remove command from queue
//                 cmd->pending = false;
//                 led_queue_head = (led_queue_head + 1) % LED_QUEUE_SIZE;
                
//                 // Check if more commands pending
//                 if (led_queue_head == led_queue_tail) {
//                     led_command_ready = false;
//                     ESP_LOGI("LED_QUEUE", "All commands processed, queue empty");
//                 } else {
//                     ESP_LOGI("LED_QUEUE", "Command processed, %d remaining in queue", 
//                             (led_queue_tail - led_queue_head + LED_QUEUE_SIZE) % LED_QUEUE_SIZE);
//                 }
//             } else {
//                 led_command_ready = false;
//                 ESP_LOGI("LED_QUEUE", "Queue empty, no commands to process");
//             }
//             xSemaphoreGive(led_queue_mutex);
//         } else {
//             ESP_LOGW("LED_POLL", "Failed to acquire LED queue mutex, skipping LED command processing");
//         }
//     }

//     give_mutex(modbus_mutex_uart1, "UART1");
//     kick_watchdog();
// }
#include "esp_rom_sys.h"
#include "rom/ets_sys.h"

static void dynamic_keypad_polling_handler(void) {
    uint8_t rx_data[64];
    uint8_t frame_to_send[FRAME_SIZE];
    bool processing_led_command = false;

    if (!take_mutex_with_timeout(modbus_mutex_uart1, "UART1", pdMS_TO_TICKS(100))) {
        ESP_LOGW(TAG_RS485_DATA, "Failed to take UART1 mutex in dynamic_keypad_polling_handler");
        return;
    }

    // === ALWAYS SEND POLLING FRAME FIRST (PRIORITIZE BUTTON DETECTION) ===
    frame_to_send[0] = 0xFD;
    frame_to_send[1] = 0x03;
    frame_to_send[2] = 0x10;
    frame_to_send[3] = 0x0B;
    frame_to_send[4] = 0x00;
    frame_to_send[5] = 0x01;
    frame_to_send[6] = 0xE5;
    frame_to_send[7] = 0x34;

    RS485_WRITE_MODE1;
    vTaskDelay(pdMS_TO_TICKS(8));
    //ets_delay_us(200);
    uart_write_bytes(UART_NUM1, (const char *)frame_to_send, FRAME_SIZE);
    uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(10));
     //ets_delay_us(300);
    RS485_READ_MODE1;
    //vTaskDelay(pdMS_TO_TICKS(15));
    ESP_LOGI(TAG_RS485_DATA, "Sent");
    // === READ RESPONSE IMMEDIATELY ===
    int len = uart_read_bytes(UART_NUM1, rx_data, sizeof(rx_data), pdMS_TO_TICKS(100));
    
    // === PROCESS BUTTON PRESSES FIRST ===
    if (len > 0) {
        ESP_LOGI(TAG_RS485_DATA, "Keypad Response (%d bytes):", len);
        ESP_LOG_BUFFER_HEXDUMP("Keypad Response ->", rx_data, FRAME_SIZE, ESP_LOG_INFO);
        
        uint8_t clean_data[FRAME_SIZE];
        if (clean_devices_response(rx_data, len, clean_data, FRAME_SIZE)) {
            if (clean_data[1] == 0x03) {
                // === ADDED: Check if this keypad is in lockout period ===
              //  uint8_t panel_id = clean_data[0];
                
               // if (is_keypad_in_lockout(panel_id)) {
                //    ESP_LOGW("DUPLICATE_FILTER", "Keypad %d response ignored - within 500ms lockout period", panel_id);
               // } else {
                    // Process button presses immediately
                    check_and_log_button_changes(clean_data);
               // }
            }
        }
    }

    // === ONLY THEN PROCESS LED COMMANDS ===
    if (led_command_ready) {
        if (xSemaphoreTake(led_queue_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (led_queue_head != led_queue_tail) {
                led_command_t *cmd = &led_command_queue[led_queue_head];
                processing_led_command = true;
                
                if (cmd->device_keymode == 0) { // Scene Based Device
                    ESP_LOGI("LED_POLL", "Sending keypad %d LED/DIM command", cmd->deviceNumber);
                    
                    // Send LED command
                    RS485_WRITE_MODE1;
                    //vTaskDelay(pdMS_TO_TICKS(8));
                    ets_delay_us(200);
                    uart_write_bytes(UART_NUM1, (const char *)cmd->led_cmd, FRAME_SIZE);
                    uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(20));
                    ets_delay_us(300);
                    ESP_LOG_BUFFER_HEXDUMP("led cmd ->", cmd->led_cmd, FRAME_SIZE, ESP_LOG_INFO);
                    
                    vTaskDelay(pdMS_TO_TICKS(50)); // Reduced delay
                    
                    // Send dry command
                    ESP_LOGI("DRY-CMD", "Sending dry command");
                    ESP_LOG_BUFFER_HEXDUMP("dry cmd ->", cmd->dry_onoff_cmd, FRAME_SIZE, ESP_LOG_INFO);
                    uart_write_bytes(UART_NUM1, (const char *)cmd->dry_onoff_cmd, FRAME_SIZE);
                    uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(30));
                    RS485_READ_MODE1;
                    vTaskDelay(pdMS_TO_TICKS(3));
                } else if (cmd->device_keymode == 1) { // Channel Based Device (rotary)
                    ESP_LOGI("LED_POLL", "Sending rotary %d DIM command", cmd->deviceNumber);
                    
                    // Send rotary command
                    RS485_WRITE_MODE1;
                    //vTaskDelay(pdMS_TO_TICKS(8));
                    ets_delay_us(200);
                    uart_write_bytes(UART_NUM1, (const char *)cmd->led_cmd, FRAME_SIZE);
                    uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(20));
                    ets_delay_us(300);
                    ESP_LOG_BUFFER_HEXDUMP("rotary cmd ->", cmd->led_cmd, FRAME_SIZE, ESP_LOG_INFO);
                    RS485_READ_MODE1;
                    vTaskDelay(pdMS_TO_TICKS(3));
                }
                
                // Remove command from queue
                cmd->pending = false;
                led_queue_head = (led_queue_head + 1) % LED_QUEUE_SIZE;
                
                // Check if more commands pending
                if (led_queue_head == led_queue_tail) {
                    led_command_ready = false;
                    ESP_LOGI("LED_QUEUE", "All commands processed, queue empty");
                } else {
                    ESP_LOGI("LED_QUEUE", "Command processed, %d remaining in queue", 
                            (led_queue_tail - led_queue_head + LED_QUEUE_SIZE) % LED_QUEUE_SIZE);
                }
            } else {
                led_command_ready = false;
                ESP_LOGI("LED_QUEUE", "Queue empty, no commands to process");
            }
            xSemaphoreGive(led_queue_mutex);
        } else {
            ESP_LOGW("LED_POLL", "Failed to acquire LED queue mutex, skipping LED command processing");
        }
    }

    give_mutex(modbus_mutex_uart1, "UART1");
    kick_watchdog();
}
void queue_led_command_for_polling(uint8_t deviceNumber, uint16_t value) {
    if (led_queue_mutex == NULL) {
        ESP_LOGE("LED_QUEUE", "LED queue mutex not initialized");
        return;
    }

    int retry_count = 0;
    const int max_retries = 3;

    while (retry_count < max_retries) {
        if (xSemaphoreTake(led_queue_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Check current queue size
            uint8_t queue_size = (led_queue_tail - led_queue_head + LED_QUEUE_SIZE) % LED_QUEUE_SIZE;

            if (queue_size >= (LED_QUEUE_SIZE - 1)) {
                ESP_LOGW("LED_QUEUE", "Queue full (%d/%d), dropping command for device %d",
                         queue_size, LED_QUEUE_SIZE - 1, deviceNumber);
                xSemaphoreGive(led_queue_mutex);
                return;
            }

            // Prepare command structure
            led_command_t *cmd = &led_command_queue[led_queue_tail];
            memset(cmd, 0, sizeof(led_command_t));

            cmd->deviceNumber = deviceNumber;
            cmd->value = value;
            cmd->dry_cmd_active_onoff_btn = dry_cmd_active_onoff_btn;
            cmd->is_btn_state_on = is_btn_state_on;
            cmd->device_keymode = device_keymode;
            cmd->dry_cmd_active_rotary_btn = dry_cmd_active_rotary_btn;
            cmd->pending = true;

            // === Build LED command ===
            if (device_keymode == 0) {  // Scene-based (keypad)
                uint16_t led_value_with_backlight = value;
                if (BACKLIGHT && !(led_value_with_backlight & (1 << 8))) {
                    led_value_with_backlight |= (1 << 8);
                }

                ESP_LOGI("LED_QUEUE", "Queueing LED command for keypad %d: value 0x%04X",
                         deviceNumber, led_value_with_backlight);

                cmd->led_cmd[0] = deviceNumber;
                cmd->led_cmd[1] = 0x06;
                cmd->led_cmd[2] = 0x10;
                cmd->led_cmd[3] = 0x08;
                cmd->led_cmd[4] = (led_value_with_backlight >> 8) & 0xFF;
                cmd->led_cmd[5] = led_value_with_backlight & 0xFF;

                uint16_t crc = modbus_crc16(cmd->led_cmd, 6);
                cmd->led_cmd[6] = crc & 0xFF;
                cmd->led_cmd[7] = (crc >> 8) & 0xFF;

                // === Build Dry Contact command (conditional) ===
                bool should_send_dry = (!boot_restoring) && dry_send_enabled;

                if (should_send_dry) {
                    cmd->dry_onoff_cmd[0] = deviceNumber;
                    cmd->dry_onoff_cmd[1] = 0x47;
                    cmd->dry_onoff_cmd[2] = 0x47;
                    cmd->dry_onoff_cmd[3] = dry_cmd_active_onoff_btn;   // valid 1–8
                    cmd->dry_onoff_cmd[4] = is_btn_state_on ? 1 : 0;    // 1 = ON, 0 = OFF
                    cmd->dry_onoff_cmd[5] = 0x47;

                    crc = modbus_crc16(cmd->dry_onoff_cmd, 6);
                    cmd->dry_onoff_cmd[6] = crc & 0xFF;
                    cmd->dry_onoff_cmd[7] = (crc >> 8) & 0xFF;

                    ESP_LOG_BUFFER_HEXDUMP("Queued Dry Command ->", cmd->dry_onoff_cmd, FRAME_SIZE, ESP_LOG_INFO);
                } else {
                    ESP_LOGI("LED_QUEUE", "Skipping Dry command (boot_restoring=%d, dry_send_enabled=%d)",
                             boot_restoring, dry_send_enabled);
                }

            } else if (device_keymode == 1) {  // Channel-based (rotary)
                ESP_LOGI("LED_QUEUE", "Queueing DIM command for Rotary %d: Button %d, value %d",
                         deviceNumber, dry_cmd_active_rotary_btn, value);

                cmd->led_cmd[0] = deviceNumber;
                cmd->led_cmd[1] = 0x03;
                cmd->led_cmd[2] = 0x01;
                cmd->led_cmd[3] = 0x10;
                cmd->led_cmd[4] = dry_cmd_active_rotary_btn;
                cmd->led_cmd[5] = (uint8_t)(2.55 * value);

                uint16_t crc = modbus_crc16(cmd->led_cmd, 6);
                cmd->led_cmd[6] = crc & 0xFF;
                cmd->led_cmd[7] = (crc >> 8) & 0xFF;
            }

            // Log LED command
            ESP_LOG_BUFFER_HEXDUMP("Queued LED Command ->", cmd->led_cmd, FRAME_SIZE, ESP_LOG_INFO);

            // Push to queue
            led_queue_tail = (led_queue_tail + 1) % LED_QUEUE_SIZE;
            led_command_ready = true;

            ESP_LOGI("LED_QUEUE", "Command queued. Queue size: %d/%d",
                     queue_size + 1, LED_QUEUE_SIZE - 1);

            xSemaphoreGive(led_queue_mutex);

            // Save LED state for persistence
            save_led_state_to_nvs(deviceNumber, value);
            return; // done
        } else {
            retry_count++;
            ESP_LOGW("LED_QUEUE", "Failed to acquire LED queue mutex (attempt %d/%d)",
                     retry_count, max_retries);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    ESP_LOGE("LED_QUEUE", "Failed to queue LED command for device %d after %d attempts",
             deviceNumber, max_retries);
}

// void queue_led_command_for_polling(uint8_t deviceNumber, uint16_t value) {
//     if (led_queue_mutex == NULL) {
//         ESP_LOGE("LED_QUEUE", "LED queue mutex not initialized");
//         return;
//     }
    
//     int retry_count = 0;
//     const int max_retries = 3;
    
//     while (retry_count < max_retries) {
//         if (xSemaphoreTake(led_queue_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
//             // Check current queue size
//             uint8_t queue_size = (led_queue_tail - led_queue_head + LED_QUEUE_SIZE) % LED_QUEUE_SIZE;
            
//             // Allow queue to hold up to LED_QUEUE_SIZE-1 commands (one slot always empty for detection)
//             if (queue_size >= (LED_QUEUE_SIZE - 1)) {
//                 ESP_LOGW("LED_QUEUE", "LED command queue full (%d/%d), dropping command for device %d", 
//                          queue_size, LED_QUEUE_SIZE - 1, deviceNumber);
//                 xSemaphoreGive(led_queue_mutex);
//                 return;
//             }
            
//             // Prepare the command with all necessary data
//             led_command_t *cmd = &led_command_queue[led_queue_tail];
//             memset(cmd, 0, sizeof(led_command_t));
            
//             cmd->deviceNumber = deviceNumber;
//             cmd->value = value;
//             cmd->dry_cmd_active_onoff_btn = dry_cmd_active_onoff_btn;
//             cmd->is_btn_state_on = is_btn_state_on;
//             cmd->device_keymode = device_keymode;
//             cmd->dry_cmd_active_rotary_btn = dry_cmd_active_rotary_btn;
//             cmd->pending = true;
            
//             // Build LED command frame
//             if (device_keymode == 0) { // Scene Based Device
//                 uint16_t led_value_with_backlight = value;
//                 if (BACKLIGHT && !(led_value_with_backlight & (1 << 8))) {
//                     led_value_with_backlight |= (1 << 8);
//                 }
                
//                 ESP_LOGI("LED_QUEUE", "Queueing LED command for keypad %d: value 0x%04X", deviceNumber, led_value_with_backlight);
                
//                 // Build LED command
//                 cmd->led_cmd[0] = deviceNumber;
//                 cmd->led_cmd[1] = 0x06;
//                 cmd->led_cmd[2] = 0x10;
//                 cmd->led_cmd[3] = 0x08;
//                 cmd->led_cmd[4] = (led_value_with_backlight >> 8) & 0xFF;
//                 cmd->led_cmd[5] = led_value_with_backlight & 0xFF;
                
//                 uint16_t crc = modbus_crc16(cmd->led_cmd, 6);
//                 cmd->led_cmd[6] = crc & 0xFF;
//                 cmd->led_cmd[7] = (crc >> 8) & 0xFF;
                
//                 // Build dry contact command
//                 cmd->dry_onoff_cmd[0] = deviceNumber;
//                 cmd->dry_onoff_cmd[1] = 0x47;
//                 cmd->dry_onoff_cmd[2] = 0x47;
//                 cmd->dry_onoff_cmd[3] = dry_cmd_active_onoff_btn;
//                 cmd->dry_onoff_cmd[4] = is_btn_state_on ? 1 : 0;
//                 cmd->dry_onoff_cmd[5] = 0x47;
                
//                 crc = modbus_crc16(cmd->dry_onoff_cmd, 6);
//                 cmd->dry_onoff_cmd[6] = crc & 0xFF;
//                 cmd->dry_onoff_cmd[7] = (crc >> 8) & 0xFF;
                
//             } else if (device_keymode == 1) { // Channel Based Device (rotary)
//                 ESP_LOGI("LED_QUEUE", "Queueing Dim command for Rotary %d: Button %d, value %d",
//                         deviceNumber, dry_cmd_active_rotary_btn, value);
                
//                 // Build LED command for rotary
//                 cmd->led_cmd[0] = deviceNumber;
//                 cmd->led_cmd[1] = 0x03;
//                 cmd->led_cmd[2] = 0x01;
//                 cmd->led_cmd[3] = 0x10;
//                 cmd->led_cmd[4] = dry_cmd_active_rotary_btn;
//                 cmd->led_cmd[5] = (uint8_t)(2.55 * value);
                
//                 uint16_t crc = modbus_crc16(cmd->led_cmd, 6);
//                 cmd->led_cmd[6] = crc & 0xFF;
//                 cmd->led_cmd[7] = (crc >> 8) & 0xFF;
//             }
            
//             ESP_LOG_BUFFER_HEXDUMP("Queued LED Command ->", cmd->led_cmd, FRAME_SIZE, ESP_LOG_INFO);
//             if (device_keymode == 0) {
//                 ESP_LOG_BUFFER_HEXDUMP("Queued Dry Command ->", cmd->dry_onoff_cmd, FRAME_SIZE, ESP_LOG_INFO);
//             }
            
//             // Add to queue
//             led_queue_tail = (led_queue_tail + 1) % LED_QUEUE_SIZE;
//             led_command_ready = true;
            
//             ESP_LOGI("LED_QUEUE", "Command queued successfully. Queue size: %d/%d", 
//                     queue_size + 1, LED_QUEUE_SIZE - 1);
            
//             xSemaphoreGive(led_queue_mutex);
            
//             // Save LED state to NVS for persistence
//             save_led_state_to_nvs(deviceNumber, value);
//             return; // Success
//         } else {
//             retry_count++;
//             ESP_LOGW("LED_QUEUE", "Failed to acquire LED queue mutex (attempt %d/%d)", retry_count, max_retries);
//             vTaskDelay(pdMS_TO_TICKS(10)); // Small delay before retry
//         }
//     }
    
//     ESP_LOGE("LED_QUEUE", "Failed to queue LED command for device %d after %d attempts", deviceNumber, max_retries);
// }

bool clean_devices_response(uint8_t *rx_data, int len, uint8_t *clean_buf, int org_len) {
    if (len < 5) return false;
    int start_index = 0;
    while (start_index < len && rx_data[start_index] == 0x00) start_index++;
    int remaining_bytes = len - start_index;
    if (remaining_bytes < org_len) return false;
    uint8_t *frame = &rx_data[start_index];
    memcpy(clean_buf, frame, org_len);
    uint16_t calc_crc = modbus_crc16(clean_buf, org_len - 2);
    uint16_t recv_crc = (clean_buf[org_len - 1] << 8) | clean_buf[org_len - 2];
    if (calc_crc != recv_crc) {
        ESP_LOGE("DEVICES", "Device CRC mismatch: calculated=0x%04X, received=0x%04X", calc_crc, recv_crc);
        return false;
    }
    return true;
}

bool clean_modbus_zones_response(uint8_t *rx_data, int len, uint8_t *clean_buf) {
    if (len < 5) return false;
    int start_index = 0;
    while (start_index < len && rx_data[start_index] == 0x00) start_index++;
    int remaining_bytes = len - start_index;
    if (remaining_bytes < 5) return false;
    uint8_t *frame = &rx_data[start_index];
    uint8_t byteCount = frame[2];
    int expected_len = 3 + byteCount + 2;
    if (remaining_bytes < expected_len) return false;
    memcpy(clean_buf, frame, expected_len);
    uint16_t calc_crc = modbus_crc16(clean_buf, expected_len - 2);
    uint16_t recv_crc = (clean_buf[expected_len - 1] << 8) | clean_buf[expected_len - 2];
    if (calc_crc != recv_crc) {
        ESP_LOGE("RS485_DATA", "ZONE CRC mismatch: calculated=0x%04X, received=0x%04X", calc_crc, recv_crc);
        return false;
    }
    return true;
}

bool clean_modbus_channels_response(uint8_t *rx_data, int len, uint8_t *clean_buf) {
    // identical to zones response parsing
    return clean_modbus_zones_response(rx_data, len, clean_buf);
}
void polling_response_task(void *arg) {
    if (polling_task != NULL && polling_task != xTaskGetCurrentTaskHandle()) {
        ESP_LOGW(TAG_RS485_DATA, "Keypad polling task already exists, deleting this instance");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_RS485_DATA, "Device polling task started (Keypads: %d, Rotaries: %d)", 
             keypads_count, rotaries_count);
    polling_task_ready = true;
    xSemaphoreGive(polling_semaphore);

    // Check if we have any devices to poll
    if (keypads_count == 0 && rotaries_count == 0) {
        ESP_LOGI(TAG_RS485_DATA, "No devices configured for polling");
        vTaskDelete(NULL);
    }

    if (keypad_frame_mutex == NULL) initialize_keypad_frame_management();

    vTaskDelay(pdMS_TO_TICKS(10));
    while (1) {
        //uart_flush(UART_NUM1);
        dynamic_keypad_polling_handler();
        // Dynamic delay - shorter if LED commands are pending
        if (led_command_ready) {
            vTaskDelay(pdMS_TO_TICKS(10));  // Short delay if commands pending
        } else {
            vTaskDelay(pdMS_TO_TICKS(300));  // Normal delay
        }
    }
}

void send_led_command_priority(uint8_t deviceNumber, uint16_t value) {
    UBaseType_t original_priority = uxTaskPriorityGet(NULL);
    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    queue_led_command_for_polling(deviceNumber, value);
    vTaskPrioritySet(NULL, original_priority);
}

// // Controller polling (zones and channels)
// static void dynamic_polling_handler(void) {
//     uint8_t rx_data[256];
//     uint8_t frame_to_send[FRAME_SIZE];

//     if (!take_mutex_with_timeout(modbus_mutex_uart2, "UART2", pdMS_TO_TICKS(100))) {
//         ESP_LOGW(TAG_RS485_DATA, "Failed to take UART2 mutex in dynamic_polling_handler");
//         return;
//     }

//     if (frame_mutex != NULL && xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
//         if (command_pending) {
//             memcpy(frame_to_send, current_frame, FRAME_SIZE);
//             command_pending = false;
//             ESP_LOGI(TAG_RS485_DATA, "Sending COMMAND frame");
//             if (zones_poll_eligible && poll_deviceType == 1) poll_option = 0;
//             else if(channels_poll_eligible) poll_option = 1;
//         } else {
//             // default zone poll
//             if (zones_poll_eligible && poll_option == 0) {
//                 frame_to_send[0] = default_slave_id;
//                 frame_to_send[1] = 0x03;
//                 frame_to_send[2] = 0x1B;
//                 frame_to_send[3] = 0x58;
//                 frame_to_send[4] = 0x00;
//                 frame_to_send[5] = 0x40;
//                 uint16_t crc = modbus_crc16(frame_to_send, 6);
//                 frame_to_send[6] = crc & 0xFF;
//                 frame_to_send[7] = (crc >> 8) & 0xFF;
//                 ESP_LOGI(TAG_RS485_DATA, "Sending ZONE POLLING frame");
//                 if(channels_poll_eligible) poll_option = 1;
//                 else poll_option = 0;
//             } 
//             if (channels_poll_eligible && poll_option == 1) {
//                 // channel polling - use first eligible frame if exists
//                 if (global_valid_count > 0 && global_valid_frames[0] != NULL) {
//                     uint8_t *frame_src = global_valid_frames[0];
//                     frame_to_send[0] = default_slave_id;
//                     frame_to_send[1] = 0x03;
//                     frame_to_send[2] = frame_src[2];
//                     frame_to_send[3] = frame_src[3];
//                     frame_to_send[4] = frame_src[4];
//                     frame_to_send[5] = frame_src[5];
//                     uint16_t crc = modbus_crc16(frame_to_send, 6);
//                     frame_to_send[6] = crc & 0xFF;
//                     frame_to_send[7] = (crc >> 8) & 0xFF;
//                     ESP_LOGI(TAG_RS485_DATA, "Sending CHANNEL-%d POLLING frame", eligible_channel_poll_values[0]);
//                     ch_range_ID = eligible_channel_poll_values[0];
//                 }
//                 if (global_valid_count > 1) poll_option = 2;
//                 else if (!zones_poll_eligible && global_valid_count == 1) poll_option = 1;
//                 else poll_option = 0;
//             } else if (poll_option == 2) {
//                 if (global_valid_count > 1 && global_valid_frames[1] != NULL) {
//                     uint8_t *frame_src = global_valid_frames[1];
//                     frame_to_send[0] = default_slave_id;
//                     frame_to_send[1] = 0x03;
//                     frame_to_send[2] = frame_src[2];
//                     frame_to_send[3] = frame_src[3];
//                     frame_to_send[4] = frame_src[4];
//                     frame_to_send[5] = frame_src[5];
//                     uint16_t crc = modbus_crc16(frame_to_send, 6);
//                     frame_to_send[6] = crc & 0xFF;
//                     frame_to_send[7] = (crc >> 8) & 0xFF;
//                     ESP_LOGI(TAG_RS485_DATA, "Sending CHANNEL-%d POLLING frame", eligible_channel_poll_values[1]);
//                     ch_range_ID = eligible_channel_poll_values[1];
//                 }
//                 if(zones_poll_eligible) poll_option = 0;
//                 else poll_option = 1;
//             }
//         }
//         xSemaphoreGive(frame_mutex);
//     } else {
//         // fallback if frame_mutex not acquired (send zone poll)
//         if(zones_poll_eligible && poll_option == 0){
//             frame_to_send[0] = default_slave_id;
//             frame_to_send[1] = 0x03;
//             frame_to_send[2] = 0x1B;
//             frame_to_send[3] = 0x58;
//             frame_to_send[4] = 0x00;
//             frame_to_send[5] = 0x40;
//             uint16_t crc = modbus_crc16(frame_to_send, 6);
//             frame_to_send[6] = crc & 0xFF;
//             frame_to_send[7] = (crc >> 8) & 0xFF;
//             ESP_LOGI(TAG_RS485_DATA, "Sending ZONE POLLING frame");
//             if(channels_poll_eligible) poll_option = 1;
//             else poll_option = 0;
//         }
//         if (channels_poll_eligible && poll_option == 1) {
//             // channel polling - use first eligible frame if exists
//             if (global_valid_count > 0 && global_valid_frames[0] != NULL) {
//                 uint8_t *frame_src = global_valid_frames[0];
//                 frame_to_send[0] = default_slave_id;
//                 frame_to_send[1] = 0x03;
//                 frame_to_send[2] = frame_src[2];
//                 frame_to_send[3] = frame_src[3];
//                 frame_to_send[4] = frame_src[4];
//                 frame_to_send[5] = frame_src[5];
//                 uint16_t crc = modbus_crc16(frame_to_send, 6);
//                 frame_to_send[6] = crc & 0xFF;
//                 frame_to_send[7] = (crc >> 8) & 0xFF;
//                 ESP_LOGI(TAG_RS485_DATA, "Sending CHANNEL-%d POLLING frame", eligible_channel_poll_values[0]);
//                 ch_range_ID = eligible_channel_poll_values[0];
//             }
//             if (global_valid_count > 1) poll_option = 2;
//             else if (!zones_poll_eligible && global_valid_count == 1) poll_option = 1;
//             else poll_option = 0;
//         } else if (poll_option == 2) {
//             if (global_valid_count > 1 && global_valid_frames[1] != NULL) {
//                 uint8_t *frame_src = global_valid_frames[1];
//                 frame_to_send[0] = default_slave_id;
//                 frame_to_send[1] = 0x03;
//                 frame_to_send[2] = frame_src[2];
//                 frame_to_send[3] = frame_src[3];
//                 frame_to_send[4] = frame_src[4];
//                 frame_to_send[5] = frame_src[5];
//                 uint16_t crc = modbus_crc16(frame_to_send, 6);
//                 frame_to_send[6] = crc & 0xFF;
//                 frame_to_send[7] = (crc >> 8) & 0xFF;
//                 ESP_LOGI(TAG_RS485_DATA, "Sending CHANNEL-%d POLLING frame", eligible_channel_poll_values[1]);
//                 ch_range_ID = eligible_channel_poll_values[1];
//             }
//             if(zones_poll_eligible) poll_option = 0;
//             else poll_option = 1;
//         }
//     }

//     RS485_WRITE_MODE2;
//     vTaskDelay(pdMS_TO_TICKS(10));
//     uart_write_bytes(UART_NUM2, (const char *)frame_to_send, FRAME_SIZE);
//     uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(10));
//     RS485_READ_MODE2;
//     vTaskDelay(pdMS_TO_TICKS(5));

//     int len = uart_read_bytes(UART_NUM2, rx_data, sizeof(rx_data), pdMS_TO_TICKS(200));
//     if (len > 0) {
//         ESP_LOGI("len_check", "len_check-%d", len);
//       //  for (int i = 0; i < len; i++) printf("%02X ", rx_data[i]);
//       //  printf("\n");

//         if (frame_to_send[1] == 0x03) {
//             uint8_t clean_data[300];
//             if (clean_modbus_zones_response(rx_data, len, clean_data) &&
//                 clean_data[1] == 0x03 && clean_data[2] == 0x80) {
//                 check_and_log_zones_changes(default_slave_id, clean_data);
//                 give_mutex(modbus_mutex_uart2, "UART2");
//                 return;
//             }
//             // else if (clean_modbus_channels_response(rx_data, len, clean_data) &&
//             //          clean_data[1] == 0x03 && (clean_data[2] == 0xFA || clean_data[2] == 0x18)) {
//             //     check_and_log_channels_changes(default_slave_id, clean_data, ch_range_ID);
//             // }
//             else if (clean_modbus_channels_response(rx_data, len, clean_data) &&
//                     clean_data[1] == 0x03) {

//                 int byteCount = clean_data[2];
//                 if (byteCount > 0 && byteCount <= 250) {  // Handle both 255-byte and short (53-byte) responses
//                     check_and_log_channels_changes(default_slave_id, clean_data, ch_range_ID);
//                 } else {
//                     ESP_LOGW("CHANNELS", "Unexpected byteCount %d for range %d", byteCount, ch_range_ID);
//                 }
//             }
//         }
//     }
//     give_mutex(modbus_mutex_uart2, "UART2");
//     kick_watchdog();
// }

static void dynamic_polling_handler(void) {
    uint8_t rx_data[256];
    uint8_t frame_to_send[FRAME_SIZE];

    if (!take_mutex_with_timeout(modbus_mutex_uart2, "UART2", pdMS_TO_TICKS(100))) {
        ESP_LOGW(TAG_RS485_DATA, "Failed to take UART2 mutex in dynamic_polling_handler");
        return;
    }

    if (frame_mutex != NULL && xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (command_pending) {
            memcpy(frame_to_send, current_frame, FRAME_SIZE);
            command_pending = false;
            ESP_LOGI(TAG_RS485_DATA, "Sending COMMAND frame");
            if (zones_poll_eligible && poll_deviceType == 1) poll_option = 0;
            else if(channels_poll_eligible) poll_option = 1;
        } else {
            // Regular polling logic
            if (zones_poll_eligible && poll_option == 0) {
                frame_to_send[0] = default_slave_id;
                frame_to_send[1] = 0x03;
                frame_to_send[2] = 0x1B;
                frame_to_send[3] = 0x58;
                frame_to_send[4] = 0x00;
                frame_to_send[5] = 0x40;
                uint16_t crc = modbus_crc16(frame_to_send, 6);
                frame_to_send[6] = crc & 0xFF;
                frame_to_send[7] = (crc >> 8) & 0xFF;
                //ESP_LOGI(TAG_RS485_DATA, "Sending ZONE POLLING frame");
                
                // Always cycle to channels next if available, otherwise stay with zones
                if (channels_poll_eligible) {
                    poll_option = 1;
                } else {
                    poll_option = 0; // Stay with zones if no channels
                }
            } 
            else if (channels_poll_eligible && poll_option == 1) {
                // channel polling - use first eligible frame if exists
                if (global_valid_count > 0 && global_valid_frames[0] != NULL) {
                    uint8_t *frame_src = global_valid_frames[0];
                    frame_to_send[0] = default_slave_id;
                    frame_to_send[1] = 0x03;
                    frame_to_send[2] = frame_src[2];
                    frame_to_send[3] = frame_src[3];
                    frame_to_send[4] = frame_src[4];
                    frame_to_send[5] = frame_src[5];
                    uint16_t crc = modbus_crc16(frame_to_send, 6);
                    frame_to_send[6] = crc & 0xFF;
                    frame_to_send[7] = (crc >> 8) & 0xFF;
                    ESP_LOGI(TAG_RS485_DATA, "Sending CHANNEL-%d POLLING frame", eligible_channel_poll_values[0]);
                    ch_range_ID = eligible_channel_poll_values[0];
                }
                
                // Cycle to next option: if we have multiple channels, go to second channel, otherwise back to zones
                if (global_valid_count > 1) {
                    poll_option = 2;
                } else {
                    // Only one channel range, go back to zones if available
                    poll_option = (zones_poll_eligible) ? 0 : 1;
                }
            } 
            else if (poll_option == 2) {
                if (global_valid_count > 1 && global_valid_frames[1] != NULL) {
                    uint8_t *frame_src = global_valid_frames[1];
                    frame_to_send[0] = default_slave_id;
                    frame_to_send[1] = 0x03;
                    frame_to_send[2] = frame_src[2];
                    frame_to_send[3] = frame_src[3];
                    frame_to_send[4] = frame_src[4];
                    frame_to_send[5] = frame_src[5];
                    uint16_t crc = modbus_crc16(frame_to_send, 6);
                    frame_to_send[6] = crc & 0xFF;
                    frame_to_send[7] = (crc >> 8) & 0xFF;
                    ESP_LOGI(TAG_RS485_DATA, "Sending CHANNEL-%d POLLING frame", eligible_channel_poll_values[1]);
                    ch_range_ID = eligible_channel_poll_values[1];
                }
                
                // After second channel, always go back to zones
                poll_option = 0;
            }
            else {
                // Fallback: if no valid poll_option, default to zones
                if (zones_poll_eligible) {
                    frame_to_send[0] = default_slave_id;
                    frame_to_send[1] = 0x03;
                    frame_to_send[2] = 0x1B;
                    frame_to_send[3] = 0x58;
                    frame_to_send[4] = 0x00;
                    frame_to_send[5] = 0x40;
                    uint16_t crc = modbus_crc16(frame_to_send, 6);
                    frame_to_send[6] = crc & 0xFF;
                    frame_to_send[7] = (crc >> 8) & 0xFF;
                    ESP_LOGI(TAG_RS485_DATA, "Sending ZONE POLLING frame (fallback)");
                    poll_option = (channels_poll_eligible) ? 1 : 0;
                } else if (channels_poll_eligible) {
                    // Only channels available, use first one
                    if (global_valid_count > 0 && global_valid_frames[0] != NULL) {
                        uint8_t *frame_src = global_valid_frames[0];
                        frame_to_send[0] = default_slave_id;
                        frame_to_send[1] = 0x03;
                        frame_to_send[2] = frame_src[2];
                        frame_to_send[3] = frame_src[3];
                        frame_to_send[4] = frame_src[4];
                        frame_to_send[5] = frame_src[5];
                        uint16_t crc = modbus_crc16(frame_to_send, 6);
                        frame_to_send[6] = crc & 0xFF;
                        frame_to_send[7] = (crc >> 8) & 0xFF;
                        ESP_LOGI(TAG_RS485_DATA, "Sending CHANNEL-%d POLLING frame (fallback)", eligible_channel_poll_values[0]);
                        ch_range_ID = eligible_channel_poll_values[0];
                    }
                    poll_option = (global_valid_count > 1) ? 2 : 1;
                }
            }
        }
        xSemaphoreGive(frame_mutex);
    } else {
        // Fallback if frame_mutex not acquired
        if (zones_poll_eligible && poll_option == 0) {
            frame_to_send[0] = default_slave_id;
            frame_to_send[1] = 0x03;
            frame_to_send[2] = 0x1B;
            frame_to_send[3] = 0x58;
            frame_to_send[4] = 0x00;
            frame_to_send[5] = 0x40;
            uint16_t crc = modbus_crc16(frame_to_send, 6);
            frame_to_send[6] = crc & 0xFF;
            frame_to_send[7] = (crc >> 8) & 0xFF;
            ESP_LOGI(TAG_RS485_DATA, "Sending ZONE POLLING frame (mutex fallback)");
            if (channels_poll_eligible) poll_option = 1;
        } else if (channels_poll_eligible && poll_option == 1) {
            if (global_valid_count > 0 && global_valid_frames[0] != NULL) {
                uint8_t *frame_src = global_valid_frames[0];
                frame_to_send[0] = default_slave_id;
                frame_to_send[1] = 0x03;
                frame_to_send[2] = frame_src[2];
                frame_to_send[3] = frame_src[3];
                frame_to_send[4] = frame_src[4];
                frame_to_send[5] = frame_src[5];
                uint16_t crc = modbus_crc16(frame_to_send, 6);
                frame_to_send[6] = crc & 0xFF;
                frame_to_send[7] = (crc >> 8) & 0xFF;
                ESP_LOGI(TAG_RS485_DATA, "Sending CHANNEL-%d POLLING frame (mutex fallback)", eligible_channel_poll_values[0]);
                ch_range_ID = eligible_channel_poll_values[0];
            }
            if (global_valid_count > 1) poll_option = 2;
            else poll_option = (zones_poll_eligible) ? 0 : 1;
        } else if (poll_option == 2) {
            if (global_valid_count > 1 && global_valid_frames[1] != NULL) {
                uint8_t *frame_src = global_valid_frames[1];
                frame_to_send[0] = default_slave_id;
                frame_to_send[1] = 0x03;
                frame_to_send[2] = frame_src[2];
                frame_to_send[3] = frame_src[3];
                frame_to_send[4] = frame_src[4];
                frame_to_send[5] = frame_src[5];
                uint16_t crc = modbus_crc16(frame_to_send, 6);
                frame_to_send[6] = crc & 0xFF;
                frame_to_send[7] = (crc >> 8) & 0xFF;
                ESP_LOGI(TAG_RS485_DATA, "Sending CHANNEL-%d POLLING frame (mutex fallback)", eligible_channel_poll_values[1]);
                ch_range_ID = eligible_channel_poll_values[1];
            }
            poll_option = 0;
        } else {
            // Reset to zones if poll_option is invalid
            if (zones_poll_eligible) {
                frame_to_send[0] = default_slave_id;
                frame_to_send[1] = 0x03;
                frame_to_send[2] = 0x1B;
                frame_to_send[3] = 0x58;
                frame_to_send[4] = 0x00;
                frame_to_send[5] = 0x40;
                uint16_t crc = modbus_crc16(frame_to_send, 6);
                frame_to_send[6] = crc & 0xFF;
                frame_to_send[7] = (crc >> 8) & 0xFF;
                ESP_LOGI(TAG_RS485_DATA, "Sending ZONE POLLING frame (reset fallback)");
                poll_option = (channels_poll_eligible) ? 1 : 0;
            }
        }
    }

    RS485_WRITE_MODE2;
    vTaskDelay(pdMS_TO_TICKS(10));
    uart_write_bytes(UART_NUM2, (const char *)frame_to_send, FRAME_SIZE);
    uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(10));
    
    // Debug print the frame being sent
   // ESP_LOGI(TAG_RS485_DATA, "Sent frame:");
    // for (int i = 0; i < FRAME_SIZE; i++) {
    //     printf("%02X=", frame_to_send[i]);
    // }
    // printf("\n");
    
    RS485_READ_MODE2;
    vTaskDelay(pdMS_TO_TICKS(5));

    int len = uart_read_bytes(UART_NUM2, rx_data, sizeof(rx_data), pdMS_TO_TICKS(200));
    if (len > 0) {
        ESP_LOGI("len_check", "len_check-%d", len);
        // for (int i = 0; i < len; i++) printf("%02X ", rx_data[i]);
        // printf("\n");

        if (frame_to_send[1] == 0x03) {
            uint8_t clean_data[300];
            if (clean_modbus_zones_response(rx_data, len, clean_data) &&
                clean_data[1] == 0x03 && clean_data[2] == 0x80) {
                check_and_log_zones_changes(default_slave_id, clean_data);
                give_mutex(modbus_mutex_uart2, "UART2");
                return;
            }
            else if (clean_modbus_channels_response(rx_data, len, clean_data) &&
                    clean_data[1] == 0x03) {

                int byteCount = clean_data[2];
                if (byteCount > 0 && byteCount <= 250) {  // Handle both 255-byte and short (53-byte) responses
                    check_and_log_channels_changes(default_slave_id, clean_data, ch_range_ID);
                } else {
                    ESP_LOGW("CHANNELS", "Unexpected byteCount %d for range %d", byteCount, ch_range_ID);
                }
            }
        }
    }
    give_mutex(modbus_mutex_uart2, "UART2");
    kick_watchdog();
}
void send_scene_to_controller(uint8_t slaveId, uint8_t zone, uint8_t scene_no) {
    if (frame_mutex == NULL) {
        ESP_LOGE(TAG_BTN, "Frame mutex not initialized");
        return;
    }
    
    if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Prepare the command frame
        uint8_t scene_to_trigger = scene_no == 0 ? 0 : scene_no;
        uint16_t decimal_value = ZONE_WRITE_REG_VAL + zone;
        
        current_frame[0] = slaveId;
        current_frame[1] = 0x06;
        current_frame[2] = (decimal_value >> 8) & 0xFF;
        current_frame[3] = decimal_value & 0xFF;
        current_frame[4] = 0x00;
        current_frame[5] = scene_to_trigger;
        
        uint16_t crc = modbus_crc16(current_frame, 6);
        current_frame[6] = crc & 0xFF;
        current_frame[7] = (crc >> 8) & 0xFF;
        
        // Set command pending flag
        command_pending = true;
        
        ESP_LOGI(TAG_RS485_DATA, "Command prepared: Scene %d to Zone %d, ID %d", 
                 scene_to_trigger, zone, slaveId);
        
        xSemaphoreGive(frame_mutex);
    } else {
        ESP_LOGE(TAG_BTN, "Failed to acquire frame mutex for scene command");
    }
}

// void send_channel_to_controller(uint8_t slaveId, uint16_t channel, uint16_t value) {
//     uint8_t frame[FRAME_SIZE] = {0};
//     frame[0] = slaveId;
//     frame[1] = 0x06;
//     frame[2] = (channel >> 8) & 0xFF;
//     frame[3] = channel & 0xFF;
//     frame[4] = (value >> 8) & 0xFF;
//     frame[5] = value & 0xFF;

//     uint16_t crc = modbus_crc16(frame, 6);
//     frame[6] = crc & 0xFF;
//     frame[7] = (crc >> 8) & 0xFF;

//     if (!take_mutex_with_timeout(modbus_mutex_uart2, "UART2_CHANNEL", pdMS_TO_TICKS(100))) {
//         ESP_LOGE("CHANNEL", "Failed to acquire UART2 mutex for channel command");
//         return;
//     }

//     RS485_WRITE_MODE2;
//     vTaskDelay(pdMS_TO_TICKS(10));
//     ESP_LOG_BUFFER_HEXDUMP("Channel Command ->", frame, FRAME_SIZE, ESP_LOG_INFO);
//     uart_write_bytes(UART_NUM2, (const char *)frame, FRAME_SIZE);
//     uart_wait_tx_done(UART_NUM2, pdMS_TO_TICKS(50));
//     give_mutex(modbus_mutex_uart2, "UART2_CHANNEL");
//     kick_watchdog();
// }

void send_channel_to_controller(uint8_t slaveId, uint16_t channel_no, uint8_t ch_val) {
    if (frame_mutex == NULL) {
        ESP_LOGE(TAG_BTN, "Frame mutex not initialized");
        return;
    }
    
    if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Prepare the command frame
        uint16_t decimal_value = CH_WRITE_REG_VAL + channel_no - 1;
        // 0x01 0x06 0x00 0x00 0x00 0xFE 0x08 0x4A->dim_value:100
        // Formula: (int)(2.55*value)
        current_frame[0] = slaveId;
        current_frame[1] = 0x06;
        current_frame[2] = (decimal_value >> 8) & 0xFF;
        current_frame[3] = decimal_value & 0xFF;
        current_frame[4] = 0x00;
        current_frame[5] = ch_val;
        
        uint16_t crc = modbus_crc16(current_frame, 6);
        current_frame[6] = crc & 0xFF;
        current_frame[7] = (crc >> 8) & 0xFF;
        
        // Set command pending flag
        command_pending = true;
        ESP_LOGI(TAG_RS485_DATA, "Command prepared: Channel %d - Percent %d -%02X to ID %d", 
                 channel_no, ch_val, ch_val, slaveId);
        /*for(int i=0;i<FRAME_SIZE;i++)
        {
            printf("%02X ", current_frame[i]);
        }*/
        
        xSemaphoreGive(frame_mutex);
    } else {
        ESP_LOGE(TAG_BTN, "Failed to acquire frame mutex for scene command");
    }
}

void polling_response_task_controller(void *arg) {
    if (zone_polling_task != NULL && zone_polling_task != xTaskGetCurrentTaskHandle()) {
        ESP_LOGW(TAG_RS485_DATA, "Zone polling task already exists, deleting this instance");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_RS485_DATA, "Dynamic polling task started");
    if (frame_mutex == NULL) initialize_frame_management();
    if (keypads_count == 0 && rotaries_count == 0) {
        ESP_LOGI(TAG_RS485_DATA, "No devices configured for polling");
        vTaskDelete(NULL);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
    uart_flush(UART_NUM2);
    while (1) {
        dynamic_polling_handler();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ========== LED helpers ==========
void send_led_group_on(uint8_t keypadNumber, const uint8_t* leds, uint8_t count, uint8_t backlight_on) {
    int keypad_index = -1;
    for (int i = 0; i < keypads_count; i++) {
        if (keypad_configs[i].keypadNumber == keypadNumber) { keypad_index = i; break; }
    }
    if (keypad_index < 0) { ESP_LOGE("LED", "Keypad %d not found", keypadNumber); return; }

    uint16_t *state = &led_states[keypad_index];
    for (uint8_t i = 0; i < count; i++) {
        if (leds[i] >= 1 && leds[i] <= 8) *state |= (1 << (leds[i] - 1));
    }

    // ✅ Always keep backlight ON if BACKLIGHT enabled
    if (BACKLIGHT) *state |= (1 << 8);
    else if (backlight_on) *state |= (1 << 8);
    else *state &= ~(1 << 8);

    ESP_LOGI("LED", "New LED state for keypad %d: 0x%04X", keypadNumber, *state);
}

void send_led_group_off(uint8_t keypadNumber, const uint8_t* leds, uint8_t count, uint8_t backlight_on) {
    int keypad_index = -1;
    for (int i = 0; i < keypads_count; i++) {
        if (keypad_configs[i].keypadNumber == keypadNumber) { keypad_index = i; break; }
    }
    if (keypad_index < 0) { ESP_LOGE("LED", "Keypad %d not found", keypadNumber); return; }

    uint16_t *state = &led_states[keypad_index];
    for (uint8_t i = 0; i < count; i++) {
        if (leds[i] >= 1 && leds[i] <= 8) *state &= ~(1 << (leds[i] - 1));
    }

    // ✅ Preserve or force backlight if BACKLIGHT defined
    if (BACKLIGHT) *state |= (1 << 8);
    else if (backlight_on) *state |= (1 << 8);
    else *state &= ~(1 << 8);

    ESP_LOGI("LED", "New LED state for keypad %d: 0x%04X", keypadNumber, *state);
}


void initialize_led_states() {
    boot_restoring = true;   // block dry-contact during LED restore
    for (int i = 0; i < keypads_count; i++) {
        // Try to load LED state from dedicated NVS storage first
        uint16_t saved_led_state = get_led_state_from_nvs_enhanced(keypad_configs[i].keypadNumber);
        
        if (saved_led_state != 0) {
            led_states[i] = saved_led_state;
            ESP_LOGI("LED_INIT", "Restored LED state for keypad %d: 0x%04X", 
                     keypad_configs[i].keypadNumber, saved_led_state);
        } else {
            // Default initialization
            led_states[i] = 0x0000;
            if (BACKLIGHT) led_states[i] |= (1 << 8);
            ESP_LOGI("LED_INIT", "Initialized default LED state for keypad %d: 0x%04X", 
                     keypad_configs[i].keypadNumber, led_states[i]);
        }
    }
    boot_restoring = false;  // LED restore done
}


void update_keypad_leds_for_scene(uint8_t keypadNumber, uint8_t scene_id) {
    ESP_LOGI("LED_UPDATE", "Updating LEDs for keypad %d, scene %d", keypadNumber, scene_id);
    keypad_config_t *cfg = NULL;
    int keypad_index = -1;
    for (int i = 0; i < keypads_count; i++) {
        if (keypad_configs[i].keypadNumber == keypadNumber) { cfg = &keypad_configs[i]; keypad_index = i; break; }
    }
    if (cfg == NULL) { ESP_LOGE("LED_UPDATE", "Keypad %d not found", keypadNumber); return; }

    if (scene_id == 0x00) {
        uint8_t all_leds[] = {1,2,3,4,5,6,7,8};
        send_led_group_off(keypadNumber, all_leds, 8, BACKLIGHT);
        if (keypad_index >= 0) {
            led_states[keypad_index] = 0x0000;
            if (BACKLIGHT) led_states[keypad_index] |= (1 << 8);
        }
    } else {
        uint8_t button_to_turn_on = 0;
        for (int btn = 0; btn < NUM_BUTTONS; btn++) {
            if (cfg->keyOnScene[btn] == scene_id) { button_to_turn_on = btn + 1; break; }
        }
        if (button_to_turn_on > 0 && keypad_index >= 0) {
            led_states[keypad_index] |= (1 << (button_to_turn_on - 1));
            if (BACKLIGHT) led_states[keypad_index] |= (1 << 8);
            send_led_command_priority(keypadNumber, led_states[keypad_index]);
        }
    }
}
void check_and_log_channels_changes(uint8_t slaveId, uint8_t *response, uint16_t channel_start_no) {
    if (response[1] != 0x03) {
        ESP_LOGW("CHANNELS", "Invalid function code: %02X", response[1]);
        return;
    }

    int byteCount = response[2];
    int channels_in_response = byteCount / 2;
    if (channels_in_response <= 0) {
        ESP_LOGW("CHANNELS", "Invalid channel count (%d)", channels_in_response);
        return;
    }

    // ✅ Find proper start channel from range IDs
    uint16_t start_channel = 1;
    for (int i = 0; i < sizeof(channel_poll_range_ids)/sizeof(channel_poll_range_ids[0]); i++) {
        if (channel_poll_range_ids[i] == channel_start_no) {
            if (i == 0)
                start_channel = 1;
            else
                start_channel = channel_poll_range_ids[i - 1] + 1;
            break;
        }
    }

    uint16_t end_channel = start_channel + channels_in_response - 1;

    ESP_LOGI("CHANNELS", "=== Processing Channels %d–%d (Range ID: %d) ===", 
             start_channel, end_channel, channel_start_no);

    for (int i = 0; i < channels_in_response; i++) {
        int offset = 3 + (i * 2);
        if (offset + 1 >= byteCount + 3) break;

        uint16_t ch_value = (response[offset] << 8) | response[offset + 1];
        uint16_t channel_no = start_channel + i;

        if (ch_value == 0x0100) continue;

        uint16_t store_index = (channel_no <= 250) ? channel_no : (channel_no - 250);
        if (ch_value == last_channel_values[slaveId][store_index]) continue;

        last_channel_values[slaveId][store_index] = ch_value;
        uint8_t percentage = (uint8_t)round(ch_value / 2.55);

        bool mapped = false;
        for (int r = 0; r < rotaries_count; r++) {
            rotary_config_t *rc = &rotary_configs[r];
            for (int k = 0; k < 3; k++) {
                if (rc->keyValue[k] == channel_no) {
                    mapped = true;
                    ESP_LOGI("CHANNEL_VALUES", "Channel %d: %d%% → Rotary %d Key %d", 
                             channel_no, percentage, rc->rotaryNumber, k + 1);
                    device_keymode = 1;
                    dry_cmd_active_rotary_btn = k + 1;
                    send_led_command_priority(rc->rotaryNumber, percentage);
                    //break;
                }
            }
           // if (mapped) break;
        }

        if (!mapped) {
            ESP_LOGI("CHANNEL_VALUES", "Channel %d: %d%% (No rotary mapping)", 
                     channel_no, percentage);
        }
    }

    ESP_LOGI("CHANNELS", "=== Completed processing range %d ===\n", channel_start_no);
}


// void check_and_log_channels_changes(uint8_t slaveId, uint8_t *response, uint16_t channel_start_no) {
//     if (response[1] != 0x03) {
//         ESP_LOGW("CHANNELS", "Invalid function code: %02X", response[1]);
//         return;
//     }
//     int byteCount = response[2];
//     int channels_in_response = byteCount / 2;
//     if (channels_in_response <= 0 || channels_in_response > MAX_CHANNELS_PER_RANGE) return;

//     uint16_t start_channel = (channel_start_no - 125) + 1;
    
//     ESP_LOGI("CHANNELS", "=== Processing Channels %d-%d (Range ID: %d) ===", 
//              start_channel, start_channel + channels_in_response - 1, channel_start_no);
    
//     // First, log all rotary mappings for channels in this range
//     bool found_mappings = false;
//     for (int i = 0; i < channels_in_response; i++) {
//         uint16_t channel_no = start_channel + i;
        
//         // Check if any rotary has this channel mapped
//         for (int r = 0; r < rotaries_count; r++) {
//             rotary_config_t *rc = &rotary_configs[r];
//             for (int k = 0; k < 3; k++) {
//                 if (rc->keyValue[k] == channel_no) {
//                     if (!found_mappings) {
//                         ESP_LOGI("CHANNEL_MAPPING", "Rotary mappings in range %d:", channel_start_no);
//                         found_mappings = true;
//                     }
//                     ESP_LOGI("CHANNEL_MAPPING", "  Channel %d → Rotary %d Key %d", 
//                             channel_no, rc->rotaryNumber, k + 1);
//                 }
//             }
//         }
//     }
    
//     if (!found_mappings) {
//         ESP_LOGI("CHANNEL_MAPPING", "No rotary mappings found in range %d", channel_start_no);
//     }

//     // Then process the channel values
//     ESP_LOGI("CHANNEL_VALUES", "Channel values in range %d:", channel_start_no);
//     for (int i = 0; i < channels_in_response; i++) {
//         int offset = 3 + (i * 2);
//         uint16_t ch_value = (response[offset] << 8) | response[offset + 1];
//         uint16_t channel_no = start_channel + i;
//         if (ch_value == 0x100) continue;
        
//         uint16_t store_index = (channel_no <= 250) ? channel_no : (channel_no - 250);
//         if (ch_value == last_channel_values[slaveId][store_index]) {
//             // Value unchanged, but still log if it has rotary mapping
//             for (int r = 0; r < rotaries_count; r++) {
//                 rotary_config_t *rc = &rotary_configs[r];
//                 for (int k = 0; k < 3; k++) {
//                     if (rc->keyValue[k] == channel_no) {
//                         uint8_t percentage = (uint8_t)round(ch_value / 2.55);
//                         ESP_LOGI("CHANNEL_VALUES", "  Channel %d: %d%% (Unchanged) → Rotary %d Key %d", 
//                                 channel_no, percentage, rc->rotaryNumber, k + 1);
//                         break;
//                     }
//                 }
//             }
//             continue;
//         }
        
//         last_channel_values[slaveId][store_index] = ch_value;
//         uint8_t percentage = (uint8_t)round(ch_value / 2.55);
        
//         // Log the channel value and any associated rotary mappings
//         bool has_rotary_mapping = false;
//         for (int r = 0; r < rotaries_count; r++) {
//             rotary_config_t *rc = &rotary_configs[r];
//             for (int k = 0; k < 3; k++) {
//                 if (rc->keyValue[k] == channel_no) {
//                     ESP_LOGI("CHANNEL_VALUES", "  Channel %d: %d%% (Changed) → Rotary %d Key %d", 
//                             channel_no, percentage, rc->rotaryNumber, k + 1);
//                     has_rotary_mapping = true;
                    
//                     // Log the actual value change
//                     uint16_t prev_value = 0;
//                     if (last_channel_values[slaveId][store_index] != ch_value) {
//                         // Find previous value for comparison
//                         for (int prev_slave = 0; prev_slave <= MAX_SLAVES; prev_slave++) {
//                             if (last_channel_values[prev_slave][store_index] != 0) {
//                                 prev_value = last_channel_values[prev_slave][store_index];
//                                 break;
//                             }
//                         }
//                         uint8_t prev_percentage = (uint8_t)round(prev_value / 2.55);
//                         ESP_LOGI("CHANNEL_CHANGE", "    Rotary %d Key %d: %d%% → %d%%", 
//                                 rc->rotaryNumber, k + 1, prev_percentage, percentage);
//                         device_keymode = 1;
//                         dry_cmd_active_rotary_btn = k + 1;
//                         send_led_command_priority(rc->rotaryNumber, percentage);
//                     }
//                     break;
//                 }
//             }
//         }
        
//         if (!has_rotary_mapping) {
//             ESP_LOGI("CHANNEL_VALUES", "  Channel %d: %d%% (Changed) → No rotary mapping", 
//                     channel_no, percentage);
//         }
        
//         ESP_LOGI("CHANNELS", "Slave %d - Channel %d - (0x%04X, %d%%)", 
//                 slaveId, channel_no, ch_value, percentage);
//     }
    
//     ESP_LOGI("CHANNELS", "=== Completed processing range %d ===\n", channel_start_no);
// }

// void check_and_log_zones_changes(uint8_t slaveId, uint8_t *response) {
//     if (response[1] != 0x03) {
//         ESP_LOGW("ZONES", "Invalid function code: %02X", response[1]);
//         return;
//     }
//     int byteCount = response[2];
//     int zones_in_response = byteCount / 2;
//     struct { uint16_t bitmask; uint8_t scene_id; } scene_mapping[] = {
//         {0x0001, 0x00}, {0x0002, 0x01}, {0x0004, 0x02}, {0x0008, 0x03},
//         {0x0010, 0x04}, {0x0020, 0x05}, {0x0040, 0x06}, {0x0080, 0x07},
//         {0x0100, 0x08}, {0x0200, 0x09}, {0x0400, 0x0A}, {0x0800, 0x0B},
//         {0x1000, 0x0C}, {0x2000, 0x0D}, {0x4000, 0x0E}, {0x8000, 0x0F}
//     };

//     for (int z = 0; z < zones_in_response; z++) {
//         int offset = 3 + (z * 2);
//         uint16_t scene_value = (response[offset] << 8) | response[offset + 1];
//         int zoneNumber = z + 1;
//         if (slaveId > MAX_SLAVES || zoneNumber > MAX_ZONES) continue;
//         if (scene_value == last_scene_values[slaveId][zoneNumber]) continue;
//         last_scene_values[slaveId][zoneNumber] = scene_value;

//         uint8_t scene_id = 0xFF;
//         for (int i = 0; i < 16; i++) {
//             if (scene_value & scene_mapping[i].bitmask) {
//                 scene_id = scene_mapping[i].scene_id; 
//                 break;
//             }
//         }
//         if (scene_id == 0xFF) continue;

//         ESP_LOGI("ZONES", "=== Slave %d Zone %d → Scene %d ===", slaveId, zoneNumber, scene_id);
//         device_keymode = 0;
        
//         if (scene_id == 0x00) {
//             // Scene 0 - Turn OFF all LEDs for all keypads in this zone
//             ESP_LOGI("SCENE_0", "Scene 0 detected - turning OFF all LEDs for keypads in zone %d", zoneNumber);
            
//             // First, log all keypads that will be affected
//             ESP_LOGI("SCENE_MAPPING", "Keypads in Zone %d that will be turned OFF:", zoneNumber);
//             for (int k = 0; k < keypads_count; k++) {
//                 keypad_config_t *cfg = &keypad_configs[k];
//                 if (cfg->slaveId == slaveId && cfg->zoneNumber == zoneNumber) {
//                     ESP_LOGI("SCENE_MAPPING", "  - Keypad %d (All LEDs OFF)", cfg->keypadNumber);
//                 }
//             }
            
//             // === SET GLOBAL FLAGS FOR SECONDARY COMMAND ===
//             is_btn_state_on = false;
//             dry_cmd_active_onoff_btn = 0xFF;
//             device_keymode = 0; // Scene based device
            
//             // Then perform the actual OFF action with proper queue management
//             for (int k = 0; k < keypads_count; k++) {
//                 keypad_config_t *cfg = &keypad_configs[k];
//                 if (cfg->slaveId == slaveId && cfg->zoneNumber == zoneNumber) {
//                     uint16_t original_state = led_states[k];
//                     uint16_t new_led_state = 0x0000;
                    
//                     // === FIX: Preserve backlight when sending Scene 0 ===
//                     if (BACKLIGHT) {
//                         new_led_state |= (1 << 8);
//                         ESP_LOGI("SCENE_ACTION", "Keypad %d: Backlight preserved (BACKLIGHT defined)", cfg->keypadNumber);
//                     }
                    
//                     if (new_led_state != original_state) {
//                         led_states[k] = new_led_state;
//                         ESP_LOGI("SCENE_ACTION", "Keypad %d: Turning OFF ALL LEDs (was 0x%04X, now 0x%04X)", 
//                                 cfg->keypadNumber, original_state, new_led_state);
                        
//                         // Queue LED command for this keypad
//                         ESP_LOGI("SCENE_ACTION", "Queueing LED command for keypad %d", cfg->keypadNumber);
//                         queue_led_command_for_polling(cfg->keypadNumber, new_led_state);
                        
//                         // Save LED state to NVS
//                         save_led_state_to_nvs(cfg->keypadNumber, new_led_state);
                        
//                         // Small delay to prevent queue overflow and allow processing
//                         vTaskDelay(pdMS_TO_TICKS(50));
                        
//                     } else {
//                         ESP_LOGI("SCENE_ACTION", "Keypad %d: No LED state change needed (already 0x%04X)", 
//                                 cfg->keypadNumber, original_state);
                        
//                         // Still send the command to ensure synchronization
//                         ESP_LOGI("SCENE_ACTION", "Queueing sync LED command for keypad %d", cfg->keypadNumber);
//                         queue_led_command_for_polling(cfg->keypadNumber, new_led_state);
//                         vTaskDelay(pdMS_TO_TICKS(30));
//                     }
//                 }
//             }
            
//             ESP_LOGI("SCENE_0", "All keypads in zone %d processed for Scene 0", zoneNumber);
//         }               
//         else {
//             // Handle scenes 1-15
//             // First, log all scene mappings for this zone
//             ESP_LOGI("SCENE_MAPPING", "Scene %d mappings in Zone %d:", scene_id, zoneNumber);
            
//             // REMOVE flag setting from mapping loop - just do logging here
//             for (int k = 0; k < keypads_count; k++) {
//                 keypad_config_t *cfg = &keypad_configs[k];
//                 if (cfg->slaveId == slaveId && cfg->zoneNumber == zoneNumber) {
//                     bool has_on_scene = false;
//                     bool has_off_scene = false;
                    
//                     // Check for ON scenes (LOG ONLY - no flag setting)
//                     for (int btn = 0; btn < NUM_BUTTONS; btn++) {
//                         if (cfg->keyOnScene[btn] == scene_id) {
//                             if (!has_on_scene) {
//                                 ESP_LOGI("SCENE_MAPPING", "  - Keypad %d:", cfg->keypadNumber);
//                                 has_on_scene = true;
//                             }
//                             ESP_LOGI("SCENE_MAPPING", "      Button %d → ON (Scene %d)", btn + 1, scene_id);
//                         }
//                     }
                    
//                     // Check for OFF scenes (LOG ONLY - no flag setting)
//                     for (int btn = 0; btn < NUM_BUTTONS; btn++) {
//                         if (cfg->keyOffScene[btn] == scene_id) {
//                             if (!has_off_scene && !has_on_scene) {
//                                 ESP_LOGI("SCENE_MAPPING", "  - Keypad %d:", cfg->keypadNumber);
//                                 has_off_scene = true;
//                             }
//                             ESP_LOGI("SCENE_MAPPING", "      Button %d → OFF (Scene %d)", btn + 1, scene_id);
//                         }
//                     }
                    
//                     if (!has_on_scene && !has_off_scene) {
//                         ESP_LOGI("SCENE_MAPPING", "  - Keypad %d: No scene mapping", cfg->keypadNumber);
//                     }
//                 }
//             }
            
//             // Then perform the actual scene actions
//             ESP_LOGI("SCENE_ACTION", "Executing Scene %d actions:", scene_id);
//             for (int k = 0; k < keypads_count; k++) {
//                 keypad_config_t *cfg = &keypad_configs[k];
//                 if (cfg->slaveId == slaveId && cfg->zoneNumber == zoneNumber) {
//                     bool led_state_changed = false;
//                     uint16_t new_led_state = led_states[k];
//                     uint16_t original_state = led_states[k];
                    
//                     // Process ON scenes first
//                     for (int btn = 0; btn < NUM_BUTTONS; btn++) {
//                         if (cfg->keyOnScene[btn] == scene_id) {
//                             uint16_t mask = (1 << btn);
//                             if (!(new_led_state & mask)) {
//                                 new_led_state |= mask;
//                                 ESP_LOGI("SCENE_ACTION", "Keypad %d: Button %d → LED ON (Scene %d)", 
//                                         cfg->keypadNumber, btn + 1, scene_id);
//                                 led_state_changed = true;
//                                 // SET FLAGS HERE for this specific keypad
//                                 is_btn_state_on = true;
//                                 dry_cmd_active_onoff_btn = btn + 1;
//                             } else {
//                                 ESP_LOGI("SCENE_ACTION", "Keypad %d: Button %d → LED already ON (Scene %d)", 
//                                         cfg->keypadNumber, btn + 1, scene_id);
//                                 // Still set flags even if no change
//                                 is_btn_state_on = true;
//                                 dry_cmd_active_onoff_btn = btn + 1;
//                             }
//                         }
//                     }
                    
//                     // Process OFF scenes second (overrides ON if same button has both)
//                     for (int btn = 0; btn < NUM_BUTTONS; btn++) {
//                         if (cfg->keyOffScene[btn] == scene_id) {
//                             uint16_t mask = (1 << btn);
//                             if (new_led_state & mask) {
//                                 new_led_state &= ~mask;
//                                 ESP_LOGI("SCENE_ACTION", "Keypad %d: Button %d → LED OFF (Scene %d)", 
//                                         cfg->keypadNumber, btn + 1, scene_id);
//                                 led_state_changed = true;
//                                 // SET FLAGS HERE for this specific keypad
//                                 is_btn_state_on = false;
//                                 dry_cmd_active_onoff_btn = btn + 1;
//                             } else {
//                                 ESP_LOGI("SCENE_ACTION", "Keypad %d: Button %d → LED already OFF (Scene %d)", 
//                                         cfg->keypadNumber, btn + 1, scene_id);
//                                 // Still set flags even if no change
//                                 is_btn_state_on = false;
//                                 dry_cmd_active_onoff_btn = btn + 1;
//                             }
//                         }
//                     }
                    
//                     // Handle backlight
//                     uint16_t backlight_mask = (1 << 8);
//                     if (BACKLIGHT) {
//                         if (!(new_led_state & backlight_mask)) {
//                             new_led_state |= backlight_mask;
//                             if (!led_state_changed) {
//                                 ESP_LOGI("SCENE_ACTION", "Keypad %d: Backlight ON", cfg->keypadNumber);
//                             }
//                             led_state_changed = true;
//                         }
//                     } else {
//                         if (new_led_state & backlight_mask) {
//                             new_led_state &= ~backlight_mask;
//                             if (!led_state_changed) {
//                                 ESP_LOGI("SCENE_ACTION", "Keypad %d: Backlight OFF", cfg->keypadNumber);
//                             }
//                             led_state_changed = true;
//                         }
//                     }
                    
//                     // Update if state changed
//                     if (led_state_changed || new_led_state != original_state) {
//                         led_states[k] = new_led_state;
                        
//                         ESP_LOGI("LED_STATE", "Keypad %d: LED state changed 0x%04X → 0x%04X", 
//                                 cfg->keypadNumber, original_state, new_led_state);
                        
//                         // Queue LED command with delay to prevent queue overflow
//                         queue_led_command_for_polling(cfg->keypadNumber, new_led_state);
//                         save_led_state_to_nvs(cfg->keypadNumber, new_led_state);
                        
//                         // Small delay to allow queue processing
//                         vTaskDelay(pdMS_TO_TICKS(30));
//                     } else {
//                         ESP_LOGI("SCENE_ACTION", "Keypad %d: No LED state change needed", cfg->keypadNumber);
//                     }
//                 }
//             }
//         }
//         ESP_LOGI("ZONES", "=== Scene %d processing complete ===\n", scene_id);
//     }
// }
void check_and_log_zones_changes(uint8_t slaveId, uint8_t *response) {
    if (response[1] != 0x03) {
        ESP_LOGW("ZONES", "Invalid function code: %02X", response[1]);
        return;
    }
    int byteCount = response[2];
    int zones_in_response = byteCount / 2;
    struct { uint16_t bitmask; uint8_t scene_id; } scene_mapping[] = {
        {0x0001, 0x00}, {0x0002, 0x01}, {0x0004, 0x02}, {0x0008, 0x03},
        {0x0010, 0x04}, {0x0020, 0x05}, {0x0040, 0x06}, {0x0080, 0x07},
        {0x0100, 0x08}, {0x0200, 0x09}, {0x0400, 0x0A}, {0x0800, 0x0B},
        {0x1000, 0x0C}, {0x2000, 0x0D}, {0x4000, 0x0E}, {0x8000, 0x0F}
    };

    for (int z = 0; z < zones_in_response; z++) {
        int offset = 3 + (z * 2);
        uint16_t scene_value = (response[offset] << 8) | response[offset + 1];
        int zoneNumber = z + 1;
        if (slaveId > MAX_SLAVES || zoneNumber > MAX_ZONES) continue;
        if (scene_value == last_scene_values[slaveId][zoneNumber]) continue;
        last_scene_values[slaveId][zoneNumber] = scene_value;

        uint8_t scene_id = 0xFF;
        for (int i = 0; i < 16; i++) {
            if (scene_value & scene_mapping[i].bitmask) {
                scene_id = scene_mapping[i].scene_id; 
                break;
            }
        }
        if (scene_id == 0xFF) continue;

        ESP_LOGI("ZONES", "=== Slave %d Zone %d → Scene %d ===", slaveId, zoneNumber, scene_id);
        device_keymode = 0;
        
        if (scene_id == 0x00) {
            // Scene 0 - Turn OFF all LEDs for all keypads in this zone (OVERRIDE LOCKOUT)
            ESP_LOGI("SCENE_0", "Scene 0 detected - turning OFF all LEDs for keypads in zone %d (OVERRIDING LOCKOUT)", zoneNumber);
            
            // First, log all keypads that will be affected
            ESP_LOGI("SCENE_MAPPING", "Keypads in Zone %d that will be turned OFF:", zoneNumber);
            for (int k = 0; k < keypads_count; k++) {
                keypad_config_t *cfg = &keypad_configs[k];
                if (cfg->slaveId == slaveId && cfg->zoneNumber == zoneNumber) {
                    ESP_LOGI("SCENE_MAPPING", "  - Keypad %d (All LEDs OFF)", cfg->keypadNumber);
                }
            }
            
            // === SET GLOBAL FLAGS FOR SECONDARY COMMAND ===
            is_btn_state_on = false;
            dry_cmd_active_onoff_btn = 0xFF;
            device_keymode = 0; // Scene based device
            
            // Then perform the actual OFF action - IGNORE LOCKOUT FOR SCENE 0
            for (int k = 0; k < keypads_count; k++) {
                keypad_config_t *cfg = &keypad_configs[k];
                if (cfg->slaveId == slaveId && cfg->zoneNumber == zoneNumber) {
                    
                    // === SCENE 0 OVERRIDE: DON'T CHECK LOCKOUT ===
                    // For Scene 0, always process ALL keypads regardless of lockout
                    bool was_locked = is_keypad_in_lockout(cfg->keypadNumber);
                    if (was_locked) {
                        ESP_LOGI("SCENE_0_OVERRIDE", "OVERRIDING lockout for keypad %d - Scene 0 takes priority", cfg->keypadNumber);
                    }
                    
                    uint16_t original_state = led_states[k];
                    uint16_t new_led_state = 0x0000;
                    
                    // === FIX: Preserve backlight when sending Scene 0 ===
                    if (BACKLIGHT) {
                        new_led_state |= (1 << 8);
                        ESP_LOGI("SCENE_ACTION", "Keypad %d: Backlight preserved (BACKLIGHT defined)", cfg->keypadNumber);
                    }
                    
                    if (new_led_state != original_state) {
                        led_states[k] = new_led_state;
                        ESP_LOGI("SCENE_ACTION", "Keypad %d: Turning OFF ALL LEDs (was 0x%04X, now 0x%04X)", 
                                cfg->keypadNumber, original_state, new_led_state);
                        
                        // Queue LED command for this keypad
                        ESP_LOGI("SCENE_ACTION", "Queueing LED command for keypad %d", cfg->keypadNumber);
                        queue_led_command_for_polling(cfg->keypadNumber, new_led_state);
                        
                        // Save LED state to NVS
                        save_led_state_to_nvs(cfg->keypadNumber, new_led_state);
                        
                        // Small delay to prevent queue overflow and allow processing
                        vTaskDelay(pdMS_TO_TICKS(50));
                        
                    } else {
                        ESP_LOGI("SCENE_ACTION", "Keypad %d: No LED state change needed (already 0x%04X)", 
                                cfg->keypadNumber, original_state);
                        
                        // Still send the command to ensure synchronization
                        ESP_LOGI("SCENE_ACTION", "Queueing sync LED command for keypad %d", cfg->keypadNumber);
                        queue_led_command_for_polling(cfg->keypadNumber, new_led_state);
                        vTaskDelay(pdMS_TO_TICKS(30));
                    }
                }
            }
            
            ESP_LOGI("SCENE_0", "All keypads in zone %d processed for Scene 0 (lockout overridden)", zoneNumber);
        }               
        else {
            // Handle scenes 1-15 - USE NORMAL LOCKOUT RULES
            // First, log all scene mappings for this zone
            ESP_LOGI("SCENE_MAPPING", "Scene %d mappings in Zone %d:", scene_id, zoneNumber);
            
            for (int k = 0; k < keypads_count; k++) {
                keypad_config_t *cfg = &keypad_configs[k];
                if (cfg->slaveId == slaveId && cfg->zoneNumber == zoneNumber) {
                    bool has_on_scene = false;
                    bool has_off_scene = false;
                    
                    // Check for ON scenes (LOG ONLY - no flag setting)
                    for (int btn = 0; btn < NUM_BUTTONS; btn++) {
                        if (cfg->keyOnScene[btn] == scene_id) {
                            if (!has_on_scene) {
                                ESP_LOGI("SCENE_MAPPING", "  - Keypad %d:", cfg->keypadNumber);
                                has_on_scene = true;
                            }
                            ESP_LOGI("SCENE_MAPPING", "      Button %d → ON (Scene %d)", btn + 1, scene_id);
                        }
                    }
                    
                    // Check for OFF scenes (LOG ONLY - no flag setting)
                    for (int btn = 0; btn < NUM_BUTTONS; btn++) {
                        if (cfg->keyOffScene[btn] == scene_id) {
                            if (!has_off_scene && !has_on_scene) {
                                ESP_LOGI("SCENE_MAPPING", "  - Keypad %d:", cfg->keypadNumber);
                                has_off_scene = true;
                            }
                            ESP_LOGI("SCENE_MAPPING", "      Button %d → OFF (Scene %d)", btn + 1, scene_id);
                        }
                    }
                    
                    if (!has_on_scene && !has_off_scene) {
                        ESP_LOGI("SCENE_MAPPING", "  - Keypad %d: No scene mapping", cfg->keypadNumber);
                    }
                }
            }
            
            // Then perform the actual scene actions - USE NORMAL LOCKOUT
            ESP_LOGI("SCENE_ACTION", "Executing Scene %d actions:", scene_id);
            for (int k = 0; k < keypads_count; k++) {
                keypad_config_t *cfg = &keypad_configs[k];
                if (cfg->slaveId == slaveId && cfg->zoneNumber == zoneNumber) {
                    
                    // === NORMAL LOCKOUT CHECK FOR SCENES 1-15 ===
                    if (is_keypad_in_lockout(cfg->keypadNumber)) {
                        ESP_LOGI("SCENE_LOCKOUT", "Skipping scene command for keypad %d - recently pressed", cfg->keypadNumber);
                        continue;
                    }
                    
                    bool led_state_changed = false;
                    uint16_t new_led_state = led_states[k];
                    uint16_t original_state = led_states[k];
                    
                    // Process ON scenes first
                    for (int btn = 0; btn < NUM_BUTTONS; btn++) {
                        if (cfg->keyOnScene[btn] == scene_id) {
                            uint16_t mask = (1 << btn);
                            if (!(new_led_state & mask)) {
                                new_led_state |= mask;
                                ESP_LOGI("SCENE_ACTION", "Keypad %d: Button %d → LED ON (Scene %d)", 
                                        cfg->keypadNumber, btn + 1, scene_id);
                                led_state_changed = true;
                                // SET FLAGS HERE for this specific keypad
                                is_btn_state_on = true;
                                dry_cmd_active_onoff_btn = btn + 1;
                            } else {
                                ESP_LOGI("SCENE_ACTION", "Keypad %d: Button %d → LED already ON (Scene %d)", 
                                        cfg->keypadNumber, btn + 1, scene_id);
                                // Still set flags even if no change
                                is_btn_state_on = true;
                                dry_cmd_active_onoff_btn = btn + 1;
                            }
                        }
                    }
                    
                    // Process OFF scenes second (overrides ON if same button has both)
                    for (int btn = 0; btn < NUM_BUTTONS; btn++) {
                        if (cfg->keyOffScene[btn] == scene_id) {
                            uint16_t mask = (1 << btn);
                            if (new_led_state & mask) {
                                new_led_state &= ~mask;
                                ESP_LOGI("SCENE_ACTION", "Keypad %d: Button %d → LED OFF (Scene %d)", 
                                        cfg->keypadNumber, btn + 1, scene_id);
                                led_state_changed = true;
                                // SET FLAGS HERE for this specific keypad
                                is_btn_state_on = false;
                                dry_cmd_active_onoff_btn = btn + 1;
                            } else {
                                ESP_LOGI("SCENE_ACTION", "Keypad %d: Button %d → LED already OFF (Scene %d)", 
                                        cfg->keypadNumber, btn + 1, scene_id);
                                // Still set flags even if no change
                                is_btn_state_on = false;
                                dry_cmd_active_onoff_btn = btn + 1;
                            }
                        }
                    }
                    
                    // Handle backlight
                    uint16_t backlight_mask = (1 << 8);
                    if (BACKLIGHT) {
                        if (!(new_led_state & backlight_mask)) {
                            new_led_state |= backlight_mask;
                            if (!led_state_changed) {
                                ESP_LOGI("SCENE_ACTION", "Keypad %d: Backlight ON", cfg->keypadNumber);
                            }
                            led_state_changed = true;
                        }
                    } else {
                        if (new_led_state & backlight_mask) {
                            new_led_state &= ~backlight_mask;
                            if (!led_state_changed) {
                                ESP_LOGI("SCENE_ACTION", "Keypad %d: Backlight OFF", cfg->keypadNumber);
                            }
                            led_state_changed = true;
                        }
                    }
                    
                    // Update if state changed
                    if (led_state_changed || new_led_state != original_state) {
                        led_states[k] = new_led_state;
                        
                        ESP_LOGI("LED_STATE", "Keypad %d: LED state changed 0x%04X → 0x%04X", 
                                cfg->keypadNumber, original_state, new_led_state);
                        
                        // Queue LED command with delay to prevent queue overflow
                        queue_led_command_for_polling(cfg->keypadNumber, new_led_state);
                        save_led_state_to_nvs(cfg->keypadNumber, new_led_state);
                        
                        // Small delay to allow queue processing
                        vTaskDelay(pdMS_TO_TICKS(30));
                    } else {
                        ESP_LOGI("SCENE_ACTION", "Keypad %d: No LED state change needed", cfg->keypadNumber);
                    }
                }
            }
        }
        ESP_LOGI("ZONES", "=== Scene %d processing complete ===\n", scene_id);
    }
}
void update_keypad_leds_for_zone(uint8_t slaveId, uint8_t zone, uint8_t scene) {
    ESP_LOGI("LED_UPDATE", "Updating LEDs for slave %d zone %d scene %d", slaveId, zone, scene);
    
    for (int i = 0; i < keypads_count; i++) {
        keypad_config_t *cfg = &keypad_configs[i];
        if (cfg->slaveId == slaveId && cfg->zoneNumber == zone) {
            uint16_t led_state = 0;
            for (int btn = 0; btn < NUM_BUTTONS; btn++) {
                if (cfg->keyOnScene[btn] == scene) {
                    led_state |= (1 << btn);
                }
            }
            if (BACKLIGHT) led_state |= (1 << 8);
            
            ESP_LOGI("LED_UPDATE", "Keypad %d: scene %d -> LED state 0x%04X", cfg->keypadNumber, scene, led_state);
            send_led_command_priority(cfg->keypadNumber, led_state);
        }
    }
}

esp_err_t active_panel_get_handler(httpd_req_t *req) {
    char resp[32];
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Reset if timeout has passed
    if (current_time - last_button_press_time > ACTIVE_PANEL_TIMEOUT_MS) {
        web_active_keypad_no = 0;
    }
    
    if (web_active_keypad_no > 0) {
        snprintf(resp, sizeof(resp), "%d", web_active_keypad_no);
    } else {
        snprintf(resp, sizeof(resp), "--");
    }
    
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

// === HTTP Handlers ===
esp_err_t ota_upload_handler(httpd_req_t *req)
{
    char buf[1024];
    int remaining = req->content_len;
    bool image_started = false;
    esp_err_t err;

    ESP_LOGI("OTA", "Starting OTA update. Size: %d bytes", remaining);

    const esp_partition_t *update_partition =
        esp_ota_get_next_update_partition(NULL);

    if (!update_partition) {
        ESP_LOGE("OTA", "No OTA partition found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle = 0;

    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE("OTA", "esp_ota_begin failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    while (remaining > 0) {
        int received = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
        if (received <= 0) continue;

        int offset = 0;

        if (!image_started) {
            for (int i = 0; i < received; i++) {
                if ((uint8_t)buf[i] == 0xE9) {
                    offset = i;
                    image_started = true;
                    ESP_LOGI("OTA", "Firmware start found at offset %d", offset);
                    break;
                }
            }

            if (!image_started) {
                remaining -= received;
                continue;
            }
        }

        err = esp_ota_write(ota_handle, buf + offset, received - offset);
        if (err != ESP_OK) {
            ESP_LOGE("OTA", "esp_ota_write failed (%s)", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return ESP_FAIL;
        }

        remaining -= received;
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE("OTA", "esp_ota_end failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE("OTA", "esp_ota_set_boot_partition failed (%s)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OTA Successful! Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();

    return ESP_OK;
}


esp_err_t index_get_handler(httpd_req_t *req) {
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

bool is_keypad_in_nvs(uint8_t keypadNumber) {
    nvs_handle_t nvs;
    char key[16];
    snprintf(key, sizeof(key), "keypad%d", keypadNumber);
    bool exists = false;
    
    if (nvs_open("keypad_ns", NVS_READONLY, &nvs) == ESP_OK) {
        size_t len = sizeof(keypad_config_t);
        keypad_config_t dummy;
        if (nvs_get_blob(nvs, key, &dummy, &len) == ESP_OK) {
            exists = true;
        }
        nvs_close(nvs);
    }
    
    ESP_LOGI("NVS", "Keypad %d exists in NVS: %s", keypadNumber, exists ? "YES" : "NO");
    return exists;
}

bool is_rotary_in_nvs(uint8_t rotaryNumber) {
    nvs_handle_t nvs;
    char key[16];
    snprintf(key, sizeof(key), "rotary%d", rotaryNumber);
    bool exists = false;
    
    if (nvs_open("rotary_ns", NVS_READONLY, &nvs) == ESP_OK) {
        size_t len = sizeof(rotary_config_t);
        rotary_config_t dummy;
        if (nvs_get_blob(nvs, key, &dummy, &len) == ESP_OK) {
            exists = true;
        }
        nvs_close(nvs);
    }
    
    ESP_LOGI("NVS", "Rotary %d exists in NVS: %s", rotaryNumber, exists ? "YES" : "NO");
    return exists;
}

void load_channel_polling_ranges_from_nvs() {
    global_valid_count = 0; // Reset before loading new ranges
    memset(global_valid_frames, 0, sizeof(global_valid_frames)); // Clear pointers
    
    nvs_handle_t nvs;
    size_t size = sizeof(eligible_channel_poll_values);
    esp_err_t err = nvs_open("poll_ns", NVS_READONLY, &nvs);
    
    if (err == ESP_OK) {
        // Load the eligible ranges array
        err = nvs_get_blob(nvs, "eligible_ranges", eligible_channel_poll_values, &size);
        if (err == ESP_OK) {
            // Load the count
            err = nvs_get_u8(nvs, "eligible_count", &eligible_count);
            if (err == ESP_OK) {
                ESP_LOGI("CHANNEL_POLL", "Loaded %d ranges from NVS:", eligible_count);
                for (int i = 0; i < eligible_count; i++) {
                    ESP_LOGI("CHANNEL_POLL", "→ %d", eligible_channel_poll_values[i]);
                    
                    // Map to global_valid_frames
                    for (int j = 0; j < MAX_CHANNEL_POLL_RANGES; j++) {
                        if (eligible_channel_poll_values[i] == channel_poll_range_ids[j]) {
                            if (global_valid_count < 2) {
                                global_valid_frames[global_valid_count++] = channel_frames[j];
                                ESP_LOGI("CHANNEL_POLL", "Mapped Range %d → Frame[%d]", 
                                         eligible_channel_poll_values[i], j);
                            }
                            break;
                        }
                    }
                }
            } else {
                ESP_LOGW("CHANNEL_POLL", "Failed to load eligible_count from NVS: %s", esp_err_to_name(err));
                eligible_count = 0;
            }
        } else {
            ESP_LOGW("CHANNEL_POLL", "Failed to load eligible_ranges from NVS: %s", esp_err_to_name(err));
            eligible_count = 0;
        }
        nvs_close(nvs);
    } else {
        ESP_LOGW("CHANNEL_POLL", "Failed to open poll_ns namespace: %s", esp_err_to_name(err));
        eligible_count = 0;
    }
    
    if (eligible_count == 0) {
        ESP_LOGI("CHANNEL_POLL", "No polling ranges found in NVS, starting fresh");
    }
}

typedef struct {
    uint16_t range_id;
    int range_index;
} rotary_range_config_t;

// Helper to find which range a given channel belongs to
rotary_range_config_t get_channel_range_info(uint16_t channel)
{
    rotary_range_config_t info = { .range_id = 0, .range_index = -1 };
    for (int i = 0; i < 9; i++) {
        if (channel <= channel_poll_range_ids[i]) {
            info.range_id = channel_poll_range_ids[i];
            info.range_index = i;
            return info;
        }
    }
    return info; // invalid if not found
}

void save_channel_polling_ranges_to_nvs() {
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("poll_ns", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        // Save the eligible ranges array
        err = nvs_set_blob(nvs, "eligible_ranges", eligible_channel_poll_values, sizeof(eligible_channel_poll_values));
        if (err == ESP_OK) {
            // Save the count
            err = nvs_set_u8(nvs, "eligible_count", eligible_count);
            if (err == ESP_OK) {
                nvs_commit(nvs);
                ESP_LOGI("CHANNEL_POLL", "Saved %d polling ranges to NVS", eligible_count);
                for (int i = 0; i < eligible_count; i++) {
                    ESP_LOGI("CHANNEL_POLL", "→ Range %d", eligible_channel_poll_values[i]);
                }
            }
        }
        if (err != ESP_OK) {
            ESP_LOGE("CHANNEL_POLL", "Failed to save polling ranges to NVS: %s", esp_err_to_name(err));
        }
        nvs_close(nvs);
    } else {
        ESP_LOGE("CHANNEL_POLL", "Failed to open poll_ns namespace: %s", esp_err_to_name(err));
    }
}

void update_channel_polling_ranges(uint16_t user_channel, bool is_delete, uint8_t rotaryNumber) {
    // Map channel → range ID
    uint16_t range_id = 0;
    for (int i = 0; i < MAX_CHANNEL_POLL_RANGES; i++) {
        int range_end = channel_poll_range_ids[i];
        int range_start = (i == 0) ? 1 : channel_poll_range_ids[i - 1] + 1;
        if (user_channel >= range_start && user_channel <= range_end) {
            range_id = channel_poll_range_ids[i];
            break;
        }
    }

    if (range_id == 0) return; // invalid

    if (!is_delete) {
        // --- ADDING ---
        bool already_exists = false;
        for (int i = 0; i < eligible_count; i++) {
            if (eligible_channel_poll_values[i] == range_id) {
                already_exists = true;
                break;
            }
        }

        if (!already_exists && eligible_count < 2) {
            eligible_channel_poll_values[eligible_count++] = range_id;
            ESP_LOGI("CHANNEL_POLL", "Added new range %d → eligible_count=%d", range_id, eligible_count);
        } else if (already_exists) {
            ESP_LOGI("CHANNEL_POLL", "Range %d already exists — no change", range_id);
        } else {
            ESP_LOGW("CHANNEL_POLL", "Eligible count limit reached (2). Skipping %d", range_id);
        }

    } else {
        // --- DELETING ---
        bool still_used = false;
        
        // Check if other rotaries use this range
        for (int r = 0; r < rotaries_count; r++) {
            if (rotary_configs[r].rotaryNumber == rotaryNumber) continue; // Skip the rotary being deleted
            
            for (int k = 0; k < 3; k++) { // up to 3 keys per rotary
                uint16_t ch = rotary_configs[r].keyValue[k];
                if (ch >= 1 && ch <= 1024) {
                    uint16_t tmp_id = 0;
                    for (int i = 0; i < MAX_CHANNEL_POLL_RANGES; i++) {
                        int range_end = channel_poll_range_ids[i];
                        int range_start = (i == 0) ? 1 : channel_poll_range_ids[i - 1] + 1;
                        if (ch >= range_start && ch <= range_end) {
                            tmp_id = channel_poll_range_ids[i];
                            break;
                        }
                    }
                    if (tmp_id == range_id) {
                        still_used = true;
                        ESP_LOGI("CHANNEL_POLL", "Range %d still used by rotary %d (channel %d)", 
                                range_id, rotary_configs[r].rotaryNumber, ch);
                        break;
                    }
                }
            }
            if (still_used) break;
        }

        if (!still_used) {
            // Remove the range from eligible ranges
            for (int i = 0; i < eligible_count; i++) {
                if (eligible_channel_poll_values[i] == range_id) {
                    // Shift all subsequent elements left
                    for (int j = i; j < eligible_count - 1; j++) {
                        eligible_channel_poll_values[j] = eligible_channel_poll_values[j + 1];
                    }
                    eligible_count--;
                    ESP_LOGI("CHANNEL_POLL", "Removed unused range %d (eligible_count=%d)", range_id, eligible_count);
                    break;
                }
            }
        } else {
            ESP_LOGI("CHANNEL_POLL", "Range %d still used by another rotary — not deleted", range_id);
        }
    }

    // Update frame mapping
    global_valid_count = eligible_count;
    for (int i = 0; i < eligible_count; i++) {
        for (int j = 0; j < MAX_CHANNEL_POLL_RANGES; j++) {
            if (eligible_channel_poll_values[i] == channel_poll_range_ids[j]) {
                global_valid_frames[i] = channel_frames[j];
                break;
            }
        }
    }

    // Save to NVS after updating
    save_channel_polling_ranges_to_nvs();
}


void start_polling_tasks_if_needed(void) {
    if (!polling_task_started && (keypads_count > 0 || rotaries_count > 0)) {
        ESP_LOGI(TAG_RS485_DATA, "Starting polling tasks (Keypads: %d, Rotaries: %d)", 
                 keypads_count, rotaries_count);
        
        // Make sure any existing task handles are cleared
        polling_task = NULL;
        zone_polling_task = NULL;
        
        // Start tasks with proper delays
        vTaskDelay(pdMS_TO_TICKS(50));
        xTaskCreate(polling_response_task, "keypad_polling", 4096, NULL, 8, &polling_task);
        
        vTaskDelay(pdMS_TO_TICKS(50));
        xTaskCreate(polling_response_task_controller, "zone_polling", 4096, NULL, 7, &zone_polling_task);
        
        polling_task_started = true;
        
        // Wait for keypad polling task to be ready
        if (xSemaphoreTake(polling_semaphore, pdMS_TO_TICKS(2000)) == pdTRUE) {
            ESP_LOGI(TAG_RS485_DATA, "Polling task is ready");
        } else {
            ESP_LOGE(TAG_RS485_DATA, "Polling task failed to start");
        }
    }
}


// Helper function to save keypad configuration
bool save_keypad_configuration(keypad_config_t *config) {
    // Store in keypad_configs array
    bool exists = false;
    for (int i = 0; i < keypads_count; i++) {
        if (keypad_configs[i].keypadNumber == config->keypadNumber) {
            memcpy(&keypad_configs[i], config, sizeof(keypad_config_t));
            exists = true;
            ESP_LOGI("NVS", "Updated existing keypad %d", config->keypadNumber);
            default_slave_id = config->slaveId;
            break;
        }
    }
    
    if (!exists && keypads_count < MAX_DEVICES) {
        memcpy(&keypad_configs[keypads_count], config, sizeof(keypad_config_t));
        keypads_count++;
        ESP_LOGI("NVS", "Added new keypad %d (total: %d)", config->keypadNumber, keypads_count);
    }

    // Also store in device_configs for unified management
    if (devices_count < MAX_DEVICES) {
        device_configs[devices_count].deviceType = DEVICE_TYPE_KEYPAD;
        memcpy(&device_configs[devices_count].config.keypad, config, sizeof(keypad_config_t));
        devices_count++;
    }

    // Save to NVS
    nvs_handle_t nvs;
    char key[16];
    snprintf(key, sizeof(key), "keypad%d", config->keypadNumber);
    if (nvs_open("keypad_ns", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_blob(nvs, key, config, sizeof(keypad_config_t));
        nvs_commit(nvs);
        nvs_close(nvs);
        ESP_LOGI("NVS", "Saved keypad %d to NVS", config->keypadNumber);
        return true;
    }
    return false;
}
// Helper function to save rotary configuration  
bool save_rotary_configuration(rotary_config_t *config) {
    // Store in rotary_configs array
    bool exists = false;
    for (int i = 0; i < rotaries_count; i++) {
        if (rotary_configs[i].rotaryNumber == config->rotaryNumber) {
            memcpy(&rotary_configs[i], config, sizeof(rotary_config_t));
            exists = true;
            ESP_LOGI("NVS", "Updated existing rotary %d", config->rotaryNumber);
            break;
        }
    }
    
    if (!exists && rotaries_count < MAX_DEVICES) {
        memcpy(&rotary_configs[rotaries_count], config, sizeof(rotary_config_t));
        rotaries_count++;
        ESP_LOGI("NVS", "Added new rotary %d (total: %d)", config->rotaryNumber, rotaries_count);
    }

    // Also store in device_configs for unified management
    if (devices_count < MAX_DEVICES) {
        device_configs[devices_count].deviceType = DEVICE_TYPE_ROTARY;
        memcpy(&device_configs[devices_count].config.rotary, config, sizeof(rotary_config_t));
        devices_count++;
    }

    // Save to NVS
    nvs_handle_t nvs;
    char key[16];
    snprintf(key, sizeof(key), "rotary%d", config->rotaryNumber);
    if (nvs_open("rotary_ns", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_blob(nvs, key, config, sizeof(rotary_config_t));
        nvs_commit(nvs);
        nvs_close(nvs);
        ESP_LOGI("NVS", "Saved rotary %d to NVS", config->rotaryNumber);
        return true;
    }
    return false;
}



// === Keypad Configuration Handler (POST) ===
esp_err_t submit_keypad_handler(httpd_req_t *req) {
    char content[1024];
    int ret, remaining = req->content_len;
    int total_read = 0;
    
    //ESP_LOGI("WEB", "Keypad config POST received, content length: %d", remaining);
    
    // Read POST data
    while (remaining > 0) {
        ret = httpd_req_recv(req, content + total_read, remaining);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE("WEB", "Failed to read POST data: %d", ret);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read POST data");
            return ESP_FAIL;
        }
        remaining -= ret;
        total_read += ret;
    }
    content[total_read] = '\0';
    
    ESP_LOGI("WEB", "Received keypad POST data: %s", content);
    
    keypad_config_t config = {0};
    bool has_required_params = false;
    
    // Parse URL-encoded form data
    // char *token = strtok(content, "&");
    // while (token != NULL) {
    //     char *key = token;
    //     char *value = strchr(token, '=');
    //     if (value) {
    //         *value = '\0';
    //         value++;
            
    //         ESP_LOGI("WEB", "Parsing: %s = %s", key, value);
            
    //         if (strcmp(key, "slaveId") == 0) {
    //             config.slaveId = atoi(value);
    //             has_required_params = true;
    //         } 
    //         else if (strcmp(key, "keypadNumber") == 0) {
    //             config.keypadNumber = atoi(value);
    //             has_required_params = true;
    //         } 
    //         else if (strcmp(key, "zoneNumber") == 0) {
    //             config.zoneNumber = atoi(value);
    //         }
    //         else {
    //             // Handle key scene values
    //             for (int i = 1; i <= BUTTONS_PER_KEYPAD; i++) {
    //                 char on_key[16], off_key[16];
    //                 snprintf(on_key, sizeof(on_key), "key%dOnScene", i);
    //                 snprintf(off_key, sizeof(off_key), "key%dOffScene", i);
                    
    //                 // In the parsing section - ONLY check for empty strings, allow explicit 0
    //                 if (strcmp(key, on_key) == 0) {
    //                     config.keyOnScene[i-1] = (strlen(value) > 0) ? atoi(value) : 0;
    //                     // But if it's empty, set to a high number that won't conflict
    //                     if (strlen(value) == 0) {
    //                         config.keyOnScene[i-1] = 255; // Use 255 for unassigned
    //                     }
    //                 }
    //                 else if (strcmp(key, off_key) == 0) {
    //                     config.keyOffScene[i-1] = (strlen(value) > 0) ? atoi(value) : 0;
    //                     if (strlen(value) == 0) {
    //                         config.keyOffScene[i-1] = 255; // Use 255 for unassigned
    //                     }
    //                 }
    //             }
    //         }
    //     }
    //     token = strtok(NULL, "&");
    // }
    char *token = strtok(content, "&");
    while (token != NULL) {
        char *key = token;
        char *value = strchr(token, '=');
        if (value) {
            *value = '\0';
            value++;
            
            ESP_LOGI("WEB", "Parsing: %s = %s", key, value);
            
            if (strcmp(key, "slaveId") == 0) {
                config.slaveId = atoi(value);
                has_required_params = true;
            } 
            else if (strcmp(key, "keypadNumber") == 0) {
                config.keypadNumber = atoi(value);
                has_required_params = true;
            } 
            else if (strcmp(key, "zoneNumber") == 0) {
                config.zoneNumber = atoi(value);
            }
            else {
                // === CHANGE THIS SECTION ===
                // Handle key scene values - use -1 for unassigned/empty fields
                for (int i = 1; i <= BUTTONS_PER_KEYPAD; i++) {
                    char on_key[16], off_key[16];
                    snprintf(on_key, sizeof(on_key), "key%dOnScene", i);
                    snprintf(off_key, sizeof(off_key), "key%dOffScene", i);
                    
                    if (strcmp(key, on_key) == 0) {
                        if (strlen(value) > 0) {
                            config.keyOnScene[i-1] = atoi(value);  // Can be 0 if user entered "0"
                        } else {
                            config.keyOnScene[i-1] = -1;  // -1 for empty fields
                        }
                    }
                    else if (strcmp(key, off_key) == 0) {
                        if (strlen(value) > 0) {
                            config.keyOffScene[i-1] = atoi(value);  // Can be 0 if user entered "0"
                        } else {
                            config.keyOffScene[i-1] = -1;  // -1 for empty fields
                        }
                    }
                }
                // === END OF CHANGE ===
            }
        }
        token = strtok(NULL, "&");
    }
    if (!has_required_params) {
        ESP_LOGE("WEB", "Missing required parameters");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing slaveId or keypadNumber");
        return ESP_OK;
    }

    // === SEND RESPONSE IMMEDIATELY (BEFORE any long operations) ===
    httpd_resp_sendstr(req, "Keypad Configuration Sent!");
    
    // === NOW do the potentially long operations AFTER sending response ===
    
    // Check if device already exists in NVS
    if (!is_keypad_in_nvs(config.keypadNumber)) {

        if (keypads_count + rotaries_count >= MAX_DEVICES) {  // <--- enforce your limit
            ESP_LOGW("WEB", "Device limit (%d) reached — skipping new keypad %d", 
                    MAX_DEVICES, config.keypadNumber);
            return ESP_OK; // Stop here, no configure_panel_address() call
        }

        ESP_LOGI("WEB", "New keypad %d - configuring panel address", config.keypadNumber);

        // Store the configuration for later use
        memcpy(&last_submitted_config, &config, sizeof(config));
        last_config_valid = true;
        panel_new_address = config.keypadNumber;

        // Configure panel address for KEYPAD
        configure_panel_address(DEVICE_TYPE_KEYPAD);
    }

    else {
        ESP_LOGI("WEB", "Keypad %d already exists in NVS, updating configuration", config.keypadNumber);
        
        // Update the configuration directly since device already exists
        if (save_keypad_configuration(&config)) {
            ESP_LOGI("WEB", "Keypad %d configuration updated successfully", config.keypadNumber);
            httpd_resp_sendstr(req, "Keypad Configuration Saved Successfully!");
        } else {
            ESP_LOGE("WEB", "Failed to update keypad %d configuration", config.keypadNumber);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save configuration");
        }
    }
    
    return ESP_OK;
}

// === Rotary Configuration Handler (POST) ===
esp_err_t submit_rotary_handler(httpd_req_t *req) {
    char content[512];
    int ret, remaining = req->content_len;
    int total_read = 0;
    
    //ESP_LOGI("WEB", "Rotary config POST received, content length: %d", remaining);
    
    // Read POST data
    while (remaining > 0) {
        ret = httpd_req_recv(req, content + total_read, remaining);
        if (ret <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            ESP_LOGE("WEB", "Failed to read POST data: %d", ret);
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Failed to read POST data");
            return ESP_FAIL;
        }
        remaining -= ret;
        total_read += ret;
    }
    content[total_read] = '\0';
    
    ESP_LOGI("WEB", "Received rotary POST data: %s", content);
    
    rotary_config_t config = {0};
    bool has_required_params = false;
    
    // Parse URL-encoded form data
    char *token = strtok(content, "&");
    while (token != NULL) {
        char *key = token;
        char *value = strchr(token, '=');
        if (value) {
            *value = '\0';
            value++;
            
            ESP_LOGI("WEB", "Parsing: %s = %s", key, value);
            
            if (strcmp(key, "slaveId") == 0) {
                config.slaveId = atoi(value);
                has_required_params = true;
            } 
            else if (strcmp(key, "rotaryNumber") == 0) {
                config.rotaryNumber = atoi(value);
                has_required_params = true;
            }
            else if (strcmp(key, "key1Number") == 0) {
                config.keyValue[0] = atoi(value);
                poll_buff.channel1_pollVal = config.keyValue[0];
            }
            else if (strcmp(key, "key2Number") == 0) {
                config.keyValue[1] = atoi(value);
                poll_buff.channel2_pollVal = config.keyValue[1];
            }
            else if (strcmp(key, "key3Number") == 0) {
                config.keyValue[2] = atoi(value);
                poll_buff.channel3_pollVal = config.keyValue[2];
            }
        }
        token = strtok(NULL, "&");
    }

    if (!has_required_params) {
        ESP_LOGE("WEB", "Missing required parameters");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing slaveId or rotaryNumber");
        return ESP_OK;
    }
    
    // === NEW CODE: Store the configuration for later use ===
    memcpy(&last_submitted_rotary_config, &config, sizeof(config));
    last_rotary_config_valid = true;
    panel_new_address = config.rotaryNumber;

    // Store channel values in poll_buff for backup
    poll_buff.channel1_pollVal = config.keyValue[0];
    poll_buff.channel2_pollVal = config.keyValue[1]; 
    poll_buff.channel3_pollVal = config.keyValue[2];

    ESP_LOGI("WEB", "Stored rotary config: channels %d,%d,%d", 
             config.keyValue[0], config.keyValue[1], config.keyValue[2]);
    
    // === SEND RESPONSE IMMEDIATELY (BEFORE any long operations) ===
    httpd_resp_sendstr(req, "Rotary Configuration Sent!");
    
    // Check if device already exists in NVS
    if (!is_rotary_in_nvs(config.rotaryNumber)) {

        if (rotaries_count + keypads_count >= MAX_DEVICES) {  // <--- enforce your limit
            ESP_LOGW("WEB", "Device limit (%d) reached — skipping new rotary %d", 
                    MAX_DEVICES, config.rotaryNumber);
            return ESP_OK; // Skip configuring new rotary
        }

        ESP_LOGI("WEB", "New rotary %d - configuring panel address", config.rotaryNumber);
        
        // Configure panel address for ROTARY
        configure_panel_address(DEVICE_TYPE_ROTARY);
    }
 
    else {
        ESP_LOGI("WEB", "Rotary %d already exists in NVS, updating configuration", config.rotaryNumber);

        // Save updated config
        if (save_rotary_configuration(&config)) {
            ESP_LOGI("WEB", "Rotary %d configuration updated successfully", config.rotaryNumber);

            // Update eligible polling ranges
            for (int k = 0; k < 3; k++) {
                if (config.keyValue[k] > 0) {
                    update_channel_polling_ranges(config.keyValue[k], false, config.rotaryNumber);
                }
            }

            // If any eligible ranges exist, enable polling
            if (eligible_count > 0) {
                channels_poll_eligible = true;
                ESP_LOGI("CHANNEL_POLL", "Rotary %d added -> %d eligible ranges found", config.rotaryNumber, eligible_count);
                for (int i = 0; i < eligible_count; i++) {
                    ESP_LOGI("CHANNEL_POLL", " → %d", eligible_channel_poll_values[i]);
                }
            }

            // Start polling tasks if not running
            if (!polling_task_started) {
                ESP_LOGI("RS485_DATA", "Starting polling tasks (Keypads: %d, Rotaries: %d)", keypads_count, rotaries_count);

                if (polling_task == NULL) {
                    xTaskCreate(polling_response_task, "polling_task", 4096, NULL, 8, &polling_task);
                }

                if (zone_polling_task == NULL) {
                    xTaskCreate(polling_response_task_controller, "polling_controller_task", 4096, NULL, 7, &zone_polling_task);
                }

                polling_task_started = true;
            }

            httpd_resp_sendstr(req, "Rotary Configuration Saved Successfully!");
        } 
        else {
            ESP_LOGE("WEB", "Failed to update rotary %d configuration", config.rotaryNumber);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save configuration");
        }
    }

    return ESP_OK;
}


static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    char ssid[32];
    strcpy(ssid, AP_SSID);
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen((char *)wifi_config.ap.ssid);
    strncpy((char *)wifi_config.ap.password, AP_PASS, sizeof(wifi_config.ap.password));
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "AP started. SSID:%s Password:%s IP:192.168.4.1", (char *)wifi_config.ap.ssid, (char *)wifi_config.ap.password);
    
}

httpd_handle_t start_MODBUS_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t active_panel_uri = {
            .uri      = "/active_panel",
            .method   = HTTP_GET,
            .handler  = active_panel_get_handler
        };
        httpd_register_uri_handler(server, &active_panel_uri);

        httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_get_handler };
        
        // === HANDLERS FOR KEYPAD AND ROTARY ===
        // Change to POST since forms typically submit via POST
        httpd_uri_t submit_keypad_uri = { 
            .uri = "/submit_keypad", 
            .method = HTTP_POST,  // Changed to POST
            .handler = submit_keypad_handler 
        };
        httpd_uri_t submit_rotary_uri = { 
            .uri = "/submit_rotary", 
            .method = HTTP_POST,  // Changed to POST
            .handler = submit_rotary_handler 
        };
        
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &submit_keypad_uri);
        httpd_register_uri_handler(server, &submit_rotary_uri);

        httpd_uri_t delete_device_uri = {
            .uri       = "/delete_device",
            .method    = HTTP_GET,
            .handler   = delete_device_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &delete_device_uri);

        httpd_uri_t onoff_config_uri = {
            .uri      = "/onoff_config",
            .method   = HTTP_GET,
            .handler  = onoff_config_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &onoff_config_uri);

        httpd_uri_t curtain_config_uri = {
            .uri      = "/curtain_config",
            .method   = HTTP_GET,
            .handler  = curtain_config_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &curtain_config_uri);
        httpd_uri_t ota_upload = {
            .uri = "/ota",
            .method = HTTP_POST,
            .handler = ota_upload_handler
        };
        httpd_register_uri_handler(server, &ota_upload);

        ESP_LOGI("WEB", "Modbus config server started");
    }
    return server;
}

bool is_device_in_nvs(uint8_t deviceNumber) {
    nvs_handle_t nvs;
    char key[16];
    snprintf(key, sizeof(key), "device%d", deviceNumber);

    bool exists = false;
    if (nvs_open("device_ns", NVS_READONLY, &nvs) == ESP_OK) {
        size_t len = sizeof(device_config_t);
        device_config_t dummy;
        if (nvs_get_blob(nvs, key, &dummy, &len) == ESP_OK) {
            exists = true;
        }
        nvs_close(nvs);
    }
    return exists;
}

void save_keypad_config_after_address_change() {
    keypad_config_t cfg;
    bool found_existing = false;
    memset(&cfg, 0, sizeof(cfg));

    // Step 1: Try to find existing in RAM
    for (int j = 0; j < keypads_count; j++) {
        if (keypad_configs[j].keypadNumber == panel_new_address) {
            memcpy(&cfg, &keypad_configs[j], sizeof(cfg));
            found_existing = true;
            break;
        }
    }

    // Step 2: If not found, use last submitted config
    if (!found_existing && last_config_valid) {
        memcpy(&cfg, &last_submitted_config, sizeof(cfg));
    }

    // Step 3: Always update addressing info
    cfg.keypadNumber = panel_new_address;
    //cfg.slaveId      = default_slave_id;
    // Keep zoneNumber from submitted config

    // Save to NVS
    nvs_handle_t nvs;
    char key[16];
    snprintf(key, sizeof(key), "keypad%d", panel_new_address);
    if (nvs_open("keypad_ns", NVS_READWRITE, &nvs) == ESP_OK) {
        esp_err_t err = nvs_set_blob(nvs, key, &cfg, sizeof(cfg));
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Successfully saved keypad config for %s", key);
                
                // Verify the save worked
                size_t verify_size = sizeof(keypad_config_t);
                keypad_config_t verify_cfg;
                if (nvs_get_blob(nvs, key, &verify_cfg, &verify_size) == ESP_OK) {
                    ESP_LOGI(TAG, "Verified keypad %d saved to NVS: Slave=%d, Zone=%d", 
                            verify_cfg.keypadNumber, verify_cfg.slaveId, verify_cfg.zoneNumber);
                    default_slave_id = verify_cfg.slaveId;
                }
            }
        }
        nvs_close(nvs);
    }

    // Update in RAM
    bool found = false;
    for (int j = 0; j < keypads_count; j++) {
        if (keypad_configs[j].keypadNumber == panel_new_address) {
            memcpy(&keypad_configs[j], &cfg, sizeof(cfg));
            found = true;
            break;
        }
    }
    if (!found && keypads_count < MAX_DEVICES) {
        memcpy(&keypad_configs[keypads_count], &cfg, sizeof(cfg));
        keypads_count++;
        ESP_LOGI(TAG, "Added new keypad %d (total: %d)", panel_new_address, keypads_count);
        keypad_ids_Buff[keypad_id_cnt++] = panel_new_address;
        
        // Start polling tasks if this is the first keypad
        zones_poll_eligible = true;
        poll_option = 0;
        start_polling_tasks_if_needed();                            
    }
}

void save_rotary_config_after_address_change() {
    rotary_config_t cfg;
    bool found_existing = false;
    memset(&cfg, 0, sizeof(cfg));

    // Step 1: Try to find existing in RAM
    for (int j = 0; j < rotaries_count; j++) {
        if (rotary_configs[j].rotaryNumber == panel_new_address) {
            memcpy(&cfg, &rotary_configs[j], sizeof(cfg));
            found_existing = true;
            break;
        }
    }

    // Step 2: If not found, use last submitted rotary config
    if (!found_existing && last_rotary_config_valid) {
        memcpy(&cfg, &last_submitted_rotary_config, sizeof(cfg));
        ESP_LOGI(TAG, "Using last submitted rotary config for rotary %d", panel_new_address);
    }

    // Step 3: Always update addressing info
    cfg.rotaryNumber = panel_new_address;
    //cfg.slaveId = default_slave_id;

    // Step 4: Copy channel values from poll_buff if available
    if (poll_buff.channel1_pollVal > 0) cfg.keyValue[0] = poll_buff.channel1_pollVal;
    if (poll_buff.channel2_pollVal > 0) cfg.keyValue[1] = poll_buff.channel2_pollVal; 
    if (poll_buff.channel3_pollVal > 0) cfg.keyValue[2] = poll_buff.channel3_pollVal;

    // Save to NVS
    nvs_handle_t nvs;
    char key[16];
    snprintf(key, sizeof(key), "rotary%d", panel_new_address);
    if (nvs_open("rotary_ns", NVS_READWRITE, &nvs) == ESP_OK) {
        esp_err_t err = nvs_set_blob(nvs, key, &cfg, sizeof(cfg));
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Successfully saved rotary config for %s", key);
                
                // Log the channel values that were saved
                ESP_LOGI(TAG, "Saved rotary %d channels: %d, %d, %d", 
                        cfg.rotaryNumber, cfg.keyValue[0], cfg.keyValue[1], cfg.keyValue[2]);
                
                // Verify the save worked
                size_t verify_size = sizeof(rotary_config_t);
                rotary_config_t verify_cfg;
                if (nvs_get_blob(nvs, key, &verify_cfg, &verify_size) == ESP_OK) {
                    ESP_LOGI(TAG, "Verified rotary %d saved to NVS: Slave=%d, Channels=%d,%d,%d", 
                            verify_cfg.rotaryNumber, verify_cfg.slaveId,
                            verify_cfg.keyValue[0], verify_cfg.keyValue[1], verify_cfg.keyValue[2]);
                            default_slave_id = cfg.slaveId;
                }
            }
        }
        nvs_close(nvs);
    }

    // Update in RAM
    bool found = false;
    for (int j = 0; j < rotaries_count; j++) {
        if (rotary_configs[j].rotaryNumber == panel_new_address) {
            memcpy(&rotary_configs[j], &cfg, sizeof(cfg));
            found = true;
            break;
        }
    }
    if (!found && rotaries_count < MAX_DEVICES) {
        memcpy(&rotary_configs[rotaries_count], &cfg, sizeof(cfg));
        rotaries_count++;
        ESP_LOGI(TAG, "Added new rotary %d (total: %d)", panel_new_address, rotaries_count);
        
        // Update channel polling ranges with the actual channel values
        for (int k = 0; k < 3; k++) {
            if (rotary_configs[rotaries_count - 1].keyValue[k] > 0) {
                ESP_LOGI(TAG, "Adding channel %d to polling ranges", 
                        rotary_configs[rotaries_count - 1].keyValue[k]);
                update_channel_polling_ranges(rotary_configs[rotaries_count - 1].keyValue[k], false, panel_new_address);
            }
        }
        save_channel_polling_ranges_to_nvs();
        // If we have at least one eligible range, mark channels polling as eligible
        if (eligible_count > 0) {
            channels_poll_eligible = true;
            ESP_LOGI("CHANNEL_POLL", "Rotary %d added -> %d eligible ranges found", panel_new_address, eligible_count);
        }

        poll_option = 1;
        start_polling_tasks_if_needed();
    }
}

//=========Configuraing Panel Address==========
void configure_panel_address(uint8_t device_type) {
    if (DEVICE_LIMIT_REACHED()) {
        ESP_LOGW(TAG, "Device limit (%d) reached — skipping new device configuration", MAX_DEVICES);
        return; // Skip further configuration/storage
    }
    if (xSemaphoreTake(modbus_mutex_uart1, pdMS_TO_TICKS(3000)) == pdTRUE) {
        RS485_WRITE_MODE1;
        
        // Send address change command
        uint8_t new_address[8] = {0xFF, 0x06, 0x10, 0x50, 0x00, panel_new_address, 0x00, 0x00};
        uint16_t crc = modbus_crc16(new_address, 6);
        new_address[6] = crc & 0xFF;
        new_address[7] = (crc >> 8) & 0xFF;
        
        uart_write_bytes(UART_NUM1, (const char*)new_address, FRAME_SIZE);
        uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(50));
        ESP_LOG_BUFFER_HEXDUMP("New address frame ->", new_address, FRAME_SIZE, ESP_LOG_INFO);
        
        // Send save address command
        uint8_t save_address[8] = {0xFF, 0x06, 0x21, 0x99, 0x10, 0x50, 0x4B, 0xFB};
        uart_write_bytes(UART_NUM1, (const char*)save_address, FRAME_SIZE);
        uart_wait_tx_done(UART_NUM1, pdMS_TO_TICKS(50));
        vTaskDelay(pdMS_TO_TICKS(20));

        // --- Start ACK waiting window ---
        uint8_t rx_data[1024];
        uint8_t target_ack[8] = {0xFF, 0x06, 0x10, 0x50, 0xFF, 0xFF, 0x99, 0x75};
        bool ack_received = false;
        int64_t start = esp_timer_get_time();
        int64_t timeout = 60 * 1000000; // 60 seconds

        while ((esp_timer_get_time() - start) < timeout) {
            RS485_READ_MODE1;
            vTaskDelay(pdMS_TO_TICKS(10));
            int len = uart_read_bytes(UART_NUM1, rx_data, sizeof(rx_data), pdMS_TO_TICKS(50));
            if (len > 0) {
                for (int i = 0; i <= len - 8; i++) {
                    if (memcmp(&rx_data[i], target_ack, 8) == 0) {
                        ESP_LOGI(TAG_RS485_DATA, "Panel ACK received for %02X (Type: %s)", 
                                panel_new_address, 
                                device_type == DEVICE_TYPE_KEYPAD ? "KEYPAD" : "ROTARY");

                        // --- Save configuration based on device type ---
                        if (device_type == DEVICE_TYPE_KEYPAD) {
                            save_keypad_config_after_address_change();
                        } else if (device_type == DEVICE_TYPE_ROTARY) {
                            save_rotary_config_after_address_change();
                        }
                        
                        ack_received = true;
                        break;
                    }
                }
            }
            if (ack_received) break;
            RS485_WRITE_MODE1;
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        if (!ack_received) {
            ESP_LOGW(TAG_RS485_DATA, "No ACK received from panel after 60 seconds, skipping NVS update");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        xSemaphoreGive(modbus_mutex_uart1);
    } else {
        ESP_LOGE(TAG_RS485_DATA, "Failed to acquire UART1 mutex for panel configuration");
    }
}

bool save_keypad_config_to_nvs(uint8_t slaveId,
                               uint8_t keypadNumber,
                               uint8_t zoneNumber,
                               keypad_config_t *cfg) {
    nvs_handle_t nvs;
    char key[16];
    snprintf(key, sizeof(key), "keypad%d", keypadNumber);
    bool is_new = true;
    if (nvs_open("keypad_ns", NVS_READWRITE, &nvs) == ESP_OK) {
        if (nvs_set_blob(nvs, key, cfg, sizeof(keypad_config_t)) == ESP_OK) {
            nvs_commit(nvs);
            ESP_LOGI("NVM", "Saved config for %s", key);
            // check if already exists in RAM
            is_new = true;
            for (int i = 0; i < keypads_count; i++) {
                if (keypad_configs[i].keypadNumber == keypadNumber) {
                    memcpy(&keypad_configs[i], cfg, sizeof(keypad_config_t));
                    is_new = false;   // existing keypad updated
                    break;
                }
            }
            if (is_new) {
                if (keypads_count < MAX_DEVICES) {
                    memcpy(&keypad_configs[keypads_count], cfg, sizeof(keypad_config_t));
                    //keypads_count++;   // increment here so tasks see it immediately
                } else {
                    ESP_LOGE("NVM", "Max keypads reached!");
                    is_new = false;
                }
            }
        }
        nvs_close(nvs);
    }
    return is_new;
}

void read_all_rotary_configs_from_nvs() {
    nvs_iterator_t it = NULL;
    esp_err_t err = nvs_entry_find(NVS_DEFAULT_PART_NAME, "rotary_ns", NVS_TYPE_BLOB, &it);
    rotaries_count = 0; // Reset count
    ESP_LOGI(TAG, "==== Reading all rotary configurations from NVS ====");
    
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No rotary entries found in NVS or error: %s", esp_err_to_name(err));
        return;
    }
    
    while (err == ESP_OK && it != NULL && rotaries_count < MAX_DEVICES) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        ESP_LOGI("NVS", "Found rotary entry: %s", info.key);
        
        nvs_handle_t nvs;
        if (nvs_open("rotary_ns", NVS_READONLY, &nvs) == ESP_OK) {
            rotary_config_t config;
            size_t required_size = sizeof(rotary_config_t);
            esp_err_t get_err = nvs_get_blob(nvs, info.key, &config, &required_size);
            
            if (get_err == ESP_OK) {
                ESP_LOGI("NVM", "Successfully loaded rotary: %s | Slave: %d, Rotary: %d", 
                        info.key, config.slaveId, config.rotaryNumber);
                default_slave_id = config.slaveId;
                
                // Store in rotary_configs array
                memcpy(&rotary_configs[rotaries_count], &config, sizeof(rotary_config_t));
                
                // Also populate device_configs array for unified device management
                if (devices_count < MAX_DEVICES) {
                    device_configs[devices_count].deviceType = DEVICE_TYPE_ROTARY;
                    memcpy(&device_configs[devices_count].config.rotary, &config, sizeof(rotary_config_t));
                    devices_count++;
                }
                
                rotaries_count++;
                
                ESP_LOGI("NVM", "Added rotary %d to RAM (total: %d)", config.rotaryNumber, rotaries_count);
                
                // Log channel configuration
                for (int i = 0; i < 3; i++) {
                    if (config.keyValue[i] != 0) {
                        ESP_LOGI("NVM", "  Key %d: Channel=%d", i + 1, config.keyValue[i]);
                    }
                }
            } else {
                ESP_LOGE("NVM", "Failed to get blob for %s: %s", info.key, esp_err_to_name(get_err));
            }
            nvs_close(nvs);
        } else {
            ESP_LOGE("NVM", "Failed to open rotary_ns for %s", info.key);
        }
        
        err = nvs_entry_next(&it);
    }
    
    if (it) {
        nvs_release_iterator(it);
    }
    
    ESP_LOGI(TAG, "==== Loaded %d rotary configurations from NVS ====", rotaries_count);
}

void read_all_keypad_configs_from_nvs() {
    nvs_iterator_t it = NULL;
    esp_err_t err = nvs_entry_find(NVS_DEFAULT_PART_NAME, "keypad_ns", NVS_TYPE_BLOB, &it);
    keypads_count = 0; // Reset count
    ESP_LOGI(TAG, "==== Reading all keypad configurations from NVS ====");
    
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No keypad entries found in NVS or error: %s", esp_err_to_name(err));
        return;
    }
    
    while (err == ESP_OK && it != NULL && keypads_count < MAX_DEVICES) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        ESP_LOGI("NVS", "Found keypad entry: %s", info.key);
        
        nvs_handle_t nvs;
        if (nvs_open("keypad_ns", NVS_READONLY, &nvs) == ESP_OK) {
            keypad_config_t config;
            size_t required_size = sizeof(keypad_config_t);
            esp_err_t get_err = nvs_get_blob(nvs, info.key, &config, &required_size);
            
            if (get_err == ESP_OK) {
                ESP_LOGI("NVM", "Successfully loaded keypad: %s | Slave: %d, Keypad: %d, Zone: %d", 
                        info.key, config.slaveId, config.keypadNumber, config.zoneNumber);
                        default_slave_id = config.slaveId;
                
                // Store in keypad_configs array
                memcpy(&keypad_configs[keypads_count], &config, sizeof(keypad_config_t));
                
                // Initialize LED state for this keypad
                led_states[keypads_count] = 0x0000; // Start with all LEDs off
                if (BACKLIGHT) led_states[keypads_count] |= (1 << 8);
                
                // Also populate device_configs array for unified device management
                if (devices_count < MAX_DEVICES) {
                    device_configs[devices_count].deviceType = DEVICE_TYPE_KEYPAD;
                    memcpy(&device_configs[devices_count].config.keypad, &config, sizeof(keypad_config_t));
                    devices_count++;
                }
                
                keypad_ids_Buff[keypad_id_cnt++] = config.keypadNumber; 
                keypads_count++;
                
                ESP_LOGI("NVM", "Added keypad %d to RAM (total: %d)", config.keypadNumber, keypads_count);
                
                // Log button configuration
                for (int i = 0; i < BUTTONS_PER_KEYPAD; i++) {
                    if (config.keyOnScene[i] != 0 || config.keyOffScene[i] != 0) {
                        ESP_LOGI("NVM", "  Key %d: OnScene=%d, OffScene=%d", 
                                i + 1, config.keyOnScene[i], config.keyOffScene[i]);
                    }
                }
            } else {
                ESP_LOGE("NVM", "Failed to get blob for %s: %s", info.key, esp_err_to_name(get_err));
            }
            nvs_close(nvs);
        } else {
            ESP_LOGE("NVM", "Failed to open keypad_ns for %s", info.key);
        }
        
        err = nvs_entry_next(&it);
    }
    
    if (it) {
        nvs_release_iterator(it);
    }
    
    ESP_LOGI(TAG, "==== Loaded %d keypad configurations from NVS ====", keypads_count);
}

void app_main(void) {
    esp_log_level_set("*", ESP_LOG_NONE);
    ESP_LOGI(TAG, "Starting application...");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize UARTs
    init_uart1();
    init_uart2();

    // Initialize mutexes
    modbus_mutex_uart1 = xSemaphoreCreateMutex();
    modbus_mutex_uart2 = xSemaphoreCreateMutex();
    polling_semaphore = xSemaphoreCreateBinary();
    
    // Initialize LED command queue
    led_queue_mutex = xSemaphoreCreateMutex();
    memset(led_command_queue, 0, sizeof(led_command_queue));
    led_queue_head = 0;
    led_queue_tail = 0;
    led_command_ready = false;
    
    // Initialize frame management
    initialize_frame_management();
    initialize_keypad_frame_management();
    wifi_init_softap();
    start_MODBUS_webserver();
    
    // Load configurations (NO TASK CREATION IN THESE FUNCTIONS)
    read_all_keypad_configs_from_nvs();
    read_all_rotary_configs_from_nvs(); 
    initialize_led_states();
    load_channel_polling_ranges_from_nvs();
    vTaskDelay(pdMS_TO_TICKS(50));
    // === STEP 1: Restore LED states from NVS FIRST ===
    nvs_handle_t nvs;
    esp_err_t nvs_err;
    
    nvs_err = nvs_open("led_ns", NVS_READONLY, &nvs);
    if (nvs_err == ESP_OK) {
        ESP_LOGI("LED_RESTORE", "Successfully opened led_ns namespace");
        
        for (int i = 0; i < keypads_count; i++) {
            char key[16];
            snprintf(key, sizeof(key), "led_state_%d", keypad_configs[i].keypadNumber);
            
            uint16_t saved_state = 0;
            nvs_err = nvs_get_u16(nvs, key, &saved_state);
            
            if (nvs_err == ESP_OK) {
                led_states[i] = saved_state;
                ESP_LOGI("LED_RESTORE", "Restored LED state for keypad %d: 0x%04X", 
                         keypad_configs[i].keypadNumber, saved_state);
            } else {
                ESP_LOGW("LED_RESTORE", "Failed to read LED state for keypad %d, using default", 
                         keypad_configs[i].keypadNumber);
                led_states[i] = 0x0000;
                if (BACKLIGHT) led_states[i] |= (1 << 8);
            }
        }
        nvs_close(nvs);
    } else {
        ESP_LOGW("LED_RESTORE", "Failed to open led_ns namespace, using default LED states");
        for (int i = 0; i < keypads_count; i++) {
            led_states[i] = 0x0000;
            if (BACKLIGHT) led_states[i] |= (1 << 8);
        }
    }

    // === STEP 2: Start polling tasks if we have ANY devices ===
    if (keypads_count > 0 || rotaries_count > 0) {
        ESP_LOGI(TAG, "Starting polling tasks (Keypads: %d, Rotaries: %d)", keypads_count, rotaries_count);
        
        // Start keypad polling task (UART1) - needed for both keypads AND rotaries
        xTaskCreate(polling_response_task, "keypad_polling", 4096, NULL, 8, &polling_task);
        
        // Wait for keypad polling task to be ready
        if (xSemaphoreTake(polling_semaphore, pdMS_TO_TICKS(2000)) == pdTRUE) {
            ESP_LOGI(TAG, "Keypad polling task ready");
        } else {
            ESP_LOGE(TAG, "Keypad polling task failed to start within timeout");
        }
        
        // === STEP 3: Send RESTORED LED states through UART1 (only for keypads) ===
        if (keypads_count > 0) {
            ESP_LOGI("LED_INIT", "Sending restored LED states via UART1...");
            for (int i = 0; i < keypads_count; i++) {
                ESP_LOGI("LED_INIT", "Keypad %d → LED state 0x%04X", 
                         keypad_configs[i].keypadNumber, led_states[i]);
                
                // Set device mode and parameters for LED command
                device_keymode = 0; // Scene based
                dry_cmd_active_onoff_btn = 0xFF; // Default to all buttons
                is_btn_state_on = false; // Default state
                
                // Queue LED command for this keypad
                queue_led_command_for_polling(keypad_configs[i].keypadNumber, led_states[i]);
                
                // Small delay between queuing commands to avoid overwhelming the queue
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            
            ESP_LOGI("LED_INIT", "All LED restore commands queued");
            
            // Wait a bit more to ensure all LED commands are processed
            vTaskDelay(pdMS_TO_TICKS(1000));
            
            // Check if all commands were processed
            if (xSemaphoreTake(led_queue_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                int queue_size = (led_queue_tail - led_queue_head + LED_QUEUE_SIZE) % LED_QUEUE_SIZE;
                ESP_LOGI("LED_INIT", "LED command queue status: %d pending commands", queue_size);
                xSemaphoreGive(led_queue_mutex);
            }
            
            zones_poll_eligible = true;
            poll_option = 0;
        }
        
        if (rotaries_count > 0) {
            channels_poll_eligible = true;
            poll_option = 1;
            ESP_LOGI("CHANNEL_POLL", "Rotaries detected, enabling channel polling");
        }
        
        // === STEP 4: Start zone polling task (UART2) ===
        ESP_LOGI(TAG, "Starting zone polling task (UART2)");
        xTaskCreate(polling_response_task_controller, "zone_polling", 4096, NULL, 7, &zone_polling_task);
        
        polling_task_started = true;
        ESP_LOGI(TAG, "All tasks started");
    } else {
        ESP_LOGI(TAG, "No devices configured, skipping polling tasks");
    }

    // Start watchdog
    task_watchdog_timer = xTimerCreate("watchdog", pdMS_TO_TICKS(10000), pdTRUE, NULL, system_watchdog_callback);
    if (task_watchdog_timer != NULL) {
        xTimerStart(task_watchdog_timer, 0);
        ESP_LOGI(TAG, "Watchdog timer started");
    } else {
        ESP_LOGE(TAG, "Failed to create watchdog timer");
    }
    ESP_LOGI("CHECK", "Next OTA partition: %s", esp_ota_get_next_update_partition(NULL)->label);
    //ESP_LOGI(TAG, "Application started successfully");
    ESP_LOGE("OTA_TEST", "🔥 OTA IMAGE BOOTED 🔥");
    ESP_LOGE("OTA_TEST", "BUILD: %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "=====================================");
    ESP_LOGI(TAG, "====Jaquar Devices RS485 Gateway====");
    ESP_LOGI(TAG, "=====================================");
    dry_send_enabled = true;   // allow dry-contact commands from now on

    // Main loop - kick watchdog periodically
    while (1) {
        kick_watchdog();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}