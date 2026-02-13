
#ifndef SRC_APP_SERIAL_H_
#define SRC_APP_SERIAL_H_

#include <ev_man.h>
#include <SwTimer.h>
#include <em_gpio.h>
#include <AppTimer.h>
#include <stddef.h>  // Include for size_t definition

void Serial_Init();
extern void timeout_config_addr_CB();
extern void switch_1_timer_CB();
extern void led_toggle_timer_CB();
extern void button_press_timer_CB();
extern SSwTimer timeout_config_addr;
extern SSwTimer button_press_timer;
extern SSwTimer led_toggle_timer;
extern uint32_t g_gpioEm4Flags ;
extern bool on_off_state;
// Used by the application data file.
typedef struct SApplicationData
{
  uint8_t KeypadID;
  uint8_t Relay1BtnNo;
  uint8_t Relay2BtnNo;
  uint8_t Relay3BtnNo;
  uint8_t Relay4BtnNo;
  bool relay1State;
  bool relay2State;
  bool relay3State;
  bool relay4State;

} SApplicationData;

#define FILE_SIZE_APPLICATIONDATA     (sizeof(SApplicationData))

extern SApplicationData ApplicationData;

/**
*  @brief Configure LED Mode
*/

#define true      1
#define false     0


#define ON        0xFF
#define OFF       0x00


#define SW_OFF        1
#define SW_ON       0
extern bool addr_config_mode;
void sendFrame(uint8_t *frame);
void AppResetNvm();
void InitializeButton_rx_Timer();
void SceneButtonPress_rx(uint8_t button_rx);

//====
#endif /* SRC_APP_SERIAL_H_ */

