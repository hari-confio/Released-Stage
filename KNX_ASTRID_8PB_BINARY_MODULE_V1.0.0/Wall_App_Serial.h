
#ifndef SRC_APP_SERIAL_H_
#define SRC_APP_SERIAL_H_

#include <ev_man.h>
#include <SwTimer.h>
#include <em_gpio.h>

//#define SERIAL_TX_PORT gpioPortC
//#define SERIAL_TX_PIN 6
//
//#define SERIAL_RX_PORT gpioPortC
//#define SERIAL_RX_PIN 7

#include <stddef.h>  // Include for size_t definition
#define RELAY_8_PORT gpioPortC
#define RELAY_8_PIN 7

void StartTx(uint8_t *frame,uint16_t length);
//void StartTx();

void delay(uint32_t milliseconds);
extern uint8_t txFrame1on[];

extern uint8_t txFrame1off[];
extern uint8_t txFrame2on[];

extern uint8_t txFrame2off[];
extern uint8_t txFrame3on[];

extern uint8_t txFrame3off[];
extern uint8_t txFrame4on[];


extern uint8_t txFrame4off[];
extern uint8_t txFrame5on[];

extern uint8_t txFrame5off[];

extern uint8_t txFrame6off[];
extern uint8_t txFrame6on[];
void Serial_Init();
//void StartTx(char *frame);
// Define the frame to be transmitted

//
//
/*
#define SERIAL_TX_PORT gpioPortD
#define SERIAL_TX_PIN 12
////
#define SERIAL_RX_PORT gpioPortD
#define SERIAL_RX_PIN 13
*/
//
//
typedef unsigned char byte;

void handle_switch_event(uint8_t switch_number);

//void handle_switch_event(uint8_t switch_number, uint8_t *txFrameOn, uint8_t *txFrameOff);

//void create_read_frame();
//void create_frame(const SApplicationData1* AppData1,byte switch_num,int on_off_val);
// void knx_frame();
 typedef unsigned char byte;

 void crc_fun(byte newValue) ;
 byte CalculateChecksum(byte frame[], byte size) ;
//=======dim
 //===================================
#define PORT_DIM      gpioPortF//PWM for dimming
#define PIN_DIM       6
#define PORT_SENSOR_INT   gpioPortD//Sensor Interrupt
#define PIN_SENSOR_INT    11
  extern uint32_t sw_1_cur_pos;
  extern uint32_t sw_2_cur_pos;
  extern uint32_t sw_3_cur_pos;
  extern uint32_t sw_4_cur_pos;
  extern uint32_t sw_5_cur_pos;
  extern uint32_t sw_6_cur_pos;



  #define InputMode     gpioModeInputPullFilter
  extern void switch_1_timer_CB();
 void Sw_CB();
 void Sw_interrupt_CB(uint8_t id);
 extern void Sensor_interrupt_CB(uint8_t id);


  void Set_Dim_levels(uint8_t Dim);
  void set_proximity_level();
   void Dim_Init();
  extern uint32_t g_gpioEm4Flags ;



 /**
  *  @brief Configure LED Mode
  */
 #define OutputMode      gpioModePushPull

 #define true      1
 #define false     0


 #define ON        0xFF
 #define OFF       0x00


 #define SW_OFF        1
 #define SW_ON       0
 void extract_and_print_group_addresses(uint8_t *buffer, uint8_t length,uint8_t button_num);
extern uint8_t button_pressed_count;
void sendFrame(uint8_t *frame);
void AppResetNvm();
extern uint8_t button_pressed_num ; // Global variable to track button presses
void InitializeButton_rx_Timer();
void SceneButtonPress_rx(uint8_t button_rx);

 //====
#endif /* SRC_APP_SERIAL_H_ */

