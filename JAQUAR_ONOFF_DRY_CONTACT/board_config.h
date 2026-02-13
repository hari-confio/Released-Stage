
#ifndef SRC_BOARD_CONFIG_H_
#define SRC_BOARD_CONFIG_H_



/*
 * Hardware Version V3
 *
 * PB1 -- PA3
 * PB2 -- PC8
 * PB3 -- PC9
 * PB4 -- PC10
 * PB5 -- PF3
 * PB6 -- PF4
 * PB7 -- PF5
 * PB8 -- PF6
 *

 *
 * SCL -- PC6
 * SDA -- PC7
 *
 * INT -- PD11
 *
 * DIM -- PD12
 */


#include <stdint.h>

#include "em_gpio.h"
#include "ev_man.h"

//Port A
#define UART0_TX_PORT		gpioPortA
#define UART0_TX_PIN		0

#define UART0_RX_PORT 		gpioPortA
#define UART0_RX_PIN		1




//#ifndef PORT_PB1
//#define PORT_PB1                            gpioPortA
//#endif
//#ifndef PIN_PB1
//#define PIN_PB1                             3
//#endif
//// [GPIO_PB1_GPIO]$
//
//// <gpio> PB2_GPIO
//// $[GPIO_PB2_GPIO]
//#ifndef PORT_PB2
//#define PORT_PB2                            gpioPortC
//#endif
//#ifndef PIN_PB2
//#define PIN_PB2                             8
//#endif
//// [GPIO_PB2_GPIO]$
//
//// <gpio> PB3_GPIO
//// $[GPIO_PB3_GPIO]
//#ifndef PORT_PB3
//#define PORT_PB3                            gpioPortC
//#endif
//#ifndef PIN_PB3
//#define PIN_PB3                             9
//#endif
//// [GPIO_PB3_GPIO]$
//
//// <gpio> PB4_GPIO
//// $[GPIO_PB4_GPIO]
//#ifndef PORT_PB4
//#define PORT_PB4                            gpioPortC
//#endif
//#ifndef PIN_PB4
//#define PIN_PB4                             10
//#endif
//
//
//
//#ifndef PORT_PB5
//#define PORT_PB5                            gpioPortF
//#endif
//#ifndef PIN_PB5
//#define PIN_PB5                             4
//#endif
//
//#ifndef PORT_PB6
//#define PORT_PB6                            gpioPortF
//#endif
//#ifndef PIN_PB6
//#define PIN_PB6                             5
//#endif

#ifndef BUTTON_PORT
#define BUTTON_PORT                            gpioPortD
#endif
#ifndef BUTTON_PIN
#define BUTTON_PIN                             14
#endif

#define SERIAL_TX_PORT                         gpioPortC
#define SERIAL_TX_PIN                          11

#define SERIAL_RX_PORT                         gpioPortC
#define SERIAL_RX_PIN                          8

#define RE_PORT                                gpioPortC
#define RE_PIN                                 9
#define DE_PORT                                gpioPortC
#define DE_PIN                                 10

#define APP_RELAY_1_PORT    gpioPortA
#define APP_RELAY_1_PIN     3

#define APP_RELAY_2_PORT    gpioPortC
#define APP_RELAY_2_PIN     6

#define APP_RELAY_3_PORT    gpioPortF
#define APP_RELAY_3_PIN     3

#define APP_RELAY_4_PORT    gpioPortC
#define APP_RELAY_4_PIN     7

#define LED1_PORT                                gpioPortD // RED
#define LED1_PIN                                 15
#define LED2_PORT                                gpioPortA // GREEN
#define LED2_PIN                                 2

//#ifndef PORT_PB1
//#define PORT_PB1                            gpioPortA
//#endif
//#ifndef PIN_PB1
//#define PIN_PB1                             3
//#endif
//// [GPIO_PB1_GPIO]$
//
//// <gpio> PB2_GPIO
//// $[GPIO_PB2_GPIO]
//#ifndef PORT_PB2
//#define PORT_PB2                            gpioPortC
//#endif
//#ifndef PIN_PB2
//#define PIN_PB2                             8
//#endif
//// [GPIO_PB2_GPIO]$
//
//// <gpio> PB3_GPIO
//// $[GPIO_PB3_GPIO]
//#ifndef PORT_PB3
//#define PORT_PB3                            gpioPortC
//#endif
//#ifndef PIN_PB3
//#define PIN_PB3                             9
//#endif
//// [GPIO_PB3_GPIO]$
//
//// <gpio> PB4_GPIO
//// $[GPIO_PB4_GPIO]
//#ifndef PORT_PB4
//#define PORT_PB4                            gpioPortC
//#endif
//#ifndef PIN_PB4
//#define PIN_PB4                             10
//#endif


//
//#ifndef PORT_PB5
//#define PORT_PB5                            gpioPortF
//#endif
//#ifndef PIN_PB5
//#define PIN_PB5                             4
//#endif

//#ifndef PORT_PB5
//#define PORT_PB5                            gpioPortF
//#endif
//#ifndef PIN_PB5
//#define PIN_PB5                             3
//#endif
//
////#ifndef PORT_PB6
////#define PORT_PB6                            gpioPortF
////#endif
////#ifndef PIN_PB6
////#define PIN_PB6                             5
////#endif
//#ifndef PORT_PB6
//#define PORT_PB6                            gpioPortF
//#endif
//#ifndef PIN_PB6
//#define PIN_PB6                             4
//#endif
//
//#ifndef PORT_PB7
//#define PORT_PB7                            gpioPortF
//#endif
//#ifndef PIN_PB7
//#define PIN_PB7                             5
//#endif
//
//#ifndef PORT_PB8
//#define PORT_PB8                            gpioPortD
//#endif
//#ifndef PIN_PB8
//#define PIN_PB8                             12
//#endif


#define true 			1
#define false			0


#define ON				0xFF
#define OFF				0x00


#define SW_OFF				1
#define SW_ON				0
#define FRAME_SIZE 8       // Maximum expected frame size
extern uint8_t btn_press_cnt;
extern uint8_t nvm_data_buff[5];
extern bool relay_states[4];
extern bool last_relay_state[4]; // store last known state
typedef enum APP_BUTTON_EVENT_
{
  APP_EVENT_PB_DOWN = DEFINE_EVENT_KEY_NBR,
  APP_EVENT_PB_UP,
  APP_EVENT_PB_SHORT_PRESS,
  APP_EVENT_PB_HOLD,
  APP_EVENT_PB_LONG_PRESS,
  APP_EVENT_INCLUSION,
  APP_EVENT_HARD_RESET,

  APP_EVENT_SW1_DOWN,
  APP_EVENT_SW1_UP,
  APP_EVENT_SW1_SHORT_PRESS,
  APP_EVENT_SW1_HOLD,


  APP_EVENT_SW2_DOWN,
  APP_EVENT_SW2_UP,
  APP_EVENT_SW2_SHORT_PRESS,
  APP_EVENT_SW2_HOLD,
  APP_EVENT_SW2_LONG_PRESS,

  APP_EVENT_SW3_DOWN,
  APP_EVENT_SW3_UP,
  APP_EVENT_SW3_SHORT_PRESS,
  APP_EVENT_SW3_HOLD,
  APP_EVENT_SW3_LONG_PRESS,

  APP_EVENT_SW4_DOWN,
  APP_EVENT_SW4_UP,
  APP_EVENT_SW4_SHORT_PRESS,
  APP_EVENT_SW4_HOLD,
  APP_EVENT_SW4_LONG_PRESS,

  EVENT_BTN_MAXIMUM /**< EVENT_BTN_MAX define the last enum type*/
} APP_BUTTON_EVENT;

//typedef enum APP_EVENT_APP_
//{
//  EVENT_EMPTY = 127,//DEFINE_EVENT_APP_NBR,(EVENTS ARE MISMATCHING BECAUSE I HAVE MODIFIED ALL THE EVENTS AS PER MY REQUIREMENT
//  EVENT_APP_INIT,
//  EVENT_APP_REFRESH_MMI,
//  EVENT_APP_FLUSHMEM_READY,
//  EVENT_APP_NEXT_EVENT_JOB,
//  EVENT_APP_FINISH_EVENT_JOB,
//  EVENT_APP_SEND_OVERLOAD_NOTIFICATION,
//  EVENT_APP_SMARTSTART_IN_PROGRESS,
//  EVENT_APP_LEARN_IN_PROGRESS
//}
//EVENT_APP;

typedef enum APP_BUTTON_STATE_
{
	DOWN,
	UP,
	SHORT_PRESS,
	HOLD,
	LONG_PRESS
}APP_BUTTON_STATE;

typedef enum APP_BUTTON_TYPE_
{
	NO_TYPE,
	PUSH_BUTTON_TYPE,
	TOGGLE_TYPE,
	SINGLE_STATE_SWITCH_TYPE
} APP_BUTTON_TYPE;

typedef enum APP_BUTTON_ID_
{
	BUTTON_1,
	BUTTON_2,
	BUTTON_3,
	BUTTON_4,
	TOTAL_NUM_OF_BUTTONS
} APP_BUTTON_ID;



void Init_HW();
uint32_t Board_HW_Init(void);

#endif /* SRC_BOARD_CONFIG_H_ */
