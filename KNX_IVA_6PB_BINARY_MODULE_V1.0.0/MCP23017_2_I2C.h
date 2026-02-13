
#ifndef SRC_MCP23017_2_I2C_H_
#define SRC_MCP23017_2_I2C_H_

#include <stdint.h>

//{0 1 0 0 A2 A1 A0 R/W}
#define MCP23017_ADDRESS_2    0x4E // A2 = A1 = A0 = 1 [0 1 0 0 1 1 1 0] = 0x4E



//  Registers                         //  DESCRIPTION                  DATASHEET
//Defined in MCP23017_I2C.h
/*
#define APP_RELAY_1_PORT	  MCP23017_GPIO_B
#define APP_RELAY_1_PIN	    1

#define APP_RELAY_2_PORT	  MCP23017_GPIO_B
#define APP_RELAY_2_PIN	    5

#define APP_RELAY_3_PORT	  MCP23017_GPIO_B
#define APP_RELAY_3_PIN	    0

#define APP_RELAY_4_PORT	  MCP23017_GPIO_B
#define APP_RELAY_4_PIN	    3

#define APP_RELAY_5_PORT    MCP23017_GPIO_B
#define APP_RELAY_5_PIN     4

#define APP_RELAY_6_PORT    MCP23017_GPIO_B
#define APP_RELAY_6_PIN     2

#define APP_RELAY_7_PORT    MCP23017_GPIO_A
#define APP_RELAY_7_PIN     7

#define APP_RELAY_8_PORT    MCP23017_GPIO_A
#define APP_RELAY_8_PIN     6

*/
void MCP23017_2_Init();
void MCP23017_2_GPIO_PinOutSet(uint8_t port, uint8_t pin, uint8_t value);
void MCP23017_2_GPIO_PinOutToggle(uint8_t port, uint8_t pin);
uint8_t MCP23017_2_ReadInputs_PortB(void);
uint8_t MCP23017_2_ReadInput_PinB(uint8_t pin);
//#define APP_RELAY_1_PORT    MCP23017_GPIO_B
//#define APP_RELAY_1_PIN     3
//
//#define APP_RELAY_2_PORT    MCP23017_GPIO_B
//#define APP_RELAY_2_PIN     4
//
//#define APP_RELAY_3_PORT    MCP23017_GPIO_B
//#define APP_RELAY_3_PIN     5
//
//#define APP_RELAY_4_PORT    MCP23017_GPIO_B
//#define APP_RELAY_4_PIN     6
//
//#define APP_RELAY_5_PORT    MCP23017_GPIO_B
//#define APP_RELAY_5_PIN     7
//
//#define APP_RELAY_6_PORT    MCP23017_GPIO_A
//#define APP_RELAY_6_PIN     3
//
//#define APP_RELAY_7_PORT    MCP23017_GPIO_A
//#define APP_RELAY_7_PIN     0
//
//#define APP_RELAY_8_PORT    MCP23017_GPIO_A
//#define APP_RELAY_8_PIN     1
//
//#define APP_RELAY_7_PORT1    MCP23017_GPIO_A
//#define APP_RELAY_7_PIN1     7
//
//#define APP_RELAY_8_PORT1    MCP23017_GPIO_A
//#define APP_RELAY_8_PIN1    6



#define APP_RELAY_1_PORT    gpioPortB
#define APP_RELAY_1_PIN     13

#define APP_RELAY_2_PORT    gpioPortC
#define APP_RELAY_2_PIN     11

#define APP_RELAY_3_PORT    gpioPortF
#define APP_RELAY_3_PIN     7

#define APP_RELAY_4_PORT    gpioPortD
#define APP_RELAY_4_PIN     9

#define APP_RELAY_5_PORT    gpioPortD
#define APP_RELAY_5_PIN     10

#define APP_RELAY_6_PORT    gpioPortD
#define APP_RELAY_6_PIN     13

#define APP_RELAY_7_PORT    gpioPortD
#define APP_RELAY_7_PIN     14

#define APP_RELAY_8_PORT    gpioPortD
#define APP_RELAY_8_PIN     15
//=============

//========================
#ifndef PB7_GPIO_PORT
#define PB7_GPIO_PORT                            gpioPortA
#endif
#ifndef PB7_GPIO_PIN
#define PB7_GPIO_PIN                             7
#endif

#ifndef PB8_GPIO_PORT
#define PB8_GPIO_PORT                            gpioPortA
#endif
#ifndef PB8_GPIO_PIN
#define PB8_GPIO_PIN                             6
#endif

#define PB7_INT_NO       7
#define PB8_INT_NO       6

//======================
#define TASK_STACK_SIZE_GPIO_STATE_PRINT 1000  // [bytes]

void InitGpioStatePrintTask(void);
void GpioStatePrintTask(void *pvParameters);
//void MCP23017_2_GetPinStates(uint8_t *pin7State, uint8_t *pin8State);
void MCP23017_2_GetPinStates();

//#define DEFINE_SW  0x8
//typedef enum
//{
//  EVENT_EMPTY =DEFINE_SW,
//
//  BOARD_BUTTON_PB7,
//  BOARD_BUTTON_PB8,
//
//} button_id1_t;
////==============
void MCP23017_2_writeReg(uint8_t reg, uint8_t value);

//======================
#endif /* SRC_MCP23017_I2C_H_ */
