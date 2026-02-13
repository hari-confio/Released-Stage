
#ifndef SRC_MCP23017_I2C_H_
#define SRC_MCP23017_I2C_H_

#include <stdint.h>

#define MCP23017_ADDRESS      0x40


//PC6 (SCL)
#define MCP23017_SCL_PORT     gpioPortC
#define MCP23017_SCL_PIN      6

//PC7 (SDA)
#define MCP23017_SDA_PORT     gpioPortC
#define MCP23017_SDA_PIN      7

#define MCP23017_PIN_ON   0
#define MCP23017_PIN_OFF  1

//  Registers                         //  DESCRIPTION                  DATASHEET
#define MCP23017_DDR_A        0x00    //  Data Direction Register A       P18
#define MCP23017_DDR_B        0x01    //  Data Direction Register B       P18
#define MCP23017_POL_A        0x02    //  Input Polarity A                P18
#define MCP23017_POL_B        0x03    //  Input Polarity B                P18
#define MCP23017_GPINTEN_A    0x04    //  NOT USED interrupt enable       P19
#define MCP23017_GPINTEN_B    0x05    //  NOT USED
#define MCP23017_DEFVAL_A     0x06    //  NOT USED interrupt def          P19
#define MCP23017_DEFVAL_B     0x07    //  NOT USED
#define MCP23017_INTCON_A     0x08    //  NOT USED interrupt control      P20
#define MCP23017_INTCON_B     0x09    //  NOT USED
#define MCP23017_IOCR         0x0A    //  IO control register             P20
#define MCP23017_IOCR2        0x0B    //  NOT USED
#define MCP23017_PUR_A        0x0C    //  Pull Up Resistors A             P22
#define MCP23017_PUR_B        0x0D    //  Pull Up Resistors A             P22
#define MCP23017_INTF_A       0x0E    //  NOT USED interrupt flag         P22
#define MCP23017_INTF_B       0x0F    //  NOT USED
#define MCP23017_INTCAP_A     0x10    //  NOT USED interrupt capture      P23
#define MCP23017_INTCAP_B     0x11    //  NOT USED
#define MCP23017_GPIO_A       0x12    //  General Purpose IO A            P23
#define MCP23017_GPIO_B       0x13    //  General Purpose IO B            P23
#define MCP23017_OLAT_A       0x14    //  NOT USED output latch           P24
#define MCP23017_OLAT_B       0x15    //  NOT USED



#define APP_LED_ACTIVE_WHITE    (1)
#define APP_LED_ACTIVE_YELLOW   (0)

#if APP_LED_ACTIVE_WHITE && APP_LED_ACTIVE_YELLOW
#error "Both Yellow and White are active"
#endif



#if APP_LED_ACTIVE_WHITE
  #define APP_LED_1_ON_PORT MCP23017_GPIO_B
  #define APP_LED_1_ON_PIN  0
  #define APP_LED_1_OFF_PORT  MCP23017_GPIO_B
  #define APP_LED_1_OFF_PIN 1

  #define APP_LED_2_ON_PORT MCP23017_GPIO_B
  #define APP_LED_2_ON_PIN  2
  #define APP_LED_2_OFF_PORT  MCP23017_GPIO_B
  #define APP_LED_2_OFF_PIN 3


  #define APP_LED_3_ON_PORT MCP23017_GPIO_B
  #define APP_LED_3_ON_PIN  4
  #define APP_LED_3_OFF_PORT  MCP23017_GPIO_B
  #define APP_LED_3_OFF_PIN 5


  #define APP_LED_4_ON_PORT MCP23017_GPIO_B
  #define APP_LED_4_ON_PIN  6
  #define APP_LED_4_OFF_PORT  MCP23017_GPIO_B
  #define APP_LED_4_OFF_PIN 7


  #define APP_LED_5_ON_PORT MCP23017_GPIO_A
  #define APP_LED_5_ON_PIN  7
  #define APP_LED_5_OFF_PORT  MCP23017_GPIO_A
  #define APP_LED_5_OFF_PIN 6


  #define APP_LED_6_ON_PORT MCP23017_GPIO_A
  #define APP_LED_6_ON_PIN  5
  #define APP_LED_6_OFF_PORT  MCP23017_GPIO_A
  #define APP_LED_6_OFF_PIN 4

  #define APP_LED_7_ON_PORT MCP23017_GPIO_A
  #define APP_LED_7_ON_PIN  3
  #define APP_LED_7_OFF_PORT  MCP23017_GPIO_A
  #define APP_LED_7_OFF_PIN 2

  #define APP_LED_8_ON_PORT MCP23017_GPIO_A
  #define APP_LED_8_ON_PIN  1
  #define APP_LED_8_OFF_PORT  MCP23017_GPIO_A
  #define APP_LED_8_OFF_PIN 0
#endif

//A0 A1 A2 A3
#if APP_LED_ACTIVE_YELLOW
  #define APP_LED_1_OFF_PORT MCP23017_GPIO_B
  #define APP_LED_1_OFF_PIN  0
  #define APP_LED_1_ON_PORT  MCP23017_GPIO_B
  #define APP_LED_1_ON_PIN 1

  #define APP_LED_2_OFF_PORT MCP23017_GPIO_B
  #define APP_LED_2_OFF_PIN  2
  #define APP_LED_2_ON_PORT  MCP23017_GPIO_B
  #define APP_LED_2_ON_PIN 3


  #define APP_LED_3_OFF_PORT MCP23017_GPIO_B
  #define APP_LED_3_OFF_PIN  4
  #define APP_LED_3_ON_PORT  MCP23017_GPIO_B
  #define APP_LED_3_ON_PIN 5


  #define APP_LED_4_OFF_PORT MCP23017_GPIO_B
  #define APP_LED_4_OFF_PIN  6
  #define APP_LED_4_ON_PORT  MCP23017_GPIO_B
  #define APP_LED_4_ON_PIN 7


  #define APP_LED_5_OFF_PORT MCP23017_GPIO_A
  #define APP_LED_5_OFF_PIN  7
  #define APP_LED_5_ON_PORT  MCP23017_GPIO_A
  #define APP_LED_5_ON_PIN 6


  #define APP_LED_6_OFF_PORT MCP23017_GPIO_A
  #define APP_LED_6_OFF_PIN  5
  #define APP_LED_6_ON_PORT  MCP23017_GPIO_A
  #define APP_LED_6_ON_PIN 4

  #define APP_LED_7_OFF_PORT MCP23017_GPIO_A
  #define APP_LED_7_OFF_PIN  3
  #define APP_LED_7_ON_PORT  MCP23017_GPIO_A
  #define APP_LED_7_ON_PIN 2

  #define APP_LED_8_OFF_PORT MCP23017_GPIO_A
  #define APP_LED_8_OFF_PIN  1
  #define APP_LED_8_ON_PORT  MCP23017_GPIO_A
  #define APP_LED_8_ON_PIN 0
#endif




void Set_Relay(uint8_t led,uint8_t value);
void Set_Led(uint8_t led,uint8_t value);

void MCP23017_Init();
//void MCP23017_Send_Byte(uint8_t data);
void MCP23017_GPIO_PinOutSet(uint8_t port, uint8_t pin, uint8_t value);
void MCP23017_GPIO_PinOutToggle(uint8_t port, uint8_t pin);


#endif /* SRC_MCP23017_I2C_H_ */
