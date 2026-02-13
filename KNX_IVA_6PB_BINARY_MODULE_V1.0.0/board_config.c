

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <board_config.h>
#include "DebugPrintConfig.h"
#define DEBUGPRINT

#include "DebugPrint.h"

#include <stdint.h>

#include "stdbool.h"

#include <ZW_application_transport_interface.h>

#include "zaf_event_helper.h"

#include <AppTimer.h>

#include "gpiointerrupt.h"

#include "em_timer.h"
#include "em_emu.h"
#include "SizeOf.h"
#include "em_cmu.h"
#include "MCP23017_I2C.h"
#include "MCP23017_2_I2C.h"

 #include "Wall_App_Serial.h"

//#include "MCP23017_2_I2C.h"

/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/

//Disabling USART0 saves much power when an application is battery powered
//#define DISABLE_USART0



#ifdef APP_DEBUG
static uint8_t m_aDebugPrintBuffer[96];
#endif
/**
 * @brief GPIO EM4 wakeup flags (bitmask containing button(s) that woke us up from EM4)
 */
uint32_t g_gpioEm4Flags = 0;



/**
 * @brief counter to increment for every 100ms to do factory reset of the device
 */
uint8_t hcount = 0;
uint8_t hcount_1 = 0;

/**
 *  @brief Timer to hatd reset the device from the gateway
 */
SSwTimer switch_6_timer;
SSwTimer switch_5_timer;
SSwTimer sw_timer;

uint32_t sw_1_cur_pos;
uint32_t sw_2_cur_pos;
uint32_t sw_3_cur_pos;
uint32_t sw_4_cur_pos;
uint32_t sw_5_cur_pos;
uint32_t sw_6_cur_pos;
uint32_t sw_7_cur_pos;
uint32_t sw_8_cur_pos;

//==========
void switch_6_timer_CB()
{
  DPRINT("CBT2\n");
  hcount = 0;
}
void switch_5_timer_CB()
{
  DPRINT("CBT4\n");
  hcount_1 = 0;
}
void Sw_CB()
{

  if(sw_1_cur_pos != GPIO_PinInGet(PORT_PB1,PIN_PB1))
  {
    sw_1_cur_pos = GPIO_PinInGet(PORT_PB1,PIN_PB1);

    if(false == GPIO_PinInGet(PORT_PB1,PIN_PB1)) // Button pressed
    {

        // Fire DOWN event immediately on press
        ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW1_DOWN);
    }
    else if(true == GPIO_PinInGet(PORT_PB1,PIN_PB1)) // Button released
    {

        // Fire UP event regardless of hold
        ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW1_UP);

    }
  }


  if(sw_2_cur_pos != GPIO_PinInGet(PORT_PB2,PIN_PB2))
  {
    sw_2_cur_pos = GPIO_PinInGet(PORT_PB2,PIN_PB2);

    if(false == GPIO_PinInGet(PORT_PB2,PIN_PB2)) // Button pressed
    {

        ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW2_DOWN);
    }
    else if(true == GPIO_PinInGet(PORT_PB2,PIN_PB2)) // Button released
    {

        ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW2_UP);

    }
  }


  if(sw_3_cur_pos != GPIO_PinInGet(PORT_PB3,PIN_PB3))
  {
    sw_3_cur_pos = GPIO_PinInGet(PORT_PB3,PIN_PB3);

    if(false == GPIO_PinInGet(PORT_PB3,PIN_PB3)) // Button pressed
    {

        ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW3_DOWN);
    }
    else if(true == GPIO_PinInGet(PORT_PB3,PIN_PB3)) // Button released
    {

        ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW3_UP);

    }
  }

  if(sw_4_cur_pos != GPIO_PinInGet(PORT_PB4,PIN_PB4))
  {
    sw_4_cur_pos = GPIO_PinInGet(PORT_PB4,PIN_PB4);

    if(false == GPIO_PinInGet(PORT_PB4,PIN_PB4)) // Button pressed
    {


        ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW4_DOWN);
    }
    else if(true == GPIO_PinInGet(PORT_PB4,PIN_PB4)) // Button released
    {

        ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW4_UP);

    }
  }
  if(sw_5_cur_pos != GPIO_PinInGet(PORT_PB5,PIN_PB5))
  {
    sw_5_cur_pos = GPIO_PinInGet(PORT_PB5,PIN_PB5);

    if(false == GPIO_PinInGet(PORT_PB5,PIN_PB5))
    {
        {
                                hcount_1++;
                                 {

                                  TimerStart(&switch_5_timer,1000);
                                }
              }
      ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW5_DOWN);
    }
    else
    {
      ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW5_UP);
    }
  }

  if(sw_6_cur_pos != GPIO_PinInGet(PORT_PB6,PIN_PB6))
  {
    sw_6_cur_pos = GPIO_PinInGet(PORT_PB6,PIN_PB6);

    if(false == GPIO_PinInGet(PORT_PB6,PIN_PB6))
    {
        {
                    hcount++;
                     {
                      TimerStart(&switch_6_timer,1000);
                    }

                  }
      ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW6_DOWN);
    }
    else
    {
      ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW6_UP);
    }
  }
//  if(sw_5_cur_pos != GPIO_PinInGet(PORT_PB5,PIN_PB5))
//  {
//    sw_5_cur_pos = GPIO_PinInGet(PORT_PB5,PIN_PB5);
//
//    if(false == GPIO_PinInGet(PORT_PB5,PIN_PB5)) // Button pressed
//    {
//
//        // Fire DOWN event immediately on press
//        ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW5_DOWN);
//    }
//    else if(true == GPIO_PinInGet(PORT_PB5,PIN_PB5)) // Button released
//    {
//
//        // Fire UP event regardless of hold
//        ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW5_UP);
//
//
//    }
//  }
//
//  if(sw_6_cur_pos != GPIO_PinInGet(PORT_PB6,PIN_PB6))
//  {
//    sw_6_cur_pos = GPIO_PinInGet(PORT_PB6,PIN_PB6);
//
//    if(false == GPIO_PinInGet(PORT_PB6,PIN_PB6)) // Button pressed
//    {                                hcount++;
//
//
//        // Fire DOWN event immediately on press
//        ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW6_DOWN);
//    }
//    else if(true == GPIO_PinInGet(PORT_PB6,PIN_PB6)) // Button released
//    {
//
//        // Fire UP event regardless of hold
//        ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW6_UP);
//
//
//    }
//  }
  if(sw_7_cur_pos != GPIO_PinInGet(PORT_PB7,PIN_PB7))
  {
      sw_7_cur_pos = GPIO_PinInGet(PORT_PB7,PIN_PB7);
    if(false == GPIO_PinInGet(PORT_PB7,PIN_PB7))
    {

      ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW7_DOWN);
      //GPIO_PinOutClear(APP_RELAY_7_PORT, APP_RELAY_7_PIN);
    }
    else
    {
      ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW7_UP);
      //GPIO_PinOutSet(APP_RELAY_7_PORT, APP_RELAY_7_PIN);
    }
  }
  if(sw_8_cur_pos != GPIO_PinInGet(PORT_PB8,PIN_PB8))
  {
      sw_8_cur_pos = GPIO_PinInGet(PORT_PB8,PIN_PB8);
    if(false == GPIO_PinInGet(PORT_PB8,PIN_PB8))
    {

      ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW8_DOWN);
      //GPIO_PinOutClear(APP_RELAY_8_PORT, APP_RELAY_8_PIN);
    }
    else
    {
      ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW8_UP);
      //GPIO_PinOutSet(APP_RELAY_8_PORT, APP_RELAY_8_PIN);
    }
  }

// if(hcount == 6)//include/exclude event
//  {
//    hcount = 0;
//    DPRINTF("IN LEARN MODE \n");
//    ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_INCLUSION);
//  }
////  if(hcount_1 == 14)//include/exclude event
////   {
////     hcount_1 = 0;
////     Set_Led(1,1);
////     Set_Led(2,1);
////     Set_Led(3,1);
////     Set_Led(4,1);
////     Set_Led(5,1);
////     Set_Led(6,1);
////     Set_Led(7,1);
////     Set_Led(8,1);
////     Set_Led(1,0);
////       Set_Led(2,0);
////       Set_Led(3,0);
////       Set_Led(4,0);
////       Set_Led(5,0);
////       Set_Led(6,0);
////       Set_Led(7,0);
////       Set_Led(8,0);
////     AppResetNvm();
////     // ZAF_EventHelperEventEnqueueFromISR(EVENT_SYSTEM_RESET);
////   }
///
  if(hcount == 12)//include/exclude event



  {
    hcount = 0;
    DPRINTF("IN LEARN MODE \n");
    ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_INCLUSION);
  }
  if(hcount_1 == 18)//include/exclude event
   {
     hcount_1 = 0;
     Set_Led(1,1);
     Set_Led(2,1);
     Set_Led(3,1);
     Set_Led(4,1);
     Set_Led(5,1);
     Set_Led(6,1);
     Set_Led(7,1);
     Set_Led(8,1);
     Set_Led(1,0);
       Set_Led(2,0);
       Set_Led(3,0);
       Set_Led(4,0);
       Set_Led(5,0);
       Set_Led(6,0);
       Set_Led(7,0);
       Set_Led(8,0);
     AppResetNvm();
     // ZAF_EventHelperEventEnqueueFromISR(EVENT_SYSTEM_RESET);
   }
}
void Sensor_interrupt_CB(uint8_t pin)
{
UNUSED(pin);


//  if(pin == 11)//Sensor Intr Pin is 0
//  {
    if(!GPIO_PinInGet(PORT_SENSOR_INT, PIN_SENSOR_INT))
    {
        DPRINTF("sensor_intr %d  HIGH",pin);
      set_proximity_level();
    }
  //}
}
void Sw_interrupt_CB(uint8_t id)
{
  UNUSED(id);

  TimerStart(&sw_timer,100);
}



//Board HW init
void Init_HW()
{
  CMU_ClockEnable(cmuClock_GPIO, true);



#if defined(APP_DEBUG)
  ZAF_UART0_init(UART0_TX_PORT, UART0_TX_PIN, UART0_RX_PORT, UART0_RX_PIN);
  ZAF_UART0_enable(115200, true, false);
  DebugPrintConfig(m_aDebugPrintBuffer, sizeof(m_aDebugPrintBuffer), ZAF_UART0_tx_send);
#else
  GPIO_PinModeSet(UART0_TX_PORT, UART0_TX_PIN, OutputMode, 0);
  GPIO_PinModeSet(UART0_RX_PORT, UART0_RX_PIN, OutputMode, 0);
#endif

  GPIO_PinModeSet(PORT_PB1, PIN_PB1, InputMode, 1);
  GPIO_PinModeSet(PORT_PB2, PIN_PB2, InputMode, 1);
  GPIO_PinModeSet(PORT_PB3, PIN_PB3, InputMode, 1);
  GPIO_PinModeSet(PORT_PB4, PIN_PB4, gpioModeInputPull, 0);
  GPIO_PinModeSet(PORT_PB5, PIN_PB5, InputMode, 1);
  GPIO_PinModeSet(PORT_PB6, PIN_PB6, gpioModeInputPull, 0);


  GPIO_PinModeSet(PORT_DIM, PIN_DIM, InputMode, 1);
//  GPIO_PinModeSet(PORT_SENSOR_INT, PIN_SENSOR_INT, InputMode, 1);

}
uint32_t Board_HW_Init(void)                  //GPIO INIT
{
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(PORT_PB1, PIN_PB1, InputMode, 1);
  GPIO_PinModeSet(PORT_PB2, PIN_PB2, InputMode, 1);
  GPIO_PinModeSet(PORT_PB3, PIN_PB3, InputMode, 1);
  GPIO_PinModeSet(PORT_PB4, PIN_PB4, InputMode, 1);
  GPIO_PinModeSet(PORT_PB5, PIN_PB5, InputMode, 1);
  GPIO_PinModeSet(PORT_PB6, PIN_PB6, InputMode, 1);
  GPIO_PinModeSet(PORT_PB7, PIN_PB7, InputMode, 1);
  GPIO_PinModeSet(PORT_PB8, PIN_PB8, InputMode, 1);
  //GPIO_PinModeSet(PORT_SENSOR_INT, PIN_SENSOR_INT, InputMode, 1);
  GPIO_PinModeSet(PORT_DIM, PIN_DIM, OutputMode, 0);

  /* Unlatch EM4 GPIO pin states after wakeup (OK to call even if not EM4 wakeup) */
  EMU_UnlatchPinRetention();

  /* Save the EM4 GPIO wakeup flags */
  g_gpioEm4Flags = GPIO_IntGet() & _GPIO_IF_EM4WU_MASK;
  GPIO_IntClear(g_gpioEm4Flags);

  //Register the timers
  AppTimerRegister(&switch_5_timer, false, switch_5_timer_CB);
  AppTimerRegister(&switch_6_timer, false, switch_6_timer_CB);

  AppTimerRegister(&sw_timer, false, Sw_CB);

  GPIOINT_Init();
  NVIC_SetPriority(GPIO_ODD_IRQn, 5);
  NVIC_SetPriority(GPIO_EVEN_IRQn, 5);

  //Get the switch positions
  sw_1_cur_pos = GPIO_PinInGet(PORT_PB1,PIN_PB1);
  sw_2_cur_pos = GPIO_PinInGet(PORT_PB2,PIN_PB2);
  sw_3_cur_pos = GPIO_PinInGet(PORT_PB3,PIN_PB3);
  sw_4_cur_pos = GPIO_PinInGet(PORT_PB4,PIN_PB4);
  sw_5_cur_pos = GPIO_PinInGet(PORT_PB5,PIN_PB5);
  sw_6_cur_pos = GPIO_PinInGet(PORT_PB6,PIN_PB6);
  sw_7_cur_pos = GPIO_PinInGet(PORT_PB7,PIN_PB7);
  sw_8_cur_pos = GPIO_PinInGet(PORT_PB8,PIN_PB8);



  GPIO_ExtIntConfig(PORT_PB1, PIN_PB1, PIN_PB1, true, true, true);
  GPIOINT_CallbackRegister(PIN_PB1, Sw_interrupt_CB);//3 -> 2

  GPIO_ExtIntConfig(PORT_PB2, PIN_PB2, PIN_PB2, true, true, true);
  GPIOINT_CallbackRegister(PIN_PB2, Sw_interrupt_CB);//8

  GPIO_ExtIntConfig(PORT_PB3, PIN_PB3, PIN_PB3, true, true, true);
  GPIOINT_CallbackRegister(PIN_PB3, Sw_interrupt_CB);//9

  GPIO_ExtIntConfig(PORT_PB4, PIN_PB4, PIN_PB4, true, true, true);
  GPIOINT_CallbackRegister(PIN_PB4, Sw_interrupt_CB);//10

  GPIO_ExtIntConfig(PORT_PB5, PIN_PB5, 2, true, true, true);
  GPIOINT_CallbackRegister(2, Sw_interrupt_CB);//3

  GPIO_ExtIntConfig(PORT_PB6, PIN_PB6, PIN_PB6, true, true, true);
  GPIOINT_CallbackRegister(PIN_PB6, Sw_interrupt_CB);//4

//
  GPIO_ExtIntConfig(PORT_PB7, PIN_PB7, PIN_PB7, true, true, true);
  GPIOINT_CallbackRegister(PIN_PB7, Sw_interrupt_CB);//5

  GPIO_ExtIntConfig(PORT_PB8, PIN_PB8, PIN_PB8, true, true, true);
  GPIOINT_CallbackRegister(PIN_PB8, Sw_interrupt_CB);//6

  //GPIO_ExtIntConfig(PORT_LED_G, PIN_LED_G, PIN_LED_G, true, true, true);
  //GPIOINT_CallbackRegister(PIN_LED_G, Sw_interrupt_CB);//11
  //Serial_Init();

  Dim_Init();
  interface_iic_init();
  MCP23017_Init();
  GPIO_PinModeSet(PORT_DIM, PIN_DIM, OutputMode, 0);
  GPIO_PinModeSet(PORT_SENSOR_INT, PIN_SENSOR_INT, gpioModeInputPull, 1);

  GPIO_PinModeSet(APP_RELAY_1_PORT, APP_RELAY_1_PIN, OutputMode, 1);
  GPIO_PinModeSet(APP_RELAY_2_PORT, APP_RELAY_2_PIN, OutputMode, 1);
  GPIO_PinModeSet(APP_RELAY_3_PORT, APP_RELAY_3_PIN, OutputMode, 1);
  GPIO_PinModeSet(APP_RELAY_4_PORT, APP_RELAY_4_PIN, OutputMode, 1);
  GPIO_PinModeSet(APP_RELAY_5_PORT, APP_RELAY_5_PIN, OutputMode, 1);
  GPIO_PinModeSet(APP_RELAY_6_PORT, APP_RELAY_6_PIN, OutputMode, 1);
  GPIO_PinModeSet(APP_RELAY_7_PORT, APP_RELAY_7_PIN, OutputMode, 1);
  GPIO_PinModeSet(APP_RELAY_8_PORT, APP_RELAY_8_PIN, OutputMode, 1);


  MCP23017_2_Init();

  //GPIO_PinModeSet(PORT_SENSOR_INT, PIN_SENSOR_INT, gpioModeInputPullFilter, 0);
  GPIO_ExtIntConfig(PORT_SENSOR_INT, PIN_SENSOR_INT, PIN_SENSOR_INT, true, true, true);
  GPIOINT_CallbackRegister(PIN_SENSOR_INT, Sensor_interrupt_CB);
  //app_apds9960_init();

  return 0;
}
//uint32_t Board_HW_Init(void)                  //GPIO INIT
//{
//  CMU_ClockEnable(cmuClock_GPIO, true);
//
//  GPIO_PinModeSet(PORT_PB1, PIN_PB1, InputMode, 1);
//  GPIO_PinModeSet(PORT_PB2, PIN_PB2, InputMode, 1);
//  GPIO_PinModeSet(PORT_PB3, PIN_PB3, InputMode, 1);
//  GPIO_PinModeSet(PORT_PB4, PIN_PB4, InputMode, 1);
//  GPIO_PinModeSet(PORT_PB5, PIN_PB5, InputMode, 1);
//  GPIO_PinModeSet(PORT_PB6, PIN_PB6, InputMode, 1);
////  GPIO_PinModeSet(PORT_PB7, PIN_PB7, InputMode, 1);
////  GPIO_PinModeSet(PORT_PB8, PIN_PB8, InputMode, 1);
//  //GPIO_PinModeSet(PORT_SENSOR_INT, PIN_SENSOR_INT, InputMode, 1);
//  GPIO_PinModeSet(PORT_DIM, PIN_DIM, OutputMode, 0);
//
//  /* Unlatch EM4 GPIO pin states after wakeup (OK to call even if not EM4 wakeup) */
//  EMU_UnlatchPinRetention();
//
//  /* Save the EM4 GPIO wakeup flags */
//  g_gpioEm4Flags = GPIO_IntGet() & _GPIO_IF_EM4WU_MASK;
//  GPIO_IntClear(g_gpioEm4Flags);
//
//
//
//  AppTimerRegister(&sw_timer, false, Sw_CB);
//
//  GPIOINT_Init();
//  NVIC_SetPriority(GPIO_ODD_IRQn, 5);
//  NVIC_SetPriority(GPIO_EVEN_IRQn, 5);
//
//  //Get the switch positions
//  sw_1_cur_pos = GPIO_PinInGet(PORT_PB1,PIN_PB1);
//  sw_2_cur_pos = GPIO_PinInGet(PORT_PB2,PIN_PB2);
//  sw_3_cur_pos = GPIO_PinInGet(PORT_PB3,PIN_PB3);
//  sw_4_cur_pos = GPIO_PinInGet(PORT_PB4,PIN_PB4);
//  sw_5_cur_pos = GPIO_PinInGet(PORT_PB5,PIN_PB5);
//  sw_6_cur_pos = GPIO_PinInGet(PORT_PB6,PIN_PB6);
////  sw_7_cur_pos = GPIO_PinInGet(PORT_PB7,PIN_PB7);
////  sw_8_cur_pos = GPIO_PinInGet(PORT_PB8,PIN_PB8);
//
//
//
//  GPIO_ExtIntConfig(PORT_PB1, PIN_PB1, PIN_PB1, true, true, true);
//  GPIOINT_CallbackRegister(PIN_PB1, Sw_interrupt_CB);//3 -> 2
//
//  GPIO_ExtIntConfig(PORT_PB2, PIN_PB2, PIN_PB2, true, true, true);
//  GPIOINT_CallbackRegister(PIN_PB2, Sw_interrupt_CB);//8
//
//  GPIO_ExtIntConfig(PORT_PB3, PIN_PB3, PIN_PB3, true, true, true);
//  GPIOINT_CallbackRegister(PIN_PB3, Sw_interrupt_CB);//9
//
//  GPIO_ExtIntConfig(PORT_PB4, PIN_PB4, PIN_PB4, true, true, true);
//  GPIOINT_CallbackRegister(PIN_PB4, Sw_interrupt_CB);//10
//
//  GPIO_ExtIntConfig(PORT_PB5, PIN_PB5, PIN_PB5, true, true, true);
//  GPIOINT_CallbackRegister(PIN_PB5, Sw_interrupt_CB);//3
//
//  GPIO_ExtIntConfig(PORT_PB6, PIN_PB6, PIN_PB6, true, true, true);
//  GPIOINT_CallbackRegister(PIN_PB6, Sw_interrupt_CB);//4
//
////
////  GPIO_ExtIntConfig(PORT_PB7, PIN_PB7, PIN_PB7, true, true, true);
////  GPIOINT_CallbackRegister(PIN_PB7, Sw_interrupt_CB);//5
////
////  GPIO_ExtIntConfig(PORT_PB8, PIN_PB8, PIN_PB8, true, true, true);
////  GPIOINT_CallbackRegister(PIN_PB8, Sw_interrupt_CB);//6
//
//  //GPIO_ExtIntConfig(PORT_LED_G, PIN_LED_G, PIN_LED_G, true, true, true);
//  //GPIOINT_CallbackRegister(PIN_LED_G, Sw_interrupt_CB);//11
//  //Serial_Init();
//
//  Dim_Init();
//  interface_iic_init();
//  MCP23017_Init();
//  GPIO_PinModeSet(PORT_DIM, PIN_DIM, OutputMode, 0);
// // GPIO_PinModeSet(RELAY_8_PORT, RELAY_8_PIN, gpioModePushPull, 0);
//  GPIO_PinModeSet(PORT_SENSOR_INT, PIN_SENSOR_INT, gpioModeInputPull, 1);
//
//  MCP23017_2_Init();
//
//  //GPIO_PinModeSet(PORT_SENSOR_INT, PIN_SENSOR_INT, gpioModeInputPullFilter, 0);
//  GPIO_ExtIntConfig(PORT_SENSOR_INT, PIN_SENSOR_INT, PIN_SENSOR_INT, true, true, true);
//  GPIOINT_CallbackRegister(PIN_SENSOR_INT, Sensor_interrupt_CB);
//  //app_apds9960_init();
// // GPIO_PinModeSet(RELAY_PORT_1, RELAY_PIN_1, OutputMode, 0);
//
//  return 0;
//}
