

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/
#include <board_config.h>
#include "DebugPrintConfig.h"
//#define DEBUGPRINT
#include "events.h"
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

SSwTimer timeout_config_addr;
SSwTimer button_press_timer;
SSwTimer led_toggle_timer;
SSwTimer retain_relay_states_timer;
uint32_t sw_1_cur_pos;

bool set_red_led = false;
uint8_t toggle_cnt = 0;
uint8_t on_cnt = 0;
uint8_t btn_press_cnt = 0;
void led_toggle_timer_CB()
{
  if(!set_red_led){
    if(toggle_cnt < 10){
        GPIO_PinOutToggle(LED2_PORT, LED2_PIN);
        toggle_cnt++;
        TimerStart(&led_toggle_timer, 500);
    }
    else if(toggle_cnt == 10)
      {
        TimerStop(&led_toggle_timer);
        toggle_cnt = 0;
      }
  }
  else if(set_red_led)
    {
      GPIO_PinOutClear(LED1_PORT, LED1_PIN);
      set_red_led = false;
    }
}

void button_press_timer_CB()
{
  DPRINTF("\n== Button Pressed : %d times==\n", btn_press_cnt);
  if(btn_press_cnt == 1)  //Save Addr Config
    {
      addr_config_mode = true;
      timeout_config_addr_CB();
      TimerStart(&led_toggle_timer, 100);
    }
  if(btn_press_cnt == 4)  //Soft Reset or NVM clear
    {
      DPRINTF("\n== lEARN mODE==\n");
      ZAF_EventHelperEventEnqueueFromISR(EVENT_APP_LEARNMODE);
      TimerStart(&led_toggle_timer, 100);


//      GPIO_PinOutSet(LED1_PORT, LED1_PIN);
//      set_red_led = true;
//      TimerStart(&led_toggle_timer, 5000);
//      AppResetNvm();
    }
  if(btn_press_cnt >= 8)  //Hard Reset
    {
      ZAF_EventHelperEventEnqueueFromISR(EVENT_APP_HARDRESET);
    }
    btn_press_cnt = 0;
}

void Switch_1_CB(uint8_t pin)
{

  UNUSED(pin);
  if(sw_1_cur_pos != GPIO_PinInGet(BUTTON_PORT,BUTTON_PIN))
    {
      sw_1_cur_pos = GPIO_PinInGet(BUTTON_PORT,BUTTON_PIN);

      if(false == GPIO_PinInGet(BUTTON_PORT,BUTTON_PIN))
      {
        ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW1_DOWN);
        return;
      }
      else
     {
       ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW1_UP);
       return;
     }
    }

}

void retain_relay_states_timer_CB()
{
  if(relay_states[0]) GPIO_PinOutSet(APP_RELAY_1_PORT, APP_RELAY_1_PIN);
  else if(!relay_states[0]) GPIO_PinOutClear(APP_RELAY_1_PORT, APP_RELAY_1_PIN);
  if(relay_states[1]) GPIO_PinOutSet(APP_RELAY_2_PORT, APP_RELAY_2_PIN);
  else if(!relay_states[1]) GPIO_PinOutClear(APP_RELAY_2_PORT, APP_RELAY_2_PIN);
  if(relay_states[2]) GPIO_PinOutSet(APP_RELAY_3_PORT, APP_RELAY_3_PIN);
  else if(!relay_states[2]) GPIO_PinOutClear(APP_RELAY_3_PORT, APP_RELAY_3_PIN);
  if(relay_states[3]) GPIO_PinOutSet(APP_RELAY_4_PORT, APP_RELAY_4_PIN);
  else if(!relay_states[3]) GPIO_PinOutClear(APP_RELAY_4_PORT, APP_RELAY_4_PIN);
  DPRINT("\n== Done Retaining State ==\n");
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
  GPIO_PinModeSet(UART0_TX_PORT, UART0_TX_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(UART0_RX_PORT, UART0_RX_PIN, gpioModePushPull, 0);
#endif

}

uint32_t Board_HW_Init(void)                  //GPIO INIT
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(BUTTON_PORT, BUTTON_PIN, gpioModeInputPullFilter, 1);

  GPIO_PinModeSet(LED1_PORT, LED1_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(LED2_PORT, LED2_PIN, gpioModePushPull, 0);

  GPIO_PinModeSet(APP_RELAY_1_PORT, APP_RELAY_1_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(APP_RELAY_2_PORT, APP_RELAY_2_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(APP_RELAY_3_PORT, APP_RELAY_3_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(APP_RELAY_4_PORT, APP_RELAY_4_PIN, gpioModePushPull, 0);

  /* Unlatch EM4 GPIO pin states after wakeup (OK to call even if not EM4 wakeup) */
  EMU_UnlatchPinRetention();

  /* Save the EM4 GPIO wakeup flags */
  g_gpioEm4Flags = GPIO_IntGet() & _GPIO_IF_EM4WU_MASK;
  GPIO_IntClear(g_gpioEm4Flags);

  //Register the timers
  AppTimerRegister(&led_toggle_timer, false, led_toggle_timer_CB);
  AppTimerRegister(&button_press_timer, false, button_press_timer_CB);
  AppTimerRegister(&timeout_config_addr, false, timeout_config_addr_CB);


  GPIOINT_Init();
  NVIC_SetPriority(GPIO_ODD_IRQn, 5);
  NVIC_SetPriority(GPIO_EVEN_IRQn, 5);

  Serial_Init();
  GPIO_ExtIntConfig(BUTTON_PORT, BUTTON_PIN, BUTTON_PIN, true, true, true);
  GPIOINT_CallbackRegister(BUTTON_PIN, Switch_1_CB);
  GPIO_PinOutSet(LED1_PORT, LED1_PIN);
  set_red_led = true;
  TimerStart(&led_toggle_timer, 5000);
//  retain_relayStates();
  //Initialize_Relay_States();
  AppTimerRegister(&retain_relay_states_timer, false, retain_relay_states_timer_CB);
  TimerStart(&retain_relay_states_timer, 500);
  return 0;
}
