
#include <Wall_App_Serial.h>
#include <stdbool.h>
#include <em_usart.h>
#include "em_timer.h"
#include <stdint.h>
#include "em_cmu.h"
#include <string.h>
#include "board_init.h"
#include <AppTimer.h>
#include <math.h>
#include <SwTimer.h>
#include "DebugPrintConfig.h"
#include "board_config.h"
//#define DEBUGPRINT
#include "DebugPrint.h"


#define RX_BUFF_LEN 256  // Or whatever length you need
static uint8_t Rx_Buff[RX_BUFF_LEN] = {0};  // Buffer initialized to zero
static uint8_t Rx_Buff_Len = 0;
uint8_t rotary_rx_addr = 0;
bool addr_config_mode = false, enable_addr_mode = false;
SApplicationData ApplicationData;
uint8_t nvm_data_buff[5] = {0};
bool last_relay_state[4] = {false, false, false, false};
bool on_off_state = false;
bool is_bidirectional_command = false;
void process_rs485_rx_data();
//=================
 void USART1_TX_IRQHandler(void)
 {
   //DPRINTF("\n  Transmission completed");

     uint32_t flags;
     flags = USART_IntGet(USART1);
     USART_IntClear(USART1, flags);
     if (flags & USART_IF_TXC)
     {
        // DPRINTF("\nTransmission completed");
     }
     GPIO_PinOutClear(DE_PORT, DE_PIN);
     GPIO_PinOutClear(RE_PORT, RE_PIN);
 }
void sendFrame(uint8_t *frame)
{
//  DPRINTF("\nTx start");
  GPIO_PinOutSet(DE_PORT,DE_PIN);
  GPIO_PinOutSet(RE_PORT,RE_PIN);

  for (int i = 0; i < FRAME_SIZE; i++)
  {
    DPRINTF("%x ", frame[i]);
    USART_Tx(USART1,frame[i]);
  }
 DPRINTF("\nTx Done\n");
}


void Init_Hw_Timer()
{
    // Enable TIMER1
    CMU_ClockEnable(cmuClock_TIMER1, true);

    // Set the PWM frequency
    TIMER_TopSet(TIMER1, 0xffff);

    // Create the timer count control object initializer
    TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
    timerCCInit.mode = timerCCModeCompare;

    // Configure CC channel 0
    TIMER_InitCC(TIMER1, 0, &timerCCInit);

    // Initialize and start timer with defined prescale
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    timerInit.prescale = timerPrescale2;
    TIMER_Init(TIMER1, &timerInit);

    TIMER_IntEnable(TIMER1, TIMER_IEN_CC0);
    NVIC_EnableIRQ(TIMER1_IRQn);

}

void Serial_Init()
{
  DPRINT("Serial_Init\n");
  CMU_ClockEnable(cmuClock_USART1, true);

  USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
  init.baudrate = 9600;

  // Set pin modes for USART TX and RX pins
  GPIO_PinModeSet(SERIAL_RX_PORT, SERIAL_RX_PIN, gpioModeInput, 0);   // Rx
  GPIO_PinModeSet(SERIAL_TX_PORT, SERIAL_TX_PIN, gpioModePushPull, 1); // Tx
  GPIO_PinModeSet(RE_PORT, RE_PIN, gpioModePushPull, 0);
  GPIO_PinModeSet(DE_PORT, DE_PIN, gpioModePushPull, 0);

  GPIO_PinOutClear(DE_PORT,DE_PIN);
  GPIO_PinOutClear(RE_PORT,RE_PIN);
  // Initialize USART asynchronous mode and route pins
  USART_InitAsync(USART1, &init);
  USART1->ROUTELOC0 = USART_ROUTELOC0_RXLOC_LOC12 | USART_ROUTELOC0_TXLOC_LOC16;
  USART1->ROUTEPEN |= USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;

  // Initialize USART Interrupts
  USART_IntEnable(USART1, USART_IEN_RXDATAV);
  USART_IntEnable(USART1, USART_IEN_TXC);

  // Enable USART Interrupts
  NVIC_EnableIRQ(USART1_RX_IRQn);
  NVIC_EnableIRQ(USART1_TX_IRQn);

  Init_Hw_Timer();

}

// USART interrupt handler
void USART1_RX_IRQHandler(void) {
    uint32_t flags = USART_IntGet(USART1);
    USART_IntClear(USART1, flags);
    uint8_t data;

    // Start a timer (HW timer)
    TIMER_CounterSet(TIMER1, 1);
    TIMER_Enable(TIMER1, true);

    data = (uint8_t) USART_Rx(USART1);
   // DPRINTF("%02X ", data);
    if (Rx_Buff_Len < RX_BUFF_LEN) // Ensure no overflow
    {
        Rx_Buff[Rx_Buff_Len++] = data;
        //DPRINTF("%02X ", data);
    }
    else
    {
        DPRINTF("Rx_Buff overflow");
    }

}

// Timer interrupt handler
void TIMER1_IRQHandler(void) {
    uint32_t flags = TIMER_IntGet(TIMER1);
    TIMER_IntClear(TIMER1, flags);
    TIMER_Enable(TIMER1, false);
//    for(int i=0; i<Rx_Buff_Len; i++)
//      {
//        DPRINTF("%02X ",Rx_Buff[i]);
//      }
//    DPRINT("\n");
    process_rs485_rx_data();
    Rx_Buff_Len = 0;        // Clear buffer length
}

// === CRC16 MODBUS for uint8_t buffer ===
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

bool relay_states[4] = {false, false, false, false};


void Set_Value(uint8_t button_no, uint8_t on_off_value)
{
    uint8_t relay_pins[4]  = {APP_RELAY_1_PIN, APP_RELAY_2_PIN, APP_RELAY_3_PIN, APP_RELAY_4_PIN};
    uint8_t relay_ports[4] = {APP_RELAY_1_PORT, APP_RELAY_2_PORT, APP_RELAY_3_PORT, APP_RELAY_4_PORT};
    uint8_t relay_ids[4]   = {nvm_data_buff[1], nvm_data_buff[2], nvm_data_buff[3], nvm_data_buff[4]};
    bool* relay_states_ptrs[4] = {
        &ApplicationData.relay1State,
        &ApplicationData.relay2State,
        &ApplicationData.relay3State,
        &ApplicationData.relay4State
    };

    if (button_no == 0xFF) {
        // Turn all OFF
        for (int i = 0; i < 4; i++) {
            if (last_relay_state[i]) { // only act if currently ON
                GPIO_PinOutClear(relay_ports[i], relay_pins[i]);
                relay_states[i] = false;
                *relay_states_ptrs[i] = false;
                last_relay_state[i] = false;
                DPRINTF("RELAY%d OFF (All OFF)\n", i + 1);
            }
        }
        on_off_state = 0;
        DPRINT("\n All Relays are Turning OFF \n");
        writeAppData(&ApplicationData);
        return;
    }

    for (int i = 0; i < 4; i++) {
        if (button_no == relay_ids[i]) {
            bool new_state;

            if (is_bidirectional_command && on_off_value != 0xFF) {
                // Bidirectional explicit ON/OFF
                new_state = (on_off_value == 1);
                DPRINTF("Explicit cmd: Button %d, desired=%s\n", i+1, new_state ? "ON" : "OFF");
            } else {
                // Toggle mode
                new_state = !last_relay_state[i];
                DPRINTF("Toggle cmd: Button %d, desired=%s\n", i+1, new_state ? "ON" : "OFF");
            }

            // Apply only if state actually changed
            if (new_state != last_relay_state[i]) {
                if (new_state) {
                    GPIO_PinOutSet(relay_ports[i], relay_pins[i]);
                    DPRINTF("RELAY%d ON\n", i + 1);
                } else {
                    GPIO_PinOutClear(relay_ports[i], relay_pins[i]);
                    DPRINTF("RELAY%d OFF\n", i + 1);
                }

                relay_states[i] = new_state;
                *relay_states_ptrs[i] = new_state;
                last_relay_state[i] = new_state;
                on_off_state = new_state;
                writeAppData(&ApplicationData);
            } else {
                DPRINTF("RELAY%d unchanged (already %s)\n",
                        i + 1, new_state ? "ON" : "OFF");
            }
            break;
        }
    }
}

typedef struct{
  uint8_t keypad_Id;
  uint8_t load1_Button;
  uint8_t load2_Button;
  uint8_t load3_Button;
  uint8_t load4_Button;
}rx_addr_config_t;
rx_addr_config_t rx_data;

void timeout_config_addr_CB()
{
  //{keypad_Id, 0x45, load1_Button, load2_Button, load3_Button, load4_Button, crc1, crc2};
  if(addr_config_mode && enable_addr_mode)
  {
      DPRINT("\n Address Configuration Done\n");
      ApplicationData.KeypadID = nvm_data_buff[0] = rx_data.keypad_Id;
      ApplicationData.Relay1BtnNo = nvm_data_buff[1] = rx_data.load1_Button;
      ApplicationData.Relay2BtnNo = nvm_data_buff[2] = rx_data.load2_Button;
      ApplicationData.Relay3BtnNo = nvm_data_buff[3] = rx_data.load3_Button;
      ApplicationData.Relay4BtnNo = nvm_data_buff[4] = rx_data.load4_Button;
      writeAppData(&ApplicationData);

      //{keypad_Id, 0x46, 0x46, 0x46, 0x46, 0x46, crc1, crc2}; // Address Rx Confirm Config Command
      uint8_t configuration_save_ack[8] = {rx_data.keypad_Id, 0x46, 0x46, 0x46, 0x46, 0x46, 0x00, 0x00};
      uint16_t crc = modbus_crc16(configuration_save_ack, 6);
      configuration_save_ack[6] = crc & 0xFF;
      configuration_save_ack[7] = (crc >> 8) & 0xFF;

      //for(uint8_t i=0;i<5;i++){
          sendFrame(configuration_save_ack);//Ack 1 time
      //}
      DPRINT("\n Addr Save Ack Sent\n");
      TimerStop(&timeout_config_addr);

      DPRINT("\n=======Configured Data=========\n");
      DPRINTF("\nKeypad ID : %d\n", rx_data.keypad_Id);
      DPRINTF("\nRelay-1 Button : %d\n", rx_data.load1_Button);
      DPRINTF("\nRelay-2 Button : %d\n", rx_data.load2_Button);
      DPRINTF("\nRelay-3 Button : %d\n", rx_data.load3_Button);
      DPRINTF("\nRelay-4 Button : %d\n", rx_data.load4_Button);
      DPRINT("\n==================================\n");
  }
  else
  {
      DPRINT("\n Address Configuration Not Done/Enabled\n");
  }
  addr_config_mode = enable_addr_mode = false;
  memset(Rx_Buff, 0, Rx_Buff_Len);
}

void process_rs485_rx_data()
{
/*04 03 00 02 13 01 28 AF -B1
04 03 00 02 14 02 6A 9E -B2

02 03 00 02 08 02 62 38 -B2
02 03 00 02 09 04 E3 AA -B3
02 03 00 02 0A 08 E3 5F -B4
02 03 00 02 0B 10 E2 C5 -B5
02 03 00 02 0C 20 E0 E1 -B6*/

  if (Rx_Buff[0] == nvm_data_buff[0] && Rx_Buff[1] == 0x03 && Rx_Buff[3] == 0x02) // Action Command
    {
      DPRINTF("\n====Action Command\n");
      //check CRC valid or not
      uint16_t crc = modbus_crc16(Rx_Buff, 6);
      uint8_t crc1 = crc & 0xFF;
      uint8_t crc2 = (crc >> 8) & 0xFF;
      if(crc1 == Rx_Buff[6] && crc2 == Rx_Buff[7])
      {
          is_bidirectional_command = false; // toggle
          uint8_t btn_val = 0;
          if(Rx_Buff[5] == 0x01) btn_val = 1;
          if(Rx_Buff[5] == 0x02) btn_val = 2;
          if(Rx_Buff[5] == 0x04) btn_val = 3;
          if(Rx_Buff[5] == 0x08) btn_val = 4;
          if(Rx_Buff[5] == 0x10) btn_val = 5;
          if(Rx_Buff[5] == 0x20) btn_val = 6;
          if(Rx_Buff[5] == 0x40) btn_val = 7;
          if(Rx_Buff[5] == 0x80) btn_val = 8;
          Set_Value(btn_val, 0xFF);
      }
      else
        {
          DPRINT("\n======Action: CRC valid Error!======\n");
        }

    }
  //Bidirectional Single button : {keypad_Id, 0x47, 0x47, keypad_btn_no, on/off, 0x47, crc1, crc2};
  //Bidirectional All OFF button : {keypad_Id, 0x47, 0x47, 0xFF, on/off, 0x47, crc1, crc2};
  else if (Rx_Buff[0] == nvm_data_buff[0] && Rx_Buff[1] == 0x47 && Rx_Buff[5] == 0x47) // Bidirectional Action Command
    {
      DPRINTF("\n====Bidirectional Action Command\n");
      uint16_t crc = modbus_crc16(Rx_Buff, 6);
      uint8_t crc1 = crc & 0xFF;
      uint8_t crc2 = (crc >> 8) & 0xFF;
      if(crc1 == Rx_Buff[6] && crc2 == Rx_Buff[7])
      {
          is_bidirectional_command = true;
          uint8_t button_no = Rx_Buff[3];
          uint8_t on_off_value = Rx_Buff[4];  // ON/OFF value in frame

          for(int i=0;i<8;i++)
              DPRINTF("%02X ", Rx_Buff[i]);
          DPRINT("\n");

          Set_Value(button_no, on_off_value);
      }
      else
      {
          DPRINT("\n======Bidirectional Action: CRC valid Error!======\n");
      }

    }
  //{keypad_Id, 0x45, load1_Button, load2_Button, load3_Button, load4_Button, crc1, crc2};
  else if(Rx_Buff[1] == 0x45) // Address Rx Config Command
    {
        DPRINT("\n Ready for Address Configuration \n");
        //check CRC valid or not
        uint16_t crc = modbus_crc16(Rx_Buff, 6);
        uint8_t crc1 = crc & 0xFF;
        uint8_t crc2 = (crc >> 8) & 0xFF;
        if(crc1 == Rx_Buff[6] && crc2 == Rx_Buff[7])
        {
            rx_data.keypad_Id = Rx_Buff[0];
            rx_data.load1_Button = Rx_Buff[2];
            rx_data.load2_Button = Rx_Buff[3];
            rx_data.load3_Button = Rx_Buff[4];
            rx_data.load4_Button = Rx_Buff[5];
            enable_addr_mode = true;
            TimerStart(&timeout_config_addr,60000);
        }
        else
          {
            DPRINT("\nConfig: ======CRC valid Error!======\n");
          }
    }
  //{keypad_Id, 0x46, 0x46, 0x46, 0x46, 0x46, crc1, crc2}; // Address Rx Confirm Config Command
  else if(Rx_Buff[1] == 0x46 && Rx_Buff[2] == 0x46 && Rx_Buff[3] == 0x46 && Rx_Buff[4] == 0x46 && Rx_Buff[5])
    {
      DPRINT("\n Rx Address Configuration Not Done/Enabled\n");
      uint16_t crc = modbus_crc16(Rx_Buff, 6);
      uint8_t crc1 = crc & 0xFF;
      uint8_t crc2 = (crc >> 8) & 0xFF;
      if(crc1 == Rx_Buff[6] && crc2 == Rx_Buff[7])
      {
          TimerStop(&timeout_config_addr);
          memset(Rx_Buff, 0, Rx_Buff_Len);
          addr_config_mode = enable_addr_mode = false;
      }
      else
        {
          DPRINT("\n======Config Confirm: CRC valid Error!======\n");
        }
    }
}


