
#include <Wall_App_Serial.h>
#include <stdbool.h>
#include <em_usart.h>
#include "em_timer.h"
#include <stdint.h>
#include "em_cmu.h"

#include "board_init.h"
#include <AppTimer.h>
#include <SwTimer.h>
#include "DebugPrintConfig.h"

#define DEBUGPRINT
#include "DebugPrint.h"
 #include "MCP23017_I2C.h"

//static char Rx_Buff[RX_BUFF_LEN] = {0};
#define RX_BUFF_LEN 256  // Or whatever length you need

static uint8_t Rx_Buff[RX_BUFF_LEN] = {0};  // Buffer initialized to zero

static uint16_t Rx_Buff_Len = 0;
//=================
uint8_t button_pressed_num = 0; // Global variable to track button presses

#define RE_PORT gpioPortF
#define RE_PIN 3
#define DE_PORT gpioPortF
#define DE_PIN 7
uint16_t count = 0;

SSwTimer learn_reset_timer;
void learn_reset_timer_CB();

void learn_reset_timer_CB()
{
    count = 0;
}
uint8_t txFrame1[] = {
    0x80, 0xBC, 0x81, 0x15, 0x82, 0x09, 0x83, 0x00,
    0x84, 0x01, 0x85, 0x91, 0x86, 0x00, 0x87, 0x81,
    0x48, 0x4E, 0x11
};
//

//uint8_t txFrame39[] = {
//    0x80, 0xBC, 0x81, 0x15, 0x82, 0x08, 0x83, 0x13, 0x84,
//    0x00, 0x85, 0x92, 0x86, 0x00, 0x87, 0x80, 0x88, 0x64,
//    0x49, 0x3B
//};
volatile uint16_t txIndex = 0; // Index to track the current position in txFrame
volatile uint16_t txFrameLength = sizeof(txFrame1) / sizeof(txFrame1[0]); // Length of txFrame
uint8_t *currentTxFrame; // Pointer to the current frame to transmit

// Function declarations
 void USART1_TX_IRQHandler(void);

 void USART1_TX_IRQHandler(void)
 {
  // DPRINTF("\n  Transmission completed");

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
  currentTxFrame = frame; // Set current frame to transmit

 // emberAfCorePrintln("TX Frame -> ");
  //int length = strlen(frame);
  for (int i = 0; i < txFrameLength; i++)
  {
 //  DPRINTF("%x ", currentTxFrame[i]);
    USART_Tx(USART1,currentTxFrame[i]);
  }
 //DPRINTF("\nTx Done");
}
// Function to start transmission of a specified frame

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
    DPRINT("hw_timer\n");

}
/*
void Serial_Init()
{
  DPRINT("Serial_Init");
    CMU_ClockEnable(cmuClock_USART1, true);

    USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
    init.baudrate = 19200;
    init.parity = usartEvenParity; // Set even parity

    // Set pin modes for USART TX and RX pins
    GPIO_PinModeSet(SERIAL_RX_PORT, SERIAL_RX_PIN, gpioModeInput, 0);   // Rx
    GPIO_PinModeSet(SERIAL_TX_PORT, SERIAL_TX_PIN, gpioModePushPull, 1); // Tx
    GPIO_PinModeSet(RE_PORT, RE_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(DE_PORT, DE_PIN, gpioModePushPull, 0);
    GPIO_PinOutSet(DE_PORT,DE_PIN);
    GPIO_PinOutSet(RE_PORT,RE_PIN);
    GPIO_PinOutClear(DE_PORT,DE_PIN);
    GPIO_PinOutClear(RE_PORT,RE_PIN);
    // Initialize USART asynchronous mode and route pins
    USART_InitAsync(USART1, &init);
    USART1->ROUTELOC0 = USART_ROUTELOC0_RXLOC_LOC20 | USART_ROUTELOC0_TXLOC_LOC20;
    USART1->ROUTEPEN |= USART_ROUTEPEN_TXPEN | USART_ROUTEPEN_RXPEN;

    // Initialize USART Interrupts
    USART_IntEnable(USART1, USART_IEN_RXDATAV);
    USART_IntEnable(USART1, USART_IEN_TXC);

    // Enable USART Interrupts
    NVIC_EnableIRQ(USART1_RX_IRQn);
    NVIC_EnableIRQ(USART1_TX_IRQn);

    Init_Hw_Timer();
    AppTimerRegister(&learn_reset_timer, true, learn_reset_timer_CB);
    DPRINT("cmpltd");

}*/
//===========

SSwTimer smooth_led_dim_timer;

uint8_t dim_target_value = 100;
uint8_t dim_current_value = 100;
uint32_t timeCount = 0;

uint8_t parameter_DIM_TIME = 23;
uint8_t parameter_MIN_LEVEL= 0;
uint8_t parameter_MAX_LEVEL= 100;

void smooth_led_dim_timer_CB()
{
 // DPRINTF("  smooth_led_dim_timer_CB\n");

  if(dim_target_value > dim_current_value)
  {
//      DPRINTF(" dim_target_value > dim_current_value \n");
//      DPRINTF(" dim_target_value-- %d\n",dim_target_value);
//      DPRINTF(" dim_current_value-- %d\n",dim_current_value);

    Set_Dim_levels(dim_current_value++);
    timeCount = 0;

  }
  else if(dim_target_value < dim_current_value)
  {
//      DPRINTF(" dim_current_value-- %d\n",dim_current_value);
//
//      DPRINTF(" dim_target_value < dim_current_value \n");
//      DPRINTF(" dim_target_value-- %d\n",dim_target_value);

    Set_Dim_levels(dim_current_value--);
    timeCount = 0;

  }
  else
  {
    //  DPRINTF("in timeCount == parameter_DIM_TIME*100 . \n");

    timeCount++;
    if(timeCount == parameter_DIM_TIME*100)//parameter_value_9*1000
    {
      dim_target_value = parameter_MIN_LEVEL;
    //  DPRINTF(" dim_target_value-- %d\n",dim_target_value);
 //     DPRINTF(" dim_current_value-- %d\n",dim_current_value);

    }

    if(dim_current_value == parameter_MIN_LEVEL)
    {
        //DPRINTF(" in else ... \n");
      //  DPRINTF(" dim_target_value-- %d\n",dim_target_value);

      TimerStop(&smooth_led_dim_timer);
      Set_Dim_levels(dim_current_value);
     // DPRINTF(" dim_current_value-- %d\n",dim_current_value);


    }

    if(timeCount%100 == 0)
    {
     // DPRINTF("D-%d-%d \n",parameter_DIM_TIME,timeCount/100);
    //  DPRINTF("in timecount . \n");

    }
  }
}



void set_proximity_level()
{
 // DPRINTF("  set_proximity_level\n");

  TimerStart(&smooth_led_dim_timer,10);
  dim_target_value =  parameter_MAX_LEVEL;
  timeCount = 0;
}

uint32_t triac_compare_val1_1;

void Set_Dim_levels(uint8_t Dim)
{
 // DPRINTF("  Set_Dim_levels\n");

  uint32_t max = 0xBE6E;
  triac_compare_val1_1 = (max * (uint32_t)(Dim)) / 100;
  TIMER_CompareSet(TIMER0, 1, triac_compare_val1_1);
}


void Dim_Init()
{
  DPRINTF("DIM INIT intialisation completed \n");

  CMU_ClockEnable(cmuClock_TIMER0, false);
  TIMER_Enable(TIMER0, false);
  GPIO_PinModeSet(PORT_DIM, PIN_DIM, gpioModePushPull, OFF);

//  GPIO_PinModeSet(PORT_DIM, PIN_DIM, gpioModePushPull, ON);

  // Enable clock for TIMER0 module
  CMU_ClockEnable(cmuClock_TIMER0, true);

  TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;
  timerCCInit.mode = timerCCModePWM;

  TIMER_InitCC(TIMER0, 1, &timerCCInit);  //output triac 1

  //Set First compare value fot Triac 1
  uint32_t max = 0xBE6E;//800Hz
  triac_compare_val1_1 = (max * (uint32_t)(100)) / 100;
  TIMER_CompareSet(TIMER0, 1, triac_compare_val1_1);

  //Set Top value for Timer (should be less then 20mSec)
  max = 0xBE6E;//800Hz
  TIMER_TopSet(TIMER0, max);

  TIMER0->ROUTELOC0 = (TIMER0->ROUTELOC0 & (~_TIMER_ROUTELOC0_CC1LOC_MASK)) | TIMER_ROUTELOC0_CC1LOC_LOC29;//PF6
  TIMER0->ROUTEPEN  = TIMER_ROUTEPEN_CC1PEN;

  // Initialize and start timer
  TIMER_Init_TypeDef timerInit_TIM_LEC = TIMER_INIT_DEFAULT;
  timerInit_TIM_LEC.prescale = timerPrescale1;

  TIMER_Init(TIMER0, &timerInit_TIM_LEC);
  TIMER_Enable(TIMER0, true);

  AppTimerRegister(&smooth_led_dim_timer, true, smooth_led_dim_timer_CB);
}
/*


// USART interrupt handler
void USART1_RX_IRQHandler(void) {
    uint32_t flags = USART_IntGet(USART1);
    USART_IntClear(USART1, flags);
    uint8_t data;

    // Start a timer (HW timer)
    TIMER_CounterSet(TIMER1, 1);
    TIMER_Enable(TIMER1, true);

    data = USART_Rx(USART1);
    Rx_Buff[Rx_Buff_Len] = data;  // Add the byte to the buffer
        Rx_Buff_Len++;

    // Store the received byte
//
}

// Timer interrupt handler
void TIMER1_IRQHandler(void) {
    uint32_t flags = TIMER_IntGet(TIMER1);
    TIMER_IntClear(TIMER1, flags);
    TIMER_Enable(TIMER1, false);


    Rx_Buff_Len = 0;        // Clear buffer length
}
*/

//===============================================================================

/*

#define FRAME_START 0x30    // Starting byte of the frame
#define FRAME_SIZE 75       // Maximum expected frame size
bool frame_started = false;

// USART interrupt handler
void USART1_RX_IRQHandler(void) {
    uint32_t flags = USART_IntGet(USART1);
    USART_IntClear(USART1, flags);
    uint8_t data;

    // Start a timer (HW timer)
    TIMER_CounterSet(TIMER1, 1);
    TIMER_Enable(TIMER1, true);

    data = USART_Rx(USART1);

    // Check for frame start (0x30)
    if (data == FRAME_START) {

        if (frame_started) {
            // If a frame has already started, reset the buffer
            DPRINTF("\n"); // Print newline before new frame starts
            Rx_Buff_Len = 0; // Reset the buffer length
        }
        frame_started = true; // Mark that a frame has started
    }

    // Store the received byte
    if (frame_started && Rx_Buff_Len < FRAME_SIZE) {
        Rx_Buff[Rx_Buff_Len++] = data; // Add the byte to the buffer
    }
}

// Timer interrupt handler
void TIMER1_IRQHandler(void) {
    uint32_t flags = TIMER_IntGet(TIMER1);
    TIMER_IntClear(TIMER1, flags);
    TIMER_Enable(TIMER1, false);

    // Ensure we have a frame with enough data
    if (Rx_Buff_Len > 0) {
        DPRINTF("\n");  // Printing the length in hex

        // Check if the frame starts with 0x30
        if (Rx_Buff[0] == FRAME_START) {

            // Check if 0x3C 0x00 exists at the expected positions
            if (Rx_Buff[11] == 0x3C && Rx_Buff[12] == 0x00) {

                // Extract the length from the group address field (at Rx_Buff[14] and Rx_Buff[15])
                uint8_t length = Rx_Buff[14];  // Length of the group addresses (hex)
//                if (length > 0) {
//                              parametersReceivedFromEts = true;
//                }
                // Print the length of group addresses
                DPRINTF("Group Address Length: 0x%.2X\n", length);  // Printing the length in hex
                extract_and_print_group_addresses(Rx_Buff, length,button_pressed_num);

                // Iterate over the group addresses (each group address is 2 bytes long)
              //  DPRINTF("Group Addresses: ");
                for (int i = 0; i < length; i++) {
                    // Each group address consists of two bytes: Rx_Buff[15 + 2*i] and Rx_Buff[16 + 2*i]
                    DPRINTF("0x%.2X%.2X ", Rx_Buff[15 + 2*i], Rx_Buff[16 + 2*i]);  // Printing each address pair in hex
             //   set_ets_parameters();
                }

                DPRINTF("\n");
            }
        }
     }

    // Reset buffer state for the next frame
    frame_started = false;  // Reset the frame started flag
    Rx_Buff_Len = 0;        // Clear buffer length
}



*/


#define FRAME_START 0x30    // Starting byte of the frame
#define FRAME_SIZE 75       // Maximum expected frame size
bool frame_started = false;
bool frame_started_1 = false;
#define FRAME_START_1 0xbc    // Starting byte of the frame

// USART interrupt handler
void USART1_RX_IRQHandler(void) {
    uint32_t flags = USART_IntGet(USART1);
    USART_IntClear(USART1, flags);
    uint8_t data;

    // Start a timer (HW timer)
    TIMER_CounterSet(TIMER1, 1);
    TIMER_Enable(TIMER1, true);

    data = USART_Rx(USART1);

    // Check for frame start (0x30)
    if (data == FRAME_START) {

        if (frame_started) {
            // If a frame has already started, reset the buffer
            DPRINTF("\n"); // Print newline before new frame starts
            Rx_Buff_Len = 0; // Reset the buffer length
        }
        frame_started = true; // Mark that a frame has started
    }
    if (data == FRAME_START_1) {
          if (frame_started_1) {
              // If a frame has already started, reset the buffer
              DPRINTF("\n"); // Print newline before new frame starts
              Rx_Buff_Len = 0; // Reset the buffer length
          }
          frame_started_1 = true; // Mark that a frame has started
      }
    // Store the received byte
    if (frame_started && Rx_Buff_Len < FRAME_SIZE) {
        Rx_Buff[Rx_Buff_Len++] = data; // Add the byte to the buffer
    }
    if (frame_started_1 && Rx_Buff_Len < FRAME_SIZE) {
         Rx_Buff[Rx_Buff_Len++] = data; // Add the byte to the buffer
     }
}

// Timer interrupt handler
void TIMER1_IRQHandler(void) {
    uint32_t flags = TIMER_IntGet(TIMER1);
    TIMER_IntClear(TIMER1, flags);
    TIMER_Enable(TIMER1, false);

    // Ensure we have a frame with enough data
/*    if (Rx_Buff_Len > 0) {
        DPRINTF("\n");  // Printing the length in hex

        // Check if the frame starts with 0x30
        if (Rx_Buff[0] == FRAME_START) {

            // Check if 0x3C 0x00 exists at the expected positions
            if (Rx_Buff[11] == 0x3C && Rx_Buff[12] == 0x00) {

                // Extract the length from the group address field (at Rx_Buff[14] and Rx_Buff[15])
                uint8_t length = Rx_Buff[14];  // Length of the group addresses (hex)
//                if (length > 0) {
//                              parametersReceivedFromEts = true;
//                }
                // Print the length of group addresses
                DPRINTF("Group Address Length: 0x%.2X\n", length);  // Printing the length in hex
                extract_and_print_group_addresses(Rx_Buff, length,button_pressed_num);

                // Iterate over the group addresses (each group address is 2 bytes long)
              //  DPRINTF("Group Addresses: ");
                for (int i = 0; i < length; i++) {
                    // Each group address consists of two bytes: Rx_Buff[15 + 2*i] and Rx_Buff[16 + 2*i]
                    DPRINTF("0x%.2X%.2X ", Rx_Buff[15 + 2*i], Rx_Buff[16 + 2*i]);  // Printing each address pair in hex
             //   set_ets_parameters();
                }

                DPRINTF("\n");
            }
        }
        if(Rx_Buff[0] == FRAME_START_1)
               {
                 for (int i = 0; i < 9; i++) {
                           DPRINTF("%.2x ", Rx_Buff[i]);
                       }
                       DPRINTF("\n");
                 uint8_t switchState = Rx_Buff[4];  // Store the 4th index value (0-based index)
                 uint8_t onOffState = Rx_Buff[7];
                 uint8_t switchIndex = getSwitchIndexBasedOnDestSubVal(switchState);
                 if (switchIndex > 0) {

                        if (onOffState == 0x80) {
                            set_proximity_level();

                                    Set_Led(switchIndex, 0);  // Turn LED off

                                  DPRINTF("switch: %.2x is turned off\n",switchIndex);

                                } else if (onOffState == 0x81) {
                                    set_proximity_level();

                                     Set_Led(switchIndex, 1);  // Turn LED off

                                    DPRINTF("switch: %.2x is turned on\n",switchIndex);

                                }
                 }
                 // Extract dest1 and dest2 from Rx_Buff
                 uint8_t dest1 = Rx_Buff[3];  // Third byte
                 uint8_t dest2 = Rx_Buff[4];  // Fourth byte

                 // Calculate group address
                 uint16_t group_address = (dest1 << 8) | dest2;

                 // Extract dest_main, dest_middle, and dest_sub from group_address
                 uint8_t dest_main = (group_address >> 11) & 0x1F;  // Top 5 bits
                 uint8_t dest_middle = (group_address >> 8) & 0x07; // Next 3 bits
                 uint8_t dest_sub = group_address & 0xFF;           // Lower 8 bits

                 // Use the function to check for a match
                 uint8_t switchIndex = checkApplicationDataMatch(dest_main, dest_middle, dest_sub);

                 // Determine LED state based on onOffState
                 uint8_t onOffState = Rx_Buff[7];  // On/Off state from the frame

                 if (switchIndex > 0) {
                     set_proximity_level();

                     // Set LED based on onOffState
                     Set_Led(switchIndex, (onOffState == 0x81) ? 1 : 0);
                   //  update_switch_status(switchIndex, (onOffState == 0x81) ? true : false);

                     DPRINTF("Match found. Setting LED %d %s.\n", switchIndex, (onOffState == 0x81) ? "ON" : "OFF");
                 } else {
                     DPRINTF("No match found for dest_main: %d, dest_middle: %d, dest_sub: %d.\n", dest_main, dest_middle, dest_sub);
                 }

               }
     }*/
    frame_started_1 = false;  // Reset the frame started flag

    // Reset buffer state for the next frame
    frame_started = false;  // Reset the frame started flag
    Rx_Buff_Len = 0;        // Clear buffer length
}




