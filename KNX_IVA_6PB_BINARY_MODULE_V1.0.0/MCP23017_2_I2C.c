
#define DEBUGPRINT
#include "DebugPrint.h"
//#include "app/framework/include/af.h"
#include "MCP23017_I2C.h"
#include "MCP23017_2_I2C.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "driver_interface.h"
#include "gpiointerrupt.h"
#include "ZW_UserTask.h" // Include the header where ZW_UserTask_* is defined
#include <events.h>
#include <board_config.h>

#include"FreeRTOS.h"
#include "task.h"
#include <assert.h>
#define TASK_STACK_SIZE_GPIO_STATE_PRINT 1000  // [bytes]

static TaskHandle_t m_xTaskHandleGpioStatePrint = NULL;

// Task and stack buffer allocation for the GPIO state print task
static StaticTask_t GpioStatePrintTaskBuffer;
static uint8_t GpioStatePrintStackBuffer[TASK_STACK_SIZE_GPIO_STATE_PRINT];

//void delay_ms(uint32_t ms) {
//    uint32_t count = 0;
//    while (count < ms * 1000) {
//        count++;
//    }
//}

//Keeping local ram variable to avoid i2c read operation
static uint8_t LOCAL_2_REG_PORT_A = 0x00;
static uint8_t LOCAL_2_REG_PORT_B = 0x00;


uint8_t MCP23017_2_readReg(uint8_t reg) {
    I2C_TransferSeq_TypeDef i2cTransfer;
    I2C_TransferReturn_TypeDef result;
    uint8_t i2c_rxBuffer[1] = {0};

    i2cTransfer.addr          = MCP23017_ADDRESS_2;
    i2cTransfer.flags         = I2C_FLAG_WRITE_READ;
    i2cTransfer.buf[0].data   = &reg;
    i2cTransfer.buf[0].len    = 1;
    i2cTransfer.buf[1].data   = i2c_rxBuffer;
    i2cTransfer.buf[1].len    = 1;
    result = I2C_TransferInit(I2C0, &i2cTransfer);

    while (result == i2cTransferInProgress) {
        result = I2C_Transfer(I2C0);
    }
    //DPRINTF("MCP23017_2_readReg.... %02X  Done   %02X\n",reg, i2c_rxBuffer[0]);
     // if(i2c_rxBuffer[0] != 0)
     //   {
         // DPRINTF("MCP23017_2_readReg.... %02X  Done   %02X\n",reg, i2c_rxBuffer[0]);
      //  }
    return i2c_rxBuffer[0];
}

uint8_t relayLed1State, relayLed2State, relayLed3State, relayLed4State, relayLed5State, relayLed6State, relayLed7State, relayLed8State;
//void set_led_with_BinaryModule(uint8_t relay, uint8_t NewState)
//{
//  if (relay == 1)
//  {
//      if (relayLed1State == NewState)
//      {
//          DPRINTF("led-1:%u\n", NewState);
//      }
//  }
//   if (relay == 2)
//  {
//      if (relayLed2State == NewState)
//      {
//          DPRINTF("led-2:%u\n", NewState);
//      }
//  }
//   if (relay == 3)
//  {
//      if (relayLed3State == NewState)
//      {
//          DPRINTF("led-3:%u\n", NewState);
//      }
//  }
//   if (relay == 4)
//  {
//      if (relayLed4State == NewState)
//      {
//          DPRINTF("led-4:%u\n", NewState);
//      }
//  }
//   if (relay == 5)
//  {
//      if (relayLed5State == NewState)
//      {
//          DPRINTF("led-5:%u\n", NewState);
//      }
//  }
//   if (relay == 6)
//  {
//      if (relayLed6State == NewState)
//      {
//          DPRINTF("led-6:%u\n", NewState);
//      }
//  }
//   if (relay == 7)
//  {
//      if (relayLed7State == NewState)
//      {
//          DPRINTF("led-7:%u\n", NewState);
//      }
//  }
//   if (relay == 8)
//  {
//      if (relayLed8State == NewState)
//      {
//          DPRINTF("led-8:%u\n\n", NewState);
//      }
//  }
//
//}

void set_led_with_BinaryModule(uint8_t relay, uint8_t NewState)
{
    uint8_t* currentState = NULL;

    switch (relay) {
        case 1: currentState = &relayLed1State; break;
        case 2: currentState = &relayLed2State; break;
        case 3: currentState = &relayLed3State; break;
        case 4: currentState = &relayLed4State; break;
        case 5: currentState = &relayLed5State; break;
        case 6: currentState = &relayLed6State; break;
        case 7: currentState = &relayLed7State; break;
        case 8: currentState = &relayLed8State; break;
        default: return;
    }

    if (*currentState != NewState) {
        *currentState = NewState;
        DPRINTF("Binary LED-%u:%u\n", relay, NewState);
        set_proximity_level();
        if(relay == 1)
          {
            DPRINTF("----LED-1---\n");
            if(NewState == 1)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_1_ON_PORT, APP_LED_1_ON_PIN, MCP23017_PIN_ON);
                MCP23017_GPIO_PinOutSet(APP_LED_1_OFF_PORT, APP_LED_1_OFF_PIN, MCP23017_PIN_OFF);
              }
            else if(NewState == 0)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_1_ON_PORT, APP_LED_1_ON_PIN, MCP23017_PIN_OFF);
                MCP23017_GPIO_PinOutSet(APP_LED_1_OFF_PORT, APP_LED_1_OFF_PIN, MCP23017_PIN_ON);
              }
          }
        else if(relay == 2)
          {
            DPRINTF("----LED-2---\n");
            if(NewState == 1)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_2_ON_PORT, APP_LED_2_ON_PIN, MCP23017_PIN_ON);
                MCP23017_GPIO_PinOutSet(APP_LED_2_OFF_PORT, APP_LED_2_OFF_PIN, MCP23017_PIN_OFF);
              }
            else if(NewState == 0)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_2_ON_PORT, APP_LED_2_ON_PIN, MCP23017_PIN_OFF);
                MCP23017_GPIO_PinOutSet(APP_LED_2_OFF_PORT, APP_LED_2_OFF_PIN, MCP23017_PIN_ON);
              }
          }
        else if(relay == 3)
          {
            DPRINTF("----LED-3---\n");
            if(NewState == 1)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_3_ON_PORT, APP_LED_3_ON_PIN, MCP23017_PIN_ON);
                MCP23017_GPIO_PinOutSet(APP_LED_3_OFF_PORT, APP_LED_3_OFF_PIN, MCP23017_PIN_OFF);
              }
            else if(NewState == 0)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_3_ON_PORT, APP_LED_3_ON_PIN, MCP23017_PIN_OFF);
                MCP23017_GPIO_PinOutSet(APP_LED_3_OFF_PORT, APP_LED_3_OFF_PIN, MCP23017_PIN_ON);
              }
          }
        else if(relay == 4)
          {
            DPRINTF("----LED-4---\n");
            if(NewState == 1)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_4_ON_PORT, APP_LED_4_ON_PIN, MCP23017_PIN_ON);
                MCP23017_GPIO_PinOutSet(APP_LED_4_OFF_PORT, APP_LED_4_OFF_PIN, MCP23017_PIN_OFF);
              }
            else if(NewState == 0)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_4_ON_PORT, APP_LED_4_ON_PIN, MCP23017_PIN_OFF);
                MCP23017_GPIO_PinOutSet(APP_LED_4_OFF_PORT, APP_LED_4_OFF_PIN, MCP23017_PIN_ON);
              }
          }
        else if(relay == 5)
          {
            DPRINTF("----LED-5---\n");
            if(NewState == 1)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_5_ON_PORT, APP_LED_5_ON_PIN, MCP23017_PIN_ON);
                MCP23017_GPIO_PinOutSet(APP_LED_5_OFF_PORT, APP_LED_5_OFF_PIN, MCP23017_PIN_OFF);
              }
            else if(NewState == 0)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_5_ON_PORT, APP_LED_5_ON_PIN, MCP23017_PIN_OFF);
                MCP23017_GPIO_PinOutSet(APP_LED_5_OFF_PORT, APP_LED_5_OFF_PIN, MCP23017_PIN_ON);
              }
          }
        else if(relay == 6)
          {
            DPRINTF("----LED-6---\n");
            if(NewState == 1)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_6_ON_PORT, APP_LED_6_ON_PIN, MCP23017_PIN_ON);
                MCP23017_GPIO_PinOutSet(APP_LED_6_OFF_PORT, APP_LED_6_OFF_PIN, MCP23017_PIN_OFF);
              }
            else if(NewState == 0)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_6_ON_PORT, APP_LED_6_ON_PIN, MCP23017_PIN_OFF);
                MCP23017_GPIO_PinOutSet(APP_LED_6_OFF_PORT, APP_LED_6_OFF_PIN, MCP23017_PIN_ON);
              }
          }
        else if(relay == 7)
          {
            DPRINTF("----LED-7---\n");
            if(NewState == 1)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_7_ON_PORT, APP_LED_7_ON_PIN, MCP23017_PIN_ON);
                MCP23017_GPIO_PinOutSet(APP_LED_7_OFF_PORT, APP_LED_7_OFF_PIN, MCP23017_PIN_OFF);
              }
            else if(NewState == 0)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_7_ON_PORT, APP_LED_7_ON_PIN, MCP23017_PIN_OFF);
                MCP23017_GPIO_PinOutSet(APP_LED_7_OFF_PORT, APP_LED_7_OFF_PIN, MCP23017_PIN_ON);
              }
          }
        else if(relay == 8)
          {
            DPRINTF("----LED-8---\n");
            if(NewState == 1)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_8_ON_PORT, APP_LED_8_ON_PIN, MCP23017_PIN_ON);
                MCP23017_GPIO_PinOutSet(APP_LED_8_OFF_PORT, APP_LED_8_OFF_PIN, MCP23017_PIN_OFF);
              }
            else if(NewState == 0)
              {
                MCP23017_GPIO_PinOutSet(APP_LED_8_ON_PORT, APP_LED_8_ON_PIN, MCP23017_PIN_OFF);
                MCP23017_GPIO_PinOutSet(APP_LED_8_OFF_PORT, APP_LED_8_OFF_PIN, MCP23017_PIN_ON);
              }
          }
    }
}

//uint8_t MCP23017_2_readReg(uint8_t reg)
//{
//  I2C_TransferSeq_TypeDef i2cTransfer;
//  I2C_TransferReturn_TypeDef result;
//
//  //uint8_t i2c_txBuffer[1] = {reg};
//  //uint8_t i2c_txBufferSize = sizeof(i2c_txBuffer);
//  uint8_t i2c_rxBuffer[1] = {0};
//  //uint8_t i2c_rxBufferIndex;
//
//  // Initializing I2C transfer
//  i2cTransfer.addr          = MCP23017_ADDRESS_2;
//  i2cTransfer.flags         = I2C_FLAG_WRITE_READ;
//  i2cTransfer.buf[0].data   = &reg;
//  i2cTransfer.buf[0].len    = 1;
//  i2cTransfer.buf[1].data   = i2c_rxBuffer;
//  i2cTransfer.buf[1].len    = 1;
//  result = I2C_TransferInit(I2C0, &i2cTransfer);
//
//  // Sending data
//  while (result == i2cTransferInProgress)
//  {
//    result = I2C_Transfer(I2C0);
//  }
//  //DPRINTF("err %d\n", result);
//  // if(i2c_rxBuffer[0] != 0)
//  //   {
//       DPRINTF("MCP23017_2_readReg.... %02X  Done   %02X\n",reg, i2c_rxBuffer[0]);
//   //  }
//    return i2c_rxBuffer[0];
//}
void MCP23017_2_writeReg(uint8_t reg, uint8_t value)
{
  apds9960_interface_iic_write(MCP23017_ADDRESS_2, reg, &value, 1);
   // DPRINTF("inside writing..\n");

}

void MCP23017_2_GPIO_PinOutToggle(uint8_t port, uint8_t pin)
{
  uint8_t mask = 1 << pin;
  if(port == MCP23017_GPIO_A)
  {
    LOCAL_2_REG_PORT_A = (uint8_t)(LOCAL_2_REG_PORT_A ^ mask);
    MCP23017_2_writeReg(MCP23017_GPIO_A,LOCAL_2_REG_PORT_A);
  }
  if(port == MCP23017_GPIO_B)
  {
    LOCAL_2_REG_PORT_B = (uint8_t)(LOCAL_2_REG_PORT_B ^ mask);
    MCP23017_2_writeReg(MCP23017_GPIO_B,LOCAL_2_REG_PORT_B);
  }
}
void MCP23017_2_GPIO_PinOutSet(uint8_t port, uint8_t pin, uint8_t value)
{
  uint8_t mask = 1 << pin;
  if(port == MCP23017_GPIO_A)
  {
    LOCAL_2_REG_PORT_A = value?(uint8_t)(LOCAL_2_REG_PORT_A | mask):(uint8_t)(LOCAL_2_REG_PORT_A & ~mask);
    MCP23017_2_writeReg(MCP23017_GPIO_A,LOCAL_2_REG_PORT_A);
  //  DPRINTF("inside MCP23017_2_GPIO_PinOutSet..\n");

  }
  if(port == MCP23017_GPIO_B)
  {
    LOCAL_2_REG_PORT_B = value?(uint8_t)(LOCAL_2_REG_PORT_B | mask):(uint8_t)(LOCAL_2_REG_PORT_B & ~mask);
    MCP23017_2_writeReg(MCP23017_GPIO_B,LOCAL_2_REG_PORT_B);
  }
}


// 1. Initialize all Port B pins as inputs
void MCP23017_2_Init()
{
    //DPRINTF("MCP23017_2_Init_AllInputs......");

    MCP23017_2_writeReg(MCP23017_PUR_B, 0xFF);     // Enable pull-ups
    MCP23017_2_writeReg(MCP23017_DDR_B, 0xFF);     // Set all B pins as input
    LOCAL_2_REG_PORT_B = 0x00;
    MCP23017_2_writeReg(MCP23017_GPIO_B, LOCAL_2_REG_PORT_B);

    //DPRINTF("MCP23017_2_Init_AllInputs......Done!");
}

// 2. Read all Port B input pins
uint8_t MCP23017_2_ReadInputs_PortB()
{
    uint8_t value = MCP23017_2_readReg(MCP23017_GPIO_B);
   // DPRINTF("MCP23017 Port B Input Value: 0x%02X", value);
    return value;
}

// 3. Read a specific pin from Port B
uint8_t MCP23017_2_ReadInput_PinB(uint8_t pin)
{
    if (pin > 7) return 0xFF; // Invalid pin
    uint8_t portValue = MCP23017_2_readReg(MCP23017_GPIO_B);
    uint8_t pinValue = (portValue >> pin) & 0x01;
   // DPRINTF("Pin B%u = %u", pin, pinValue);
    return pinValue;
}

//void MCP23017_2_Init()
//{
//  DPRINTF("MCP23017_2_Init......");
//  //  disable address increment (datasheet)
//  //MCP23017_writeReg(MCP23017_IOCR, 0b00100000);
//  //  Force INPUT_PULLUP
//  MCP23017_2_writeReg(MCP23017_PUR_A, 0xFF);
//  MCP23017_2_writeReg(MCP23017_PUR_B, 0xFF);
//  //  Ports Direction bit 0-Output,  bit 1-Input
//  MCP23017_2_writeReg(MCP23017_DDR_A, 0x00);
////  MCP23017_2_writeReg(MCP23017_DDR_B, 0x01);
//  MCP23017_2_writeReg(MCP23017_DDR_B, 0xFF);
//
////   MCP23017_2_writeReg(MCP23017_GPINTEN_A, 0xFF);  // Enable interrupts on all pins of port A
////      MCP23017_2_writeReg(MCP23017_DEFVAL_A, 0x00);   // Default comparison value
////      MCP23017_2_writeReg(MCP23017_INTCON_A, 0x00);   // Compare against previous value
////    MCP23017_2_writeReg(MCP23017_IOCR, 0x44);    //  Enable interrupt mirroring and open-drain output
//
//  LOCAL_2_REG_PORT_A = 0x00;
//  LOCAL_2_REG_PORT_B = 0x00;
//
//  MCP23017_2_writeReg(MCP23017_GPIO_A,LOCAL_2_REG_PORT_A);
//  MCP23017_2_writeReg(MCP23017_GPIO_B,LOCAL_2_REG_PORT_B);
// // MCP23017_2_writeReg(MCP23017_GPIO_B,0X01);
////    MCP23017_2_writeReg(MCP23017_PUR_B, 0x01);   // 0b00000001
//
//  //MCP23017_2_GPIO_PinOutSet(MCP23017_GPIO_B, 0, 1);
////  MCP23017_2_GPIO_PinOutSet(MCP23017_GPIO_B, 0, 1);
////  MCP23017_2_GPIO_PinOutSet(APP_RELAY_7_PORT1, APP_RELAY_7_PIN1, 1);
////  MCP23017_2_GPIO_PinOutSet(APP_RELAY_8_PORT1, APP_RELAY_8_PIN1, 1);
//
////  MCP23017_2_GPIO_PinOutSet(APP_RELAY_1_PORT, APP_RELAY_1_PIN, 1);
////  MCP23017_2_GPIO_PinOutSet(APP_RELAY_2_PORT, APP_RELAY_2_PIN, 1);
////  MCP23017_2_GPIO_PinOutSet(APP_RELAY_3_PORT, APP_RELAY_3_PIN, 1);
////  MCP23017_2_GPIO_PinOutSet(APP_RELAY_4_PORT, APP_RELAY_4_PIN, 1);
////  MCP23017_2_GPIO_PinOutSet(APP_RELAY_5_PORT, APP_RELAY_5_PIN, 1);
////  MCP23017_2_GPIO_PinOutSet(APP_RELAY_6_PORT, APP_RELAY_6_PIN, 1);
////  MCP23017_2_GPIO_PinOutSet(APP_RELAY_7_PORT, APP_RELAY_7_PIN, 1);
////  MCP23017_2_GPIO_PinOutSet(APP_RELAY_8_PORT, APP_RELAY_8_PIN, 1);
//
//
//  DPRINTF("MCP23017_2_Init......Done !!!");
// // MCP23017_2_readReg(MCP23017_INTF_A);  // Read interrupt flag
//
//
//
// }

void GpioStatePrintTask(void *pvParameters)
{
    (void)pvParameters; // Suppress unused parameter warning
   // DPRINTF("GpioStatePrintTask!!!");
//    uint8_t previousPin7State = 0xFF; // Initial state set to a value that is different from actual states
//    uint8_t previousPin8State = 0xFF; // Initial state set to a value that is different from actual states
//    bool pin7EventEnqueued = false;
//    bool pin8EventEnqueued = false;
    vTaskDelay(pdMS_TO_TICKS(100));
    while (1)
    {
      //  uint8_t currentPin7State, currentPin8State;
      //  MCP23017_2_GetPinStates();
      //  MCP23017_2_GPIO_PinInGet(MCP23017_GPIO_B, 0);
//        vTaskDelay(pdMS_TO_TICKS(1000));
        uint8_t portB_value = MCP23017_2_ReadInputs_PortB();

        for (uint8_t i = 0; i < 8; i++)
        {
            uint8_t pin_value = MCP23017_2_ReadInput_PinB(i);

           // DPRINTF("Relay %u status: %s\n", i + 1, pin_value ? "HIGH" : "LOW");
            set_led_with_BinaryModule((i+1), pin_value);
        }
//        // Handle Pin 7 state change
//        if (currentPin7State != previousPin7State)
//        {
//            // Detect transition from 1 to 0
//            if (previousPin7State == 1 && currentPin7State == 0 && !pin7EventEnqueued)
//            {
//                DPRINTF("Pin 7 -->: %d \n", currentPin7State);
//              //  ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW7_DOWN);
//                pin7EventEnqueued = true; // Ensure event is only enqueued once per transition
//            }
//            else if (currentPin7State == 1) // Reset the flag when pin returns to 1
//            {
//                pin7EventEnqueued = false;
//             //   ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW7_UP);
//
//            }
//
//            previousPin7State = currentPin7State; // Update the previous state
//        }
//
//        // Handle Pin 8 state change
//        if (currentPin8State != previousPin8State)
//        {
//            // Detect transition from 0 to 1
//            if (previousPin8State == 1 && currentPin8State == 0 && !pin8EventEnqueued)
//            {
//                DPRINTF("Pin 8 -->: %d \n", currentPin8State);
//              //  ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW8_DOWN);
//                pin8EventEnqueued = true; // Ensure event is only enqueued once per transition
//            }
//            else if (currentPin8State == 1) // Reset the flag when pin returns to 0
//            {
//                pin8EventEnqueued = false;
//              //  ZAF_EventHelperEventEnqueueFromISR(APP_EVENT_SW8_UP);
//
//            }
//
//            previousPin8State = currentPin8State; // Update the previous state
//        }
//
//        // Add a small delay to debounce the button press and avoid excessive CPU usage
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 milliseconds
    }
}



void InitGpioStatePrintTask(void)
{
 // DPRINTF(" InitGpioStatePrintTask \n");

    // Create the buffer bundle for the new task
    ZW_UserTask_Buffer_t gpioStatePrintTaskBuffer;
    gpioStatePrintTaskBuffer.taskBuffer = &GpioStatePrintTaskBuffer;
    gpioStatePrintTaskBuffer.stackBuffer = GpioStatePrintStackBuffer;
    gpioStatePrintTaskBuffer.stackBufferLength = TASK_STACK_SIZE_GPIO_STATE_PRINT;

    // Create the task setting-structure
    ZW_UserTask_t task;
    task.pTaskFunc = (TaskFunction_t)GpioStatePrintTask;
    task.pTaskName = "GpioStatePrint";
    task.pUserTaskParam = NULL;
    task.priority = USERTASK_PRIORITY_HIGHEST;
    task.taskBuffer = &gpioStatePrintTaskBuffer;

    // Create the task
    ZW_UserTask_CreateTask(&task, &m_xTaskHandleGpioStatePrint);
}






//------------------------------------
void MCP23017_2_GetPinStates() {
//void MCP23017_2_GetPinStates(uint8_t *pin7State, uint8_t *pin8State) {
    uint16_t gpioState = MCP23017_2_readReg(MCP23017_GPIO_B);
    if(gpioState !=0){
   /// DPRINTF("MCP23017_GPIO_B Register Value: 0x%02X\n", gpioState);
   // DPRINTF("\n\n");
    }
    //UNUSED(pin7State);
    //UNUSED(pin8State);
//    *pin7State = (gpioState >> APP_RELAY_7_PIN1) & 0x01;
//    *pin8State = (gpioState >> APP_RELAY_8_PIN1) & 0x01;
}

void Set_Relay(uint8_t led,uint8_t value)
{
  if(led == 1)
  {

      if(value == 1)
      {
         // MCP23017_2_GPIO_PinOutSet(APP_RELAY_1_PORT, APP_RELAY_1_PIN, 1);
          GPIO_PinOutSet(APP_RELAY_1_PORT, APP_RELAY_1_PIN);
           DPRINTF(" relay 1 is on \n");

      }
      else if( value == 0)

      {
          //MCP23017_2_GPIO_PinOutSet(APP_RELAY_1_PORT, APP_RELAY_1_PIN, 0);
          GPIO_PinOutClear(APP_RELAY_1_PORT, APP_RELAY_1_PIN);
           DPRINTF(" relay 1 is off \n");

      }

  }

  if(led == 2)
  {

      if(value == 1)
      {
          GPIO_PinOutSet(APP_RELAY_2_PORT, APP_RELAY_2_PIN);
         // MCP23017_2_GPIO_PinOutSet(APP_RELAY_2_PORT, APP_RELAY_2_PIN, 1);
          DPRINTF(" relay 2 is on \n");

      }
      else if( value == 0)

      {
          GPIO_PinOutClear(APP_RELAY_2_PORT, APP_RELAY_2_PIN);
          //MCP23017_2_GPIO_PinOutSet(APP_RELAY_2_PORT, APP_RELAY_2_PIN, 0);
          DPRINTF(" relay 2 is off \n");

      }

  }

  if(led == 3)
  {

      if(value == 1)
      {
          GPIO_PinOutSet(APP_RELAY_3_PORT, APP_RELAY_3_PIN);
          //MCP23017_2_GPIO_PinOutSet(APP_RELAY_3_PORT, APP_RELAY_3_PIN, 1);
          DPRINTF(" relay 3 is on \n");

      }
      else if( value == 0)

      {
          GPIO_PinOutClear(APP_RELAY_3_PORT, APP_RELAY_3_PIN);

          //MCP23017_2_GPIO_PinOutSet(APP_RELAY_3_PORT, APP_RELAY_3_PIN, 0);
          DPRINTF(" relay 3 is off \n");

      }

  }

  if(led == 4)
  {

      if(value == 1)
      {
          GPIO_PinOutSet(APP_RELAY_4_PORT, APP_RELAY_4_PIN);
         // MCP23017_2_GPIO_PinOutSet(APP_RELAY_4_PORT, APP_RELAY_4_PIN, 1);
          DPRINTF(" relay 4 is on \n");

      }
      else if( value == 0)

      {
          GPIO_PinOutClear(APP_RELAY_4_PORT, APP_RELAY_4_PIN);

         // MCP23017_2_GPIO_PinOutSet(APP_RELAY_4_PORT, APP_RELAY_4_PIN, 0);
          DPRINTF(" relay 4is off \n");

      }

  }


  if(led == 5)
  {

      if(value == 1)
      {
          GPIO_PinOutSet(APP_RELAY_5_PORT, APP_RELAY_5_PIN);
          //MCP23017_2_GPIO_PinOutSet(APP_RELAY_5_PORT, APP_RELAY_5_PIN, 1);
          DPRINTF(" relay 5 is on \n");

      }
      else if( value == 0)

      {
          GPIO_PinOutClear(APP_RELAY_5_PORT, APP_RELAY_5_PIN);

          //MCP23017_2_GPIO_PinOutSet(APP_RELAY_5_PORT, APP_RELAY_5_PIN, 0);
          DPRINTF(" relay 5 is off \n");

      }

  }

  if(led == 6)
  {

      if(value == 1)
      {
          GPIO_PinOutSet(APP_RELAY_6_PORT, APP_RELAY_6_PIN);
          //MCP23017_2_GPIO_PinOutSet(APP_RELAY_6_PORT, APP_RELAY_6_PIN, 1);
          DPRINTF(" relay 6 is on \n");

      }
      else if( value == 0)

      {
          GPIO_PinOutClear(APP_RELAY_6_PORT, APP_RELAY_6_PIN);

         // MCP23017_2_GPIO_PinOutSet(APP_RELAY_6_PORT, APP_RELAY_6_PIN, 0);
          DPRINTF(" relay 6 is off \n");

      }

  }
  if(led == 7)
  {

      if(value == 1)
      {
          GPIO_PinOutSet(APP_RELAY_7_PORT, APP_RELAY_7_PIN);
         // MCP23017_2_GPIO_PinOutSet(APP_RELAY_7_PORT, APP_RELAY_7_PIN, 1);
          DPRINTF(" relay 7 is on \n");

      }
      else if( value == 0)

      {
          GPIO_PinOutClear(APP_RELAY_7_PORT, APP_RELAY_7_PIN);

          //MCP23017_2_GPIO_PinOutSet(APP_RELAY_7_PORT, APP_RELAY_7_PIN, 0);
          DPRINTF(" relay 7 is off \n");

      }

  }
  if(led == 8)
  {

      if(value == 1)
      {
          GPIO_PinOutSet(APP_RELAY_8_PORT, APP_RELAY_8_PIN);
          //MCP23017_2_GPIO_PinOutSet(APP_RELAY_8_PORT, APP_RELAY_8_PIN, 1);
          DPRINTF(" relay 8 is on \n");

      }
      else if( value == 0)

      {
          GPIO_PinOutClear(APP_RELAY_8_PORT, APP_RELAY_8_PIN);

          //MCP23017_2_GPIO_PinOutSet(APP_RELAY_8_PORT, APP_RELAY_8_PIN, 0);
          DPRINTF(" relay 8 is off \n");

      }

  }
}

