
#define DEBUGPRINT
#include "DebugPrint.h"


#include "MCP23017_I2C.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "driver_interface.h"

//Keeping local ram variable to avoid i2c read operation
static uint8_t LOCAL_REG_PORT_A = 0x00;
static uint8_t LOCAL_REG_PORT_B = 0x00;

void MCP23017_readReg(uint8_t reg)
{
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  //uint8_t i2c_txBuffer[1] = {reg};
  //uint8_t i2c_txBufferSize = sizeof(i2c_txBuffer);
  uint8_t i2c_rxBuffer[1] = {0};
  //uint8_t i2c_rxBufferIndex;

  // Initializing I2C transfer
  i2cTransfer.addr          = MCP23017_ADDRESS;
  i2cTransfer.flags         = I2C_FLAG_WRITE_READ;
  i2cTransfer.buf[0].data   = &reg;
  i2cTransfer.buf[0].len    = 1;
  i2cTransfer.buf[1].data   = i2c_rxBuffer;
  i2cTransfer.buf[1].len    = 1;
  result = I2C_TransferInit(I2C0, &i2cTransfer);

  // Sending data
  while (result == i2cTransferInProgress)
  {
    result = I2C_Transfer(I2C0);
  }
//DPRINTF("err in 1 %d\n", result);
//  DPRINTF("MCP23017_readReg.... %02X  Done   %02X\n",reg, i2c_rxBuffer[0]);
}


void MCP23017_writeReg(uint8_t reg, uint8_t value)
{
  apds9960_interface_iic_write(MCP23017_ADDRESS, reg, &value, 1);
}

void MCP23017_GPIO_PinOutToggle(uint8_t port, uint8_t pin)
{
  uint8_t mask = 1 << pin;
  if(port == MCP23017_GPIO_A)
  {
    LOCAL_REG_PORT_A = (uint8_t)(LOCAL_REG_PORT_A ^ mask);
    MCP23017_writeReg(MCP23017_GPIO_A,LOCAL_REG_PORT_A);
  }
  if(port == MCP23017_GPIO_B)
  {
    LOCAL_REG_PORT_B = (uint8_t)(LOCAL_REG_PORT_B ^ mask);
    MCP23017_writeReg(MCP23017_GPIO_B,LOCAL_REG_PORT_B);
  }
}
void MCP23017_GPIO_PinOutSet(uint8_t port, uint8_t pin, uint8_t value)
{
  uint8_t mask = 1 << pin;
  if(port == MCP23017_GPIO_A)
  {
    LOCAL_REG_PORT_A = value?(uint8_t)(LOCAL_REG_PORT_A | mask):(uint8_t)(LOCAL_REG_PORT_A & ~mask);
    MCP23017_writeReg(MCP23017_GPIO_A,LOCAL_REG_PORT_A);
  }
  if(port == MCP23017_GPIO_B)
  {
    LOCAL_REG_PORT_B = value?(uint8_t)(LOCAL_REG_PORT_B | mask):(uint8_t)(LOCAL_REG_PORT_B & ~mask);
    MCP23017_writeReg(MCP23017_GPIO_B,LOCAL_REG_PORT_B);
  }
}


void MCP23017_Init()
{
  DPRINTF("inside MCP23017 init..\n");

  //  disable address increment (datasheet)
  //MCP23017_writeReg(MCP23017_IOCR, 0b00100000);
  //  Force INPUT_PULLUP
  MCP23017_writeReg(MCP23017_PUR_A, 0xFF);
  MCP23017_writeReg(MCP23017_PUR_B, 0xFF);

  //  Ports Direction bit 0-Output,  bit 1-Input
  MCP23017_writeReg(MCP23017_DDR_A, 0x00);
  MCP23017_writeReg(MCP23017_DDR_B, 0x00);

  LOCAL_REG_PORT_A = 0x00;
  LOCAL_REG_PORT_B = 0x00;

  MCP23017_writeReg(MCP23017_GPIO_A,LOCAL_REG_PORT_A);
  MCP23017_writeReg(MCP23017_GPIO_B,LOCAL_REG_PORT_B);

  MCP23017_GPIO_PinOutSet(APP_LED_1_ON_PORT, APP_LED_1_ON_PIN, 1);
  MCP23017_GPIO_PinOutSet(APP_LED_2_ON_PORT, APP_LED_2_ON_PIN, 1);
  MCP23017_GPIO_PinOutSet(APP_LED_3_ON_PORT, APP_LED_3_ON_PIN, 1);
  MCP23017_GPIO_PinOutSet(APP_LED_4_ON_PORT, APP_LED_4_ON_PIN, 1);
  MCP23017_GPIO_PinOutSet(APP_LED_5_ON_PORT, APP_LED_5_ON_PIN, 1);
  MCP23017_GPIO_PinOutSet(APP_LED_6_ON_PORT, APP_LED_6_ON_PIN, 1);
 MCP23017_GPIO_PinOutSet(APP_LED_7_ON_PORT, APP_LED_7_ON_PIN, 1);
  MCP23017_GPIO_PinOutSet(APP_LED_8_ON_PORT, APP_LED_8_ON_PIN, 1);

  MCP23017_GPIO_PinOutSet(APP_LED_1_OFF_PORT, APP_LED_1_OFF_PIN, 0);
  MCP23017_GPIO_PinOutSet(APP_LED_2_OFF_PORT, APP_LED_2_OFF_PIN, 0);
  MCP23017_GPIO_PinOutSet(APP_LED_3_OFF_PORT, APP_LED_3_OFF_PIN, 0);
  MCP23017_GPIO_PinOutSet(APP_LED_4_OFF_PORT, APP_LED_4_OFF_PIN, 0);
  MCP23017_GPIO_PinOutSet(APP_LED_5_OFF_PORT, APP_LED_5_OFF_PIN, 0);
  MCP23017_GPIO_PinOutSet(APP_LED_6_OFF_PORT, APP_LED_6_OFF_PIN, 0);
  MCP23017_GPIO_PinOutSet(APP_LED_7_OFF_PORT, APP_LED_7_OFF_PIN, 0);
  MCP23017_GPIO_PinOutSet(APP_LED_8_OFF_PORT, APP_LED_8_OFF_PIN, 0);
  DPRINTF("MCP23017b Init completed..\n");
//  MCP23017_2_readReg(MCP23017_GPIO_B);

}



void Set_Led(uint8_t led,uint8_t value)
{

  if(led == 1)
  {

      if(value == 1)
      {
        MCP23017_GPIO_PinOutSet(APP_LED_1_ON_PORT, APP_LED_1_ON_PIN, MCP23017_PIN_ON);
        MCP23017_GPIO_PinOutSet(APP_LED_1_OFF_PORT, APP_LED_1_OFF_PIN, MCP23017_PIN_OFF);
//        MCP23017_GPIO_PinOutSet(APP_LED_2_ON_PORT, APP_LED_2_ON_PIN, MCP23017_PIN_OFF);
//            MCP23017_GPIO_PinOutSet(APP_LED_2_OFF_PORT, APP_LED_2_OFF_PIN, MCP23017_PIN_ON);
      }
      else if(value == 0)
      {
        MCP23017_GPIO_PinOutSet(APP_LED_1_ON_PORT, APP_LED_1_ON_PIN, MCP23017_PIN_OFF);
        MCP23017_GPIO_PinOutSet(APP_LED_1_OFF_PORT, APP_LED_1_OFF_PIN, MCP23017_PIN_ON);
      }

  }



  if(led == 2)
  {

      if(value == 1)
      {
        MCP23017_GPIO_PinOutSet(APP_LED_2_ON_PORT, APP_LED_2_ON_PIN, MCP23017_PIN_ON);
        MCP23017_GPIO_PinOutSet(APP_LED_2_OFF_PORT, APP_LED_2_OFF_PIN, MCP23017_PIN_OFF);
//        MCP23017_GPIO_PinOutSet(APP_LED_2_ON_PORT, APP_LED_2_ON_PIN, MCP23017_PIN_OFF);
//                   MCP23017_GPIO_PinOutSet(APP_LED_2_OFF_PORT, APP_LED_2_OFF_PIN, MCP23017_PIN_ON);
      }
      else if(value == 0)
      {
        MCP23017_GPIO_PinOutSet(APP_LED_2_ON_PORT, APP_LED_2_ON_PIN, MCP23017_PIN_OFF);
        MCP23017_GPIO_PinOutSet(APP_LED_2_OFF_PORT, APP_LED_2_OFF_PIN, MCP23017_PIN_ON);
      }

  }
  if(led == 3)
  {

      if(value == 1)
      {
        MCP23017_GPIO_PinOutSet(APP_LED_3_ON_PORT, APP_LED_3_ON_PIN, MCP23017_PIN_ON);
        MCP23017_GPIO_PinOutSet(APP_LED_3_OFF_PORT, APP_LED_3_OFF_PIN, MCP23017_PIN_OFF);
//        MCP23017_GPIO_PinOutSet(APP_LED_2_ON_PORT, APP_LED_2_ON_PIN, MCP23017_PIN_OFF);
//                   MCP23017_GPIO_PinOutSet(APP_LED_2_OFF_PORT, APP_LED_2_OFF_PIN, MCP23017_PIN_ON);
      }
      else if( value == 0)
      {
        MCP23017_GPIO_PinOutSet(APP_LED_3_ON_PORT, APP_LED_3_ON_PIN, MCP23017_PIN_OFF);
        MCP23017_GPIO_PinOutSet(APP_LED_3_OFF_PORT, APP_LED_3_OFF_PIN, MCP23017_PIN_ON);
      }

  }

  if(led == 4)
  {

      if(value == 1)
      {
        MCP23017_GPIO_PinOutSet(APP_LED_4_ON_PORT, APP_LED_4_ON_PIN, MCP23017_PIN_ON);
        MCP23017_GPIO_PinOutSet(APP_LED_4_OFF_PORT, APP_LED_4_OFF_PIN, MCP23017_PIN_OFF);
//        MCP23017_GPIO_PinOutSet(APP_LED_2_ON_PORT, APP_LED_2_ON_PIN, MCP23017_PIN_OFF);
//                   MCP23017_GPIO_PinOutSet(APP_LED_2_OFF_PORT, APP_LED_2_OFF_PIN, MCP23017_PIN_ON);
      }
      else if( value == 0)
      {
        MCP23017_GPIO_PinOutSet(APP_LED_4_ON_PORT, APP_LED_4_ON_PIN, MCP23017_PIN_OFF);
        MCP23017_GPIO_PinOutSet(APP_LED_4_OFF_PORT, APP_LED_4_OFF_PIN, MCP23017_PIN_ON);
      }

  }

  if(led == 5)
  {

      if(value == 1)
      {
        MCP23017_GPIO_PinOutSet(APP_LED_5_ON_PORT, APP_LED_5_ON_PIN, MCP23017_PIN_ON);
        MCP23017_GPIO_PinOutSet(APP_LED_5_OFF_PORT, APP_LED_5_OFF_PIN, MCP23017_PIN_OFF);
//        MCP23017_GPIO_PinOutSet(APP_LED_2_ON_PORT, APP_LED_2_ON_PIN, MCP23017_PIN_OFF);
//                   MCP23017_GPIO_PinOutSet(APP_LED_2_OFF_PORT, APP_LED_2_OFF_PIN, MCP23017_PIN_ON);
      }
      else if( value == 0)
      {
        MCP23017_GPIO_PinOutSet(APP_LED_5_ON_PORT, APP_LED_5_ON_PIN, MCP23017_PIN_OFF);
        MCP23017_GPIO_PinOutSet(APP_LED_5_OFF_PORT, APP_LED_5_OFF_PIN, MCP23017_PIN_ON);
      }

  }

  if(led == 6)
  {

      if(value == 1)
      {
        MCP23017_GPIO_PinOutSet(APP_LED_6_ON_PORT, APP_LED_6_ON_PIN, MCP23017_PIN_ON);
        MCP23017_GPIO_PinOutSet(APP_LED_6_OFF_PORT, APP_LED_6_OFF_PIN, MCP23017_PIN_OFF);
//        MCP23017_GPIO_PinOutSet(APP_LED_2_ON_PORT, APP_LED_2_ON_PIN, MCP23017_PIN_OFF);
//                   MCP23017_GPIO_PinOutSet(APP_LED_2_OFF_PORT, APP_LED_2_OFF_PIN, MCP23017_PIN_ON);
      }
      else if( value == 0)

      {
        MCP23017_GPIO_PinOutSet(APP_LED_6_ON_PORT, APP_LED_6_ON_PIN, MCP23017_PIN_OFF);
        MCP23017_GPIO_PinOutSet(APP_LED_6_OFF_PORT, APP_LED_6_OFF_PIN, MCP23017_PIN_ON);
      }

  }

  if(led == 7)
   {

       if(value == 1)
       {
         MCP23017_GPIO_PinOutSet(APP_LED_7_ON_PORT, APP_LED_7_ON_PIN, MCP23017_PIN_ON);
         MCP23017_GPIO_PinOutSet(APP_LED_7_OFF_PORT, APP_LED_7_OFF_PIN, MCP23017_PIN_OFF);
 //        MCP23017_GPIO_PinOutSet(APP_LED_2_ON_PORT, APP_LED_2_ON_PIN, MCP23017_PIN_OFF);
 //                   MCP23017_GPIO_PinOutSet(APP_LED_2_OFF_PORT, APP_LED_2_OFF_PIN, MCP23017_PIN_ON);
       }
       else if( value == 0)

       {
         MCP23017_GPIO_PinOutSet(APP_LED_7_ON_PORT, APP_LED_7_ON_PIN, MCP23017_PIN_OFF);
         MCP23017_GPIO_PinOutSet(APP_LED_7_OFF_PORT, APP_LED_7_OFF_PIN, MCP23017_PIN_ON);
       }

   }

   if(led == 8)
   {

       if(value == 1)
       {
         MCP23017_GPIO_PinOutSet(APP_LED_8_ON_PORT, APP_LED_8_ON_PIN, MCP23017_PIN_ON);
         MCP23017_GPIO_PinOutSet(APP_LED_8_OFF_PORT, APP_LED_8_OFF_PIN, MCP23017_PIN_OFF);
 //        MCP23017_GPIO_PinOutSet(APP_LED_2_ON_PORT, APP_LED_2_ON_PIN, MCP23017_PIN_OFF);
 //                   MCP23017_GPIO_PinOutSet(APP_LED_2_OFF_PORT, APP_LED_2_OFF_PIN, MCP23017_PIN_ON);
       }
       else if( value == 0)

       {
         MCP23017_GPIO_PinOutSet(APP_LED_8_ON_PORT, APP_LED_8_ON_PIN, MCP23017_PIN_OFF);
         MCP23017_GPIO_PinOutSet(APP_LED_8_OFF_PORT, APP_LED_8_OFF_PIN, MCP23017_PIN_ON);
       }

   }


}

/*


void Set_Relay(uint8_t led,uint8_t value)
{
  if(led == 1)
  {

      if(value == 1)
      {
          MCP23017_GPIO_PinOutSet(APP_LED_8_OFF_PORT, APP_LED_8_OFF_PIN, MCP23017_PIN_ON);

      }
      else if( value == 0)

      {
          MCP23017_GPIO_PinOutSet(APP_LED_8_OFF_PORT, APP_LED_8_OFF_PIN, MCP23017_PIN_OFF);

      }

  }

  if(led == 2)
  {

      if(value == 1)
      {
          MCP23017_GPIO_PinOutSet(APP_LED_8_ON_PORT, APP_LED_8_ON_PIN, MCP23017_PIN_ON);

      }
      else if( value == 0)

      {
          MCP23017_GPIO_PinOutSet(APP_LED_8_ON_PORT, APP_LED_8_ON_PIN, MCP23017_PIN_OFF);

      }

  }

  if(led == 3)
  {

      if(value == 1)
      {
          MCP23017_GPIO_PinOutSet(APP_LED_7_OFF_PORT, APP_LED_7_OFF_PIN, MCP23017_PIN_ON);

      }
      else if( value == 0)

      {
          MCP23017_GPIO_PinOutSet(APP_LED_7_OFF_PORT, APP_LED_7_OFF_PIN, MCP23017_PIN_OFF);

      }

  }

  if(led == 4)
  {

      if(value == 1)
      {
          MCP23017_GPIO_PinOutSet(APP_LED_7_ON_PORT, APP_LED_7_ON_PIN, MCP23017_PIN_ON);

      }
      else if( value == 0)

      {
          MCP23017_GPIO_PinOutSet(APP_LED_7_ON_PORT, APP_LED_7_ON_PIN, MCP23017_PIN_OFF);

      }

  }
}
*/
