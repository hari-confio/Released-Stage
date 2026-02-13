
#include <driver_interface.h>
#include <stdlib.h>
#include "em_i2c.h"
#include "em_cmu.h"
#include <string.h>
#include <board.h>

#define DEBUGPRINT
#include "DebugPrint.h"


//#include "App_Scr/App_Main.h"
/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
//static void ButtonPressCallback1(uint8_t intNo)
//{
//  /* Called from interrupt */
//  switch (intNo)
//  {
//
//
//#if defined(PB7_GPIO_PIN)
//case PB7_INT_NO:
//  Board_PushbuttonEventHandler(BOARD_BUTTON_PB7);
//  break;
//#endif
//
//#if defined(PB8_GPIO_PIN)
//case PB8_INT_NO:
//  Board_PushbuttonEventHandler(BOARD_BUTTON_PB8);
//  break;
//#endif
//
//
//    default:
//      break;
//  }
//}
uint8_t interface_iic_init(void)
{
  DPRINTF("INDIDE iic Init....\n");
  CMU_ClockEnable(cmuClock_I2C0, true);
  // Use default settings
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
  // Use ~400khz SCK
  i2cInit.freq = I2C_FREQ_STANDARD_MAX;
  // Use 6:3 low high SCK ratio
  i2cInit.clhr = i2cClockHLRStandard;

  GPIO_PinModeSet(IIC_SCL_PORT, IIC_SCL_PIN, gpioModeWiredAndPullUpFilter, 1);
  GPIO_PinModeSet(IIC_SDA_PORT, IIC_SDA_PIN, gpioModeWiredAndPullUpFilter, 1);
//
////=======================================
//  GPIO_PinModeSet(PB7_GPIO_PORT, PB7_GPIO_PIN, gpioModeInputPullFilter, 1);
//  GPIO_PinModeSet(PB8_GPIO_PORT, PB8_GPIO_PIN, gpioModeInputPullFilter, 1);
//
//  GPIO_ExtIntConfig(PB7_GPIO_PORT, PB7_GPIO_PIN, PB7_GPIO_PIN, true, true, true);
//  GPIO_ExtIntConfig(PB8_GPIO_PORT, PB8_GPIO_PIN, PB8_GPIO_PIN, true, true, true);
//  GPIOINT_CallbackRegister(PB7_GPIO_PIN, ButtonPressCallback1);
//  GPIOINT_CallbackRegister(PB8_GPIO_PIN, ButtonPressCallback1);
//
////==================================

  // Enable I2C SDA and SCL pins, see the readme or the device datasheet for I2C pin mappings
  // Route I2C pins to GPIO

  I2C0->ROUTEPEN = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN;
  I2C0->ROUTELOC0 = (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SDALOC_MASK)) | I2C_ROUTELOC0_SDALOC_LOC12;
  I2C0->ROUTELOC0 = (I2C0->ROUTELOC0 & (~_I2C_ROUTELOC0_SCLLOC_MASK)) | I2C_ROUTELOC0_SCLLOC_LOC10;


  // Initializing the I2C
  I2C_Init(I2C0, &i2cInit);
  // Setting up to enable slave mode
  //I2C0->CTRL |= I2C_CTRL_SLAVE;

  DPRINTF("iic Init.... Done\n");
  return 0;
}

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */


void printHex(uint8_t *data, uint16_t len)
{
  UNUSED(data);
  for(uint16_t i=0; i<len; i++)
  {
    DPRINTF("%02X ", data[i]);
  }
  DPRINTF("\n");
}

/**
 * @brief      interface iic bus read
 * @param[in]  addr is iic device write address
 * @param[in]  reg is iic register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t apds9960_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  //emberAfCorePrintlnF("apds9960_read....%02X    %02X\n",addr, reg);
  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  // Initializing I2C transfer
  i2cTransfer.addr          = addr;
  i2cTransfer.flags         = I2C_FLAG_WRITE_READ;
  i2cTransfer.buf[0].data   = &reg;
  i2cTransfer.buf[0].len    = 1;
  i2cTransfer.buf[1].data   = buf;
  i2cTransfer.buf[1].len    = len;
  result = I2C_TransferInit(I2C0, &i2cTransfer);

  // Sending data
  while (result == i2cTransferInProgress)
  {
    result = I2C_Transfer(I2C0);
  }
 // DPRINTF("err in read %d\n", result);
  //printHex(buf, len);
//  DPRINTF("apds9960_read....   Done\n");

  return 0;

}



/**
 * @brief     interface iic bus write
 * @param[in] addr is iic device write address
 * @param[in] reg is iic register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t apds9960_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
  //emberAfCorePrintln("writeReg....%02X %02X\n",addr, reg);
  // Transfer structure
  I2C_TransferSeq_TypeDef i2cTransfer;
  I2C_TransferReturn_TypeDef result;

  uint8_t *i2c_txBuffer = (uint8_t*)calloc(len+1, sizeof(uint8_t));
  i2c_txBuffer[0] = reg;
  memcpy(&i2c_txBuffer[1],buf,len);
  uint8_t i2c_rxBuffer[1];

  //printHex(i2c_txBuffer, len+1);

  // Initializing I2C transfer
  i2cTransfer.addr          = addr;
  i2cTransfer.flags         = I2C_FLAG_WRITE;
  i2cTransfer.buf[0].data   = i2c_txBuffer;
  i2cTransfer.buf[0].len    = len+1;
  i2cTransfer.buf[1].data   = i2c_rxBuffer;
  i2cTransfer.buf[1].len    = 0;
  result = I2C_TransferInit(I2C0, &i2cTransfer);

  // Sending data
  while (result == i2cTransferInProgress)
  {
    //  DPRINTF("in loop....   Done\n");
    result = I2C_Transfer(I2C0);
  }
//  DPRINTF("err %d\n", result);
 // DPRINTF("writeReg....   Done\n");

  free(i2c_txBuffer);
    return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms
 * @note      none
 */
//void apds9960_interface_delay_ms(uint32_t ms)
//{
//  //UNUSED(ms);
//}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void apds9960_interface_debug_print(const char *const fmt, ...)
{
  UNUSED(fmt);
  DPRINTF(fmt);
}

/**
 * @brief     interface receive callback
 * @param[in] type is the interrupt type
 * @note      none
 */
