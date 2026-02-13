 /**
  *
  * @Suthor  :   G PAVAN KALYAN
  * @Email   :   RAMAGIRIPAVAN123@GMAIL.COM
  *
 */

#include <string.h>
#include <stddef.h>
#include <stdbool.h>
#include <CC_Configuration_Parameters.h>
#include <ZW_TransportEndpoint.h>
#include <ZAF_Common_interface.h>
#include <stddef.h>
#include "DebugPrint.h"
#include <ZAF_types.h>
 #include <ZAF_tx_mutex.h>

#include "DebugPrintConfig.h"
//#define DEBUGPRINT
//#define SLI_CC_CONFIGURATION_MAX_STR_LENGTH (256)

static received_frame_status_t CC_Configuration_handler_dali(RECEIVE_OPTIONS_TYPE_EX *pRxOpt,
                                                        ZW_APPLICATION_TX_BUFFER *pCmd,
                                                        uint16_t cmdLength)
{
  UNUSED(cmdLength);
 // UNUSED(pRxOpt);
//  DPRINTF("CC_Configuration_handler cmd :: %d\n",pCmd->ZW_Common.cmd);
  received_frame_status_t frame_status = RECEIVED_FRAME_STATUS_NO_SUPPORT;

  switch (pCmd->ZW_Common.cmd) {
    case CONFIGURATION_GET:
      //frame_status = cc_configuration_command_get(pRxOpt, pCmd, cmdLength);
      if (true == Check_not_legal_response_job(pRxOpt))
      {
        return RECEIVED_FRAME_STATUS_FAIL;
      }
      uint16_t p=pCmd->ZW_ConfigurationSet1byteFrame.parameterNumber;
      UNUSED(p);
    //  DPRINTF("%d is the parameter number\n",p);
      ZAF_TRANSPORT_TX_BUFFER  TxBuf;
      ZW_APPLICATION_TX_BUFFER *pTxBuf = &(TxBuf.appTxBuf);
      memset((uint16_t*)pTxBuf, 0, sizeof(ZW_APPLICATION_TX_BUFFER) );

      TRANSMIT_OPTIONS_TYPE_SINGLE_EX *pTxOptionsEx;
      RxToTxOptions(pRxOpt, &pTxOptionsEx);
      pTxBuf->ZW_ConfigurationReport1byteFrame.cmdClass = COMMAND_CLASS_CONFIGURATION;
      pTxBuf->ZW_ConfigurationReport1byteFrame.cmd = CONFIGURATION_REPORT;
      pTxBuf->ZW_ConfigurationReport1byteFrame.parameterNumber = pCmd->ZW_ConfigurationGetFrame.parameterNumber;
      pTxBuf->ZW_ConfigurationReport1byteFrame.level = 0x01;//reserved 5bits; size 3bits
      pTxBuf->ZW_ConfigurationReport1byteFrame.configurationValue1 = (uint16_t)GetConfigurationParameterValue(pCmd->ZW_ConfigurationGetFrame.parameterNumber);


    //  DPRINTF("\r\n config get = %d\n",pTxBuf->ZW_ConfigurationReport1byteFrame.configurationValue1);

      if(ZAF_ENQUEUE_STATUS_SUCCESS != Transport_SendResponseEP(
        (uint8_t *)pTxBuf,
        sizeof(ZW_CONFIGURATION_REPORT_1BYTE_FRAME),
        pTxOptionsEx,
        NULL))
        {
      //Job failed
   //       DPRINTF(" RECEIVED_FRAME_STATUS_SUCCESS...\n");
          return RECEIVED_FRAME_STATUS_SUCCESS;
        }
  //    DPRINTF(" RECEIVED_FRAME_STATUS_FAIL...\n");

          return RECEIVED_FRAME_STATUS_FAIL;
      break;

    case CONFIGURATION_SET:
      DPRINTF(" CONFIGURATION_SET ...\n");
    //  DPRINTF("Level field value: 0x%02X\n", pCmd->ZW_ConfigurationSet1byteFrame.level);

      //frame_status = cc_configuration_command_set(pRxOpt, pCmd, cmdLength);
      if(pCmd->ZW_ConfigurationSet1byteFrame.level & 0x80)//checking default bit
      {
          DPRINTF("default val...\n");

          DPRINTF("\r\n  set default :%d",pCmd->ZW_ConfigurationSet1byteFrame.level);
        // SetConfigurationParameterDefaultValue(pCmd->ZW_ConfigurationSet1byteFrame.parameterNumber);
      }
      else
      {
         // DPRINTF("inside SetConfiguration ...\n");

         //uint16_t p=pCmd->ZW_ConfigurationSet1byteFrame.parameterNumber;
         //SetConfigurationParameterValue(pCmd->ZW_ConfigurationSet1byteFrame.parameterNumber,
                                   //pCmd->ZW_ConfigurationSet1byteFrame.configurationValue1);
        // UNUSED(p);
      //     DPRINTF("completed SetConfigurationParameterValue...\n");

      }
       return RECEIVED_FRAME_STATUS_SUCCESS;
       break;


    }
 // DPRINTF("%d is the return number\n",frame_status);

  return frame_status;
}

 REGISTER_CC(COMMAND_CLASS_CONFIGURATION, CONFIGURATION_VERSION, CC_Configuration_handler_dali);
