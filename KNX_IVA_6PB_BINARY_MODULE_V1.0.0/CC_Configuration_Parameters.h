
#include <ZW_classcmd.h>
#include <CC_Common.h>
#include <ZAF_types.h>
#include <ZW_TransportEndpoint.h>
#include <ZAF_TSE.h>



//received_frame_status_t CC_Configuration_handler_dali(
//  RECEIVE_OPTIONS_TYPE_EX * pRxOpt,
//  ZW_APPLICATION_TX_BUFFER * pCmd,
//  uint16_t cmdLength);

uint16_t GetConfigurationParameterValue(uint16_t parameterNumber);
uint32_t GetConfigurationParameterValue1(uint16_t parameterNumber);
void SetConfigurationParameterValue(uint16_t parameterNumber, uint32_t parameterValue);
void SetConfigurationParameterDefaultValue(uint16_t parameterNumber);
void switch_type(uint16_t endpoint);



/*
typedef struct _APPLICATION_DATA1
{
//  uint16_t parameter_num;


  uint16_t source_main_val;
  uint16_t source_mid_val;
  uint16_t source_sub_val;

  uint16_t dest_main_val_1;
  uint16_t dest_mid_val_1;
  uint16_t dest_sub_val_1;


  uint16_t source_main_num;
  uint16_t source_mid_num;
  uint16_t source_sub_num;

  uint16_t dest_main_val_2;
  uint16_t dest_mid_val_2;
  uint16_t dest_sub_val_2;
  /////////////////
  uint16_t dest_main_val_3;
  uint16_t dest_mid_val_3;
  uint16_t dest_sub_val_3;

  uint16_t dest_main_val_4;
  uint16_t dest_mid_val_4;
  uint16_t dest_sub_val_4;

  uint16_t dest_main_val_5;
  uint16_t dest_mid_val_5;
  uint16_t dest_sub_val_5;

  uint16_t dest_main_val_6;
  uint16_t dest_mid_val_6;
  uint16_t dest_sub_val_6;

  uint16_t dest_main_val_7;
  uint16_t dest_mid_val_7;
  uint16_t dest_sub_val_7;

  uint16_t dest_main_val_8;
  uint16_t dest_mid_val_8;
  uint16_t dest_sub_val_8;

  uint16_t dest_main_num_1;
  uint16_t dest_mid_num_1;
  uint16_t dest_sub_num_1;



  uint16_t dest_main_num_2;
  uint16_t dest_mid_num_2;
  uint16_t dest_sub_num_2;
  /////////////////
  uint16_t dest_main_num_3;
  uint16_t dest_mid_num_3;
  uint16_t dest_sub_num_3;

  uint16_t dest_main_num_4;
  uint16_t dest_mid_num_4;
  uint16_t dest_sub_num_4;

  uint16_t dest_main_num_5;
  uint16_t dest_mid_num_5;
  uint16_t dest_sub_num_5;

  uint16_t dest_main_num_6;
  uint16_t dest_mid_num_6;
  uint16_t dest_sub_num_6;

  uint16_t dest_main_num_7;
  uint16_t dest_mid_num_7;
  uint16_t dest_sub_num_7;

  uint16_t dest_main_num_8;
  uint16_t dest_mid_num_8;
  uint16_t dest_sub_num_8;

}SApplicationData1;
 extern SApplicationData1 ApplicationData1;
 extern SApplicationData1 appData1;


 SApplicationData1 readAppData_1(void);

*/
