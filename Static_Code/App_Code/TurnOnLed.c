#include "Rte_Swc_TurnOnLed.h"

VAR(uint8, Swc_TurnOnLed_VAR_CLEARED) control_dial_led = 0;
VAR(boolean, Swc_TurnOnLed_VAR_CLEARED) control_io_led1 = FALSE;
VAR(boolean, Swc_TurnOnLed_VAR_CLEARED) control_io_led2 = FALSE;
VAR(boolean, Swc_TurnOnLed_VAR_CLEARED) control_io_led3 = FALSE;
VAR(boolean, Swc_TurnOnLed_VAR_CLEARED) control_io_led4 = FALSE;

FUNC(void, Swc_TurnOnLed_CODE) Re_TurnOnLed(void)
{
  Rte_Receive_R_SendDir_Dir(&control_dial_led);
  if(0 == control_dial_led){
    control_io_led1 = STD_LOW;
    control_io_led2 = STD_LOW;
    control_io_led3 = STD_LOW;
    control_io_led4 = STD_LOW;
  }
  else if(1 == control_dial_led){
    control_io_led1 = STD_HIGH;
    control_io_led2 = STD_LOW;
    control_io_led3 = STD_LOW;
    control_io_led4 = STD_LOW;
  }
  else if(2 == control_dial_led){
    control_io_led1 = STD_LOW;
    control_io_led2 = STD_HIGH;
    control_io_led3 = STD_LOW;
    control_io_led4 = STD_LOW;
  }
  else if(3 == control_dial_led){
    control_io_led1 = STD_LOW;
    control_io_led2 = STD_LOW;
    control_io_led3 = STD_HIGH;
    control_io_led4 = STD_LOW;
  }
  else if(4 == control_dial_led){
    control_io_led1 = STD_LOW;
    control_io_led2 = STD_LOW;
    control_io_led3 = STD_LOW;
    control_io_led4 = STD_HIGH;
  }

  Rte_Call_R_IoLed1_WriteDirect(control_io_led1);
  Rte_Call_R_IoLed2_WriteDirect(control_io_led2);
  Rte_Call_R_IoLed3_WriteDirect(control_io_led3);
  Rte_Call_R_IoLed4_WriteDirect(control_io_led4);
}
