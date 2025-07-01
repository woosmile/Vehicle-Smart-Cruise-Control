#include "Rte_Swc_ConMotor.h"

VAR(uint8, Swc_ConMotor_VAR_CLEARED) control_dial_motor = 0;

FUNC(void, Swc_ConMotor_CODE) Re_ConMotor(void)
{
  Rte_Receive_R_SendSpeed_Speed(&control_dial_motor);

//  if(30 <= control_dial_motor && control_dial_motor < 79)       Rte_Call_R_IoMotor_SetDutyCycle(0x1500);
//  else if(81 < control_dial_motor && control_dial_motor <= 150) Rte_Call_R_IoMotor_SetDutyCycle(0x1A80);
//  else                                                          Rte_Call_R_IoMotor_SetDutyCycle(0x1680);

	if(30 <= control_dial_motor && control_dial_motor < 79)       Rte_Call_R_IoMotor_SetDutyCycle(0x1500);
	else if(81 < control_dial_motor && control_dial_motor <= 150) Rte_Call_R_IoMotor_SetDutyCycle(0x1A80);
	else                                                          Rte_Call_R_IoMotor_SetDutyCycle(0x1680);
}

//#include "Rte_Swc_ConMotor.h"
//
//VAR(uint8, Swc_ConMotor_VAR_CLEARED) control_dial_motor = 0;
//VAR(uint16, Swc_ConMotor_VAR_CLEARED) mapped_control_dial_motor = 0;
//
//FUNC(void, Swc_ConMotor_CODE) Re_ConMotor(void)
//{
//  Rte_Receive_R_SendSpeed_Speed(&control_dial_motor);
//  mapped_control_dial_motor = control_dial_motor * 128;           //
//
//  Rte_Call_R_IoMotor_SetDutyCycle(mapped_control_dial_motor);
//}
