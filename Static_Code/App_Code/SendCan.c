#include "Rte_Swc_SendCan.h"

VAR(uint8, Swc_SendCan_VAR_CLEARED) Sig1 = 0;
VAR(uint8, Swc_SendCan_VAR_CLEARED) Sig2 = 0;
VAR(boolean, Swc_SendCan_VAR_CLEARED) Sig3 = FALSE;

FUNC(void, Swc_SendCan_CODE) Re_SendCan(void) /* Rx_Dir <--> Tx_RP */
{
  Rte_Read_R_RcCommand_ECU2_Msg_Dir_Sig1(&Sig1);
  Rte_Read_R_RcCommand_ECU2_Msg_Dir_Sig2(&Sig2);
  Rte_Receive_R_RecVib_Shock(&Sig3);

  Rte_Send_P_SendDir_Dir(Sig1);
  Rte_Send_P_SendSpeed_Speed(Sig2);

  Rte_Write_P_RcStatus_ECU1_Msg_Rp_Sig1(Sig1);
  Rte_Write_P_RcStatus_ECU1_Msg_Rp_Sig2(Sig2);
  Rte_Write_P_RcStatus_ECU1_Msg_Rp_Sig3(Sig3);
}
