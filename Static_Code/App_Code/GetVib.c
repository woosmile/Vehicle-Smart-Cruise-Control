#include "Rte_Swc_GetVib.h"

VAR(boolean, Swc_GetVib_VAR_CLEARED) Sig_shock = FALSE;

FUNC(void, Swc_GetVib) Re_GetVib(void) {
	Rte_Call_R_IoShock_ReadDirect(&Sig_shock);

	Rte_Send_P_RecVib_Shock(Sig_shock);
}
