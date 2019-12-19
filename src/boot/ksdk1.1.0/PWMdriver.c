#include "PWMdriver.h"
#include "fsl_port_hal.h"

void enablePWMpins()
{
	// PTB10 -- > TPM0_CH1
	// PTB11 -- > TPM0_CH0
	// PTB13 -- > TPM1_CH1
	// PTA0  -- > TPM1_CH0
	PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortMuxAlt2);

	// Changing mode of PTA0 stops the data line working
	// PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt2);

	//CLOCK_SYS_TPM initialised in DRV_INIT
	TPM_DRV_Init(1U, (tpm_general_config_t *)&tpmConfig);
	TPM_DRV_SetClock(1U, kTpmClockSourceModuleMCGIRCLK, kTpmDividedBy1);
	TPM_DRV_Init(0U, (tpm_general_config_t *)&tpmConfig);
	TPM_DRV_SetClock(0U, kTpmClockSourceModuleMCGIRCLK, kTpmDividedBy1);
}
