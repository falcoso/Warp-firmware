#include "fsl_tpm_driver.h"
#include "fsl_lptmr_hal.h"
#include "fsl_lptmr_driver.h"

void enablePWMpins();

const lptmr_user_config_t lptmrConfig = {
    .timerMode = kLptmrTimerModeTimeCounter,
    .pinSelect = kLptmrPinSelectInput0,
    .pinPolarity =kLptmrPinPolarityActiveHigh,
    .freeRunningEnable = true,
    .prescalerEnable = false,
    .prescalerClockSource = kClockLptmrSrcLpoClk,
    .prescalerValue = kLptmrPrescalerDivide2 ,
    .isInterruptEnabled = false
};

volatile lptmr_state_t lptmrState; // set when the timer is initialised

volatile tpm_general_config_t tpmConfig = {
		.isDBGMode = false,
		.isGlobalTimeBase = false,
		.isTriggerMode = false,
		.isStopCountOnOveflow = false,
		.isCountReloadOnTrig = false,
		.triggerSource = 0
};

volatile tpm_pwm_param_t pwmSettings[] = {
	{	//TPM0_CH0
		.mode = kTpmEdgeAlignedPWM,
		.edgeMode = kTpmHighTrue,
		.uFrequencyHZ = 400,
		.uDutyCyclePercent = 99,
	},
	{	//TPM0_CH1
		.mode = kTpmEdgeAlignedPWM,
		.edgeMode = kTpmHighTrue,
		.uFrequencyHZ = 400,
		.uDutyCyclePercent = 99,
	}
};