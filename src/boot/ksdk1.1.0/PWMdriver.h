#include "fsl_tpm_driver.h"

void enablePWMpins();
volatile tpm_general_config_t tpmConfig = {
	.isDBGMode = false,
	.isGlobalTimeBase = true,
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