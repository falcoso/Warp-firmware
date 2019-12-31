#include "PWMdriver.h"
#include "fsl_port_hal.h"
#include "gpio_pins.h"

//override default GPIO pins
enum
{
	kL298Nreverse1 = GPIO_MAKE_PIN(HW_GPIOA, 5),
	kL298Nreverse2 = GPIO_MAKE_PIN(HW_GPIOA, 7),
};

const tpm_general_config_t tpmConfig = {
		.isDBGMode = false,
		.isGlobalTimeBase = false,
		.isTriggerMode = false,
		.isStopCountOnOveflow = false,
		.isCountReloadOnTrig = false,
		.triggerSource = 0
};

volatile tpm_pwm_param_t pwmSettings = {
		.mode = kTpmEdgeAlignedPWM,
		.edgeMode = kTpmHighTrue,
		.uFrequencyHZ = 400,
		.uDutyCyclePercent = 0,
};


void enablePWMpins()
{
	// PTB10 -- > TPM0_CH1
	// PTB11 -- > TPM0_CH0
	// PTB13 -- > TPM1_CH1
	// PTA0  -- > TPM1_CH0
	PORT_HAL_SetMuxMode(PORTB_BASE, 10, kPortMuxAlt2);
	PORT_HAL_SetMuxMode(PORTB_BASE, 11, kPortMuxAlt2);
	TPM_DRV_Init(0U, (tpm_general_config_t *)&tpmConfig);
	TPM_DRV_SetClock(0U, kTpmClockSourceModuleMCGIRCLK, kTpmDividedBy1);
	if (!TPM_DRV_PwmStart(0, (tpm_pwm_param_t *)&pwmSettings, 0))
		SEGGER_RTT_printf(0, "PWM FAILED TPM0 CH%d\n", 0);

	// Changing mode of PTA0 stops the data line working
	// PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt2);
	// PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortMuxAlt2);
	// TPM_DRV_Init(1U, (tpm_general_config_t *)&tpmConfig);
	// TPM_DRV_SetClock(1U, kTpmClockSourceModuleMCGIRCLK, kTpmDividedBy1);
}

void setDutyCycle(uint16_t level, bool devtpm, bool chn)
{
	uint16_t uMod, uCnv;
	uint32_t tpmBaseAddr = g_tpmBaseAddr[devtpm];

	pwmSettings.uDutyCyclePercent = level;

	uMod = TPM_DRV_GetClock(devtpm) / pwmSettings.uFrequencyHZ - 1;
	uCnv = uMod * pwmSettings.uDutyCyclePercent / 100;
	TPM_HAL_SetChnCountVal(tpmBaseAddr, chn, uCnv);
}

void motorControl(int8_t level)
{
	// set GPIO pins to reverse polarity if level is -ve
	if(level<0)
	{
		GPIO_DRV_SetPinOutput(kL298Nreverse1);
		GPIO_DRV_SetPinOutput(kL298Nreverse2);
		level += 100;
	}
	else
	{
		GPIO_DRV_ClearPinOutput(kL298Nreverse1);
		GPIO_DRV_ClearPinOutput(kL298Nreverse2);
	}

	// modify duty cycle and reset PWM
	pwmSettings.uDutyCyclePercent = level;
	setDutyCycle(level, 0, 0);
}
