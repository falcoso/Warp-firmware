#include "PWMdriver.h"
#include "fsl_port_hal.h"
#include "gpio_pins.h"

//override default GPIO pins
enum
{
	kL298Nreverse1 = GPIO_MAKE_PIN(HW_GPIOA, 5),
	kL298Nreverse2 = GPIO_MAKE_PIN(HW_GPIOA, 7),
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

	// Changing mode of PTA0 stops the data line working
	// PORT_HAL_SetMuxMode(PORTA_BASE, 0, kPortMuxAlt2);
	// PORT_HAL_SetMuxMode(PORTB_BASE, 13, kPortMuxAlt2);
	// TPM_DRV_Init(1U, (tpm_general_config_t *)&tpmConfig);
	// TPM_DRV_SetClock(1U, kTpmClockSourceModuleMCGIRCLK, kTpmDividedBy1);
}

void motorControl(int8_t level)
{
	// set GPIO pins to reverse polarity if level is -ve
	if(level<0)
	{
		GPIO_DRV_SetPinOutput(kL298Nreverse1);
		GPIO_DRV_SetPinOutput(kL298Nreverse2);
		level = 100+level;
	}
	else
	{
		GPIO_DRV_ClearPinOutput(kL298Nreverse1);
		GPIO_DRV_ClearPinOutput(kL298Nreverse2);
	}

	// modify duty cycle and reset PWM
	for (uint8_t i=0; i<2; i++)
	{
		pwmSettings[i].uDutyCyclePercent = level;
		SEGGER_RTT_printf(0, "CH%d Duty Cycle\n", pwmSettings[i].uDutyCyclePercent);
	}

	for (uint8_t i=0; i<2; i++)
	{
		if (!TPM_DRV_PwmStart(0, (tpm_pwm_param_t *)&pwmSettings[i], i))
			SEGGER_RTT_printf(0, "PWM FAILED TPM0 CH%d\n", i);
	}
}
