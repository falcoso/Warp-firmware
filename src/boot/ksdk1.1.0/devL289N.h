#include "fsl_tpm_driver.h"
#include "fsl_lptmr_hal.h"
#include "fsl_lptmr_driver.h"
#include "SEGGER_RTT.h"

/*
 * Sets the pins for outputting PWM, currently only PTB10 and PTB11 are used
 * which connect to the two channels of TPM0, also controlling the R&G elements
 * of the RGB LED.
 */
void enablePWMpins();

/*
 * Sets the duty cycle for TPM<devtpm>_CH<chn> to level
 */
void setDutyCycle(uint16_t level, bool devtpm, bool chn)

/*
 * Ouputs a value -100 - 100 on both motors connected to the PWM pins.
 */
void motorControl(int8_t level);

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

