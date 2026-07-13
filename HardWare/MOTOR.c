#include "MOTOR.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include "ti_msp_dl_config.h"

#define PULSE_PER_REV  600   /* 300? ? 2????? */

volatile uint32_t enc_left = 0;

uint16_t debug_pulses = 0;
uint16_t debug_rpm = 0;
uint16_t debug_out = 0;

/* ===== ????? A ???? ===== */
void GROUP1_IRQHandler(void)
{
    if (DL_GPIO_getEnabledInterruptStatus(GPIOA, GOIO_GET_GET_2A_PIN))
    {
        enc_left++;
        DL_GPIO_clearInterruptStatus(GPIOA, GOIO_GET_GET_2A_PIN);
    }
}

void Motor_Init(void)
{
    PWM_0_INST->COMMONREGS.CCPD = 0x03;
    PWM_0_INST->COUNTERREGS.OCTL_01[0] |= (1 << 5);
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_1_INDEX] = 0;
    DL_TimerG_startCounter(PWM_0_INST);
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_1_PIN);
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_2_PIN);

    /* ???? A ???? */
    DL_GPIO_disableInterrupt(GPIOA,
        GOIO_GET_GET_2B_PIN | GOIO_GET_GET_1A_PIN | GOIO_GET_GET_1B_PIN);
    DL_GPIO_clearInterruptStatus(GPIOA, GOIO_GET_GET_2A_PIN);
    DL_GPIO_enableInterrupt(GPIOA, GOIO_GET_GET_2A_PIN);
    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_26_EDGE_RISE_FALL);

    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_SetPriority(GPIOA_INT_IRQn, 0);

    enc_left = 0;
}

uint32_t Motor_GetPulses(void) { return enc_left; }

void Motor_Stop(void)
{
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_1_INDEX] = 0;
}
