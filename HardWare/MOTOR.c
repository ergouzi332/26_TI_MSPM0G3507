#include "MOTOR.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include "ti_msp_dl_config.h"

#define PULSE_PER_REV  520

volatile uint32_t enc_left = 0;
volatile uint32_t enc_right = 0;

void GROUP1_IRQHandler(void)
{
    uint32_t flags = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        GOIO_GET_GET_2A_PIN | GOIO_GET_GET_2B_PIN |
        GOIO_GET_GET_1A_PIN | GOIO_GET_GET_1B_PIN);

    if (flags & GOIO_GET_GET_2A_PIN) { enc_left++; }
    if (flags & GOIO_GET_GET_2B_PIN) { enc_left++; }
    if (flags & GOIO_GET_GET_1A_PIN) { enc_right++; }
    if (flags & GOIO_GET_GET_1B_PIN) { enc_right++; }

    DL_GPIO_clearInterruptStatus(GPIOA,
        GOIO_GET_GET_2A_PIN | GOIO_GET_GET_2B_PIN |
        GOIO_GET_GET_1A_PIN | GOIO_GET_GET_1B_PIN);
}

void Motor_Init(void)
{
    PWM_0_INST->COMMONREGS.CCPD = 0x03;
    PWM_0_INST->COUNTERREGS.OCTL_01[0] |= (1 << 5);
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_0_INDEX] = 0;
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_1_INDEX] = 0;
    DL_TimerG_startCounter(PWM_0_INST);

    DL_GPIO_setPins(MOTOR_DIR_PORT,
        MOTOR_DIR_BIN_1_PIN | MOTOR_DIR_BIN_2_PIN |
        MOTOR_DIR_AIN_1_PIN | MOTOR_DIR_AIN_2_PIN);

    DL_GPIO_clearInterruptStatus(GPIOA,
        GOIO_GET_GET_2A_PIN | GOIO_GET_GET_2B_PIN |
        GOIO_GET_GET_1A_PIN | GOIO_GET_GET_1B_PIN);
    DL_GPIO_enableInterrupt(GPIOA,
        GOIO_GET_GET_2A_PIN | GOIO_GET_GET_2B_PIN |
        GOIO_GET_GET_1A_PIN | GOIO_GET_GET_1B_PIN);

    DL_GPIO_setUpperPinsPolarity(GPIOA,
        DL_GPIO_PIN_26_EDGE_RISE |
        DL_GPIO_PIN_21_EDGE_RISE |
        DL_GPIO_PIN_27_EDGE_RISE |
        DL_GPIO_PIN_22_EDGE_RISE);

    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_SetPriority(GPIOA_INT_IRQn, 0);

    enc_left = 0; enc_right = 0;
}

void Motor_SetForward(void)
{
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_1_PIN | MOTOR_DIR_AIN_1_PIN);
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_2_PIN | MOTOR_DIR_AIN_2_PIN);
}

void Motor_SetBrake(void)
{
    DL_GPIO_setPins(MOTOR_DIR_PORT,
        MOTOR_DIR_BIN_1_PIN | MOTOR_DIR_BIN_2_PIN |
        MOTOR_DIR_AIN_1_PIN | MOTOR_DIR_AIN_2_PIN);
}

// === 读后清零 ===
uint32_t Motor_GetLeftPulses(void)
{
    uint32_t val;
    __disable_irq();
    val = enc_left;
    enc_left = 0;
    __enable_irq();
    return val;
}

uint32_t Motor_GetRightPulses(void)
{
    uint32_t val;
    __disable_irq();
    val = enc_right;
    enc_right = 0;
    __enable_irq();
    return val;
}

void Motor_SetPWM(uint16_t left, uint16_t right)
{
    if (left  > 1000) left  = 1000;
    if (right > 1000) right = 1000;
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_1_INDEX] = left;
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_0_INDEX] = right;
}

void Motor_Stop(void)
{
    Motor_SetBrake();
    Motor_SetPWM(0, 0);
}
