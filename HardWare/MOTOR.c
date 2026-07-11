#include "MOTOR.h"
#include <ti/devices/msp/msp.h>
#include "ti_msp_dl_config.h"

void Motor_Init(void)
{
    // TODO: 配置 PWM 定时器
    // PWM_0_INST (TIMG0) -> PA12(CCP0), PA13(CCP1)

    // 方向引脚初始化为停止状态
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_2_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN_1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN_2_PIN);

    // 启动 PWM 定时器（占空比 0）
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_0_INDEX] = 0;
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_1_INDEX] = 0;
    DL_TimerG_startCounter(PWM_0_INST);
}

void Motor_SetSpeed(int16_t speed)
{
    // TODO: 根据 speed (±1000) 设置 PWM 占空比和方向
    (void)speed;
}

void Motor_Stop(void)
{
    // TODO: 停止电机
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_0_INDEX] = 0;
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_1_INDEX] = 0;
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_2_PIN);
}
