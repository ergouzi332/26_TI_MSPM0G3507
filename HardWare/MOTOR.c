#include "MOTOR.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include "ti_msp_dl_config.h"

// 编码器每转总脉冲数，电机单圈编码器脉冲值
#define PULSE_PER_REV  520

// 左右轮编码器全局脉冲计数，中断内修改加volatile防止编译器优化
volatile uint32_t enc_left = 0;
volatile uint32_t enc_right = 0;

/**
 * @brief GPIOA组中断服务函数，编码器AB相上升沿触发
 * 四路编码器引脚中断进入，每检测到边沿脉冲计数+1（四倍频计数逻辑）
 */
void GROUP1_IRQHandler(void)
{
    // 读取本次触发的中断标志位
    uint32_t flags = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        GOIO_GET_GET_2A_PIN | GOIO_GET_GET_2B_PIN |
        GOIO_GET_GET_1A_PIN | GOIO_GET_GET_1B_PIN);

    // 左轮编码器2A相边沿，左轮脉冲+1
    if (flags & GOIO_GET_GET_2A_PIN) { enc_left++; }
    // 左轮编码器2B相边沿，左轮脉冲+1
    if (flags & GOIO_GET_GET_2B_PIN) { enc_left++; }
    // 右轮编码器1A相边沿，右轮脉冲+1
    if (flags & GOIO_GET_GET_1A_PIN) { enc_right++; }
    // 右轮编码器1B相边沿，右轮脉冲+1
    if (flags & GOIO_GET_GET_1B_PIN) { enc_right++; }

    // 清除所有编码器引脚中断标志，否则会重复进中断
    DL_GPIO_clearInterruptStatus(GPIOA,
        GOIO_GET_GET_2A_PIN | GOIO_GET_GET_2B_PIN |
        GOIO_GET_GET_1A_PIN | GOIO_GET_GET_1B_PIN);
}

/**
 * @brief 电机底层初始化：PWM输出、方向控制IO、编码器中断配置
 */
void Motor_Init(void)
{
    // PWM分频配置，设置PWM时钟分频参数
    PWM_0_INST->COMMONREGS.CCPD = 0x03;
    // 开启定时器比较输出使能
    PWM_0_INST->COUNTERREGS.OCTL_01[0] |= (1 << 5);
    // 左右轮PWM初始占空比0，电机静止
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_0_INDEX] = 0;
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_1_INDEX] = 0;
    // 启动PWM定时器开始计数
    DL_TimerG_startCounter(PWM_0_INST);

    // 电机方向控制引脚默认全部置高，上电刹车状态
    DL_GPIO_setPins(MOTOR_DIR_PORT,
        MOTOR_DIR_BIN_1_PIN | MOTOR_DIR_BIN_2_PIN |
        MOTOR_DIR_AIN_1_PIN | MOTOR_DIR_AIN_2_PIN);

    // 清除编码器引脚残留中断标志
    DL_GPIO_clearInterruptStatus(GPIOA,
        GOIO_GET_GET_2A_PIN | GOIO_GET_GET_2B_PIN |
        GOIO_GET_GET_1A_PIN | GOIO_GET_GET_1B_PIN);
    // 使能四路编码器引脚GPIO中断
    DL_GPIO_enableInterrupt(GPIOA,
        GOIO_GET_GET_2A_PIN | GOIO_GET_GET_2B_PIN |
        GOIO_GET_GET_1A_PIN | GOIO_GET_GET_1B_PIN);

    // 配置编码器引脚中断触发方式：上升沿触发
    DL_GPIO_setUpperPinsPolarity(GPIOA,
        DL_GPIO_PIN_26_EDGE_RISE |
        DL_GPIO_PIN_21_EDGE_RISE |
        DL_GPIO_PIN_27_EDGE_RISE |
        DL_GPIO_PIN_22_EDGE_RISE);

    // 清除NVIC挂起中断，开启GPIOA中断，设置最高中断优先级0
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_SetPriority(GPIOA_INT_IRQn, 0);

    // 上电清零左右轮编码器计数
    enc_left = 0; enc_right = 0;
}

/**
 * @brief 设置电机正转前进模式
 * AIN1/BIN1拉低，AIN2/BIN2拉高，左右轮同向前进
 */
void Motor_SetForward(void)
{
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_1_PIN | MOTOR_DIR_AIN_1_PIN);
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_2_PIN | MOTOR_DIR_AIN_2_PIN);
}

/**
 * @brief 设置电机刹车模式
 * 四路方向引脚全部拉高，H桥上下管同时导通，电机短接制动
 */
void Motor_SetBrake(void)
{
    DL_GPIO_setPins(MOTOR_DIR_PORT,
        MOTOR_DIR_BIN_1_PIN | MOTOR_DIR_BIN_2_PIN |
        MOTOR_DIR_AIN_1_PIN | MOTOR_DIR_AIN_2_PIN);
}

/**
 * @brief 读取并清零左轮编码器脉冲（读取本次20ms周期累计脉冲）
 * @return 本次周期左轮总脉冲数
 * 关中断防止读取过程中脉冲计数被中断修改，读完立即清零
 */
uint32_t Motor_GetLeftPulses(void)
{
    uint32_t val;
    __disable_irq();    // 关闭全局中断，保证原子操作
    val = enc_left;
    enc_left = 0;
    __enable_irq();     // 恢复全局中断
    return val;
}

/**
 * @brief 读取并清零右轮编码器脉冲
 * @return 本次周期右轮总脉冲数
 */
uint32_t Motor_GetRightPulses(void)
{
    uint32_t val;
    __disable_irq();
    val = enc_right;
    enc_right = 0;
    __enable_irq();
    return val;
}

/**
 * @brief 设置左右轮PWM占空比输出
 * @param left 左轮PWM数值 0~1000
 * @param right 右轮PWM数值 0~1000
 * 内部限幅，超过1000强制封顶1000
 */
void Motor_SetPWM(uint16_t left, uint16_t right)
{
    if (left  > 1000) left  = 1000;
    if (right > 1000) right = 1000;
    // CC1对应左轮，CC0对应右轮
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_1_INDEX] = left;
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_0_INDEX] = right;
}

/**
 * @brief 整车停止函数：刹车+PWM清零
 */
void Motor_Stop(void)
{
    Motor_SetBrake();
    Motor_SetPWM(0, 0);
}