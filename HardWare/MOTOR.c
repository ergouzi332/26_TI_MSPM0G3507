#include "MOTOR.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include "ti_msp_dl_config.h"
#define ENCODER_PPR         260
static volatile uint32_t enc_pulses = 0;

#define PID_Kp              1.4f
#define PID_Ki              0.6f
#define PID_Kd              0.0f
#define MIN_PWM_DUTY        80


static int16_t target_rpm = 0;
static int16_t actual_rpm = 0;
static float pid_integral = 0.0f;
volatile uint8_t mpu_flag = 0;  /* TIMG6 ISR ? 1?????? */
void MPU6050_UpdateYawFast(float dt);  /* ISR ??? */
static float pid_prev_err = 0.0f;
static float soft_target = 0.0f;
static uint8_t speed_update_cnt = 0;

/* ======================== ????????????????======================== */
void CAPTURE_B_INST_IRQHandler(void)
{
    if (DL_TimerG_getEnabledInterruptStatus(CAPTURE_B_INST,
        DL_TIMERG_INTERRUPT_CC0_DN_EVENT))
    {
        uint32_t cc0 = CAPTURE_B_INST->COUNTERREGS.CC_01[DL_TIMER_CC_0_INDEX];
        static uint32_t last_cc0 = 0;
        if (cc0 != last_cc0) { enc_pulses++; last_cc0 = cc0; }
        DL_TimerG_clearInterruptStatus(CAPTURE_B_INST,
            DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
    }
}

/* ======================== ?????PWM ======================== */
static inline void pwm_hw_set(uint16_t duty)
{
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_1_INDEX] = duty;
}

/* ======================== ????????????======================== */
void Motor_Init(void)
{
    /* PA13 -> TIMG0 CCP1 */

    /* ?????CCP0 + CCP1 ??????????????????????????? */
    PWM_0_INST->COMMONREGS.CCPD = 0x03;

    /* CCP1 ??????????????<<5?????RC_FUNCVAL(0)???NV_DISABLE(0) */
    PWM_0_INST->COUNTERREGS.OCTL_01[0] = (1 << 5);

    /* ???????????= 0????????? */
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_1_INDEX] = 0;

    /* ?????PWM ???????*/
    DL_TimerG_startCounter(PWM_0_INST);

    /* ???????TIMG7 */
    CAPTURE_B_INST->COUNTERREGS.LOAD = 0xFFFF;
    DL_TimerG_startCounter(CAPTURE_B_INST);
    DL_TimerG_enableInterrupt(CAPTURE_B_INST, DL_TIMERG_INTERRUPT_CC0_DN_EVENT);
    NVIC_EnableIRQ(CAPTURE_B_INST_INT_IRQN);

    /* TIMG6?100ms ????????? MPU6050 */
    CAPTURE_A_INST->COUNTERREGS.LOAD = 0xFFFF;
    DL_TimerG_enableInterrupt(CAPTURE_A_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    NVIC_EnableIRQ(CAPTURE_A_INST_INT_IRQN);
    DL_TimerG_startCounter(CAPTURE_A_INST);


    /* ???????????*/
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_1_PIN);
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_2_PIN);
}

/* ======================== PID ????????? ======================== */
void Motor_SpeedUpdate(void)
{
    speed_update_cnt++;
    if (speed_update_cnt >= 10) {
        speed_update_cnt = 0;

        uint32_t primask = __get_PRIMASK(); __disable_irq();
        uint32_t pulses = enc_pulses; enc_pulses = 0;
        __set_PRIMASK(primask);

        float raw_rpm = (float)pulses * 60.0f / 260.0f / 0.1f;
        static float smooth_rpm = 0.0f;
        smooth_rpm += (raw_rpm - smooth_rpm) * 0.1f;
        if (smooth_rpm > 500.0f) smooth_rpm = 500.0f;
        actual_rpm = (int16_t)(smooth_rpm + 0.5f);

        /* ???????? 10 RPM */
        if (soft_target < (float)target_rpm) {
            soft_target += 45.0f;
            if (soft_target > (float)target_rpm) soft_target = (float)target_rpm;
        } else if (soft_target > (float)target_rpm) {
            soft_target -= 45.0f;
            if (soft_target < (float)target_rpm) soft_target = (float)target_rpm;
        }
        float err = soft_target - smooth_rpm;
        float deriv = (err - pid_prev_err) / 0.1f;
        pid_prev_err = err;

        float output = PID_Kp * err + PID_Ki * pid_integral + PID_Kd * deriv;
        bool saturated_up = (output >= 1000.0f);
        if (output < 0.0f) output = 0.0f;
        if (output > 1000.0f) output = 1000.0f;
        if (target_rpm > 0 && smooth_rpm < 5.0f && output < (float)MIN_PWM_DUTY)
            output = (float)MIN_PWM_DUTY;

        if (!saturated_up) {
            pid_integral += err * 0.1f;
            float max_int = 1000.0f / PID_Ki;
            if (pid_integral > max_int) pid_integral = max_int;
            if (pid_integral < -max_int) pid_integral = -max_int;
        }

        pwm_hw_set((uint16_t)output);

    }
}

void Motor_SetTargetRPM(int16_t rpm)
{
    if (rpm < 0) rpm = 0;
    if (rpm > 500) rpm = 500;
    target_rpm = rpm;
    soft_target = 0.0f;
    pid_integral = 0.0f;
    pid_prev_err = 0.0f;
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_1_PIN);
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_2_PIN);
}

int16_t Motor_GetTargetRPM(void) { return target_rpm; }
int16_t Motor_GetActualRPM(void) { return actual_rpm; }


/* ======================== TIMG6 100ms ????? mpu_flag? ======================== */
void CAPTURE_A_INST_IRQHandler(void)
{
    if (DL_TimerG_getEnabledInterruptStatus(CAPTURE_A_INST,
        DL_TIMERG_INTERRUPT_LOAD_EVENT))
    {
        static uint16_t t6_cnt = 0;
        if (++t6_cnt >= 49) { t6_cnt = 0; mpu_flag = 1; }
        DL_TimerG_clearInterruptStatus(CAPTURE_A_INST,
            DL_TIMERG_INTERRUPT_LOAD_EVENT);
    }
}













