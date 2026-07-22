#include "MOTOR.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include "ti_msp_dl_config.h"

// 缂栫爜鍣ㄦ瘡杞€昏剦鍐叉暟锛岀數鏈哄崟鍦堢紪鐮佸櫒鑴夊啿鍊?
#define PULSE_PER_REV  520

// 宸﹀彸杞紪鐮佸櫒鍏ㄥ眬鑴夊啿璁℃暟锛屼腑鏂唴淇敼鍔爒olatile闃叉缂栬瘧鍣ㄤ紭鍖?
volatile uint32_t enc_left = 0;
volatile uint32_t enc_right = 0;

/**
 * @brief GPIOA缁勪腑鏂湇鍔″嚱鏁帮紝缂栫爜鍣ˋB鐩镐笂鍗囨部瑙﹀彂
 * 鍥涜矾缂栫爜鍣ㄥ紩鑴氫腑鏂繘鍏ワ紝姣忔娴嬪埌杈规部鑴夊啿璁℃暟+1锛堝洓鍊嶉璁℃暟閫昏緫锛?
 */
void GROUP1_IRQHandler(void)
{
    // 璇诲彇鏈瑙﹀彂鐨勪腑鏂爣蹇椾綅
    uint32_t flags = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        GOIO_GET_GET_2A_PIN | GOIO_GET_GET_2B_PIN |
        GOIO_GET_GET_1A_PIN | GOIO_GET_GET_1B_PIN);

    // 宸﹁疆缂栫爜鍣?A鐩歌竟娌匡紝宸﹁疆鑴夊啿+1
    if (flags & GOIO_GET_GET_2A_PIN) { enc_left++; }
    // 宸﹁疆缂栫爜鍣?B鐩歌竟娌匡紝宸﹁疆鑴夊啿+1
    if (flags & GOIO_GET_GET_2B_PIN) { enc_left++; }
    // 鍙宠疆缂栫爜鍣?A鐩歌竟娌匡紝鍙宠疆鑴夊啿+1
    if (flags & GOIO_GET_GET_1A_PIN) { enc_right++; }
    // 鍙宠疆缂栫爜鍣?B鐩歌竟娌匡紝鍙宠疆鑴夊啿+1
    if (flags & GOIO_GET_GET_1B_PIN) { enc_right++; }

    // 娓呴櫎鎵€鏈夌紪鐮佸櫒寮曡剼涓柇鏍囧織锛屽惁鍒欎細閲嶅杩涗腑鏂?
    DL_GPIO_clearInterruptStatus(GPIOA,
        GOIO_GET_GET_2A_PIN | GOIO_GET_GET_2B_PIN |
        GOIO_GET_GET_1A_PIN | GOIO_GET_GET_1B_PIN);
}

/**
 * @brief 鐢垫満搴曞眰鍒濆鍖栵細PWM杈撳嚭銆佹柟鍚戞帶鍒禝O銆佺紪鐮佸櫒涓柇閰嶇疆
 */
void Motor_Init(void)
{
    // PWM鍒嗛閰嶇疆锛岃缃甈WM鏃堕挓鍒嗛鍙傛暟
    PWM_0_INST->COMMONREGS.CCPD = 0x03;
    // 寮€鍚畾鏃跺櫒姣旇緝杈撳嚭浣胯兘
    PWM_0_INST->COUNTERREGS.OCTL_01[0] |= (1 << 5);
    // 宸﹀彸杞甈WM鍒濆鍗犵┖姣?锛岀數鏈洪潤姝?
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_0_INDEX] = 0;
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_1_INDEX] = 0;
    // 鍚姩PWM瀹氭椂鍣ㄥ紑濮嬭鏁?
    DL_TimerG_startCounter(PWM_0_INST);

    // 鐢垫満鏂瑰悜鎺у埗寮曡剼榛樿鍏ㄩ儴缃珮锛屼笂鐢靛埞杞︾姸鎬?
    DL_GPIO_setPins(MOTOR_DIR_PORT,
        MOTOR_DIR_BIN_1_PIN | MOTOR_DIR_BIN_2_PIN |
        MOTOR_DIR_AIN_1_PIN | MOTOR_DIR_AIN_2_PIN);

    // 娓呴櫎缂栫爜鍣ㄥ紩鑴氭畫鐣欎腑鏂爣蹇?
    DL_GPIO_clearInterruptStatus(GPIOA,
        GOIO_GET_GET_2A_PIN | GOIO_GET_GET_2B_PIN |
        GOIO_GET_GET_1A_PIN | GOIO_GET_GET_1B_PIN);
    // 浣胯兘鍥涜矾缂栫爜鍣ㄥ紩鑴欸PIO涓柇
    DL_GPIO_enableInterrupt(GPIOA,
        GOIO_GET_GET_2A_PIN | GOIO_GET_GET_2B_PIN |
        GOIO_GET_GET_1A_PIN | GOIO_GET_GET_1B_PIN);

    // 閰嶇疆缂栫爜鍣ㄥ紩鑴氫腑鏂Е鍙戞柟寮忥細涓婂崌娌胯Е鍙?
    DL_GPIO_setUpperPinsPolarity(GPIOA,
        DL_GPIO_PIN_26_EDGE_RISE |
        DL_GPIO_PIN_21_EDGE_RISE |
        DL_GPIO_PIN_27_EDGE_RISE |
        DL_GPIO_PIN_22_EDGE_RISE);

    // 娓呴櫎NVIC鎸傝捣涓柇锛屽紑鍚疓PIOA涓柇锛岃缃渶楂樹腑鏂紭鍏堢骇0
    NVIC_ClearPendingIRQ(GPIOA_INT_IRQn);
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
    NVIC_SetPriority(GPIOA_INT_IRQn, 0);

    // 涓婄數娓呴浂宸﹀彸杞紪鐮佸櫒璁℃暟
    enc_left = 0; enc_right = 0;
}

/**
 * @brief 璁剧疆鐢垫満姝ｈ浆鍓嶈繘妯″紡
 * AIN1/BIN1鎷変綆锛孉IN2/BIN2鎷夐珮锛屽乏鍙宠疆鍚屽悜鍓嶈繘
 */
void Motor_SetForward(void)
{
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_1_PIN | MOTOR_DIR_AIN_1_PIN);
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_2_PIN | MOTOR_DIR_AIN_2_PIN);
}

/**
 * @brief 璁剧疆鐢垫満鍒硅溅妯″紡
 * 鍥涜矾鏂瑰悜寮曡剼鍏ㄩ儴鎷夐珮锛孒妗ヤ笂涓嬬鍚屾椂瀵奸€氾紝鐢垫満鐭帴鍒跺姩
 */
void Motor_SetBrake(void)
{
    DL_GPIO_setPins(MOTOR_DIR_PORT,
        MOTOR_DIR_BIN_1_PIN | MOTOR_DIR_BIN_2_PIN |
        MOTOR_DIR_AIN_1_PIN | MOTOR_DIR_AIN_2_PIN);
}

/**
 * @brief 璇诲彇骞舵竻闆跺乏杞紪鐮佸櫒鑴夊啿锛堣鍙栨湰娆?0ms鍛ㄦ湡绱鑴夊啿锛?
 * @return 鏈鍛ㄦ湡宸﹁疆鎬昏剦鍐叉暟
 * 鍏充腑鏂槻姝㈣鍙栬繃绋嬩腑鑴夊啿璁℃暟琚腑鏂慨鏀癸紝璇诲畬绔嬪嵆娓呴浂
 */
uint32_t Motor_GetLeftPulses(void)
{
    uint32_t val;
    __disable_irq();    // 鍏抽棴鍏ㄥ眬涓柇锛屼繚璇佸師瀛愭搷浣?
    val = enc_left;
    enc_left = 0;
    __enable_irq();     // 鎭㈠鍏ㄥ眬涓柇
    return val;
}

/**
 * @brief 璇诲彇骞舵竻闆跺彸杞紪鐮佸櫒鑴夊啿
 * @return 鏈鍛ㄦ湡鍙宠疆鎬昏剦鍐叉暟
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
 * @brief 璁剧疆宸﹀彸杞甈WM鍗犵┖姣旇緭鍑?
 * @param left 宸﹁疆PWM鏁板€?0~1000
 * @param right 鍙宠疆PWM鏁板€?0~1000
 * 鍐呴儴闄愬箙锛岃秴杩?000寮哄埗灏侀《1000
 */
void Motor_SetPWM(uint16_t left, uint16_t right)
{
    if (left  > 1000) left  = 1000;
    if (right > 1000) right = 1000;
    // CC1瀵瑰簲宸﹁疆锛孋C0瀵瑰簲鍙宠疆
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_1_INDEX] = left;
    PWM_0_INST->COUNTERREGS.CC_01[DL_TIMER_CC_0_INDEX] = right;
}

/**
 * @brief 鏁磋溅鍋滄鍑芥暟锛氬埞杞?PWM娓呴浂
 */

void Motor_PivotLeft(void)
{
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN_1_PIN | MOTOR_DIR_AIN_2_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_1_PIN);
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_2_PIN);
}

/**
 * @brief 坦克左转：左轮反转、右轮正转，原地旋转
 * AIN1=1, AIN2=0 -> 左轮反转; BIN1=0, BIN2=1 -> 右轮正转
 */
void Motor_TankLeft(void)
{
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN_1_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_AIN_2_PIN);
    DL_GPIO_clearPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_1_PIN);
    DL_GPIO_setPins(MOTOR_DIR_PORT, MOTOR_DIR_BIN_2_PIN);
}

void Motor_Stop(void)
{
    Motor_SetBrake();
    Motor_SetPWM(0, 0);
}
