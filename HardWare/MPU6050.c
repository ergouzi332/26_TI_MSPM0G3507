#include "MPU6050.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/dl_i2c.h>
#include "ti_msp_dl_config.h"
#include <math.h>
#include <stdlib.h>

// 未定义I2C实例则默认使用I2C0
#ifndef MPU_I2C0_INST
#define MPU_I2C0_INST I2C0
#endif

// MPU6050 I2C器件地址，AD0引脚低电平0x68，高电平0x69
#define MPU6050_ADDR_LOW      0x68
#define MPU6050_ADDR_HIGH     0x69

// 全局静态变量区
static MPU_RawData mpu_raw;               // MPU6050原始传感器数据结构体
static float gz_offset = 0.0f;            // 陀螺仪Z轴静态零偏补偿值
static float yaw_angle = 0.0f;            // 积分得到的航向角Yaw，单位°
static float debug_dps = 0.0f;            // 调试用角速度值 °/s
static int32_t debug_raw_dps100 = 0;      // 放大100倍的角速度，方便整型串口输出
static int16_t last_gz = 0;               // 上一次读取的Z轴原始ADC值
static int mpu_present = 0;               // MPU设备存在标记：1=检测到，0=未识别
static uint8_t mpu_whoami = 0;            // WHO_AM_I寄存器ID
static uint8_t mpu_addr = MPU6050_ADDR_LOW;// 当前使用的I2C设备地址
static uint8_t mpu_last_status = 0;       // 上一次I2C通信状态
static int cal_done = 0;                  // 陀螺仪自动校准完成标记

// 陀螺仪自动校准临时变量
int32_t debug_cal_sum = 0;    // 多组gz原始值累加和
int debug_cal_cnt = 0;        // 有效采样计数

/**
 * @brief I2C总线等待函数，等待总线空闲；总线卡死则复位I2C外设
 */
static void i2c_wait(void)
{
    // 简易延时消抖
    volatile uint32_t w = 5000; while (--w);
    // I2C通信超时阈值
    uint32_t tout = 200000;
    // 循环等待I2C控制器非忙状态
    while ((DL_I2C_getControllerStatus(MPU_I2C0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) && --tout);

    // 超时判定总线挂死，复位并重新初始化I2C
    if (tout == 0) {
        DL_I2C_reset(MPU_I2C0_INST);
        SYSCFG_DL_MPU_I2C0_init();
        MPU_I2C0_INST->MASTER.MFIFOCTL = 0x00;
    }
}

/**
 * @brief MPU6050单寄存器写入
 * @param reg 寄存器地址
 * @param dat 待写入数据
 */
void MPU6050_WriteReg(uint8_t reg, uint8_t dat)
{
    DL_I2C_setTargetAddress(MPU_I2C0_INST, mpu_addr);
    MPU_I2C0_INST->MASTER.MTXDATA = reg;   // 先发送寄存器地址
    MPU_I2C0_INST->MASTER.MTXDATA = dat;   // 再发送写入数据
    // 启动I2C主机发送，长度2字节
    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    i2c_wait();
    mpu_last_status = 0x00;
}

/**
 * @brief MPU6050单寄存器读取
 * @param reg 待读取寄存器地址
 * @return 寄存器返回单字节数据
 */
uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t val = 0;

    DL_I2C_setTargetAddress(MPU_I2C0_INST, mpu_addr);
    MPU_I2C0_INST->MASTER.MTXDATA = reg;
    // 发送寄存器地址（写阶段1字节）
    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    i2c_wait();

    // 短延时等待从机切换收发
    { volatile uint32_t w = 1000; while (--w); }

    // 切换为读模式，读取1字节寄存器数据
    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    i2c_wait();

    val = (uint8_t)MPU_I2C0_INST->MASTER.MRXDATA;
    mpu_last_status = 0x00;
    return val;
}

/**
 * @brief 读取陀螺仪Z轴原始16位ADC数据
 * @return int16_t 高低字节拼接后的原始gz值
 */
int16_t MPU6050_ReadGZ(void)
{
    uint8_t h = MPU6050_ReadReg(0x47); // Z轴高字节寄存器
    uint8_t l = MPU6050_ReadReg(0x48); // Z轴低字节寄存器
    return (int16_t)((h << 8) | l);    // 拼接为有符号16位整数
}

// 外部查询接口：检测MPU是否在线，固定返回1
int      MPU6050_IsPresent(void)      { return 1; }
// 获取WHO_AM_I寄存器存储值
uint8_t  MPU6050_GetWhoAmI(void)      { return mpu_whoami; }
// 获取上一次I2C通信状态
uint8_t  MPU6050_GetLastStatus(void)  { return mpu_last_status; }
// 拷贝原始传感器数据到外部结构体
void     MPU6050_GetRawData(MPU_RawData *out) { *out = mpu_raw; }
// 获取当前I2C通信地址
uint8_t  MPU6050_GetScanAddress(void) { return mpu_addr; }

/**
 * @brief MPU6050完整初始化，上电复位、配置采样率、滤波、量程
 */
void MPU6050_Init(void)
{
    // 上电稳定延时
    DL_Common_delayCycles(4800000U);
    MPU_I2C0_INST->MASTER.MFIFOCTL = 0x05;
    DL_Common_delayCycles(1000U);

    int ok = 0;
    // 循环30次尝试读取ID，检测芯片是否存在
    for (int i = 0; i < 30; i++) {
        uint8_t who = MPU6050_ReadReg(0x75);
        if (who == 0x68) { ok = 1; break; }
        { volatile uint32_t d = 50000; while (--d); }
    }

    // 初次识别失败，重新初始化I2C再重试20次
    if (!ok) {
        SYSCFG_DL_MPU_I2C0_init();
        MPU_I2C0_INST->MASTER.MFIFOCTL = 0x05;
        DL_Common_delayCycles(100000U);
        for (int i = 0; i < 20; i++) {
            uint8_t who = MPU6050_ReadReg(0x75);
            if (who == 0x68) { ok = 1; break; }
            { volatile uint32_t d = 50000; while (--d); }
        }
    }

    // 多次重试仍未检测到MPU，直接退出初始化
    if (!ok) { mpu_present = 0; return; }
    mpu_present = 1;
    mpu_whoami = 0x68;

    // PWR_MGMT_1 0x6B=0x00 唤醒MPU，退出睡眠模式
    MPU6050_WriteReg(0x6B, 0x00);   DL_Common_delayCycles(3200000U);
    // CONFIG 0x1A=0x03 低通滤波带宽44Hz
    MPU6050_WriteReg(0x1A, 0x03);   DL_Common_delayCycles(32000U);
    // SMPLRT_DIV 0x19=0x07 采样分频，输出速率1kHz/(7+1)=125Hz
    MPU6050_WriteReg(0x19, 0x07);   DL_Common_delayCycles(32000U);
    // GYRO_CONFIG 0x1B=0x00 陀螺仪量程±250°/s
    MPU6050_WriteReg(0x1B, 0x00);   DL_Common_delayCycles(32000U);
    // ACCEL_CONFIG 0x1C=0x00 加速度计量程±2g
    MPU6050_WriteReg(0x1C, 0x00);   DL_Common_delayCycles(32000U);

    // 清空I2C发送FIFO
    MPU_I2C0_INST->MASTER.MFIFOCTL = 0x00;
    __NOP();

    yaw_angle = 0.0f; // 初始化航向角归零
}

/**
 * @brief 陀螺仪静止自动校准，采集多组静止gz计算零偏offset
 */
void MPU6050_CalibrateGyro(void)
{
    debug_cal_sum = 0;
    debug_cal_cnt = 0;

    // 采集600组Z轴原始数据
    for (int i = 0; i < 600; i++) {
        int16_t gz = MPU6050_ReadGZ();
        // 剔除大幅晃动数据，仅采集接近静止样本
        if (abs((int)gz) < 3000) {
            debug_cal_sum += gz;
            debug_cal_cnt++;
        }
        DL_Common_delayCycles(160000U);
    }

    // 有效样本大于300才计算零偏，保证校准精度
    if (debug_cal_cnt > 300) {
        int32_t off_x100 = (debug_cal_sum * 100) / debug_cal_cnt;
        gz_offset = (float)off_x100 / 100.0f;
        cal_done = 1;
    } else {
        // 采样晃动过多，校准失败，零偏置0
        gz_offset = 0.0f;
        cal_done = 0;
    }
}

/**
 * @brief 内部读取GZ并积分更新Yaw航向角
 * @param dt 积分时间步长，单位s
 */
void MPU6050_UpdateYawFast(float dt)
{
    if (mpu_present != 1) return;
    int16_t gz_raw = MPU6050_ReadGZ();
    // 减去零偏，转换原始值到 °/s 角速度，±250°/s 对应32768量程
    float gz_dps = (gz_raw - gz_offset) * 250.0f / 32768.0f;
    yaw_angle += gz_dps * dt;
    // 航向角限幅 -180 ~ +180°
    if (yaw_angle > 180.0f)  yaw_angle -= 360.0f;
    if (yaw_angle < -180.0f) yaw_angle += 360.0f;
}

/**
 * @brief 外部传入GZ原始值，积分更新航向角（主循环20ms周期调用）
 * @param gz_raw 外部读取的Z轴原始ADC值
 * @param dt 积分步长时间，单位s
 */
void MPU6050_UpdateYawFromRaw(int16_t gz_raw, float dt)
{
    last_gz = gz_raw;
    // 原始值扣除静态零偏，转为 °/s
    float gz_dps = ((float)gz_raw - gz_offset) * 250.0f / 32768.0f;
    // 微小角速度死区，消除静止积分漂移
    if (fabsf(gz_dps) < 0.5f) { gz_dps = 0.0f; }
    // 角速度积分得到角度
    yaw_angle += gz_dps * dt;
    // 角度循环限幅 -180~180
    if (yaw_angle > 180.0f)  yaw_angle -= 360.0f;
    if (yaw_angle < -180.0f) yaw_angle += 360.0f;
}

// 获取放大100倍角速度（整型调试输出）
int32_t  MPU6050_GetDps100(void)       { return debug_raw_dps100; }
// 获取上一次Z轴原始ADC值
int16_t  MPU6050_GetLastGz(void)       { return last_gz; }
// 获取当前积分航向角Yaw，单位°
float    MPU6050_GetYaw(void)      { return yaw_angle; }
// 航向角强制归零
void     MPU6050_ResetYaw(void)     { yaw_angle = 0.0f; }
// 获取当前Z轴零偏补偿值
float    MPU6050_GetGzOffset(void)     { return gz_offset; }
// 手动设置Z轴零偏补偿（小车静止时动态校准调用）
void     MPU6050_SetGzOffset(float off) { gz_offset = off; }
// 判断陀螺仪自动校准是否完成
int      MPU6050_GetCalDone(void)      { return cal_done; }
// 调试接口：读取默认地址WHO_AM_I寄存器
uint8_t  MPU6050_DebugWho(void)        { return MPU6050_ReadReg(0x75); }
// 调试接口：切换I2C地址后读取WHO_AM_I
uint8_t  MPU6050_DebugWhoAt(uint8_t addr) { mpu_addr = addr; return MPU6050_ReadReg(0x75); }