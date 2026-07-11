#include "MPU6050.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/dl_i2c.h>
#include "ti_msp_dl_config.h"
#include <math.h>
#include <stdlib.h>

#ifndef MPU_I2C0_INST
#define MPU_I2C0_INST I2C0
#endif

#define MPU6050_ADDR_LOW      0x68
#define MPU6050_ADDR_HIGH     0x69

static MPU_RawData mpu_raw;
static float gz_offset = 0.0f;
static float yaw_angle = 0.0f;
static float debug_dps = 0.0f;
static int32_t debug_raw_dps100 = 0;
static int16_t last_gz = 0;
static int mpu_present = 0;
static uint8_t mpu_whoami = 0;
static uint8_t mpu_addr = MPU6050_ADDR_LOW;
static uint8_t mpu_last_status = 0;
static int cal_done = 0;

/* ???? */
int32_t debug_cal_sum = 0;
int debug_cal_cnt = 0;

static void i2c_wait(void)
{
    volatile uint32_t w = 5000; while (--w);
    while (DL_I2C_getControllerStatus(MPU_I2C0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY);
}

void MPU6050_WriteReg(uint8_t reg, uint8_t dat)
{
    DL_I2C_setTargetAddress(MPU_I2C0_INST, mpu_addr);
    MPU_I2C0_INST->MASTER.MTXDATA = reg;
    MPU_I2C0_INST->MASTER.MTXDATA = dat;
    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    i2c_wait();
    mpu_last_status = 0x00;
}

uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t val = 0;

    DL_I2C_setTargetAddress(MPU_I2C0_INST, mpu_addr);
    MPU_I2C0_INST->MASTER.MTXDATA = reg;
    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    i2c_wait();

    { volatile uint32_t w = 1000; while (--w); }

    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    i2c_wait();

    val = (uint8_t)MPU_I2C0_INST->MASTER.MRXDATA;
    mpu_last_status = 0x00;
    return val;
}

int16_t MPU6050_ReadGZ(void)
{
    uint8_t h = MPU6050_ReadReg(0x47);
    uint8_t l = MPU6050_ReadReg(0x48);
    return (int16_t)((h << 8) | l);
}

int      MPU6050_IsPresent(void)      { return 1; }
uint8_t  MPU6050_GetWhoAmI(void)      { return mpu_whoami; }
uint8_t  MPU6050_GetLastStatus(void)  { return mpu_last_status; }
void     MPU6050_GetRawData(MPU_RawData *out) { *out = mpu_raw; }
uint8_t  MPU6050_GetScanAddress(void) { return mpu_addr; }

void MPU6050_Init(void)
{
    DL_Common_delayCycles(4800000U);
    MPU_I2C0_INST->MASTER.MFIFOCTL = 0x05;
    DL_Common_delayCycles(1000U);

    int ok = 0;
    for (int i = 0; i < 30; i++) {
        uint8_t who = MPU6050_ReadReg(0x75);
        if (who == 0x68) { ok = 1; break; }
        { volatile uint32_t d = 50000; while (--d); }
    }

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

    if (!ok) { mpu_present = 0; return; }
    mpu_present = 1;
    mpu_whoami = 0x68;

    MPU6050_WriteReg(0x6B, 0x00);   DL_Common_delayCycles(3200000U);
    MPU6050_WriteReg(0x1A, 0x03);   DL_Common_delayCycles(32000U);
    MPU6050_WriteReg(0x19, 0x07);   DL_Common_delayCycles(32000U);
    MPU6050_WriteReg(0x1B, 0x00);   DL_Common_delayCycles(32000U);
    MPU6050_WriteReg(0x1C, 0x00);   DL_Common_delayCycles(32000U);

    /* ? TX FIFO */
    MPU_I2C0_INST->MASTER.MFIFOCTL = 0x00;
    __NOP();

    yaw_angle = 0.0f;
}

void MPU6050_CalibrateGyro(void)
{
    debug_cal_sum = 0;
    debug_cal_cnt = 0;

    for (int i = 0; i < 600; i++) {
        int16_t gz = MPU6050_ReadGZ();
        if (abs((int)gz) < 3000) {
            debug_cal_sum += gz;
            debug_cal_cnt++;
        }
        DL_Common_delayCycles(160000U);
    }

    if (debug_cal_cnt > 300) {
        int32_t off_x100 = (debug_cal_sum * 100) / debug_cal_cnt;
        gz_offset = (float)off_x100 / 100.0f;
        cal_done = 1;
    } else {
        gz_offset = 0.0f;
        cal_done = 0;
    }
}

void MPU6050_UpdateYawFast(float dt)
{
    if (mpu_present != 1) return;
    int16_t gz_raw = MPU6050_ReadGZ();
    float gz_dps = (gz_raw - gz_offset) * 250.0f / 32768.0f;
    yaw_angle += gz_dps * dt;
    if (yaw_angle > 180.0f)  yaw_angle -= 360.0f;
    if (yaw_angle < -180.0f) yaw_angle += 360.0f;
}

void MPU6050_UpdateYawFromRaw(int16_t gz_raw, float dt)
{
    last_gz = gz_raw;
    float gz_dps = ((float)gz_raw - gz_offset) * 250.0f / 32768.0f;
    if (fabsf(gz_dps) < 0.5f) { gz_dps = 0.0f; }
    yaw_angle += gz_dps * dt;
    if (yaw_angle > 180.0f)  yaw_angle -= 360.0f;
    if (yaw_angle < -180.0f) yaw_angle += 360.0f;
}

int32_t  MPU6050_GetDps100(void)       { return debug_raw_dps100; }
int16_t  MPU6050_GetLastGz(void)       { return last_gz; }
float    MPU6050_GetYaw(void)      { return yaw_angle; }
void     MPU6050_ResetYaw(void)     { yaw_angle = 0.0f; }
float    MPU6050_GetGzOffset(void)     { return gz_offset; }
void     MPU6050_SetGzOffset(float off) { gz_offset = off; }
int      MPU6050_GetCalDone(void)      { return cal_done; }
uint8_t  MPU6050_DebugWho(void)        { return MPU6050_ReadReg(0x75); }
uint8_t  MPU6050_DebugWhoAt(uint8_t addr) { mpu_addr = addr; return MPU6050_ReadReg(0x75); }

