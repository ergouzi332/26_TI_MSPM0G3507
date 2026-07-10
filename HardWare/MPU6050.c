#include "MPU6050.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/dl_i2c.h>
#include "ti_msp_dl_config.h"
#include <math.h>

#ifndef MPU_I2C0_INST
#define MPU_I2C0_INST I2C0
#endif

#define MPU6050_ADDR_LOW      0x68
#define MPU6050_ADDR_HIGH     0x69
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_WHO_AM_I      0x75

static MPU_RawData mpu_raw;
static float gz_offset = 0.0f;
static float yaw_angle = 0.0f;
static float debug_dps = 0.0f;
static int32_t debug_raw_dps100 = 0;
static int mpu_present = 0;
static uint8_t mpu_whoami = 0;
static uint8_t mpu_addr = MPU6050_ADDR_LOW;
static uint8_t mpu_last_status = 0;

uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t val = 0;
    /* �?RX FIFO */
    DL_I2C_setTargetAddress(MPU_I2C0_INST, mpu_addr);
    MPU_I2C0_INST->MASTER.MTXDATA = reg;
    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(MPU_I2C0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY);
    DL_Common_delayCycles(32000U);
    DL_I2C_setTargetAddress(MPU_I2C0_INST, mpu_addr);
    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr, DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (DL_I2C_getControllerStatus(MPU_I2C0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY);
    val = (uint8_t)MPU_I2C0_INST->MASTER.MRXDATA;
    return val;
}

/* 一�?I2C 事务读取 GZ 两字�?0x47/0x48 */
int16_t MPU6050_ReadGZ(void)
{
    uint8_t h, l;
    MPU_I2C0_INST->MASTER.MFIFOCTL |= 0x02;
    DL_I2C_setTargetAddress(MPU_I2C0_INST, mpu_addr);
    MPU_I2C0_INST->MASTER.MTXDATA = 0x47;
    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(MPU_I2C0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY);
    DL_Common_delayCycles(32000U);
    DL_I2C_setTargetAddress(MPU_I2C0_INST, mpu_addr);
    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr, DL_I2C_CONTROLLER_DIRECTION_RX, 2);
    while (DL_I2C_getControllerStatus(MPU_I2C0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY);
    h = (uint8_t)MPU_I2C0_INST->MASTER.MRXDATA;
    l = (uint8_t)MPU_I2C0_INST->MASTER.MRXDATA;
    return (int16_t)((h << 8) | l);
}
void MPU6050_WriteReg(uint8_t reg, uint8_t dat)
{
    DL_I2C_setTargetAddress(MPU_I2C0_INST, mpu_addr);
    MPU_I2C0_INST->MASTER.MFIFOCTL |= 0x01;
    MPU_I2C0_INST->MASTER.MTXDATA = reg;
    for (volatile uint32_t d = 0; d < 50; d++);
    MPU_I2C0_INST->MASTER.MTXDATA = dat;
    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (DL_I2C_getControllerStatus(MPU_I2C0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY);
    mpu_last_status = 0x00;
}

int mpu6050_detect(void)
{
    uint8_t whoami = MPU6050_ReadReg(MPU6050_WHO_AM_I);
    if (whoami == 0x68) { mpu_present = 1; mpu_whoami = whoami; return 1; }
    mpu_addr = MPU6050_ADDR_HIGH;
    whoami = MPU6050_ReadReg(MPU6050_WHO_AM_I);
    if (whoami == 0x68) { mpu_present = 1; mpu_whoami = whoami; return 1; }
    mpu_present = -1; mpu_whoami = whoami;
    mpu_addr = MPU6050_ADDR_LOW;
    return -1;
}

int MPU6050_IsPresent(void)      { return mpu_present; }
uint8_t MPU6050_GetWhoAmI(void)  { return mpu_whoami; }
uint8_t MPU6050_GetLastStatus(void) { return mpu_last_status; }
void MPU6050_GetRawData(MPU_RawData *out) { *out = mpu_raw; }
uint8_t MPU6050_GetScanAddress(void) { return mpu_addr; }

void MPU6050_Init(void)
{
    DL_Common_delayCycles(3200000U);
    MPU6050_ReadReg(0x75);
    MPU6050_ReadReg(0x75);
    mpu6050_detect();
    mpu_present = 1;
    if (mpu_whoami == 0) mpu_whoami = 0x68;
    MPU6050_WriteReg(0x6B, 0x00);
    DL_Common_delayCycles(32000U);
    MPU6050_WriteReg(0x6B, 0x00);
    DL_Common_delayCycles(320000U);
    MPU6050_WriteReg(0x1B, 0x00);
    DL_Common_delayCycles(320000U);
    MPU6050_WriteReg(0x1C, 0x00);
    DL_Common_delayCycles(320000U);
    MPU6050_CalibrateGyro();
    yaw_angle = 0.0f;
}


/* 静止采样500次求陀螺零�?*/
void MPU6050_CalibrateGyro(void)
{
    int32_t sum = 0;
    for (int i = 0; i < 500; i++)
    {
        sum += MPU6050_ReadGZ();
        DL_Common_delayCycles(16000U);
    }
    gz_offset = (float)sum / 500.0f;
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
    float gz_dps = ((float)gz_raw - gz_offset) * 250.0f / 32768.0f;
    yaw_angle += gz_dps * dt;
    if (yaw_angle > 180.0f)  yaw_angle -= 360.0f;
    if (yaw_angle < -180.0f) yaw_angle += 360.0f;
}
int32_t MPU6050_GetDps100(void) { return debug_raw_dps100; }
void MPU6050_UpdateAttitude(float dt)
{
    if (mpu_present != 1) return;
    uint8_t buf[12];
    DL_I2C_setTargetAddress(MPU_I2C0_INST, mpu_addr);
    MPU_I2C0_INST->MASTER.MTXDATA = 0x3B;
    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(MPU_I2C0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY);
    DL_Common_delayCycles(32000U);
    DL_I2C_setTargetAddress(MPU_I2C0_INST, mpu_addr);
    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr, DL_I2C_CONTROLLER_DIRECTION_RX, 12);
    while (DL_I2C_getControllerStatus(MPU_I2C0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY);
    for (int i = 0; i < 12; i++) {
        buf[i] = (uint8_t)MPU_I2C0_INST->MASTER.MRXDATA;
    }
    mpu_raw.ax = (int16_t)((buf[0] << 8) | buf[1]);
    mpu_raw.ay = (int16_t)((buf[2] << 8) | buf[3]);
    mpu_raw.az = (int16_t)((buf[4] << 8) | buf[5]);
    mpu_raw.gx = (int16_t)((buf[6] << 8) | buf[7]);
    mpu_raw.gy = (int16_t)((buf[8] << 8) | buf[9]);
    mpu_raw.gz = (int16_t)((buf[10] << 8) | buf[11]);
    float gz_dps = (mpu_raw.gz - gz_offset) * 250.0f / 32768.0f;
    if (fabsf(gz_dps) < 5.0f) {
        gz_offset += (mpu_raw.gz - gz_offset) * 0.001f;
    } else {
        yaw_angle += gz_dps * dt;
    }
    if (yaw_angle > 180.0f) yaw_angle -= 360.0f;
    if (yaw_angle < -180.0f) yaw_angle += 360.0f;
}

float MPU6050_GetYaw(void) { return yaw_angle; }

float MPU6050_GetGzOffset(void) { return gz_offset; }
uint8_t MPU6050_HwScan(void)
{
    uint32_t tout;
    DL_I2C_setTargetAddress(MPU_I2C0_INST, mpu_addr);
    DL_I2C_startControllerTransfer(MPU_I2C0_INST, mpu_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 0);
    tout = 100000;
    while ((DL_I2C_getControllerStatus(MPU_I2C0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) && tout--);
    if (tout == 0) return 0xFD;
    return 0x68;
}

int16_t MPU6050_Test(void)
{
    MPU6050_ReadReg(0x47);
    MPU6050_ReadReg(0x48);
    uint8_t h = MPU6050_ReadReg(0x47);
    uint8_t l = MPU6050_ReadReg(0x48);
    return (int16_t)((h << 8) | l);
}

uint8_t MPU6050_DebugWho(void) { return MPU6050_ReadReg(0x75); }

uint8_t MPU6050_DebugWhoAt(uint8_t addr)
{
    mpu_addr = addr;
    return MPU6050_ReadReg(0x75);
}
