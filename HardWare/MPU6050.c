#include "MPU6050.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include "ti_msp_dl_config.h"
#include <math.h>

#define MPU6050_ADDR_LOW      0x68
#define MPU6050_ADDR_HIGH     0x69
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_WHO_AM_I      0x75

#define SW_PORT             MPU6050_PORT
#define SW_PIN_SDA          MPU6050_MPU6050_SDA_PIN
#define SW_PIN_SCL          MPU6050_MPU6050_SCL_PIN
#define SW_IOMUX_SDA        MPU6050_MPU6050_SDA_IOMUX
#define SW_IOMUX_SCL        MPU6050_MPU6050_SCL_IOMUX

static MPU_RawData mpu_raw;
static float gz_offset = 0.0f;
static float yaw_angle = 0.0f;

static int mpu_present = 0;
static uint8_t mpu_whoami = 0;
static uint8_t mpu_addr = MPU6050_ADDR_LOW;
static uint8_t mpu_last_status = 0;

/* ================== ?? I2C ================== */

static void sw_delay(void)
{
    volatile uint32_t i = 120;
    while (i--) {}
}

#define SCL_LOW()   DL_GPIO_clearPins(SW_PORT, SW_PIN_SCL)
#define SCL_HIGH()  DL_GPIO_setPins(SW_PORT, SW_PIN_SCL)
#define SDA_LOW()   DL_GPIO_clearPins(SW_PORT, SW_PIN_SDA)
#define SDA_HIGH()  DL_GPIO_setPins(SW_PORT, SW_PIN_SDA)

static uint8_t sda_read_input(void)
{
    DL_GPIO_initDigitalInputFeatures(SW_IOMUX_SDA,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    sw_delay();
    return (DL_GPIO_readPins(SW_PORT, SW_PIN_SDA) != 0) ? 1 : 0;
}

static void sda_restore_output(void)
{
    DL_GPIO_initDigitalOutput(SW_IOMUX_SDA);
    DL_GPIO_enableOutput(SW_PORT, SW_PIN_SDA);
}

static void sw_start(void)
{
    SDA_HIGH(); SCL_HIGH(); sw_delay(); SDA_LOW(); sw_delay(); SCL_LOW(); sw_delay();
}

static void sw_stop(void)
{
    SDA_LOW(); sw_delay(); SCL_HIGH(); sw_delay(); SDA_HIGH(); sw_delay();
}

static void sw_write_bit(uint8_t bit)
{
    if (bit) SDA_HIGH(); else SDA_LOW();
    sw_delay(); SCL_HIGH(); sw_delay(); SCL_LOW(); sw_delay();
}

static uint8_t sw_wait_ack(void)
{
    uint8_t ack;
    SDA_HIGH(); sw_delay(); SCL_HIGH(); sw_delay();
    ack = (sda_read_input() == 0);
    sda_restore_output();
    SCL_LOW(); sw_delay();
    return ack;
}

static void sw_send_byte(uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        sw_write_bit((byte & 0x80) ? 1 : 0);
        byte <<= 1;
    }
}

static uint8_t sw_read_byte(void)
{
    uint8_t byte = 0;
    sda_read_input();
    for (uint8_t b = 0; b < 8; b++) {
        SCL_HIGH(); sw_delay();
        byte = (byte << 1) | (sda_read_input() ? 1 : 0);
        SCL_LOW(); sw_delay();
    }
    sda_restore_output();
    return byte;
}

static int sw_tx(uint8_t addr, const uint8_t *data, uint8_t len)
{
    sw_start();
    sw_send_byte((addr << 1) | 0);
    if (!sw_wait_ack()) { sw_stop(); return -1; }
    for (uint8_t i = 0; i < len; i++) {
        sw_send_byte(data[i]);
        if (!sw_wait_ack()) { sw_stop(); return -1; }
    }
    sw_stop();
    return 0;
}

static int sw_tx_rx(uint8_t addr, uint8_t regAddr, uint8_t *buf, uint8_t rxLen)
{
    if (rxLen == 0) return 0;

    sw_start();
    sw_send_byte((addr << 1) | 0);
    if (!sw_wait_ack()) { sw_stop(); return 0; }
    sw_send_byte(regAddr);
    if (!sw_wait_ack()) { sw_stop(); return 0; }

    sw_start();
    sw_send_byte((addr << 1) | 1);
    if (!sw_wait_ack()) { sw_stop(); return 0; }

    for (uint8_t i = 0; i < rxLen; i++) {
        buf[i] = sw_read_byte();
        if (i < rxLen - 1) SDA_LOW(); else SDA_HIGH();
        sw_delay(); SCL_HIGH(); sw_delay(); SCL_LOW(); sw_delay();
    }

    sw_stop();
    return rxLen;
}

uint8_t MPU6050_ScanFirstAddress(void)
{
    for (uint8_t addr = 1; addr < 0x7F; addr++)
    {
        uint8_t dummy = 0;
        if (sw_tx(addr, &dummy, 1) == 0)
            return addr;
    }
    return 0;
}

/* ================== ?? API ================== */

void MPU6050_WriteReg(uint8_t reg, uint8_t dat)
{
    uint8_t buf[2] = {reg, dat};
    mpu_last_status = (sw_tx(mpu_addr, buf, 2) < 0) ? 0xE1 : 0x00;
}

uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t val = 0;
    int ret = sw_tx_rx(mpu_addr, reg, &val, 1);
    mpu_last_status = (ret == 1) ? 0x00 : 0xE3;
    return val;
}

static void MPU6050_ReadAll(MPU_RawData *dat)
{
    uint8_t axh = MPU6050_ReadReg(0x3B);
    uint8_t axl = MPU6050_ReadReg(0x3C);
    uint8_t ayh = MPU6050_ReadReg(0x3D);
    uint8_t ayl = MPU6050_ReadReg(0x3E);
    uint8_t azh = MPU6050_ReadReg(0x3F);
    uint8_t azl = MPU6050_ReadReg(0x40);
    uint8_t gxh = MPU6050_ReadReg(0x43);
    uint8_t gxl = MPU6050_ReadReg(0x44);
    uint8_t gyh = MPU6050_ReadReg(0x45);
    uint8_t gyl = MPU6050_ReadReg(0x46);
    uint8_t gzh = MPU6050_ReadReg(0x47);
    uint8_t gzl = MPU6050_ReadReg(0x48);
    dat->ax = (int16_t)((axh << 8) | axl);
    dat->ay = (int16_t)((ayh << 8) | ayl);
    dat->az = (int16_t)((azh << 8) | azl);
    dat->gx = (int16_t)((gxh << 8) | gxl);
    dat->gy = (int16_t)((gyh << 8) | gyl);
    dat->gz = (int16_t)((gzh << 8) | gzl);
    mpu_last_status = 0x00;
}

int mpu6050_detect(void)
{
    uint8_t whoami;
    int retry;

    /* ?? 3 ???? 50ms */
    for (retry = 0; retry < 3; retry++)
    {
        mpu_addr = MPU6050_ADDR_LOW;
        whoami = MPU6050_ReadReg(MPU6050_WHO_AM_I);
        if (whoami == MPU6050_ADDR_LOW)
        {
            mpu_whoami = whoami;
            mpu_present = 1;
            return 1;
        }

        mpu_addr = MPU6050_ADDR_HIGH;
        whoami = MPU6050_ReadReg(MPU6050_WHO_AM_I);
        if (whoami == MPU6050_ADDR_LOW)
        {
            mpu_whoami = whoami;
            mpu_present = 1;
            return 1;
        }

        DL_Common_delayCycles(1600000U);  /* ~50ms ???? */
    }

    mpu_present = -1;
    mpu_whoami = whoami;
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
    mpu_present = mpu6050_detect();
    if (mpu_present != 1)
    {
        mpu_last_status = 0xFB;
        return;
    }

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);
    DL_Common_delayCycles(320000U);
    MPU6050_WriteReg(0x1A, 0x00);
    DL_Common_delayCycles(320000U);
    MPU6050_WriteReg(0x1B, 0x00);
    DL_Common_delayCycles(320000U);
    MPU6050_WriteReg(0x1C, 0x00);
    DL_Common_delayCycles(320000U);

    /* ?? */
    {
        float sum_gz = 0.0f;
        for (uint16_t i = 0; i < 200; i++)
        {
            MPU6050_ReadAll(&mpu_raw);
            sum_gz += mpu_raw.gz;
            DL_Common_delayCycles(32000U);
        }
        gz_offset = sum_gz / 200.0f;
    }
    yaw_angle = 0.0f;
}

void MPU6050_UpdateAttitude(float dt)
{
    if (mpu_present != 1) return;
    MPU6050_ReadAll(&mpu_raw);

    float gz_dps = (mpu_raw.gz - gz_offset) * 250.0f / 32768.0f;

    if (fabsf(gz_dps) < 5.0f)
    {
        gz_offset += (mpu_raw.gz - gz_offset) * 0.001f;
    }
    else
    {
        yaw_angle += gz_dps * dt;
    }

    if (yaw_angle > 180.0f)  yaw_angle -= 360.0f;
    if (yaw_angle < -180.0f) yaw_angle += 360.0f;
}

void MPU6050_UpdateYawFast(float dt)
{
    if (mpu_present != 1) return;
    uint8_t gzh = MPU6050_ReadReg(0x47);
    uint8_t gzl = MPU6050_ReadReg(0x48);
    float gz_dps = ((int16_t)((gzh << 8) | gzl) - gz_offset) * 250.0f / 32768.0f;
    static float gz_dc = 0.0f;

    /* ???? ? ???????
     * ????? ~10ms?TIMG6 ? ~100ms ????????
     * ? = 0.05 ? ???? ? 2 ?????? 0.001 (100 ?) ??? */
    gz_dc += (gz_dps - gz_dc) * 0.05f;

    yaw_angle += (gz_dps - gz_dc) * dt;

    if (yaw_angle > 180.0f) yaw_angle -= 360.0f;
    if (yaw_angle < -180.0f) yaw_angle += 360.0f;
}

float MPU6050_GetYaw(void) { return yaw_angle; }
