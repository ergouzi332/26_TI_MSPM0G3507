#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} MPU_RawData;

void MPU6050_Init(void);
void MPU6050_UpdateAttitude(float dt);
float MPU6050_GetYaw(void);
void MPU6050_WriteReg(uint8_t reg, uint8_t dat);
uint8_t MPU6050_ReadReg(uint8_t reg);
int mpu6050_detect(void);
int MPU6050_IsPresent(void);
uint8_t MPU6050_GetWhoAmI(void);
uint8_t MPU6050_GetLastStatus(void);
uint8_t MPU6050_ScanFirstAddress(void);
uint8_t MPU6050_GetScanAddress(void);
void MPU6050_GetRawData(MPU_RawData *out);
void MPU6050_UpdateYawFast(float dt);

#endif

