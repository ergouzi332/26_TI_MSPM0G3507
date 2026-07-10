/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMG0
#define PWM_0_INST_IRQHandler                                   TIMG0_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMG0_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                         DL_GPIO_PIN_12
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM34)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_13
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM35)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX



/* Defines for CAPTURE_A */
#define CAPTURE_A_INST                                                   (TIMG6)
#define CAPTURE_A_INST_IRQHandler                               TIMG6_IRQHandler
#define CAPTURE_A_INST_INT_IRQN                                 (TIMG6_INT_IRQn)
#define CAPTURE_A_INST_LOAD_VALUE                                           (0U)
/* GPIO defines for channel 0 */
#define GPIO_CAPTURE_A_C0_PORT                                             GPIOA
#define GPIO_CAPTURE_A_C0_PIN                                     DL_GPIO_PIN_21
#define GPIO_CAPTURE_A_C0_IOMUX                                  (IOMUX_PINCM46)
#define GPIO_CAPTURE_A_C0_IOMUX_FUNC                 IOMUX_PINCM46_PF_TIMG6_CCP0

/* Defines for CAPTURE_B */
#define CAPTURE_B_INST                                                   (TIMG7)
#define CAPTURE_B_INST_IRQHandler                               TIMG7_IRQHandler
#define CAPTURE_B_INST_INT_IRQN                                 (TIMG7_INT_IRQn)
#define CAPTURE_B_INST_LOAD_VALUE                                           (0U)
/* GPIO defines for channel 0 */
#define GPIO_CAPTURE_B_C0_PORT                                             GPIOA
#define GPIO_CAPTURE_B_C0_PIN                                     DL_GPIO_PIN_26
#define GPIO_CAPTURE_B_C0_IOMUX                                  (IOMUX_PINCM59)
#define GPIO_CAPTURE_B_C0_IOMUX_FUNC                 IOMUX_PINCM59_PF_TIMG7_CCP0






/* Defines for MPU_I2C0 */
#define MPU_I2C0_INST                                                       I2C0
#define MPU_I2C0_INST_IRQHandler                                 I2C0_IRQHandler
#define MPU_I2C0_INST_INT_IRQN                                     I2C0_INT_IRQn
#define MPU_I2C0_BUS_SPEED_HZ                                             100000
#define GPIO_MPU_I2C0_SDA_PORT                                             GPIOA
#define GPIO_MPU_I2C0_SDA_PIN                                      DL_GPIO_PIN_0
#define GPIO_MPU_I2C0_IOMUX_SDA                                   (IOMUX_PINCM1)
#define GPIO_MPU_I2C0_IOMUX_SDA_FUNC                    IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_MPU_I2C0_SCL_PORT                                             GPIOA
#define GPIO_MPU_I2C0_SCL_PIN                                      DL_GPIO_PIN_1
#define GPIO_MPU_I2C0_IOMUX_SCL                                   (IOMUX_PINCM2)
#define GPIO_MPU_I2C0_IOMUX_SCL_FUNC                    IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for UART_3 */
#define UART_3_INST                                                        UART3
#define UART_3_INST_FREQUENCY                                           32000000
#define UART_3_INST_IRQHandler                                  UART3_IRQHandler
#define UART_3_INST_INT_IRQN                                      UART3_INT_IRQn
#define GPIO_UART_3_RX_PORT                                                GPIOB
#define GPIO_UART_3_TX_PORT                                                GPIOB
#define GPIO_UART_3_RX_PIN                                         DL_GPIO_PIN_3
#define GPIO_UART_3_TX_PIN                                         DL_GPIO_PIN_2
#define GPIO_UART_3_IOMUX_RX                                     (IOMUX_PINCM16)
#define GPIO_UART_3_IOMUX_TX                                     (IOMUX_PINCM15)
#define GPIO_UART_3_IOMUX_RX_FUNC                      IOMUX_PINCM16_PF_UART3_RX
#define GPIO_UART_3_IOMUX_TX_FUNC                      IOMUX_PINCM15_PF_UART3_TX
#define UART_3_BAUD_RATE                                                  (9600)
#define UART_3_IBRD_32_MHZ_9600_BAUD                                       (208)
#define UART_3_FBRD_32_MHZ_9600_BAUD                                        (21)





/* Port definition for Pin Group BUZZER */
#define BUZZER_PORT                                                      (GPIOB)

/* Defines for BUZZER_OUT: GPIOB.24 with pinCMx 52 on package pin 42 */
#define BUZZER_BUZZER_OUT_PIN                                   (DL_GPIO_PIN_24)
#define BUZZER_BUZZER_OUT_IOMUX                                  (IOMUX_PINCM52)
/* Port definition for Pin Group MOTOR_DIR */
#define MOTOR_DIR_PORT                                                   (GPIOA)

/* Defines for BIN_2: GPIOA.2 with pinCMx 7 on package pin 8 */
#define MOTOR_DIR_BIN_2_PIN                                      (DL_GPIO_PIN_2)
#define MOTOR_DIR_BIN_2_IOMUX                                     (IOMUX_PINCM7)
/* Defines for BIN_1: GPIOA.7 with pinCMx 14 on package pin 13 */
#define MOTOR_DIR_BIN_1_PIN                                      (DL_GPIO_PIN_7)
#define MOTOR_DIR_BIN_1_IOMUX                                    (IOMUX_PINCM14)
/* Defines for AIN_1: GPIOA.8 with pinCMx 19 on package pin 16 */
#define MOTOR_DIR_AIN_1_PIN                                      (DL_GPIO_PIN_8)
#define MOTOR_DIR_AIN_1_IOMUX                                    (IOMUX_PINCM19)
/* Defines for AIN_2: GPIOA.9 with pinCMx 20 on package pin 17 */
#define MOTOR_DIR_AIN_2_PIN                                      (DL_GPIO_PIN_9)
#define MOTOR_DIR_AIN_2_IOMUX                                    (IOMUX_PINCM20)
/* Port definition for Pin Group Grayscale_Sensor */
#define Grayscale_Sensor_PORT                                            (GPIOB)

/* Defines for OUT: GPIOB.6 with pinCMx 23 on package pin 20 */
#define Grayscale_Sensor_OUT_PIN                                 (DL_GPIO_PIN_6)
#define Grayscale_Sensor_OUT_IOMUX                               (IOMUX_PINCM23)
/* Defines for AD0: GPIOB.7 with pinCMx 24 on package pin 21 */
#define Grayscale_Sensor_AD0_PIN                                 (DL_GPIO_PIN_7)
#define Grayscale_Sensor_AD0_IOMUX                               (IOMUX_PINCM24)
/* Defines for AD1: GPIOB.8 with pinCMx 25 on package pin 22 */
#define Grayscale_Sensor_AD1_PIN                                 (DL_GPIO_PIN_8)
#define Grayscale_Sensor_AD1_IOMUX                               (IOMUX_PINCM25)
/* Defines for AD2: GPIOB.9 with pinCMx 26 on package pin 23 */
#define Grayscale_Sensor_AD2_PIN                                 (DL_GPIO_PIN_9)
#define Grayscale_Sensor_AD2_IOMUX                               (IOMUX_PINCM26)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOA)

/* Defines for KEY_1: GPIOA.25 with pinCMx 55 on package pin 45 */
#define KEY_KEY_1_PIN                                           (DL_GPIO_PIN_25)
#define KEY_KEY_1_IOMUX                                          (IOMUX_PINCM55)
/* Defines for KEY_2: GPIOA.24 with pinCMx 54 on package pin 44 */
#define KEY_KEY_2_PIN                                           (DL_GPIO_PIN_24)
#define KEY_KEY_2_IOMUX                                          (IOMUX_PINCM54)
/* Defines for KEY_3: GPIOA.23 with pinCMx 53 on package pin 43 */
#define KEY_KEY_3_PIN                                           (DL_GPIO_PIN_23)
#define KEY_KEY_3_IOMUX                                          (IOMUX_PINCM53)
/* Port definition for Pin Group OLED */
#define OLED_PORT                                                        (GPIOA)

/* Defines for OLED_SCL: GPIOA.15 with pinCMx 37 on package pin 30 */
#define OLED_OLED_SCL_PIN                                       (DL_GPIO_PIN_15)
#define OLED_OLED_SCL_IOMUX                                      (IOMUX_PINCM37)
/* Defines for OLED_SDA: GPIOA.16 with pinCMx 38 on package pin 31 */
#define OLED_OLED_SDA_PIN                                       (DL_GPIO_PIN_16)
#define OLED_OLED_SDA_IOMUX                                      (IOMUX_PINCM38)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_CAPTURE_A_init(void);
void SYSCFG_DL_CAPTURE_B_init(void);
void SYSCFG_DL_MPU_I2C0_init(void);
void SYSCFG_DL_UART_3_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
