#ifndef HARDWARE_H
#define HARDWARE_H

/**
 * @brief Initialize board hardware using SysConfig-generated DL library.
 *
 * This function should be called once at startup to:
 *  - initialize system clocks and GPIO power via SYSCFG_DL_init()
 *  - configure I2C, UART, PWM, capture, and other peripherals
 *  - initialize board drivers in the HardWare folder
 */
void Hardware_Init(void);

#endif /* HARDWARE_H */
