#ifndef KEY_H
#define KEY_H

#include <stdint.h>

/* bit0=KEY1, bit1=KEY2, bit2=KEY3, bit3=KEY4 */
#define KEY_1   0x01
#define KEY_2   0x02
#define KEY_3   0x04
#define KEY_4   0x08

void    KEY_Init(void);
uint8_t KEY_Scan(void);    /* return pressed keys bitmask (with debounce) */
uint8_t KEY_GetState(void); /* return current key state bitmask (no debounce) */

#endif
