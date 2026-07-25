#ifndef __BUTTON_H__
#define __BUTTON_H__

#include "CH58x_common.h"

enum keys {
	KEY1 = 0,
	KEY2,
	KEY3,
	KEY4,
	KEY_INDEX,
};

#define KEY2_PIN (GPIO_Pin_22) // PB
#define KEY1_PIN (GPIO_Pin_1) // PA

#ifdef HARDWARE_B1144C
/* KEY1 is active-low on this board, same as KEY2. See btn_init(). */
#define isPressed(key) 		((key) ? \
				!GPIOB_ReadPortPin(KEY2_PIN) : \
				!GPIOA_ReadPortPin(KEY1_PIN))
#else
#define isPressed(key) 		((key) ? \
				!GPIOB_ReadPortPin(KEY2_PIN) : \
				GPIOA_ReadPortPin(KEY1_PIN))
#endif

void btn_onOnePress(int key, void (*handler)(void));
void btn_onLongPress(int key, void (*handler)(void));
void btn_init();
void btn_init_task(void);
void btn_tick(void);

#endif /* __BUTTON_H__ */
