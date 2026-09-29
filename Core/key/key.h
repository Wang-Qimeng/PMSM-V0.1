#ifndef __KEY_H__
#define __KEY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Key pin defines */
#define key0_Pin       GPIO_PIN_2
#define key0_GPIO_Port GPIOE
#define key1_Pin       GPIO_PIN_3
#define key1_GPIO_Port GPIOE
#define key2_Pin       GPIO_PIN_4
#define key2_GPIO_Port GPIOE
#define KEY_NONE       0
#define key0           1
#define key1           2
#define key2           3

void key_init(void);
uint8_t key_scan(void);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H__ */

