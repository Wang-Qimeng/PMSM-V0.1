#include "key.h"

void key_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOE_CLK_ENABLE();

	GPIO_InitStruct.Pin = key0_Pin|key1_Pin|key2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
}

uint8_t key_scan(void)
{
	static uint8_t key_prev = KEY_NONE;
	static uint32_t key_tick = 0;
	static uint8_t key_lock = 0;
	uint8_t key_now = KEY_NONE;

	if(HAL_GPIO_ReadPin(key0_GPIO_Port, key0_Pin)==GPIO_PIN_RESET)
	{
		key_now = key0;
	}
	else if(HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin)==GPIO_PIN_RESET)
	{
		key_now = key1;
	}
	else if(HAL_GPIO_ReadPin(key2_GPIO_Port, key2_Pin)==GPIO_PIN_RESET)
	{
		key_now = key2;
	}

	if(key_now != key_prev)
	{
		key_prev = key_now;
		key_tick = HAL_GetTick();
		key_lock = 0;
	}

	if(key_now != KEY_NONE && !key_lock)
	{
		if(HAL_GetTick() - key_tick >= 20)
		{
			key_lock = 1;
			return key_now;
		}
	}

	return KEY_NONE;
}