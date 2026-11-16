#ifndef __INPUT_DETECTION_H
#define __INPUT_DETECTION_H

#include "main.h"

static inline void check_aux(void)
{
    if (HAL_GPIO_ReadPin(SW_LEFT_GPIO_Port, SW_LEFT_Pin) == GPIO_PIN_SET) {
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
      tas_play(LEFT_TAS_ADDR);
      tas_play(RIGHT_TAS_ADDR);
    } else {
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
      tas_deep_sleep(LEFT_TAS_ADDR);
      tas_deep_sleep(RIGHT_TAS_ADDR);
    }
}

#endif