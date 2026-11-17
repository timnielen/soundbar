#ifndef __INPUT_DETECTION_H
#define __INPUT_DETECTION_H

#include "main.h"
#include "sounds.h"

static volatile uint32_t swLastEdge[5] = {0};
static volatile uint8_t  swPending[5]  = {0};

#define SW_LEFT_INDEX 0
#define BTN_SOURCE 1
#define BTN_VOLUME_DOWN 2
#define BTN_VOLUME_UP 3
#define BTN_ON_OFF 4


static inline void check_aux(void)
{
    uint8_t on = (HAL_GPIO_ReadPin(SW_LEFT_GPIO_Port, SW_LEFT_Pin) == GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    audio_set(on);
}

static inline void handle_inputs(void)
{
  for(uint8_t i = 0; i < 5; i++) {
    if (swPending[i] && (HAL_GetTick() - swLastEdge[i]) > 30) // 30 ms with no further edges
    {
      swPending[i] = 0;
      switch (i) {
        case SW_LEFT_INDEX:
          check_aux();
          break;
        case BTN_ON_OFF:
          mcu_sleep();
          break;
        case BTN_VOLUME_UP:
          volume_up();
          break;
        case BTN_VOLUME_DOWN:
          volume_down();
          break;
        case BTN_SOURCE:
          play_sound(ERROR_SOUND);
          break;
        default:
          break;
      }
    }
  }
}

#endif