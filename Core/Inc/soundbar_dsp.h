/*
 * soundbar_dsp.h - what OUR soundbar does with the two TAS5805M
 */
#ifndef SOUNDBAR_DSP_H
#define SOUNDBAR_DSP_H

#include <stdint.h>
#include <stdbool.h>
#include "tas5805m.h"

/* Call order (see example_main.c):
 *   1. soundbar_dsp_power_on()      PDN high
 *   2. start the I2S clocks          (ADC_MASTER high, or the ESP32)
 *   3. soundbar_dsp_start(48000)    reset, configure, play             */
void              soundbar_dsp_power_on(void);
HAL_StatusTypeDef soundbar_dsp_start(uint32_t fs_in);
HAL_StatusTypeDef soundbar_dsp_stop(void);        /* mute, Hi-Z, PDN low */

HAL_StatusTypeDef soundbar_dsp_set_volume(float db);      /* <= 0 dB    */
HAL_StatusTypeDef soundbar_dsp_set_rate(uint32_t fs_in);  /* 44.1k / 48k */

/* Call every ~100 ms while playing. Returns true if a fault was seen
 * (and cleared); *left / *right receive the raw fault registers.    */
bool soundbar_dsp_poll(tas_faults_t *left, tas_faults_t *right);

#endif
