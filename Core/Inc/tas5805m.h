/*
 * tas5805m.h - low-level driver for the two TAS5805M amplifiers/DSPs
 *
 * This layer knows the CHIP: registers, books/pages, number formats,
 * start-up order, faults. It knows nothing about speakers or crossovers;
 * that lives in soundbar_dsp.c.
 *
 * Sources: TAS5805M datasheet SLASEH5D (section numbers in comments),
 *          TI app note SLOA263A "TAS5805M Process Flows" (memory maps).
 */
#ifndef TAS5805M_H
#define TAS5805M_H

#include <stdint.h>
#include <stdbool.h>
#include "main.h"            /* HAL, CubeMX pin labels, LEFT/RIGHT_DSP_ADDR */

/* ------------------------------------------------------------------ */
/* Board configuration: adapt these to your CubeMX project             */
/* ------------------------------------------------------------------ */

extern I2C_HandleTypeDef hi2c2;          /* both amps sit on I2C2 (PB10/PB11) */
#define TAS_I2C            (&hi2c2)
#define TAS_I2C_TIMEOUT_MS 10

/* Shared PDN# line (PB0). Use the label you gave the pin in CubeMX. */
#define TAS_PDN_PORT       PDN__GPIO_Port
#define TAS_PDN_PIN        PDN__Pin

/* LEFT_DSP_ADDR / RIGHT_DSP_ADDR come from your own header.
 * The HAL wants the 7-bit address shifted left by one.
 * Set TAS_ADDR_IS_7BIT to 0 if your defines are already shifted
 * (i.e. 0x58/0x5E instead of 0x2C/0x2F).                              */
#define TAS_ADDR_IS_7BIT   0
#if TAS_ADDR_IS_7BIT
#define TAS_HAL_ADDR(a)    ((uint16_t)((a) << 1))
#else
#define TAS_HAL_ADDR(a)    ((uint16_t)(a))
#endif

/* ------------------------------------------------------------------ */
/* Book 0 / page 0 control registers (datasheet 7.6.1)                 */
/* ------------------------------------------------------------------ */
#define TAS_REG_PAGE          0x00   /* on every page: selects the page     */
#define TAS_REG_BOOK          0x7F   /* on page 0 only: selects the book    */
#define TAS_REG_RESET_CTRL    0x01
#define TAS_REG_DEVICE_CTRL_1 0x02   /* switching frequency, modulation     */
#define TAS_REG_DEVICE_CTRL_2 0x03   /* state (deep sleep/sleep/Hi-Z/play), mute, DSP reset */
#define TAS_REG_SAP_CTRL1     0x33   /* audio format, word length           */
#define TAS_REG_SAP_CTRL3     0x35   /* which input goes to which DSP path  */
#define TAS_REG_FS_MON        0x37   /* detected sample rate                */
#define TAS_REG_DIG_VOL       0x4C   /* master volume, 0.5 dB steps         */
#define TAS_REG_ANA_CTRL      0x53   /* class-D loop bandwidth              */
#define TAS_REG_AGAIN         0x54   /* analog gain                         */
#define TAS_REG_ADR_PIN_CTRL  0x60
#define TAS_REG_ADR_PIN_CONFIG 0x61
#define TAS_REG_DSP_MISC      0x66   /* EQ/DRC bypass, separate L/R coefs   */
#define TAS_REG_POWER_STATE   0x68
#define TAS_REG_CHAN_FAULT    0x70
#define TAS_REG_GLOBAL_FAULT1 0x71
#define TAS_REG_GLOBAL_FAULT2 0x72
#define TAS_REG_OT_WARNING    0x73
#define TAS_REG_FAULT_CLEAR   0x78

/* DEVICE_CTRL_2 values */
#define TAS_STATE_DEEP_SLEEP  0x00
#define TAS_STATE_SLEEP       0x01
#define TAS_STATE_HIZ         0x02
#define TAS_STATE_PLAY        0x03
#define TAS_CTRL2_MUTE        0x08   /* bit 3: soft mute (ramps the volume) */
#define TAS_CTRL2_DIS_DSP     0x10   /* bit 4: holds the DSP in reset       */

/* ------------------------------------------------------------------ */
/* DSP memory (app note SLOA263A, process flow 1 = the chip's ROM flow) */
/* ------------------------------------------------------------------ */
#define TAS_BOOK_CTRL       0x00     /* control port registers above        */
#define TAS_BOOK_DSP        0x8C     /* mixer, volume, DRC, AGL, clipper... */
#define TAS_BOOK_BQ         0xAA     /* all biquad coefficients             */

#define TAS_PAGE_VOLUME     0x2A     /* book 0x8C                           */
#define TAS_VOL_LEFT        0x24     /* 9.23 format, 1.0 = 0 dB             */
#define TAS_VOL_RIGHT       0x28

#define TAS_BQ_PER_CHANNEL  15

/* The two DSP channels of ONE chip. "LEFT" is the path to OUT_A,
 * "RIGHT" the path to OUT_B - these are the chip's names, not the
 * side of the soundbar.                                               */
typedef enum { TAS_CH_A = 0, TAS_CH_B = 1 } tas_ch_t;

/* One biquad in DSP form: already normalised (a0 = 1), feedback signs
 * already flipped, see tas_write_biquad(). double, not float: a float
 * has only 24 bits of mantissa, the DSP format 27 fraction bits.      */
typedef struct { double b0, b1, b2, a1, a2; } tas_biquad_t;

typedef struct {
    uint8_t chan;      /* 0x70 CHAN_FAULT: DC and overcurrent per channel  */
    uint8_t global1;   /* 0x71: BQ write error, clock, PVDD over/under      */
    uint8_t global2;   /* 0x72: over-temperature shutdown                   */
    uint8_t ot_warn;   /* 0x73: over-temperature warning (135 C)            */
} tas_faults_t;

/* ---- basic access ---- */
HAL_StatusTypeDef tas_write(uint8_t addr, uint8_t reg, uint8_t val);
HAL_StatusTypeDef tas_read(uint8_t addr, uint8_t reg, uint8_t *val);
HAL_StatusTypeDef tas_select(uint8_t addr, uint8_t book, uint8_t page);
HAL_StatusTypeDef tas_write_coef(uint8_t addr, uint8_t book, uint8_t page,
                                 uint8_t reg, int32_t raw);

/* ---- number formats ---- */
int32_t tas_q(double x, int frac_bits);     /* real number -> fixed point */
int32_t tas_db_to_q9_23(float db);          /* dB -> 9.23 gain          */

/* ---- life cycle ---- */
void              tas_pdn(bool on);                    /* both chips        */
HAL_StatusTypeDef tas_reset_and_enable(uint8_t addr);  /* needs I2S clocks! */
HAL_StatusTypeDef tas_set_state(uint8_t addr, uint8_t state);
HAL_StatusTypeDef tas_mute(uint8_t addr, bool mute);

/* ---- audio ---- */
HAL_StatusTypeDef tas_set_volume_db(uint8_t addr, float db);       /* master */
HAL_StatusTypeDef tas_set_channel_gain_db(uint8_t addr, tas_ch_t ch, float db);
HAL_StatusTypeDef tas_write_biquad(uint8_t addr, tas_ch_t ch, uint8_t index,
                                   const tas_biquad_t *bq);
HAL_StatusTypeDef tas_flatten_biquads(uint8_t addr, tas_ch_t ch, uint8_t first);

/* ---- monitoring ---- */
HAL_StatusTypeDef tas_read_faults(uint8_t addr, tas_faults_t *f);
HAL_StatusTypeDef tas_clear_faults(uint8_t addr);
uint32_t          tas_detected_fs(uint8_t addr);       /* 0 if unknown */

#endif
