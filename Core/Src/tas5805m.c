/*
 * tas5805m.c - low-level driver for the TAS5805M (see tas5805m.h)
 */
#include "tas5805m.h"
#include <math.h>

/* ================================================================== */
/* 1. Basic register access                                            */
/* ================================================================== */

/* One byte to one register of the CURRENTLY SELECTED book/page. */
HAL_StatusTypeDef tas_write(uint8_t addr, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(TAS_I2C, TAS_HAL_ADDR(addr), reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1, TAS_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef tas_read(uint8_t addr, uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(TAS_I2C, TAS_HAL_ADDR(addr), reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1, TAS_I2C_TIMEOUT_MS);
}

/* Books and pages (datasheet 7.5.2.5):
 *   - register 0x00 on EVERY page selects the page,
 *   - register 0x7F on PAGE 0 selects the book.
 * So: go to page 0, set the book, then set the page.
 * We do this before every access instead of remembering the current
 * page. It costs three tiny I2C writes, but the code can never be
 * "on the wrong page" because some other function moved it.          */
HAL_StatusTypeDef tas_select(uint8_t addr, uint8_t book, uint8_t page)
{
    HAL_StatusTypeDef st = tas_write(addr, TAS_REG_PAGE, 0x00);
    st |= tas_write(addr, TAS_REG_BOOK, book);
    st |= tas_write(addr, TAS_REG_PAGE, page);
    return st;
}

/* One 32-bit DSP coefficient: 4 bytes, MOST significant byte first,
 * written in one I2C transfer (the address auto-increments).         */
static HAL_StatusTypeDef write_raw32(uint8_t addr, uint8_t reg, int32_t raw)
{
    uint32_t u = (uint32_t)raw;
    uint8_t b[4] = { (uint8_t)(u >> 24), (uint8_t)(u >> 16),
                     (uint8_t)(u >> 8),  (uint8_t)u };
    return HAL_I2C_Mem_Write(TAS_I2C, TAS_HAL_ADDR(addr), reg,
                             I2C_MEMADD_SIZE_8BIT, b, 4, TAS_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef tas_write_coef(uint8_t addr, uint8_t book, uint8_t page,
                                 uint8_t reg, int32_t raw)
{
    HAL_StatusTypeDef st = tas_select(addr, book, page);
    st |= write_raw32(addr, reg, raw);
    st |= tas_select(addr, TAS_BOOK_CTRL, 0x00);   /* leave in book 0 */
    return st;
}

/* ================================================================== */
/* 2. Number formats                                                   */
/* ================================================================== */
/* The DSP works with signed fixed-point numbers in 32 bits. "9.23"
 * means 9 integer bits (incl. sign) and 23 fraction bits, so
 * 1.0 = 2^23 = 0x00800000 and the range is -256 ... +256.
 *   biquads:          5.27   (1.0 = 0x08000000, range +-16)
 *   volume, mixer:    9.23   (1.0 = 0x00800000)
 *   time constants:   1.31   (1.0 is just out of range: 0x7FFFFFFF)   */
int32_t tas_q(double x, int frac_bits)
{
    double v = x * (double)(1UL << frac_bits);
    if (v >  2147483647.0) v =  2147483647.0;   /* clamp instead of wrap */
    if (v < -2147483648.0) v = -2147483648.0;
    return (int32_t)lround(v);
}

int32_t tas_db_to_q9_23(float db)
{
    return tas_q(pow(10.0, (double)db / 20.0), 23);
}

/* ================================================================== */
/* 3. Life cycle                                                       */
/* ================================================================== */

/* PDN# is shared by both chips. Low = shutdown: all registers and
 * coefficients are lost, so after tas_pdn(true) everything has to be
 * loaded again.                                                      */
void tas_pdn(bool on)
{
    HAL_GPIO_WritePin(TAS_PDN_PORT, TAS_PDN_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (on) HAL_Delay(6);   /* datasheet 7.5.3.1 step 3: >= 5 ms */
}

HAL_StatusTypeDef tas_set_state(uint8_t addr, uint8_t state)
{
    /* writing 0x03 also clears DIS_DSP (bit 4) and MUTE (bit 3) */
    HAL_StatusTypeDef st = tas_select(addr, TAS_BOOK_CTRL, 0x00);
    st |= tas_write(addr, TAS_REG_DEVICE_CTRL_2, state & 0x03);
    return st;
}

HAL_StatusTypeDef tas_mute(uint8_t addr, bool mute)
{
    uint8_t v;
    HAL_StatusTypeDef st = tas_select(addr, TAS_BOOK_CTRL, 0x00);
    st |= tas_read(addr, TAS_REG_DEVICE_CTRL_2, &v);
    v = mute ? (v | TAS_CTRL2_MUTE) : (v & (uint8_t)~TAS_CTRL2_MUTE);
    st |= tas_write(addr, TAS_REG_DEVICE_CTRL_2, v);
    return st;
}

/* Reset the chip's logic and start the DSP.
 *
 * PRECONDITION: PDN is high (tas_pdn(true)) AND the I2S clocks
 * (BCLK, LRCLK) are already running - datasheet 7.5.3.1 step 4:
 * "Once I2S clocks are stable, set the device into HiZ state and
 * enable DSP". Writing Hi-Z to 0x03 clears DIS_DSP; if there is no
 * clock at that moment the DSP's internal data transfers start out of
 * sync.
 *
 * The sequence is the one TI's PurePath Console 3 (PPC3) puts at the
 * top of every TAS5805M configuration it exports. The lines marked
 * "undocumented" are not described in the datasheet; we copy them
 * because TI's own tool always sends them.                            */
HAL_StatusTypeDef tas_reset_and_enable(uint8_t addr)
{
    HAL_StatusTypeDef st = tas_select(addr, TAS_BOOK_CTRL, 0x00);

    st |= tas_write(addr, TAS_REG_DEVICE_CTRL_2, TAS_STATE_HIZ);
    st |= tas_write(addr, TAS_REG_RESET_CTRL, 0x11);     /* reset DSP + registers   */
    st |= tas_write(addr, TAS_REG_DEVICE_CTRL_2, TAS_STATE_HIZ);
    HAL_Delay(5);                                        /* step 5: >= 5 ms         */
    st |= tas_write(addr, TAS_REG_DEVICE_CTRL_2, TAS_STATE_DEEP_SLEEP);
    st |= tas_write(addr, 0x46, 0x01);                   /* undocumented (PPC3)     */
    st |= tas_write(addr, TAS_REG_DEVICE_CTRL_2, TAS_STATE_HIZ);

    /* ADR/FAULT pin becomes the active-low FAULT output after power-up
     * (the address was latched at PDN release). Your ADR resistor to
     * DVDD now acts as its pull-up; the STM32 can read it on PB1/PB2. */
    st |= tas_write(addr, TAS_REG_ADR_PIN_CONFIG, 0x0B); /* ADR = FAULTZ            */
    st |= tas_write(addr, TAS_REG_ADR_PIN_CTRL, 0x01);   /* ADR pin is an output    */

    st |= tas_write(addr, 0x7D, 0x11);                   /* undocumented (PPC3)     */
    st |= tas_write(addr, 0x7E, 0xFF);                   /* undocumented (PPC3)     */
    st |= tas_write(addr, TAS_REG_PAGE, 0x01);
    st |= tas_write(addr, 0x51, 0x05);                   /* undocumented (PPC3)     */
    st |= tas_write(addr, TAS_REG_PAGE, 0x00);
    return st;
}

/* ================================================================== */
/* 4. Audio                                                            */
/* ================================================================== */

/* Master volume of one chip, both channels, 0.5 dB steps
 * (register 0x4C: 0x30 = 0 dB, every +1 = -0.5 dB, 0xFF = mute).
 * Deliberately capped at 0 dB: the register could go to +24 dB, which
 * would defeat the gain staging done in soundbar_dsp.c.              */
HAL_StatusTypeDef tas_set_volume_db(uint8_t addr, float db)
{
    uint8_t code;
    if (db > 0.0f) db = 0.0f;
    if (db <= -103.0f) {
        code = 0xFF;                                    /* mute */
    } else {
        code = (uint8_t)(0x30 + (int)lroundf(-db * 2.0f));
        if (code > 0xFE) code = 0xFE;
    }
    HAL_StatusTypeDef st = tas_select(addr, TAS_BOOK_CTRL, 0x00);
    st |= tas_write(addr, TAS_REG_DIG_VOL, code);
    return st;
}

/* Per-channel gain inside the DSP (book 0x8C, page 0x2A, 9.23).
 * Used for level trim between tweeter and mids.                      */
HAL_StatusTypeDef tas_set_channel_gain_db(uint8_t addr, tas_ch_t ch, float db)
{
    return tas_write_coef(addr, TAS_BOOK_DSP, TAS_PAGE_VOLUME,
                          ch == TAS_CH_A ? TAS_VOL_LEFT : TAS_VOL_RIGHT,
                          tas_db_to_q9_23(db));
}

/* ---- Biquads -------------------------------------------------------
 * Where they live (app note SLOA263A, table 6, book 0xAA):
 *   ch A BQ 1 starts at page 0x24, register 0x18
 *   ch B BQ 1 starts at page 0x26, register 0x54
 * Each biquad = 5 coefficients x 4 bytes = 20 bytes: B0 B1 B2 A1 A2.
 *
 * Pages hold DSP data only in registers 0x08...0x7F (120 bytes); 0x00
 * is the page register and 0x01-0x07 are reserved. Counting only the
 * usable bytes, the biquads are simply packed one after the other:
 *   offset(ch A, BQ n) = 16 + 20*(n-1)
 *   offset(ch B, BQ n) = 16 + 20*(15 + n-1)
 *   page = 0x24 + offset / 120,  register = 0x08 + offset % 120
 * Check: ch A BQ 7 -> offset 136 -> page 0x25, reg 0x18 (table 6 agrees)
 *
 * The 5 coefficients must be written completely and in order, B0
 * first (datasheet 7.5.2.5). A coefficient never straddles two pages
 * (120 is a multiple of 4), but a biquad can; we then switch page in
 * the middle, exactly as TI's PPC3 exports do.
 *
 * Coefficient convention (verified bit-exact against a PPC3 export):
 *   store b0/a0, b1/a0, b2/a0, -a1/a0, -a2/a0 in 5.27.
 * The app note's table 3 says b1 and a1 are divided by 2 - that is
 * NOT what TI's tool writes for these 5.27 biquads. biquad.c already
 * returns the stored form, so this function only converts.           */
#define BQ_FIRST_PAGE 0x24
#define BQ_BYTES      20
#define PAGE_BYTES    120

HAL_StatusTypeDef tas_write_biquad(uint8_t addr, tas_ch_t ch, uint8_t index,
                                   const tas_biquad_t *bq)
{
    if (index < 1 || index > TAS_BQ_PER_CHANNEL) return HAL_ERROR;

    const double c[5] = { bq->b0, bq->b1, bq->b2, bq->a1, bq->a2 };
    uint32_t off = 16u + BQ_BYTES * ((ch == TAS_CH_B ? TAS_BQ_PER_CHANNEL : 0u)
                                     + (index - 1u));
    uint8_t page = 0;
    HAL_StatusTypeDef st = HAL_OK;

    for (int i = 0; i < 5; i++, off += 4) {
        uint8_t p   = (uint8_t)(BQ_FIRST_PAGE + off / PAGE_BYTES);
        uint8_t reg = (uint8_t)(0x08 + off % PAGE_BYTES);
        if (i == 0)         st |= tas_select(addr, TAS_BOOK_BQ, p);
        else if (p != page) st |= tas_write(addr, TAS_REG_PAGE, p);
        page = p;
        st |= write_raw32(addr, reg, tas_q(c[i], 27));
    }
    st |= tas_select(addr, TAS_BOOK_CTRL, 0x00);
    return st;
}

/* "Flat" biquad: output = input (B0 = 1, everything else 0). Used to
 * make sure unused biquads from an earlier configuration are cleared. */
HAL_StatusTypeDef tas_flatten_biquads(uint8_t addr, tas_ch_t ch, uint8_t first)
{
    const tas_biquad_t flat = { 1.0, 0.0, 0.0, 0.0, 0.0 };
    HAL_StatusTypeDef st = HAL_OK;
    for (uint8_t n = first; n <= TAS_BQ_PER_CHANNEL; n++)
        st |= tas_write_biquad(addr, ch, n, &flat);
    return st;
}

/* ================================================================== */
/* 5. Monitoring                                                       */
/* ================================================================== */
HAL_StatusTypeDef tas_read_faults(uint8_t addr, tas_faults_t *f)
{
    HAL_StatusTypeDef st = tas_select(addr, TAS_BOOK_CTRL, 0x00);
    st |= tas_read(addr, TAS_REG_CHAN_FAULT,    &f->chan);
    st |= tas_read(addr, TAS_REG_GLOBAL_FAULT1, &f->global1);
    st |= tas_read(addr, TAS_REG_GLOBAL_FAULT2, &f->global2);
    st |= tas_read(addr, TAS_REG_OT_WARNING,    &f->ot_warn);
    return st;
}

/* Fault bits stay set until cleared here, even after the cause is
 * gone (datasheet 7.5.3.3). A DC fault also keeps the outputs in Hi-Z
 * until cleared.                                                     */
HAL_StatusTypeDef tas_clear_faults(uint8_t addr)
{
    HAL_StatusTypeDef st = tas_select(addr, TAS_BOOK_CTRL, 0x00);
    st |= tas_write(addr, TAS_REG_FAULT_CLEAR, 0x80);
    return st;
}

uint32_t tas_detected_fs(uint8_t addr)
{
    uint8_t v = 0;
    if (tas_select(addr, TAS_BOOK_CTRL, 0x00) != HAL_OK) return 0;
    if (tas_read(addr, TAS_REG_FS_MON, &v) != HAL_OK) return 0;
    switch (v & 0x0F) {
        case 0x6: return 32000;
        case 0x9: return 48000;   /* 44.1 kHz is not in the table; it probably
                                     shows up here as well - check on your board */
        case 0xB: return 96000;
        default:  return 0;
    }
}
