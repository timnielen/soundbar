/*
 * soundbar_dsp.c - the sound of the soundbar
 *
 * This is the file you edit. Everything about the CHIP is in
 * tas5805m.c; everything about the SPEAKERS is here.
 *
 * Each TAS5805M drives one side of the bar:
 *   channel A (OUT_A) -> tweeter
 *   channel B (OUT_B) -> group of midrange drivers
 * Both channels of the left chip play the left input, both channels
 * of the right chip the right input.
 */
#include "soundbar_dsp.h"
#include "biquad.h"

/* ================================================================== */
/*                      >>>  EDIT FROM HERE  <<<                       */
/* ================================================================== */

/* Which output is which (swap if you wired the speakers the other way) */
#define TWEETER_CH        TAS_CH_B
#define MIDS_CH           TAS_CH_A

/* Crossover: Linkwitz-Riley 4th order = two identical Butterworth
 * biquads in a row. LR4 low-pass and high-pass add up flat AND in
 * phase, so neither channel needs inverting.                         */
#define CROSSOVER_HZ      3000.0
#define MIDS_HIGHPASS_HZ  180.0      /* protects the small mid cones   */

typedef enum { F_LOWPASS, F_HIGHPASS, F_PEAKING,
               F_LOWSHELF, F_HIGHSHELF, F_GAIN } filter_type_t;

typedef struct {
    filter_type_t type;
    double fc;        /* Hz (ignored for F_GAIN)                        */
    double q;         /* BQ_BUTTERWORTH = 0.7071; peaking: width        */
    double gain_db;   /* only for F_PEAKING, F_LOWSHELF, F_HIGHSHELF, F_GAIN */
} filter_t;

/* Up to 15 entries per channel; unused biquads are set to "flat".
 * To add EQ, append lines, e.g.
 *   { F_PEAKING, 1200.0, 2.0, -3.0 },   // -3 dB dip at 1.2 kHz       */
static const filter_t TWEETER_FILTERS[] = {
    { F_HIGHPASS, CROSSOVER_HZ, BQ_BUTTERWORTH, 0.0 },
    { F_HIGHPASS, CROSSOVER_HZ, BQ_BUTTERWORTH, 0.0 },
};

static const filter_t MIDS_FILTERS[] = {
    { F_LOWPASS,  CROSSOVER_HZ,     BQ_BUTTERWORTH, 0.0 },
    { F_LOWPASS,  CROSSOVER_HZ,     BQ_BUTTERWORTH, 0.0 },
    { F_HIGHPASS, MIDS_HIGHPASS_HZ, BQ_BUTTERWORTH, 0.0 },
    { F_HIGHPASS, MIDS_HIGHPASS_HZ, BQ_BUTTERWORTH, 0.0 },
};

/* ---- Gain staging -------------------------------------------------
 * AGAIN (reg 0x54) sets the output voltage for a FULL-SCALE digital
 * signal: 0 dB = 29.5 V peak, each step -0.5 dB. With PVDD = 20 V,
 * 29.5 V can never be reached - the top 3.4 dB would just clip.
 * Code 7 = -3.5 dB -> full scale = 29.5 V * 10^(-3.5/20) = 19.7 V.
 *
 * The mids (3.7 ohm) must stay below ~14 V peak, or the 5 A
 * overcurrent protection trips (chapter 19). 14 / 19.7 = -3.0 dB, so
 * a full-scale input can only reach 14 V on the mids. Filters can
 * overshoot a little on extreme signals; the chip's own overcurrent
 * shutdown is the backstop (it shows up in soundbar_dsp_poll()).
 * Tweeter level: start equal, then trim by measurement (chapter 23). */
#define ANALOG_GAIN_CODE  7          /* 0...31, -0.5 dB per step        */
#define TWEETER_LEVEL_DB  (-3.0f)
#define MIDS_LEVEL_DB     (-3.0f)    /* never above -3.0 at 20 V PVDD!  */

#define START_VOLUME_DB   (-30.0f)   /* volume right after start-up     */

/* ================================================================== */
/*                       >>>  EDIT UP TO HERE  <<<                     */
/* ================================================================== */

#define N(a) (sizeof(a) / sizeof((a)[0]))

static float volume_db = START_VOLUME_DB;

/* The chip's process flow runs an internal sample-rate converter to
 * 96 kHz, or 88.2 kHz for 44.1 kHz-family inputs (app note SLOA263A,
 * section 2.2.1). Biquads must be designed for THAT rate, not for the
 * I2S rate. The 96 kHz case is confirmed against a TI tool export;
 * check the 44.1 kHz case once by measuring the crossover frequency
 * with Bluetooth audio (if it is ~8 % off, return 96000 here too).   */
static double dsp_rate(uint32_t fs_in)
{
    return (fs_in % 11025u == 0u) ? 88200.0 : 96000.0;
}

static tas_biquad_t design(const filter_t *f, double fs)
{
    switch (f->type) {
    case F_LOWPASS:   return bq_lowpass  (f->fc, f->q, fs);
    case F_HIGHPASS:  return bq_highpass (f->fc, f->q, fs);
    case F_PEAKING:   return bq_peaking  (f->fc, f->q, f->gain_db, fs);
    case F_LOWSHELF:  return bq_lowshelf (f->fc, f->q, f->gain_db, fs);
    case F_HIGHSHELF: return bq_highshelf(f->fc, f->q, f->gain_db, fs);
    case F_GAIN:      return bq_gain(f->gain_db);
    }
    return bq_flat();
}

/* Write a list of filters into biquads 1, 2, 3 ... of one channel and
 * set all remaining biquads to flat.                                 */
static HAL_StatusTypeDef load_filters(uint8_t addr, tas_ch_t ch,
                                      const filter_t *list, uint8_t n,
                                      double fs)
{
    HAL_StatusTypeDef st = HAL_OK;
    if (n > TAS_BQ_PER_CHANNEL) n = TAS_BQ_PER_CHANNEL;
    for (uint8_t i = 0; i < n; i++) {
        tas_biquad_t bq = design(&list[i], fs);
        st |= tas_write_biquad(addr, ch, (uint8_t)(i + 1), &bq);
    }
    st |= tas_flatten_biquads(addr, ch, (uint8_t)(n + 1));
    return st;
}

static HAL_StatusTypeDef load_all_filters(uint8_t addr, uint32_t fs_in)
{
    double fs = dsp_rate(fs_in);
    HAL_StatusTypeDef st;
    st  = load_filters(addr, TWEETER_CH, TWEETER_FILTERS, N(TWEETER_FILTERS), fs);
    st |= load_filters(addr, MIDS_CH,    MIDS_FILTERS,    N(MIDS_FILTERS),    fs);
    return st;
}

/* Everything one chip needs, in datasheet order. The chip is in Hi-Z
 * (outputs off, DSP running) the whole time.
 *   routing: SAP_CTRL3 value. Bits 5-4 feed DSP channel A, bits 1-0
 *   channel B. 01 = "own" input, 10 = the other input.
 *   0x12: A gets L, B gets L  (left chip)
 *   0x21: A gets R, B gets R  (right chip)                           */
static HAL_StatusTypeDef configure(uint8_t addr, uint8_t routing, uint32_t fs_in)
{
    HAL_StatusTypeDef st = tas_reset_and_enable(addr);

    st |= tas_select(addr, TAS_BOOK_CTRL, 0x00);
    st |= tas_write(addr, TAS_REG_DEVICE_CTRL_1, 0x10);   /* 384 kHz, BD modulation, BTL   */
    st |= tas_write(addr, TAS_REG_ANA_CTRL, 0x00);        /* loop bandwidth 80 kHz (PPC3)  */
    st |= tas_write(addr, TAS_REG_AGAIN, ANALOG_GAIN_CODE);
    st |= tas_write(addr, TAS_REG_SAP_CTRL1, 0x02);       /* I2S, 24 bit (reset value)     */
    st |= tas_write(addr, TAS_REG_SAP_CTRL3, routing);

    /* DSP_MISC: bit 3 = separate coefficients for A and B (otherwise
     * B copies A and the mids would get the tweeter's high-pass!),
     * bit 2 = bypass FIR, bit 1 = bypass DRC, bit 0 = 0: EQ active.
     * Bit 7 is "reserved" but TI's PPC3 always sets it.
     * Must be written BEFORE the biquads.                            */
    st |= tas_write(addr, TAS_REG_DSP_MISC, 0x8E);

    st |= load_all_filters(addr, fs_in);

    st |= tas_set_channel_gain_db(addr, TWEETER_CH, TWEETER_LEVEL_DB);
    st |= tas_set_channel_gain_db(addr, MIDS_CH,    MIDS_LEVEL_DB);
    st |= tas_set_volume_db(addr, volume_db);
    return st;
}

/* ------------------------------------------------------------------ */

void soundbar_dsp_power_on(void)
{
    tas_pdn(true);                     /* both chips, includes the 5 ms */
}

HAL_StatusTypeDef soundbar_dsp_start(uint32_t fs_in)
{
    HAL_StatusTypeDef st;
    st  = configure(LEFT_DSP_ADDR,  0x12, fs_in);
    st |= configure(RIGHT_DSP_ADDR, 0x21, fs_in);
    if (st != HAL_OK) return st;       /* stay in Hi-Z if anything failed */

    st |= tas_set_state(LEFT_DSP_ADDR,  TAS_STATE_PLAY);
    st |= tas_set_state(RIGHT_DSP_ADDR, TAS_STATE_PLAY);
    st |= tas_clear_faults(LEFT_DSP_ADDR);     /* start-up leaves clock flags */
    st |= tas_clear_faults(RIGHT_DSP_ADDR);
    return st;
}

HAL_StatusTypeDef soundbar_dsp_stop(void)
{
    HAL_StatusTypeDef st;
    st  = tas_mute(LEFT_DSP_ADDR, true);
    st |= tas_mute(RIGHT_DSP_ADDR, true);
    HAL_Delay(10);                     /* datasheet 7.5.3.2: >= 6 ms ramp */
    st |= tas_set_state(LEFT_DSP_ADDR,  TAS_STATE_HIZ);
    st |= tas_set_state(RIGHT_DSP_ADDR, TAS_STATE_HIZ);
    tas_pdn(false);                    /* all settings are lost now       */
    return st;
}

HAL_StatusTypeDef soundbar_dsp_set_volume(float db)
{
    HAL_StatusTypeDef st;
    volume_db = db;
    st  = tas_set_volume_db(LEFT_DSP_ADDR,  db);
    st |= tas_set_volume_db(RIGHT_DSP_ADDR, db);
    return st;
}

/* The internal rate changes between 96 and 88.2 kHz, so all biquads
 * must be recalculated. Mute first: rewriting a running filter
 * coefficient by coefficient can click.                              */
HAL_StatusTypeDef soundbar_dsp_set_rate(uint32_t fs_in)
{
    HAL_StatusTypeDef st;
    st  = tas_mute(LEFT_DSP_ADDR, true);
    st |= tas_mute(RIGHT_DSP_ADDR, true);
    HAL_Delay(10);
    st |= load_all_filters(LEFT_DSP_ADDR,  fs_in);
    st |= load_all_filters(RIGHT_DSP_ADDR, fs_in);
    st |= tas_mute(LEFT_DSP_ADDR, false);
    st |= tas_mute(RIGHT_DSP_ADDR, false);
    return st;
}

bool soundbar_dsp_poll(tas_faults_t *left, tas_faults_t *right)
{
    tas_read_faults(LEFT_DSP_ADDR,  left);
    tas_read_faults(RIGHT_DSP_ADDR, right);

    bool fault = (left->chan  | left->global1  | left->global2) ||
                 (right->chan | right->global1 | right->global2);
    if (fault) {
        /* Clock faults recover by themselves once the clocks are back;
         * the flag just has to be cleared. DC and overcurrent faults
         * keep the outputs off until cleared.                         */
        tas_clear_faults(LEFT_DSP_ADDR);
        tas_clear_faults(RIGHT_DSP_ADDR);
    }
    return fault;
}
