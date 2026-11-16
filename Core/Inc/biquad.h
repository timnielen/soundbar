/*
 * biquad.h - design second-order filters ("biquads") for the TAS5805M
 *
 * Formulas: Robert Bristow-Johnson, "Audio EQ Cookbook" (the same ones
 * TI's PPC3 uses - checked bit-exact for a Butterworth low-pass).
 * All functions return the coefficients in the form the DSP stores
 * them (normalised, feedback signs flipped), ready for
 * tas_write_biquad().
 */
#ifndef BIQUAD_H
#define BIQUAD_H

#include "tas5805m.h"

/* fc in Hz, fs = the DSP's INTERNAL rate (see dsp_rate() in
 * soundbar_dsp.c), q = quality factor (0.7071 = Butterworth),
 * gain_db only for peaking and shelf filters.                        */
tas_biquad_t bq_flat(void);
tas_biquad_t bq_lowpass  (double fc, double q, double fs);
tas_biquad_t bq_highpass (double fc, double q, double fs);
tas_biquad_t bq_peaking  (double fc, double q, double gain_db, double fs);
tas_biquad_t bq_lowshelf (double fc, double q, double gain_db, double fs);
tas_biquad_t bq_highshelf(double fc, double q, double gain_db, double fs);
tas_biquad_t bq_gain     (double gain_db);    /* pure gain, no filtering */

#define BQ_BUTTERWORTH 0.70710678118654752   /* 1/sqrt(2) */

#endif
