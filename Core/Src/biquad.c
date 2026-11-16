/*
 * biquad.c - filter design (see biquad.h)
 *
 * Every cookbook filter has the transfer function
 *
 *          b0 + b1 z^-1 + b2 z^-2
 *   H(z) = ----------------------
 *          a0 + a1 z^-1 + a2 z^-2
 *
 * The DSP computes (direct form 1, app note figure 7):
 *   y[n] = B0 x[n] + B1 x[n-1] + B2 x[n-2] + A1 y[n-1] + A2 y[n-2]
 * Comparing both: B = b/a0 and A = -a/a0. That is all finish() does.
 */
#include "biquad.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static tas_biquad_t finish(double b0, double b1, double b2,
                           double a0, double a1, double a2)
{
    tas_biquad_t r = { b0 / a0, b1 / a0, b2 / a0, -a1 / a0, -a2 / a0 };
    return r;
}

tas_biquad_t bq_flat(void)
{
    tas_biquad_t r = { 1.0, 0.0, 0.0, 0.0, 0.0 };
    return r;
}

tas_biquad_t bq_gain(double gain_db)
{
    tas_biquad_t r = { pow(10.0, gain_db / 20.0), 0.0, 0.0, 0.0, 0.0 };
    return r;
}

tas_biquad_t bq_lowpass(double fc, double q, double fs)
{
    double w = 2.0 * M_PI * fc / fs;
    double cw = cos(w), alpha = sin(w) / (2.0 * q);
    return finish((1.0 - cw) / 2.0, 1.0 - cw, (1.0 - cw) / 2.0,
                  1.0 + alpha, -2.0 * cw, 1.0 - alpha);
}

tas_biquad_t bq_highpass(double fc, double q, double fs)
{
    double w = 2.0 * M_PI * fc / fs;
    double cw = cos(w), alpha = sin(w) / (2.0 * q);
    return finish((1.0 + cw) / 2.0, -(1.0 + cw), (1.0 + cw) / 2.0,
                  1.0 + alpha, -2.0 * cw, 1.0 - alpha);
}

/* Bell-shaped boost/cut around fc; q sets the width. */
tas_biquad_t bq_peaking(double fc, double q, double gain_db, double fs)
{
    double A = pow(10.0, gain_db / 40.0);
    double w = 2.0 * M_PI * fc / fs;
    double cw = cos(w), alpha = sin(w) / (2.0 * q);
    return finish(1.0 + alpha * A, -2.0 * cw, 1.0 - alpha * A,
                  1.0 + alpha / A, -2.0 * cw, 1.0 - alpha / A);
}

/* Boost/cut everything below fc (q = 0.7071 gives the usual shape). */
tas_biquad_t bq_lowshelf(double fc, double q, double gain_db, double fs)
{
    double A = pow(10.0, gain_db / 40.0);
    double w = 2.0 * M_PI * fc / fs;
    double cw = cos(w), alpha = sin(w) / (2.0 * q), s = 2.0 * sqrt(A) * alpha;
    return finish(A * ((A + 1) - (A - 1) * cw + s),
                  2 * A * ((A - 1) - (A + 1) * cw),
                  A * ((A + 1) - (A - 1) * cw - s),
                  (A + 1) + (A - 1) * cw + s,
                  -2 * ((A - 1) + (A + 1) * cw),
                  (A + 1) + (A - 1) * cw - s);
}

/* Boost/cut everything above fc. */
tas_biquad_t bq_highshelf(double fc, double q, double gain_db, double fs)
{
    double A = pow(10.0, gain_db / 40.0);
    double w = 2.0 * M_PI * fc / fs;
    double cw = cos(w), alpha = sin(w) / (2.0 * q), s = 2.0 * sqrt(A) * alpha;
    return finish(A * ((A + 1) + (A - 1) * cw + s),
                  -2 * A * ((A - 1) + (A + 1) * cw),
                  A * ((A + 1) + (A - 1) * cw - s),
                  (A + 1) - (A - 1) * cw + s,
                  2 * ((A - 1) - (A + 1) * cw),
                  (A + 1) - (A - 1) * cw - s);
}
