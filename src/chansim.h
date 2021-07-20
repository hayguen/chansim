#ifndef _CHANSIM_H
#define _CHANSIM_H

#include <stdlib.h>
#include "cplx.h"

#define Version "0.56-bns-4"

static inline float RNG(void)
{
        return ((float) rand() / RAND_MAX);
}

/* in fade.c */
extern void GaussInit(float frspread, int tapupdrate);
extern void FadeGains(float_complex *, float_complex *);

/* in delay.c */
extern void init_delayline(float delay_time_in_sec, int samplerate);
extern float_complex delayline(float_complex);

#endif
