#ifndef _CHANSIM_H
#define _CHANSIM_H

#include <stdlib.h>
#include <complex.h>

#define Version "0.56-bns-3"

static inline float RNG(void)
{
        return ((float) rand() / RAND_MAX);
}

/* in fade.c */
extern void GaussInit(float, int);
extern void FadeGains(float complex *, float complex *);

/* in delay.c */
extern void init_delayline(float, int);
extern float complex delayline(float complex);

#endif
