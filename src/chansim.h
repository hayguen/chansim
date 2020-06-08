#ifndef _CHANSIM_H
#define _CHANSIM_H

#include "complex.h"

#define Version "0.56-bns-3"

extern inline float RNG(void)
{
        return ((float) random() / RAND_MAX);
}

/* in fade.c */
extern void GaussInit(float, int);
extern void FadeGains(complex *, complex *);

/* in delay.c */
extern void init_delayline(float, int);
extern complex delayline(complex);

#endif
