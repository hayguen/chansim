
#define _USE_MATH_DEFINES

#include "filter.h"
#include "cplx.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#undef	DEBUG

#ifdef DEBUG
#include <stdio.h>
#endif

/*
 * Sinc done properly.
 */
static inline float sinc(float x)
{
	if (fabsf(x) < 1e-10F)
		return 1.0F;
	else
		return sinf((float)M_PI * x) / ((float)M_PI * x);
}

/*
 * Don't ask...
 */
static inline float cosc(float x)
{
	if (fabsf(x) < 1e-10F)
		return 0.0F;
	else
		return (1.0F - cosf((float)M_PI * x)) / ((float)M_PI * x);
}

/*
 * Hamming window function.
 */
static inline float hamming(float x)
{
	return 0.54F - 0.46F * cosf(2.0F * (float)M_PI * x);
}

/*
 * Create a band pass Hilbert transformer / filter with 6 dB corner
 * frequencies of 'f1' and 'f2'. (0 <= f1 < f2 <= 0.5)
 */
struct filter_s *init_filter(float f1, float f2)
{
	struct filter_s *f;
	float t, h, x;
	int i;

	if ((f = calloc(1, sizeof(struct filter_s))) == NULL)
		return NULL;

	for (i = 0; i < FilterLen; i++) {
		t = i - (FilterLen - 1) / 2.0F;
		h = i * (1.0F / (FilterLen - 1.0F));

		x = (2 * f2 * sinc((2.0F * f2) * t) -
		     2 * f1 * sinc((2.0F * f1) * t)) * hamming(h);
		f->ifilter[i] = x;
#ifdef DEBUG
		fprintf(stderr, "%.10f\t", x);
#endif

		/*
		 * The actual filter code assumes the impulse response
		 * is in time reversed order. This will be anti-
		 * symmetric so the minus sign handles that for us.
		 */
		x = (2 * f2 * cosc((2.0F * f2) * t) -
		     2 * f1 * cosc((2.0F * f1) * t)) * hamming(h);
		f->qfilter[i] = -x;
#ifdef DEBUG
		fprintf(stderr, "%.10f\n", x);
#endif
	}

	f->ptr = FilterLen;

	return f;
}

void clear_filter(struct filter_s *f)
{
	free(f);
}

