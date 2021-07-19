
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
static inline double sinc(double x)
{
	if (fabs(x) < 1e-10)
		return 1.0;
	else
		return sin(M_PI * x) / (M_PI * x);
}

/*
 * Don't ask...
 */
static inline double cosc(double x)
{
	if (fabs(x) < 1e-10)
		return 0.0;
	else
		return (1.0 - cos(M_PI * x)) / (M_PI * x);
}

/*
 * Hamming window function.
 */
static inline double hamming(double x)
{
	return 0.54 - 0.46 * cos(2 * M_PI * x);
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
		t = i - (FilterLen - 1.0) / 2.0;
		h = i * (1.0 / (FilterLen - 1.0));

		x = (2 * f2 * sinc((2.0 * f2) * t) -
		     2 * f1 * sinc((2.0 * f1) * t)) * hamming(h);
		f->ifilter[i] = x;
#ifdef DEBUG
		fprintf(stderr, "%.10f\t", x);
#endif

		/*
		 * The actual filter code assumes the impulse response
		 * is in time reversed order. This will be anti-
		 * symmetric so the minus sign handles that for us.
		 */
		x = (2 * f2 * cosc((2.0 * f2) * t) -
		     2 * f1 * cosc((2.0 * f1) * t)) * hamming(h);
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

