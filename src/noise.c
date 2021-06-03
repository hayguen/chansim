#include <stdlib.h>
#include <math.h>

#include "chansim.h"
#include "noise.h"

//----------------------------------------------------------------------------
// Filter to limit noise to the desired base band bandwidth.
// 2nd Order Butterworth IIR filter,
// Code contributed by Tomi Manninen, OH2BNS.
//----------------------------------------------------------------------------
struct noise_s *init_noise(int type, double samplerate, double cutoff)
{
	struct noise_s *n;
	double w;

	if ((n = calloc(1, sizeof(struct noise_s))) == NULL)
		return NULL;

	// calculate the IIR filter coefficients
	w = 1.0 / tan(M_PI * cutoff / samplerate);

	n->an0 = 1.0;
	n->an1 = 2.0;
	n->an2 = 1.0;

	n->bn0 = w * w + M_SQRT2 * w + 1.0;
	n->bn1 = -2.0 * w * w + 2.0;
	n->bn2 = w * w - M_SQRT2 * w + 1.0;

	n->bn1 /= n->bn0;
	n->bn2 /= n->bn0;

	// what kind of noise?
	n->noisetype = type;

	// compensate for power loss in IIR filter
	n->BGG = 1.0 / sqrt(2.0 * cutoff / samplerate);

	return n;
}

static inline float noisefilter(struct noise_s *n, float in)
{
	n->xv[0] = n->xv[1];
	n->xv[1] = n->xv[2];
	n->xv[2] = in / n->bn0;
	n->yv[0] = n->yv[1];
	n->yv[1] = n->yv[2];

	n->yv[2] = n->an0 * n->xv[2] +
		   n->an1 * n->xv[1] +
		   n->an2 * n->xv[0] -
		   n->bn1 * n->yv[1] -
		   n->bn2 * n->yv[0];

	return n->yv[2];
}

//----------------------------------------------------------------------------
// Bandlimited noise generator.
// Used for adding band-limited Gaussian, La Placian, or impulse noise.
// Note: noise filter scales automatically for sample rate and channel
// bandwidth.
//----------------------------------------------------------------------------
float BandLtdNoise(struct noise_s *n)
{
	float z = 0.0F;

	switch (n->noisetype) {
	case 0:					// Gaussian
		z = sqrtf(-2.0 * log(RNG()));
		z *= cosf(2.0 * M_PI * RNG());
		break;
	case 1:					// La Placian
		z = RNG();
		if (z < 0.5F)
			z = logf(2.0F * z) / M_SQRT2;
		else
			z = -logf(2.0F * (1.0F - z)) / M_SQRT2;
		break;
	case 2:					// Impulsive
		// This only works for SNR <= 5 or so
		z = -M_SQRT2 * logf(RNG());
		// 5 => scratchy, 8 => Geiger
		if (fabsf(z) <= 8.0F)
			z = 0.0F;		// choose whatever you fancy.
		break;
	}

	return noisefilter(n, z) * n->BGG;
}
