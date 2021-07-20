
#define _USE_MATH_DEFINES

#include "chansim.h"

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static float g, a0, a1, a2;

static float IFade0[6];		// direct-path  fading filter state vars
static float QFade0[6];
static float IFade1[6];		// delayed-path fading filter state vars
static float QFade1[6];

//----------------------------------------------------------------------------
// Rayleigh noise generator -- Gaussian noise.
// Here, rxx, has Rayleigh distribution (Schartz p.446). Remember
// its a polar coordinate thing. It is the product it and another
// jointly-independant variable, z, that's our Gaussian value (Schwartz p365).
//----------------------------------------------------------------------------
static inline void Rayleigh(float *Rx, float *Iy)
{
        float rxx, z;

        rxx = sqrtf(-2.0F * logf(RNG()));
        z = 2.0F * (float)M_PI * RNG();
        *Rx = rxx * cosf(z);
        *Iy = rxx * sinf(z);
}

//----------------------------------------------------------------------------
// Fading gains module
//
// Fade array of input/output data:
// Fade[0] is the current filter output
// Fade[0-2] are the current and past outputs
// Fade[3-5] are the current and past inputs
//----------------------------------------------------------------------------
static inline void Gauss_Filter(float *Fade)
{

        // Gaussian filter:  2-pole, 2-zero IIR
        Fade[0] = (g * (Fade[3] + 2 * Fade[4] + Fade[5]) -
		   a1 * Fade[1] - a2 * Fade[2]) / a0;

        // adjust the history terms
        Fade[2] = Fade[1];
        Fade[1] = Fade[0];
        Fade[5] = Fade[4];
        Fade[4] = Fade[3];
}

//----------------------------------------------------------------------------
//  Generate Rayleigh-distributed fade gain functions
//----------------------------------------------------------------------------
void FadeGains(float_complex *fade0, float_complex *fade1)
{
        // inputs goes into third element of IIR filter state variables
        Rayleigh(IFade0 + 3, QFade0 + 3);
        Rayleigh(IFade1 + 3, QFade1 + 3);

        // Run through gaussian filter. This actually is a LPF, which happens
        // to have the same Gaussian output properties.
        Gauss_Filter(IFade0);
        Gauss_Filter(QFade0);
        Gauss_Filter(IFade1);
        Gauss_Filter(QFade1);

	// output is from the first element of IIR state variables
	if (fade0) {
		*fade0 = make_float_complex(*IFade0, *QFade0);
	}
	if (fade1) {
		*fade1 = make_float_complex(*IFade1, *QFade1);
	}
}

//----------------------------------------------------------------------------
// Initialize Gaussian filter coefficients.
// Set up delay line tap position for second ray.
//----------------------------------------------------------------------------
void GaussInit(float frspread, int tapupdrate)
{
        int i;
        float a, c, A, C;

	if (frspread == 0.0F)
		return;

//--------------------------------------------------------------------------
// Set up fading generator
// The bandwidth for the filter is determined by the frequency spread,
// which, in this case, is set for an per symbol update rate.
//--------------------------------------------------------------------------
	// Convert Hz ==> rad/symbol
	// Radians/sec = 2*pi*Hz = 2*pi/T
	frspread *= 2.0F * (float)M_PI / tapupdrate;
	// for 2Sigma
	frspread /= (float)M_SQRT2;

	// Coefficients for 2-pole Butterworth filter.
	// With Gaussian input data, this filter's output
	// is also Gaussian.
	a = sqrtf(2.0F * (float)M_PI);
	c = 1.5F;
	A = a / frspread;
	C = c / frspread;
	C *= C;

	// These are GLOBAL
	// such that others can access it.

	// Compensates for filter Power loss
	g = sqrtf(0.5F * sqrtf(2.0F * (float)M_PI) / frspread);

	a0 = A + C + 1.0F;
	a1 = 2 * (1.0F - C);
	a2 = C + 1.0F - A;

	// Clear filter state elements.
	for (i = 0; i < 6; i++) {
		IFade0[i] = 0.0F;
		QFade0[i] = 0.0F;
		IFade1[i] = 0.0F;
		QFade1[i] = 0.0F;
	}

	// and prime the filter state
	for (i = 0; i < 1.0 / frspread; i++)
		FadeGains(NULL, NULL);
}
