#ifndef _CHANSIM_CPLX_H
#define _CHANSIM_CPLX_H

#include <complex.h>

#if __STDC_VERSION__ >= 199901L
  /* using a C99 compiler */
typedef float _Complex float_complex;

#define make_float_complex(R, I)  ( (R) + (I) * _Complex_I )
#define cplx_mulf(A, B)  ((A) * (B))
#define cplx_scale(C, FACTOR)  (C) *= (FACTOR)

#else
  /* e.g. MSVC 2019 does NOT support C99 ! */
  /* #error "Compiler does not support C99 standard!" */
  typedef struct
  {
    float re, im;
  } float_complex;

  inline float_complex make_float_complex(float real, float imag)
  {
    float_complex out = { real, imag };
    return out;
  }

  inline float_complex cplx_mulf(float_complex A, float_complex B)
  {
      float_complex out = { A.re * B.re - A.im * B.im, A.re * B.im + A.im * B.re };
      return out;
  }

  #define cplx_scale(C, FACTOR)  do { (C).re *= (FACTOR); (C).im *= (FACTOR); } while (0)

  /* #define creal(z) ((z).re) */
  /* #define cimag(z) ((z).im) */
  #define crealf(z) ((float)((z).re))
  #define cimagf(z) ((float)((z).im))

#endif

#endif
