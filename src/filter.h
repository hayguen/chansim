#ifndef _FILTER_H
#define _FILTER_H

#define FilterLen	64
#define BufferLen	1024

#ifndef	__NEED_ONLY_FILTERLEN

#include <string.h>
#include "complex.h"

/* ---------------------------------------------------------------------- */

struct filter_s {
	float ifilter[FilterLen];
	float qfilter[FilterLen];
	float ibuffer[BufferLen];
	float qbuffer[BufferLen];
	int ptr;
};

/* ---------------------------------------------------------------------- */

extern struct filter_s *init_filter(float, float);
extern void clear_filter(struct filter_s *);

/* ---------------------------------------------------------------------- */

#ifdef __i386__
#include "filter-i386.h"
#endif				/* __i386__ */

#ifndef __HAVE_ARCH_MAC
extern inline float mac(const float *a, const float *b)
{
	float sum = 0;
	unsigned int i;

	for (i = 0; i < FilterLen; i++)
		sum += (*a++) * (*b++);
	return sum;
}
#endif				/* __HAVE_ARCH_MAC */

extern inline complex filter(struct filter_s *f, complex in)
{
        float *iptr = f->ibuffer + f->ptr;
        float *qptr = f->qbuffer + f->ptr;
        complex out;

        *iptr = in.re;
        *qptr = in.im;

        out.re = mac(iptr - FilterLen, f->ifilter);
        out.im = mac(qptr - FilterLen, f->qfilter);

        f->ptr++;
        if (f->ptr == BufferLen) {
                iptr = f->ibuffer + BufferLen - FilterLen;
                qptr = f->qbuffer + BufferLen - FilterLen;
                memcpy(f->ibuffer, iptr, FilterLen * sizeof(float));
                memcpy(f->qbuffer, qptr, FilterLen * sizeof(float));
                f->ptr = FilterLen;
        }

        return out;
}

/* ---------------------------------------------------------------------- */

#endif				/* __NEED_ONLY_FILTERLEN */

#endif				/* _FILTER_H */
