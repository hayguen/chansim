#ifndef _FILTER_H
#define _FILTER_H

#define FilterLen	64
#define BufferLen	1024

#include <string.h>
#include <complex.h>

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

static inline float mac(const float *a, const float *b)
{
	float sum = 0;
	unsigned int i;

	for (i = 0; i < FilterLen; i++)
		sum += (*a++) * (*b++);
	return sum;
}

static inline float complex filter(struct filter_s *f, float complex in)
{
        float *iptr = f->ibuffer + f->ptr;
        float *qptr = f->qbuffer + f->ptr;
        float complex out;

        *iptr = crealf(in);
        *qptr = cimagf(in);

        out = mac(iptr - FilterLen, f->ifilter)
            + mac(qptr - FilterLen, f->qfilter) * _Complex_I;

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

#endif  /* _FILTER_H */
