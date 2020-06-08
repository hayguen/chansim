#include <stdlib.h>
#include <math.h>

#include "rms.h"

struct rms_s *init_rms(int len, int interval)
{
	struct rms_s *r;

	if ((r = calloc(1, sizeof(struct rms_s))) == NULL)
		return NULL;

	if ((r->buffer = calloc(len, sizeof(float))) == NULL) {
		free(r);
		return NULL;
	}

	r->bufferlen = len;
	r->interval = interval;
	r->counter = 0;
	r->ptr = 0;
	r->rms = 0.0;

	return r;
}

void clear_rms(struct rms_s *r)
{
	free(r->buffer);
	free(r);

	return;
}

static inline float calculate_rms(float *buf, int len)
{
        float sum, pwr, avg, rms;
        int i;

        sum = 0.0;
        pwr = 0.0;
        for (i = 0; i < len; i++) {
                sum += buf[i];
                pwr += buf[i] * buf[i];
        }

        avg = sum / len;
        rms = sqrt(pwr / len - avg * avg);

        return rms;
}

float rms(struct rms_s *r, float input)
{
        r->buffer[r->ptr] = input;

        r->ptr = (r->ptr + 1) % r->bufferlen;

        if (r->counter++ == r->interval) {
                r->rms = calculate_rms(r->buffer, r->bufferlen);
                r->counter = 0;
        }

        return r->rms;
}

