#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "chansim.h"

#define DELAYTAPS	256

static complex DelayLine[DELAYTAPS];

static int Ptr;
static int DelayPtr;

void init_delayline(float deltime, int samplerate)
{
	int dllen;

	/* clear the delay line */
	memset(DelayLine, 0, sizeof(DelayLine));

	/* scale from seconds to samples */
	dllen = (int) floor(deltime * samplerate + 0.5);

	if (dllen == 0)
		dllen = 1;

	if (dllen > DELAYTAPS - 1) {
		dllen = DELAYTAPS - 1;
		fprintf(stderr,
			"Warning: path delay too long, limiting to %.1f ms\n",
			(float) dllen / samplerate * 1000.0);
	}

	/* Delayed pointer dllen taps behind the input pointer */
	Ptr = 0;
	DelayPtr = DELAYTAPS - dllen;
}

complex delayline(complex in)
{
	complex out;

	/* save the new sample to the delayline */
	DelayLine[Ptr] = in;

	/* get the delayed sample */
	out = DelayLine[DelayPtr];

	/* update the pointers */
	Ptr = (Ptr + 1) % DELAYTAPS;
	DelayPtr = (DelayPtr + 1) % DELAYTAPS;

	return out;
}
