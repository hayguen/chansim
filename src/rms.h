#ifndef _RMS_H
#define _RMS_H

struct rms_s {
	float *buffer;
	int bufferlen;
        int interval;
        int counter;
        int ptr;
        float rms;
};

extern struct rms_s *init_rms(int len, int interval);
extern void clear_rms(struct rms_s *r);

extern float rms(struct rms_s *r, float input);

#endif
