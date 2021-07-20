#ifndef _RMS_H
#define _RMS_H

struct rms_s {
	float *buffer;  /* ring buffer for previous samples */
	int bufferlen;
        int interval;  /* number of new samples (= calls to rms()) to update it's return value */
        int counter;
        int ptr;
        float rms;  /* cached return for rms() */
};

extern struct rms_s *init_rms(int len, int interval);
extern void clear_rms(struct rms_s *r);

extern float rms(struct rms_s *r, float input);

#endif
