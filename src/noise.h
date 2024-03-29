#ifndef _NOISE_H
#define _NOISE_H

#define NZEROS 2
#define NPOLES 2

struct noise_s {
	float xv[NZEROS + 1];
	float yv[NPOLES + 1];
	float an0, an1, an2;
	float bn0, bn1, bn2;
	int noisetype;
	float BGG;
};

struct noise_s *init_noise(int type, float samplerate, float cutoff);
float BandLtdNoise(struct noise_s *n);

#endif
