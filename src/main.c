/***************************************************************************
                          main.c  -  description
                             -------------------
    begin                : Wed Jun 28 10:16:45 PDT 2000
    copyright            : (C) 2000 by Johan Forrer, KC7WW
    email                : forrerj@peak.org  http://www.peak.org/~forrerj
 ***************************************************************************/
//------------------------------------------------------------------
// Watterson Ionospheric Gaussian-Scatter HF Channel Model.
// Linux version: 0.56-bns-3
//------------------------------------------------------------------
//
// NOTES:-----------------------------------------------------------
// 1). This work draws heavily from Eric Johnson's contributions:
// see ftp://antanasoff.nmsu.edu/pub/hf/ale/chansim/src
// This body of published work is gratefully acknowledged.
// It is a "light" version of a commercial version that runs on
// an ADI SHARC DSP chip.
//
// Thanks also to Tomi Manninen, OH2BNS for improvements and the IIR
// filter generator code.
//
// 2). Use/abuse this code with due caution. The author is not
// responsible or liable for any consequences. It is made available
// in good faith that it may be of use. If you found bugs or make
// improvements, it would be greatly appreciated if you please pass
// that on to the author at above address.
//
// 3). Developed using "KDevelop" IDE; please see the included
// project definition file in the package.
//
// 4). Usage: sim <SNR dB>, <Type of Channel>, <IO>, <Noise type>
//     For Type of Channel use the following:
//          0 - ONLY_NOISE
//          1 - FLAT 1
//          2 - FLAT 2
//          3 - CCIR GOOD
//          4 - CCIR MODERATE
//          5 - CCIR POOR
//          6 - CCIR FLUTTER_FADING
//          7 - EXTREME
//     For IO use one of the following:
//          0 - use internal NCO (1800 Hz default)
//          1 - use sound card signals
//          2 - use stdio (pipes)
//     For Noise type use:
//          0 - Gaussian noise (Most HF simulations use this)
//          1 - LaPlacian (Some say sapproximate HF impulse noise)
//          2 - Impulse noise (Geiger-like or make it scratchy like static)
//              Here, need to set SNR to some low value like 5.0 or less.
//
// 5). Some useful background on HF channel simulation is available
// on the author's web page. Includes references to the Watterson
// paper(s), implementation details, and talks on the subject.
//
// 6). The fading function uses a simple 2-pole, 2-zero IIR filter.
// It is an approximation for a Gaussian-shaped filter response.
// Note that this filter's -3dB bandwidth is set for the fadingrate/sqrt(2)
// which is the 2Sigma PSD bandwidth for the equivalent Gaussian-shaped
// response.
//
// 7). To keep a reasonable filter rolloff, the fading filter sample rate
// i.e., the tapupdaterate needs to be keep reasonably low. This is
// a compromise. See the values selected as initialized.
//

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/soundcard.h>
#include <time.h>

#include "chansim.h"
#include "filter.h"
#include "rms.h"
#include "noise.h"

//----------------------------------------------------------------------------
// Audio I/O definitions
//----------------------------------------------------------------------------
#define BUF_SIZE	512		// "chunk" size
#define DEVICE		"/dev/dsp"
int16_t audio_buf_in[BUF_SIZE];
int16_t audio_buf_out[BUF_SIZE];
int size_in;
int size_out;

//----------------------------------------------------------------------------
// Test NCO work definitions
//----------------------------------------------------------------------------
float NCOFreq =		1800.0;	// default NCO frequency
#define NCO_GAIN	2500.0	// sets default NCO gain to not overdrive
float phase_accum =	0.0;
float delta;

//----------------------------------------------------------------------------
// Simulator definitions and memory assignments
//----------------------------------------------------------------------------
#define DIRECT		1.0	// These describe how to combine
#define DELAYED		1.0	// direct and delayed paths

int SampleRate =	8000;	// 8000 samples per second
float ChannelBW	=	3000.0;	// 3 kHz channel (used in noise shaping)
float FreqOffset =	0.0;	// Default frequency offset
float Amplitude = 	0.0;	// Signal amplitude (RMS). Zero means
				// compute at runtime
float InputGain =	1.0;	// The input signal is scaled with this
float DelTime;			// Time difference between two paths
float SigLvl;			// Signal level for given SNR
float FrSpread;			// Frequency (doppler) spread
int TapUpdRate;			// Update rate for the fading gain params

struct filter_s *Filter;	// Struct for the Hilbert transformer
struct rms_s *Rms;		// Struct for RMS calculations
struct noise_s *Noise;		// Struct for Noise generation

//------------------------------------------------------------------
// Usage stuff
//------------------------------------------------------------------
static char *UsageString = "\
Usage: chansim [-a <ampl>] [-b <bw>] [-f <nco>] [-g <gain>] [-i <IO type>] [-n <noise type>] [-o <offset>] [-r <seed>] [-s <samplerate>] <SNR> <format>
Type `chansim -h' for more information.
";

static char *HelpString = "\

      chansim - Watterson Ionospheric Gaussian-Scatter HF Channel Model
      =================================================================

                             version " Version "

Usage: chansim [options] <SNR> <format>

Arguments:
        <SNR dB>                Signal to noise ratio.

        <format>                HF channel type. (Path delay / Doppler spread)

                                0 - Noise only           (  ---  /   --- )
                                1 - Flat 1               (  ---  / 0.2 Hz)
                                2 - Flat 2               (  ---  / 1.0 Hz)
                                3 - CCIR good            (0.5 ms / 0.1 Hz)
                                4 - CCIR moderate        (1.0 ms / 0.5 Hz)
                                5 - CCIR poor            (2.0 ms /   1 Hz)
                                6 - CCIR flutter fading  (0.5 ms /  10 Hz)
                                7 - Extreme              (2.0 ms /   5 Hz)

Options:
        -a <ampl>               Set the RMS amplitude of the incoming signal.
                                Allowed range 0...1. Default is to calculate
                                it at runtime.

        -b <bw>                 Noise bandwidth. Default 3000 Hz.

        -f <nco>                Test NCO frequency. Only valid with I/O = 0.
                                Default 1800 Hz.

	-g <gain>		Input gain. Input signal is scaled with
				this factor. Default is 1.

	-i <IO type>            I/O type.

                                0 - Internal test NCO
                                1 - Soundcard I/O
                                2 - Pipe I/O (stdin/stdout)

				Default is pipe I/O.

	-n <noise type>         Noise type.

                                0 - Gaussian noise
                                1 - LaPlacian noise
                                2 - Impulse noise

				Default is Gaussian noise.

        -o <offset>             Frequency offset. Default 0 Hz.

	-r <seed>		Seed for the random number generator.
				Default is a combination of current time
				and process id.

        -s <samplerate>         Soundcard samplerate. Also used to scale
                                various filters and timings. Default 8000 sps.

";

static char *HF_Channel_type[] =
{
	"ONLY_NOISE",		// 0
	"FLAT 1",		// 1
	"FLAT 2",		// 2
	"CCIR GOOD",		// 3
	"CCIR MODERATE",	// 4
	"CCIR POOR",		// 5
	"CCIR FLUTTER_FADING",	// 6
	"EXTREME"		// 7
};

static char *IO_usage[] =
{
	"Internal NCO",		// 0
	"/dev/dsp Sound I/O",	// 1
	"STDIO"			// 2
};

static char *HF_Noise[] =
{
	"Gaussian noise",	// 0
	"LaPlacian noise",	// 1
	"Impulse noise"		// 2
};

//------------------------------------------------------------------
// Simulated HF channel.
//------------------------------------------------------------------
// 1) Form analytic signal, data saved into tapped delay line.
// 2) Compute fading gain factors (done at an update rate equal
//    to the symbol rate.)
// 3) Complex multiply fading gain factors with path components.
// 4) Add Gaussian noise component magnitude for the specified SNR.
// 5) Extract real part.
//------------------------------------------------------------------
static float simprocess(float input_signal)
{
	static complex fade0, fade1;
	static float nco = 0.0;
	static int pointsleft = 0;
	float rmsval;
	float inoise;
	complex sig, dsig, z;

	// Create analytic input signal
	sig.re = input_signal / M_SQRT2;
	sig.im = input_signal / M_SQRT2;
	sig = filter(Filter, sig);

	// Shift the frequency if requested
	if (FreqOffset != 0.0) {
		z.re = cos(nco);
		z.im = sin(nco);
		sig = cmul(sig, z);

		nco += 2.0 * M_PI * FreqOffset / SampleRate;

		if (nco > M_PI)
			nco -= 2.0 * M_PI;
		if (nco < -M_PI)
			nco += 2.0 * M_PI;
	}

	// Fading gain is activated at the "symbol" (update) rate.
	// Update direct and delayed path fading gain coefficients if needed,
	// for noise-only simulation, leave fading gain coefficients constant.
	if (--pointsleft <= 0) {
		if (FrSpread > 0.0) {
			FadeGains(&fade0, &fade1);
		} else {
			fade0.re = 1.0 / M_SQRT2;
			fade0.im = 1.0 / M_SQRT2;
			fade1.re = 1.0 / M_SQRT2;
			fade1.im = 1.0 / M_SQRT2;
		}
		pointsleft = SampleRate / TapUpdRate;
	}

	//------------------------------------------------------------------
	// Holding the fading gain constant for a symbol time,
	// Use I and Q data for two paths, complex multiply with fading gain
	// to generate effective outputs for each symbol sample point.
	//------------------------------------------------------------------
	if (DelTime > 0.0) {
		// Multipath

		// Delayed (second) path
		dsig = delayline(sig);
		dsig = cmul(dsig, fade1);

		// First path
		sig = cmul(sig, fade0);
	} else {
		// Flat fading

		// First path
		sig = cmul(sig, fade0);
		sig.re *= M_SQRT2;
		sig.im *= M_SQRT2;

		// Delayed path
		dsig.re = 0.0;
		dsig.im = 0.0;
	}

	// Compute input signal's RMS
	// This is needed to scale noise magnitude.
	if (Amplitude == 0.0)
		rmsval = rms(Rms, input_signal);
	else
		rmsval = Amplitude;

	// Noise generator generates in-phase and quadrature
	// noise components that are jointly normal, with each
	// component having RMS amplitude of unity and RMS noise power
	// is unity.
	// Note: noise gets compensated for bandwidth-limiting filter loss.
	// We also have to convert the input RMS to voltage levels.
	inoise = BandLtdNoise(Noise) * rmsval / SigLvl;

	// compute output, we don't use imaginary part here
	return DIRECT * sig.re + DELAYED * dsig.re + inoise;
	// return DIRECT * sig.re + DELAYED * dsig.re;
	// return inoise;
}

//----------------------------------------------------------------------------
// Initialize simulation paramaters.
// Initializes Gaussian filter coefficients.
//----------------------------------------------------------------------------
static void SetParms(float snr, int simform)
{
	// convert from dB to voltage ratio
	SigLvl = pow(10.0, snr / 20.0);

	switch (simform) {
	default:
	case 0:				// NOISE ONLY
		DelTime = 0.0;
		FrSpread = 0.0;
		break;
	case 1:				// FLAT 1
		DelTime = 0.0;		// 0.0 ms delay
		FrSpread = 0.2;		// 0.2 Hz spread
		break;
	case 2:				// FLAT 2
		DelTime = 0.0;		// 0.0 ms delay
		FrSpread = 1.0;		// 1.0 Hz spread
		break;
	case 3:				// CCIR GOOD
		DelTime = 0.5e-3;	// 0.5 ms delay
		FrSpread = 0.1;		// 0.1 Hz spread
		break;
	case 4:				// CCIR MODERATE
		DelTime = 1.0e-3;	// 1.0 ms delay
		FrSpread = 0.5;		// 0.5 Hz spread
		break;
	case 5:				// CCIR POOR
		DelTime = 2.0e-3;	// 2.0 ms delay
		FrSpread = 1.0;		// 1.0 Hz spread
		break;
	case 6:				// CCIR FLUTTER FADING
		DelTime = 0.5e-3;	// 0.5 ms delay
		FrSpread = 10.0;	// 10.0 Hz spread
		break;
	case 7:				// EXTREME
		DelTime = 2.0e-3;	// 2.0 ms delay
		FrSpread = 5.0;		// 5.0 Hz spread
		break;
	}
	TapUpdRate = 50.0 * FrSpread + 1.0;
}

//--------------------------------------------------------------------
// Sets up CODEC and sound system stuff.
// srate: sampling rate
// chan:  mono (=0) or stereo (=1)
// format: format of audio as defined in soundcard.h
//
static int init_audio(int srate, int chan, int format)
{
	int audio_fd, caps;
//	int enable_bits;
	int fragsize = 0x7fff0008;

	if ((audio_fd = open(DEVICE, O_RDWR)) < 0)
		return -1;

	if (ioctl(audio_fd, SNDCTL_DSP_SETDUPLEX, 0))
		return -1;
	if (ioctl(audio_fd, SNDCTL_DSP_GETCAPS, &caps))
		return -1;
	if ((caps & DSP_CAP_DUPLEX) != DSP_CAP_DUPLEX)	// full duplex audio
		return -1;
	// May not be required ... apparently does not work for ALSA
	// disable recording/playback before setting parameters
//	enable_bits = ~(PCM_ENABLE_OUTPUT | PCM_ENABLE_INPUT);
//      if (ioctl(audio_fd, SNDCTL_DSP_SETTRIGGER, &enable_bits))
//              return -1;
	if (ioctl(audio_fd, SNDCTL_DSP_SETFMT, &format))
		return -1;
	if (ioctl(audio_fd, SNDCTL_DSP_STEREO, &chan))
		return -1;
	if (ioctl(audio_fd, SNDCTL_DSP_SPEED, &srate))
		return -1;
	if (ioctl(audio_fd, SNDCTL_DSP_SETFRAGMENT, &fragsize))
		return -1;
	// May not be required ... apparently does not work for ALSA
	// enable recording/playback again
//	enable_bits = PCM_ENABLE_OUTPUT | PCM_ENABLE_INPUT;
//      if (ioctl(audio_fd, SNDCTL_DSP_SETTRIGGER, &enable_bits))
//              return -1;

	return audio_fd;
}

//
// Generate output from whatever input was selected...
//
static int gensig(int16_t *buf_ptr, int size, int iotype)
{
	int i;
	float ftemp;
	int16_t temp;

	for (i = 0; i < size; i++) {
		switch (iotype) {
		case 0:				// NCO
			temp = NCO_GAIN * cos(phase_accum);
			phase_accum += delta;
			if (phase_accum > 2.0 * M_PI)
				phase_accum -= 2.0 *M_PI;
			break;
		case 1:				// Sound IO
		case 2:				// File IO
			temp = audio_buf_in[i];
			break;
		default:			// Won't happen...
			temp = 0.0;
			break;
		}

		// Push signal though HF channel
		ftemp = temp * InputGain / 32768.0;
		ftemp = simprocess(ftemp);

		// Saturate instead of wraparound
		if (ftemp > 0.999) {
			ftemp = 0.999;
			fprintf(stderr, "chansim: positive clipping!\n");
		}
		if (ftemp < -0.999) {
			ftemp = -0.999;
			fprintf(stderr, "chansim: negative clipping!\n");
		}

		// output signal is 16-bit PCM
		buf_ptr[i] = (int16_t) (ftemp * 32768.0);
	}

	return i;
}

//===================================================================//
int main(int argc, char *argv[])
{
	int audio_fd = -1;
	int i;
	int errflag = 0;
	int Chan_type;
	int IO_type = 2;	/* default is STDIO */
	int Noise_type = 0;	/* default is gaussian */
	float SNR_parm;
	unsigned int seed;

	seed = time(NULL) + getpid();

	while ((i = getopt(argc, argv, "a:b:f:g:hi:n:o:r:s:")) != EOF) {
		switch (i) {
		case 'a':
			Amplitude = atof(optarg);
			break;
		case 'b':
			ChannelBW = atof(optarg);
			break;
		case 'f':
			NCOFreq = atof(optarg);
			break;
		case 'g':
			InputGain = atof(optarg);
			break;
		case 'i':
			IO_type = atoi(optarg);
			if (IO_type < 0 || IO_type > 2) {
				fprintf(stderr, "chansim: invalid I/O type: %d\n", IO_type);
				exit(1);
			}
			break;
		case 'n':
			Noise_type = atoi(optarg);
			if (Noise_type < 0 || Noise_type > 2) {
				fprintf(stderr, "chansim: invalid noise type: %d\n", Noise_type);
				exit(1);
			}
			break;
		case 'o':
			FreqOffset = atof(optarg);
			break;
		case 'r':
			seed = strtoul(optarg, NULL, 0);
			break;
		case 's':
			SampleRate = atoi(optarg);
			break;
		case 'h':
			printf(HelpString);
			exit(0);
			break;
		case ':':
		default:
			errflag++;
			break;
		}
	}

	if ((argc - optind) != 2)
		errflag++;

	if (errflag) {
		fprintf(stderr, UsageString);
		exit(1);
	}

	SNR_parm = atof(argv[optind++]);
	Chan_type = atoi(argv[optind++]);

	if (Chan_type < 0 || Chan_type > 7) {
		fprintf(stderr, "chansim: invalid channel type: %d\n", Chan_type);
		exit(1);
	}

	// Scale amplitude (set by user) with input gain
	Amplitude *= InputGain;

	fprintf(stderr, "Simulating %s-type HF Channel\n",
		HF_Channel_type[Chan_type]);
	fprintf(stderr, "\tS/N ratio = %.1f dB (%s)\n",
		SNR_parm,
		HF_Noise[Noise_type]);
	fprintf(stderr, "\tNoise bandwidth = %.1f Hz\n",
		ChannelBW);
	fprintf(stderr, "\tSignal amplitude = %.3f %s\n",
		Amplitude,
		Amplitude == 0.0 ? "(calculated at runtime)" : "");
	fprintf(stderr, "\tFrequency offset = %.1f Hz\n",
		FreqOffset);
	fprintf(stderr, "\tSample rate = %d sps\n",
		SampleRate);
	fprintf(stderr, "\t%s ",
		IO_usage[IO_type]);
	if (IO_type == 0)
		fprintf(stderr, "(frequency = %.1f Hz)\n", NCOFreq);
	else
		fprintf(stderr, "\n");


	delta = 2.0 * M_PI * NCOFreq / SampleRate;

	// Initialize the audio hardware to selected sampling rate
	// also Mono and signed 16 bit quantization.
	if (IO_type < 2) {
		audio_fd = init_audio(SampleRate, 0, AFMT_S16_LE);
		if (audio_fd < 0) {
			fprintf(stderr, "error audio_fd is %d\n", audio_fd);
			exit(-1);
		}
	}

	// Seed the random number generator
	srandom(seed);

	// Initialize HF channel simulation parameters
	SetParms(SNR_parm, Chan_type);

	// Initialize the noise module
	Noise = init_noise(Noise_type, SampleRate, ChannelBW);
	if (!Noise) {
		fprintf(stderr, "Noise module initialization failed\n");
		exit(1);
	}

	// Initialize HF channel Rayleigh fading coefficients
	GaussInit(FrSpread, TapUpdRate);

	// Initialize tapped delay line channel
	init_delayline(DelTime, SampleRate);

	// Calculate RMS over 256 samples, update every 64 samples
	Rms = init_rms(256, 64);
	if (!Rms) {
		fprintf(stderr, "RMS initialization failed\n");
		exit(1);
	}

	// Initialize the Hilbert transformer (200...3800Hz @ 8000sps)
	Filter = init_filter(200.0 / SampleRate, (ChannelBW + 200.0) / SampleRate);
	if (!Filter) {
		fprintf(stderr, "Filter initialization failed\n");
		exit(1);
	}

	while (1) {
		// Prepare output buffer to minimize delay between
		// sound card reads and writes. This operation overlap
		// with the write() function below.
		//
		// Fill output buffer
		size_out = gensig(audio_buf_out, size_in, IO_type);

		// Wait for a full data buffer -- this is our pacer.
		// This read() is a blocked call since the input buffer
		// is not yet full.
		if (IO_type == 0 || IO_type == 1) {
			size_in = read(audio_fd, audio_buf_in, BUF_SIZE * sizeof(int16_t));

			if (size_in == 0)
				break;

			if (size_in < 0) {
				perror("Error: read:");
				exit(-1);
			}

			if ((size_in % sizeof(int16_t)) != 0) {
				fprintf(stderr, "Error: nonintegral number of samples read\n");
				exit(-1);
			}

			size_in /= sizeof(int16_t);

			// Initiate the writing of the prepared buffer.
			// This write() is not blocked because there is space
			// left in the output buffer.
			write(audio_fd, audio_buf_out, size_out * sizeof(int16_t));
		}

		// File IO
		if (IO_type == 2) {
			size_in = fread(audio_buf_in, sizeof(int16_t), BUF_SIZE, stdin);

			if (size_in == 0)
				break;

			fwrite(audio_buf_out, sizeof(int16_t), size_out, stdout);
		}
	}
	return EXIT_SUCCESS;
}
