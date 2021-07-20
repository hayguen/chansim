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

#define _USE_MATH_DEFINES

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <process.h>
#include <io.h>

#define GETPID _getpid
#define FILEDES_READ _read
#define FILEDES_WRITE _write

#else
#define GETPID getpid
#define FILEDES_READ read
#define FILEDES_WRITE write
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#ifdef USE_SOUND
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/soundcard.h>
#endif
#include <fcntl.h>
#include <time.h>

#include "chansim.h"
#include "filter.h"
#include "rms.h"
#include "noise.h"


#ifdef WIN32

#ifdef _MSC_VER
/* mingw does define this */
typedef int ssize_t;
#endif

void usleep(unsigned int usec)
{
	HANDLE timer;
	LARGE_INTEGER ft;

	ft.QuadPart = -(10 * (__int64)usec);

	timer = CreateWaitableTimer(NULL, TRUE, NULL);
	SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
	WaitForSingleObject(timer, INFINITE);
	CloseHandle(timer);
}

#endif

//----------------------------------------------------------------------------
// Audio I/O definitions
//----------------------------------------------------------------------------
#define BUF_SIZE	512		// "chunk" size

#ifdef USE_SOUND
#define DEVICE		"/dev/dsp"
#endif
int16_t audio_buf_in[BUF_SIZE];
int16_t audio_buf_out[BUF_SIZE];
int size_in = 0;
int size_out = 0;

//----------------------------------------------------------------------------
// Test NCO work definitions
//----------------------------------------------------------------------------
float NCOFreq =		1800.0F;	// default NCO frequency
#define NCO_GAIN	2500.0F		// sets default NCO gain to not overdrive
float phase_accum =	0.0F;
float delta;

//----------------------------------------------------------------------------
// Simulator definitions and memory assignments
//----------------------------------------------------------------------------
#define DIRECT		1.0F	// These describe how to combine
#define DELAYED		1.0F	// direct and delayed paths

int SampleRate =	8000;	// 8000 samples per second
float ChannelBW	=	3000.0F;	// 3 kHz channel (used in noise shaping)
float FreqOffset =	0.0F;	// Default frequency offset
float Amplitude = 	0.0F;	// Signal amplitude (RMS). Zero means
				// compute at runtime
float InputGain =	1.0F;	// The input signal is scaled with this
float DelTime;			// Time difference between two paths
float SigLvl;			// Signal level for given SNR
float FrSpread;			// Frequency (doppler) spread
int TapUpdRate;			// Update rate for the fading gain params

struct filter_s *Filter;	// Struct for the Hilbert transformer
struct rms_s *RootMeanSqr;	// Struct for RMS calculations
struct noise_s *Noise;		// Struct for Noise generation

//------------------------------------------------------------------
// Usage stuff
//------------------------------------------------------------------
static const char *UsageString = 
"Usage: chansim [-a <ampl>] [-b <bw>] [-f <nco>] [-g <gain>] [-i <IO type>] [-n <noise type>] [-o <offset>] [-r <seed>] [-s <samplerate>] <SNR> <format>\n"
"Type 'chansim -h' for more information.\n";

static const char *HelpString =
"\n"
"chansim - Watterson Ionospheric Gaussian-Scatter HF Channel Model\n"
"version " Version "\n"
"\n"
"Usage: chansim [options] <SNR> <format>\n"
"\n"
"Arguments:\n"
"    <SNR dB>          Signal to noise ratio (option -R on Windows).\n"
"    <format>          HF channel type. (Path delay / Doppler spread)\n"
"                         (option -c on Windows)\n"
"                      0 - Noise only           (  ---  /   --- )\n"
"                      1 - Flat 1               (  ---  / 0.2 Hz)\n"
"                      2 - Flat 2               (  ---  / 1.0 Hz)\n"
"                      3 - CCIR good            (0.5 ms / 0.1 Hz)\n"
"                      4 - CCIR moderate        (1.0 ms / 0.5 Hz)\n"
"                      5 - CCIR poor            (2.0 ms /   1 Hz)\n"
"                      6 - CCIR flutter fading  (0.5 ms /  10 Hz)\n"
"                      7 - Extreme              (2.0 ms /   5 Hz)\n"
"\n"
"Options:\n"
"    -a <ampl>         Set the RMS amplitude of the incoming signal.\n"
"                      Allowed range 0...1. Default is to calculate\n"
"                      it at runtime.\n"
"    -b <bw>           Noise bandwidth. Default 3000 Hz.\n"
"    -f <nco>          Test NCO frequency. Only valid with I/O = 0.\n"
"                      Default 1800 Hz.\n"
"    -g <gain>         Input gain. Input signal is scaled with\n"
"                      this factor. Default is 1.\n"
"    -i <IO type>      I/O type.\n"
"                      0 - Internal test NCO\n"
"                      1 - Soundcard I/O (not on Windows)\n"
"                      2 - Pipe I/O (stdin/stdout)\n"
"                      Default is pipe I/O.\n"
"    -n <noise type>   Noise type.\n"
"                      0 - Gaussian noise\n"
"                      1 - LaPlacian noise\n"
"                      2 - Impulse noise\n"
"                      Default is Gaussian noise.\n"
"    -o <offset>       Frequency offset. Default 0 Hz.\n"
"    -r <seed>         Seed for the random number generator.\n"
"                      Default is a combination of current time\n"
"                      and process id.\n"
"    -s <samplerate>   Soundcard samplerate. Also used to scale\n"
"                      various filters and timings. Default 8000 sps.\n"
"\n";

static const char *HF_Channel_type[] =
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

static const char *IO_usage[] =
{
	"Internal NCO",		// 0
	"/dev/dsp Sound I/O",	// 1
	"STDIO"			// 2
};

static const char *HF_Noise[] =
{
	"Gaussian noise",	// 0
	"LaPlacian noise",	// 1
	"Impulse noise"		// 2
};

static inline float atoff(const char *s)
{
	return (float)atof(s);
}

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
	static float_complex fade0, fade1;
	static float nco = 0.0;
	static int pointsleft = 0;
	float rmsval;
	float inoise;
	float_complex sig, dsig, z;

	// Create analytic input signal
	sig = make_float_complex(input_signal / (float)M_SQRT2, input_signal / (float)M_SQRT2);
	sig = filter(Filter, sig);

	// Shift the frequency if requested
	if (FreqOffset != 0.0) {
		z = make_float_complex(cosf(nco), sinf(nco));
		sig = cplx_mulf(sig, z);

		nco += 2.0F * (float)M_PI * FreqOffset / SampleRate;

		if (nco > (float)M_PI)
			nco -= 2.0F * (float)M_PI;
		if (nco < (float)(-M_PI))
			nco += 2.0F * (float)M_PI;
	}

	// Fading gain is activated at the "symbol" (update) rate.
	// Update direct and delayed path fading gain coefficients if needed,
	// for noise-only simulation, leave fading gain coefficients constant.
	if (--pointsleft <= 0) {
		if (FrSpread > 0.0F) {
			FadeGains(&fade0, &fade1);
		} else {
			fade0 = make_float_complex(1.0F / (float)M_SQRT2, 1.0F / (float)M_SQRT2);
			fade1 = make_float_complex(1.0F / (float)M_SQRT2, 1.0F / (float)M_SQRT2);
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
		dsig = cplx_mulf(dsig, fade1);

		// First path
		sig = cplx_mulf(sig, fade0);
	} else {
		// Flat fading

		// First path
		sig = cplx_mulf(sig, fade0);
		cplx_scale(sig, (float)M_SQRT2);

		// Delayed path
		dsig = make_float_complex(0.0F, 0.0F);
	}

	// Compute input signal's RMS
	// This is needed to scale noise magnitude.
	if (Amplitude == 0.0F)
		rmsval = rms(RootMeanSqr, input_signal);
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
	return DIRECT * crealf(sig) + DELAYED * crealf(dsig) + inoise;
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
	SigLvl = powf(10.0F, snr / 20.0F);

	switch (simform) {
	default:
	case 0:				// NOISE ONLY
		DelTime = 0.0F;
		FrSpread = 0.0F;
		break;
	case 1:				// FLAT 1
		DelTime = 0.0F;		// 0.0 ms delay
		FrSpread = 0.2F;		// 0.2 Hz spread
		break;
	case 2:				// FLAT 2
		DelTime = 0.0F;		// 0.0 ms delay
		FrSpread = 1.0F;		// 1.0 Hz spread
		break;
	case 3:				// CCIR GOOD
		DelTime = 0.5e-3F;	// 0.5 ms delay
		FrSpread = 0.1F;		// 0.1 Hz spread
		break;
	case 4:				// CCIR MODERATE
		DelTime = 1.0e-3F;	// 1.0 ms delay
		FrSpread = 0.5F;		// 0.5 Hz spread
		break;
	case 5:				// CCIR POOR
		DelTime = 2.0e-3F;	// 2.0 ms delay
		FrSpread = 1.0F;		// 1.0 Hz spread
		break;
	case 6:				// CCIR FLUTTER FADING
		DelTime = 0.5e-3F;	// 0.5 ms delay
		FrSpread = 10.0F;	// 10.0 Hz spread
		break;
	case 7:				// EXTREME
		DelTime = 2.0e-3F;	// 2.0 ms delay
		FrSpread = 5.0F;		// 5.0 Hz spread
		break;
	}
	TapUpdRate = (int)(50.0F * FrSpread + 1.0F);
}

#ifdef USE_SOUND

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

#endif

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
			temp = (int16_t)( NCO_GAIN * cosf(phase_accum) );
			phase_accum += delta;
			if (phase_accum > 2.0F * (float)M_PI)
				phase_accum -= 2.0F * (float)M_PI;
			break;
		case 1:				// Sound IO
		case 2:				// File IO
			temp = audio_buf_in[i];
			break;
		default:			// Won't happen...
			temp = 0;
			break;
		}

		// Push signal though HF channel
		ftemp = temp * InputGain / 32768.0F;
		ftemp = simprocess(ftemp);

		// Saturate instead of wraparound
		if (ftemp > 0.999F) {
			ftemp = 0.999F;
			fprintf(stderr, "chansim: positive clipping!\n");
		}
		if (ftemp < -0.999F) {
			ftemp = -0.999F;
			fprintf(stderr, "chansim: negative clipping!\n");
		}

		// output signal is 16-bit PCM
		buf_ptr[i] = (int16_t) (ftemp * 32768.0F);
	}

	return i;
}

//===================================================================//
int main(int argc, char *argv[])
{
	int audio_fd = -1;
	int i;
	int errflag = 0;
	int Chan_type = 0;
	int IO_type = 2;	/* default is STDIO */
	int Noise_type = 0;	/* default is gaussian */
	float SNR_parm = 30.0F;
	unsigned int seed;
	uint32_t usleep_duration = 0U;

	seed = (unsigned)( time(NULL) + GETPID() );

#ifdef WIN32
	for (int argidx = 1; argidx < argc; ++argidx)
	{
		char* optarg = (argidx + 1 < argc) ? argv[argidx + 1] : NULL;
		char* arg = argv[argidx];
		i = 0;
		if (arg[0] == '-' && arg[2] == 0)
			i = arg[1];
		if (i && optarg)
			++argidx;
#else
	while ((i = getopt(argc, argv, "a:b:f:g:hi:n:o:r:s:")) != EOF) {
#endif
		switch (i) {
		case 'a':
			Amplitude = atoff(optarg);
			break;
		case 'b':
			ChannelBW = atoff(optarg);
			break;
		case 'f':
			NCOFreq = atoff(optarg);
			break;
		case 'g':
			InputGain = atoff(optarg);
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
			FreqOffset = atoff(optarg);
			break;
		case 'r':
			seed = strtoul(optarg, NULL, 0);
			break;
		case 's':
			SampleRate = atoi(optarg);
			break;
		case 'R':
			SNR_parm = atoff(optarg);
			break;
		case 'c':
			Chan_type = atoi(optarg);
			break;
		case 'h':
			printf("%s", HelpString);
			exit(0);
			break;
		case ':':
		default:
			errflag++;
			break;
		}
	}

#ifndef WIN32
	if ((argc - optind) != 2)
		errflag++;
#endif

	if (errflag) {
		fprintf(stderr, "%s", UsageString);
		exit(1);
	}

#ifndef WIN32
	SNR_parm = atof(argv[optind++]);
	Chan_type = atoi(argv[optind++]);
#endif

	if (Chan_type < 0 || Chan_type > 7) {
		fprintf(stderr, "chansim: invalid channel type: %d\n", Chan_type);
		exit(1);
	}

	// Scale amplitude (set by user) with input gain
	Amplitude *= InputGain;

	fprintf(stderr, "Simulating %s-type HF Channel\n", HF_Channel_type[Chan_type]);
	fprintf(stderr, "\tS/N ratio = %.1f dB (%s)\n", SNR_parm, HF_Noise[Noise_type]);
	fprintf(stderr, "\tNoise bandwidth = %.1f Hz\n", ChannelBW);
	fprintf(stderr, "\tSignal amplitude = %.3f%s\n", Amplitude,
		Amplitude == 0.0 ? " (calculated at runtime)" : "");
	fprintf(stderr, "\tFrequency offset = %.1f Hz\n", FreqOffset);
	fprintf(stderr, "\tSample rate = %d sps\n", SampleRate);
	fprintf(stderr, "\t%s ", IO_usage[IO_type]);
	if (IO_type == 0)
		fprintf(stderr, "(frequency = %.1f Hz)\n", NCOFreq);
	else
		fprintf(stderr, "\n");


	delta = 2.0F * (float)M_PI * NCOFreq / SampleRate;

	// Initialize the audio hardware to selected sampling rate
	// also Mono and signed 16 bit quantization.
	if (IO_type < 2) {
#ifdef USE_SOUND
		audio_fd = init_audio(SampleRate, 0, AFMT_S16_LE);
#else
		audio_fd = -1;
#endif
		if (audio_fd < 0) {
			if (IO_type == 1) {
				fprintf(stderr, "error audio_fd is %d\n", audio_fd);
				exit(-1);
			}
			else {
				fprintf(stderr, "error audio not available. output will go to stdout as 16 bit mono.\n");
				usleep_duration = 1000000000UL / SampleRate;
			}
		}
	}

#ifdef WIN32
	_setmode(_fileno(stdin), _O_BINARY);
	_setmode(_fileno(stdout), _O_BINARY);
#endif

	// Seed the random number generator
	srand(seed);

	// Initialize HF channel simulation parameters
	SetParms(SNR_parm, Chan_type);

	// Initialize the noise module
	Noise = init_noise(Noise_type, (float)SampleRate, ChannelBW);
	if (!Noise) {
		fprintf(stderr, "Noise module initialization failed\n");
		exit(1);
	}

	// Initialize HF channel Rayleigh fading coefficients
	GaussInit(FrSpread, TapUpdRate);

	// Initialize tapped delay line channel
	init_delayline(DelTime, SampleRate);

	// Calculate RMS over 256 samples, update every 64 samples
	RootMeanSqr = init_rms(256, 64);
	if (!RootMeanSqr) {
		fprintf(stderr, "RMS initialization failed\n");
		exit(1);
	}

	// Initialize the Hilbert transformer (200...3800Hz @ 8000sps)
	Filter = init_filter(200.0F / SampleRate, (ChannelBW + 200.0F) / SampleRate);
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

#ifdef USE_SOUND
		// Wait for a full data buffer -- this is our pacer.
		// This read() is a blocked call since the input buffer
		// is not yet full.
		if ((IO_type == 0 && audio_fd >= 0) || IO_type == 1) {
			ssize_t written;
			size_in = FILEDES_READ(audio_fd, audio_buf_in, BUF_SIZE * sizeof(int16_t));

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
			written = FILEDES_WRITE(audio_fd, audio_buf_out, size_out * sizeof(int16_t));
			if (written < 0) {
				perror("Error: write audio:");
				exit(-1);
			}
		}
#endif

		// internal test NCO - without soundcard
		if (IO_type == 0 && audio_fd < 0) {
			usleep( usleep_duration );
			size_in = BUF_SIZE;
			if (size_out)
				fwrite(audio_buf_out, sizeof(int16_t), size_out, stdout);
		}

		// File IO
		if (IO_type == 2) {
			size_in = (int)fread(audio_buf_in, sizeof(int16_t), BUF_SIZE, stdin);

			if (size_in == 0)
				break;

			fwrite(audio_buf_out, sizeof(int16_t), size_out, stdout);
		}
	}
	return 0;
}
