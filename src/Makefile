all:		chansim

CC =		gcc
LD =		gcc
CFLAGS =	-Wall -Wstrict-prototypes -std=c99 -D_GNU_SOURCE -O9
LDFLAGS =	
LIBS =		-lm
BINDIR =	/usr/local/bin

SRC =		main.c rms.c noise.c fade.c delay.c filter.c
OBJ =		$(SRC:.c=.o)


.c.o:
		$(CC) $(CFLAGS) -c $<

clean:
		rm -f *.o chansim NCO-*.bin NCO-*.wav

distclean:	clean
		rm -f .depend

install:	all
		install -m 755 -s -o root -g root chansim $(BINDIR)

chansim:	$(OBJ)
		$(LD) $(LDFLAGS) -o chansim $(OBJ) $(LIBS)

test:	chansim
		echo "running tests with 15 dB SNR"
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  15 0 >NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_0-noise-only.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  15 1 >NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_1-flat1.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  15 2 >NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_2-flat2.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  15 3 >NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_3-CCIR-good.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  15 4 >NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_4-CCIR-moderate.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  15 5 >NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_5-CCIR-poor.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  15 6 >NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_6-CCIR-flutter.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  15 7 >NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_7-extreme.bin

testwav:	test
		echo "rtl_raw2wav is available in https://github.com/hayguen/librtlsdr"
		echo "  but you can convert raw .bin files also with sox"
		echo "  or play directly with aplay"
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_0-noise-only.wav    -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_0-noise-only.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_1-flat1.wav         -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_1-flat1.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_2-flat2.wav         -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_2-flat2.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_3-CCIR-good.wav     -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_3-CCIR-good.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_4-CCIR-moderate.wav -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_4-CCIR-moderate.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_5-CCIR-poor.wav     -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_5-CCIR-poor.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_6-CCIR-flutter.wav  -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_6-CCIR-flutter.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_7-extreme.wav       -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-15dB_7-extreme.bin

test25:	chansim
		echo "running tests with 25 dB SNR"
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  25 0 >NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_0-noise-only.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  25 1 >NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_1-flat1.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  25 2 >NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_2-flat2.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  25 3 >NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_3-CCIR-good.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  25 4 >NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_4-CCIR-moderate.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  25 5 >NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_5-CCIR-poor.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  25 6 >NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_6-CCIR-flutter.bin
		-timeout 10 ./chansim -i 0 -f 700 -b 1000 -n 0 -r 1  25 7 >NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_7-extreme.bin

test25wav:	test25
		echo "rtl_raw2wav is available in https://github.com/hayguen/librtlsdr"
		echo "  but you can convert raw .bin files also with sox"
		echo "  or play directly with aplay"
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_0-noise-only.wav    -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_0-noise-only.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_1-flat1.wav         -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_1-flat1.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_2-flat2.wav         -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_2-flat2.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_3-CCIR-good.wav     -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_3-CCIR-good.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_4-CCIR-moderate.wav -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_4-CCIR-moderate.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_5-CCIR-poor.wav     -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_5-CCIR-poor.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_6-CCIR-flutter.wav  -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_6-CCIR-flutter.bin
		rtl_raw2wav -w NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_7-extreme.wav       -s 8000 -c 1 -b 16 -r NCO-700Hz_BW-1kHz_Ngauss_SNR-25dB_7-extreme.bin

depend:
		$(CC) $(CFLAGS) -MM $(SRC) > .depend

ifeq (.depend,$(wildcard .depend))
include .depend
endif

