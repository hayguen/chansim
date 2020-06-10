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
		rm -f *.o chansim

distclean:	clean
		rm -f .depend

install:	all
		install -m 755 -s -o root -g root chansim $(BINDIR)

chansim:	$(OBJ)
		$(LD) $(LDFLAGS) -o chansim $(OBJ) $(LIBS)

depend:
		$(CC) $(CFLAGS) -MM $(SRC) > .depend

ifeq (.depend,$(wildcard .depend))
include .depend
endif

