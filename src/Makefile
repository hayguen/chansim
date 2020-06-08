all:		chansim

CC =		gcc
LD =		gcc
CFLAGS =	-Wall -Wstrict-prototypes -O9 
LDFLAGS =	
LIBS =		-lm
BINDIR =	/usr/local/bin

SRC =		main.c filter.c rms.c noise.c fade.c delay.c
OBJ =		$(SRC:.c=.o)


.c.o:
		$(CC) $(CFLAGS) -c $<

clean:
		rm -f *.o *~ *.bak gmon.out

distclean:	clean
		rm -f chansim
		rm -f genfilt
		rm -f filter-i386.h
		rm -f .depend

install:	all
		install -m 755 -s -o root -g root chansim $(BINDIR)

chansim:	filter-i386.h $(OBJ)
		$(LD) $(LDFLAGS) -o chansim $(OBJ) $(LIBS)

depend:		filter-i386.h
		$(CC) $(CFLAGS) -MM $(SRC) > .depend

filter-i386.h:	genfilt
		./genfilt > filter-i386.h

genfilt.o:	genfilt.c filter.h

ifeq (.depend,$(wildcard .depend))
include .depend
endif
