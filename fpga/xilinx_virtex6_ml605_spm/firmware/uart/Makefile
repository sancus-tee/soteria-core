# Makefile

ROM       = 48K
RAM       = 10K
STACK     = 256

OBJS      = main.o
CFLAGS    = -Wall -Werror


all: main.elf


main.elf: $(OBJS)
	sancus-ld --standalone --rom-size $(ROM) --ram-size $(RAM) --sm-stack-size $(STACK) -o $@ $(OBJS)

%.o: %.c
	LD_LIBRARY_PATH=/usr/local/lib sancus-cc $(CFLAGS) -c -o $@ $<


clean:
	rm -f $(OBJS)

distclean: clean
	rm -f main.elf

 
.PHONY : all run clean distclean
