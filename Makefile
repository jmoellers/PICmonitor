PICcc = xc16-gcc
PICbin2hex = xc16-bin2hex
PICLIBS = -T p24FJ64GA002.gld
PICCFLAGS += -O -mcpu=24FJ64GA002

PICmonitor:	PICmonitor.c getErrLoc.o
	xc16-gcc $(PICCFLAGS) -o $@ PICmonitor.c getErrLoc.o $(PICLIBS)

getErrLoc.o:	getErrLoc.s
	xc16-gcc $(PICCFLAGS) -c -o $@ $< $(PICLIBS)
PICmonitor.hex:	PICmonitor
	xc16-bin2hex $<

PICmonitor.asm:	PICmonitor
	xc16-objdump --disassemble-all $< > $@
