MCU=atmega328
FORMAT=ihex
OBJCOPY=avr-objcopy

.PHONY: all clean upload eeprom list disass
.SUFFIXES: .elf .hex .eep.hex .lss .sym

all: smc3.hex smc3.eep.hex

# hex file for use with avrdude
.elf.hex:
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

# eeprom content for avrdude. eeprom upload is not supported by the
# Arduino bootloader, it requires an ISP connection.
.elf.eep.hex:
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# link
smc3.elf: smc3.o
	avr-gcc -mmcu=$(MCU) -nostartfiles -nostdlib -nodefaultlibs $^ -o $@


# compile
smc3.o: smc3.sx avr.inc
	avr-gcc -mmcu=$(MCU) -Wa,-I/usr/lib/avr/include -c $<

# debug helpers:
# - a readable assembler listing to stdout
list: smc3.o
	avr-objdump -dt smc3.o

# - a disassembly from the hex file (not the obj file)
#   This is very useful for bytewise comparision with the avra output.
disass: smc3.hex
	avr-objdump -j .sec1 -m avr5 -d smc3.hex  > smc3.disass


# upload flash only
upload: smc3.hex
	avrdude -c arduino -p m328p -U flash:w:$<

eeprom: smc3.eep.hex
	avrdude -c arduino -p m328p -U eeprom:w:$<


clean:
	rm -rf *~ *.bak *.o *.hex build
