FORMAT=ihex
OBJCOPY=avr-objcopy

.PHONY: smc3 clean upload flash eeprom list
.SUFFIXES: .elf .hex .eep .lss .sym

all: smc3.hex

.elf.hex:
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@

.elf.eep:
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

smc3.elf: smc3.o
	avr-gcc -mmcu=atmega328 -nostartfiles -nostdlib -nodefaultlibs $^ --output $@

#	avr-gcc -mmcu=atmega328 -nostartfiles -nostdlib -nodefaultlibs $^ --output $@

#	avr-gcc -mmcu=atmega328 $^ --output $@

smc3.o: smc3.sx
	avr-gcc -mmcu=atmega328 -Wa,-I/usr/lib/avr/include -c $^

# debug helpers:
list:
	avr-objdump -dt smc3.o

disass: smc3.hex
	avr-objdump -j .sec1 -m avr5 -d smc3.hex  > smc3.disass



###

smc3:
	avr-as -mmcu=atmega328 -I/usr/lib/avr/include smc3.sx

mit-as:
	avr-as  -mmcu=atmega328 -I/usr/lib/avr/include  syntax-as.S


# kann .include, erkennt dann kein _SFR_IO_ADDR und will SREG dann später
# linken. => .include ist Quatsch.
#
# mit #include und _SFR_IO_ADDR geht es.
mit-gcc:
	avr-gcc  -mmcu=atmega328 -Wa,-I/usr/lib/avr/include -c syntax-gcc.S

# wie vorher, braucht #include und _SFR_IO_ADDR
mit-cpp:
	avr-gcc -mmcu=atmega328 -Wa,-I/usr/lib/avr/include -c syntax-cpp.sx

#avr-objcopy a.out -F ihex smc3.hex

#	avrdude -c arduino -p m328p -U flash:w:smc3.hex -U eeprom:w:smc3.eep.hex


# upload flash only
upload:
	avrdude -c arduino -p m328p -U flash:w:smc3.hex

eeprom:
	avrdude -c arduino -p m328p -U eeprom:w:smc3.eep.hex


clean:
	rm -rf *~ *.bak *.cof *.hex *.lst *.map *.obj build
