.PHONY: smc3 clean upload flash eeprom disass

smc3:
	avra -l smc3.lst -m smc3.map -I inc smc3.asm

#	avrdude -c arduino -p m328p -U flash:w:smc3.hex -U eeprom:w:smc3.eep.hex

# assemble and compare the hex file with the reference
check: smc3
	@if diff smc3.hex referenz/smc3.hex ; then echo "alles ok."; fi

# upload flash only
upload:
	avrdude -c arduino -p m328p -U flash:w:smc3.hex

eeprom:
	avrdude -c arduino -p m328p -U eeprom:w:smc3.eep.hex

# - a disassembly from the hex file (not the obj file)
#   This is very useful for bytewise comparision with the avra output.
#
# The hex files needs to be normalized first, because because avra saves the
# interrupt vectors separately in individual pseudo-sections. This would
# mess up the alignment of the disassembler output with the as output.
disass: smc3.hex
	objcopy  -I ihex -O binary smc3.hex smc3.bin
	objcopy  -O ihex -I binary smc3.bin normal-avra.hex
	avr-objdump -j .sec1 -m avr5 -d -I ihex normal-avra.hex  > avra.disass

clean:
	rm -rf *~ *.bak *.cof *.hex *.lst *.map *.obj build
