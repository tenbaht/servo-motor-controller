.PHONY: smc3 clean upload flash eeprom

smc3:
	avra -l smc3.lst -m smc3.map -I inc smc3.asm

#	avrdude -c arduino -p m328p -U flash:w:smc3.hex -U eeprom:w:smc3.eep.hex

# upload flash only
upload:
	avrdude -c arduino -p m328p -U flash:w:smc3.hex

eeprom:
	avrdude -c arduino -p m328p -U eeprom:w:smc3.eep.hex


clean:
	rm -rf *~ *.bak *.cof *.hex *.lst *.map *.obj build
