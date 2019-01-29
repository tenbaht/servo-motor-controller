.PHONY: smc3 clean

smc3:
	avra -l smc3.lst -m smc3.map -I inc smc3.asm

upload:
	avrdude -c arduino -p m328p -U flash:w:smc3.hex

clean:
	rm -rf *~ *.bak *.cof *.hex *.lst *.map *.obj build
