
#include <avr/io.h>

#include "csmc3.h"
#include "encoder.h"
// --------

// avr-gcc is smart enough to optimize this into a single swap command
static unsigned char nibbleSwap(unsigned char a)
{
	return (a<<4) | (a>>4);
}


void position_capture()
{
	static uint8_t PvEnc;	// Previous encoder signal A/B
	static uint8_t PvDir;	// Previous direction
	uint8_t ZL;

	ZL = PvEnc;
	PvEnc = nibbleSwap(PIND);
	if (PvEnc & 2) PvEnc ^= 1;	// Convert it to sequencial number.
	ZL = (ZL - PvEnc) & 3;		// Decode motion

	if (ZL) {
		if (ZL == 3) {
			// enc_rev
			PvDir = -1;
			Pos--;
		} else if (ZL == 1) {
			// enc_fwd
			PvDir = 1;
			Pos++;
		} else {
			// missing code recovery
			Pos += 2*PvDir;
		}
	}
}
