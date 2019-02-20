
// --------


void position_capture()
{
	static uint8_t PvEnc;	// Previous encoder signal A/B
	static uint8_t PvDir;	// Previous direction
	uint8_t ZL;

	ZL = PvEnc;
	PvEnc = swap(PIND);
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

