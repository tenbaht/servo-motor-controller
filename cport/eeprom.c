#include "eeprom.h"

//;----------------------------------------------------------;
//; EEPROM Area

//.eseg
struct control_s parameter_bank[EEBANKS] EEMEM;


void load_parms(uint8_t idx)
{
	eeprom_read_block(Parms, &parameter_bank[idx], sizeof(struct control_s));
}

void save_parms(uint8_t idx)
{
	eeprom_update_block(Parms, &parameter_bank[idx], sizeof(struct control_s));
}
