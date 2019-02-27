#ifndef _EEPROM_H_
#define _EEPROM_H_

#include <stdint.h>
#include <avr/eeprom.h>
#include "csmc3.h"

#define EEBANKS	((E2END+1)/sizeof(struct control_s))

extern struct control_s parameter_bank[EEBANKS] EEMEM;

void load_parms(uint8_t idx);
void save_parms(uint8_t idx);


#endif
