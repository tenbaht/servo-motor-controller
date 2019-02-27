/* gebrauchte Funktionen:
 */

#ifndef _CLI_H_
#define _CLI_H_

#ifdef __cplusplus
extern "C" {
#endif

//extern const char m_prompt[] PROGMEM;
//extern const char m_error[] PROGMEM;
//extern const char m_start[] PROGMEM;

// getval(): 0=kein Wert, 1=Wert ok. Wert ist in val
uint8_t get_val();
extern int24_t val;

void task_cli(void);

void cmd_err(void);
void dp_str(const char *pstr);	//	;Display string in flash

#ifdef __cplusplus
}
#endif

#endif
