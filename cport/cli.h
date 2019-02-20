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

void task_cli(void);

void dp_str(const char *pstr);	//	;Display string in flash

#ifdef __cplusplus
}
#endif

#endif
