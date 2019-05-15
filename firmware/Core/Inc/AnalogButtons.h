#ifndef _AnalogButtons_H_
#define _AnalogButtons_H_

#include <stdbool.h>
#include <stdint.h> 
 
#define MAX_BUTTONS 4

/* Number of milliseconds for normal press detection */
#ifndef BUTTON_NORMAL_PRESS_TIME
#define BUTTON_NORMAL_PRESS_TIME  50
#endif

/* Number of milliseconds for long press detection */
#ifndef BUTTON_LONG_PRESS_TIME
#define BUTTON_LONG_PRESS_TIME    1000
#endif

#ifndef BUTTON_LONG_REPET_TIME
#define BUTTON_LONG_REPET_TIME    300
#endif

/* Button states */
#define BUTTON_STATE_START        0
#define BUTTON_STATE_PRESSED      1
#define BUTTON_STATE_WAITRELEASE  2

typedef enum {
	BUTTON_POWER  = 0,
	BUTTON_SELECT = 1,
	BUTTON_DOWN   = 2,
	BUTTON_UP     = 3,
	BUTTON_NONE   = -1
} BUTTON_ID_t;

typedef enum {
	BUTTON_PressType_OnPressed = 0x00, /*!< Button pressed */
	BUTTON_PressType_Normal,           /*!< Normal press type, released */
	BUTTON_PressType_Long              /*!< Long press type */
} BUTTON_PressType_t;

typedef struct {
	uint8_t  State;	
	uint16_t analogLowVal;
	uint16_t analogHighVal;
	void(*PressLong)(void);
	void(*PressNormal)(void);
} BUTTON_t;

void AnalogButtonUpdate(void);

extern void btnPowerPressed(void);
extern void btnSelectPressed(void);
extern void btnDownPressed(void);
extern void btnUpPressed(void);
extern void btnSelectHold(void);
extern void btnDownHold(void);
extern void btnUpHold(void);

#endif
