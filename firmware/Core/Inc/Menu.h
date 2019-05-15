#ifndef _Menu_H_
#define _Menu_H_

#include <stdbool.h>
#include <stdint.h> 

typedef struct {
	uint16_t *Value;
	const uint16_t min, max; 
} MENU_Parameters_t;

typedef enum {
  HEATER_OFF,
  HEATER_ON,
  HEATER_STANDBY
} HEATER_State_t;

typedef enum {
  DISPLAY_OFF ,
  DISPLAY_TIME,
  DISPLAY_TEMP,
  DISPLAY_END_TIME,
  DISPLAY_MENU
} DISPLAY_State_t;

typedef enum {
  MENU_IDLE,
  MENU_TEMP,
  MENU_TIME,
  MENU_SAVE
} MENU_State_t;

typedef enum {
  SWITCH_STATE_CLOSE,
	SWITCH_STATE_OPEN,      
  SWITCH_STATE_LONG_OPENING
} SWITCH_State_t;

void updateMenuState(void);
void btnPowerPressed(void);
void btnSelectPressed(void);
void btnDownPressed(void);
void btnUpPressed(void);
void btnSelectHold(void);
void btnDownHold(void);
void btnUpHold(void);

void SevenSegDecode(uint16_t Value, uint8_t indicator);

void DisplayUpdate(void);
void HeaterUpdate (void);
void HeaterTurnOff(void);
void HeaterSwitch (void);
void HeaterPid(void);

#endif
