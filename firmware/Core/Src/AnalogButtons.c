#include "main.h"
#include "Config.h"
#include "AnalogButtons.h"




uint16_t ButtonStartTime;

extern uint32_t ADC1_Result[3];

BUTTON_t buttons[MAX_BUTTONS] = {
	{ BUTTON_STATE_START, 51136, 51236,  NULL         , &btnPowerPressed  },
	{ BUTTON_STATE_START, 25512, 25612, &btnSelectHold, &btnSelectPressed },
	{ BUTTON_STATE_START, 16981, 17081, &btnDownHold  , &btnDownPressed   },
	{ BUTTON_STATE_START, 12716, 12816, &btnUpHold    , &btnUpPressed     }
};

//******************************************************************************

void AnalogButtonUpdate(void) {
  uint16_t AdcKeypad = ADC1_Result[ADC1_KEYPAD] >> 2;
  uint16_t now = HAL_GetTick();

  for (uint8_t i = 0; i < MAX_BUTTONS; i++) {
    if (AdcKeypad <= buttons[i].analogHighVal + 4 && AdcKeypad >= buttons[i].analogLowVal - 4 ) {

      if (buttons[i].State == BUTTON_STATE_START) {
        buttons[i].State = BUTTON_STATE_PRESSED;
        ButtonStartTime = now;
        return;
      }

      if (buttons[i].State == BUTTON_STATE_PRESSED) {
        if (now > (ButtonStartTime + BUTTON_LONG_PRESS_TIME)) {
          if (buttons[i].PressLong != NULL) buttons[i].PressLong();
					ButtonStartTime = now;
          buttons[i].State = BUTTON_STATE_WAITRELEASE;
        }
      }
			else if(buttons[i].State == BUTTON_STATE_WAITRELEASE) {
				if (now > (ButtonStartTime + BUTTON_LONG_REPET_TIME)) {
					if (buttons[i].PressLong != NULL) {
						if(i != BUTTON_SELECT) buttons[i].PressLong();
					}
					ButtonStartTime = now;
				}
			}
    }
    else if (buttons[i].State == BUTTON_STATE_PRESSED) {
      if ( now > (ButtonStartTime + BUTTON_NORMAL_PRESS_TIME) ) {
        buttons[i].PressNormal();
        buttons[i].State = BUTTON_STATE_WAITRELEASE;
      }
      else {
        buttons[i].State = BUTTON_STATE_START;
      }
    }
    else {
      buttons[i].State = BUTTON_STATE_START;
    }
  }
}

