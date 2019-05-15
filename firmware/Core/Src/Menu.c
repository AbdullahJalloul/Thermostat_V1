#include "main.h"
#include "Menu.h"
#include "Eeprom.h"
#include "Config.h"
#include "Buzzer.h"
#include <string.h>

//**************************************************************************

#define DeltaTime  0.5
const uint8_t HEATER_STANDBY_TIME = 10; //in 1 Seconds
const uint8_t LONG_STEP  = 10;
const uint8_t SHORT_STEP =  1;

const uint16_t  PidKp = 9200;
const uint16_t  PidKi = 4000 * DeltaTime;
const uint16_t  PidKd = 1500 / DeltaTime;
const uint16_t  iMax  = 4000;

int16_t PidITerm;
double PidLastError;
//**************************************************************************
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim14;
extern EEPROM_Data_t     ConfigData;
extern double   Temperature;
extern uint32_t PreviousTime;
//**************************************************************************
HEATER_State_t  HeaterLastState;
HEATER_State_t  HeaterState = HEATER_OFF;
DISPLAY_State_t DisplayState = DISPLAY_OFF;
MENU_State_t    MenuState = MENU_IDLE;
SWITCH_State_t  SwitchState;

MENU_Parameters_t MenuParameters[2] = {
  {&ConfigData.Temperature, MIN_TEMP , MAX_TEMP },
  {&ConfigData.Timer      , MIN_TIMER, MAX_TIMER}
};
//**************************************************************************
uint8_t  DisplayTimer, MenuTimer, DisplayBlink, OneSecond;
uint16_t HeaterTimer;
//uint32_t SwitchTimeOut;

//**************************************************************************
uint8_t selDigit, SevenSegDigit[4];
const uint8_t SevenSegCode[10] = {
  0xD7,  // 0
  0x14,  // 1
  0xCD,  // 2
  0x5D,  // 3
  0x1E,  // 4
  0x5B,  // 5
  0xDB,  // 6
  0x15,  // 7
  0xDF,  // 8
  0x5F   // 9
};

const uint8_t SevenSegEnd[4] = {
  0xCB,  // E
  0x98,  // n
  0xDC,  // d
  0x00   // led off
};

const uint8_t SevenSegSave[4] = {
  0x5B,  // S
  0x9F,  // a
  0xD6,  // v
  0x00   // led off
};
//**************************************************************************
void DisplayUpdate(void) { // Update 500ms

  switch ((uint8_t)DisplayState) {
    case DISPLAY_TIME:
      SevenSegDecode(ConfigData.Timer - HeaterTimer, LED_TIME_BIT);
      break;

    case DISPLAY_TEMP:
      SevenSegDecode(Temperature, LED_TEMP_BIT);
      break;

    case DISPLAY_END_TIME:
      if (DisplayBlink) {
        memcpy(SevenSegDigit, SevenSegEnd, 4); // on
        BuzzerLongBeep();
      }
      else {
        memset(SevenSegDigit, 0, 4); // Off
      }
      DisplayBlink ^= 1;
      break;

    case DISPLAY_MENU: {
        if (MenuState == MENU_TEMP) {
          SevenSegDecode(*MenuParameters[MENU_TEMP - 1].Value, LED_TEMP_BIT);
        }
        else if (MenuState == MENU_TIME) {
          SevenSegDecode(*MenuParameters[MENU_TIME - 1].Value, LED_TIME_BIT);
        }
        else if (MenuState == MENU_SAVE) {
          MenuState = MENU_IDLE;
          DisplayState = DISPLAY_TEMP;
          DisplayTimer = 0;
          HeaterTimer = 0;
          memcpy(SevenSegDigit, SevenSegSave, 4); // Save
          EEPROM_Write();
          BuzzerLongBeep();
          PreviousTime = HAL_GetTick() + 1000;
        }
      }
      break;
  }
}
//**************************************************************************
void HeaterUpdate(void) { // Update 500ms

  if (OneSecond) {
    if (MenuState == MENU_IDLE) {
      if (HeaterState == HEATER_ON) { // HEATER_ON or HEATER_STANDBY

        if (++DisplayTimer >= DISPLAY_TIMER) {
          DisplayTimer = 0;
          DisplayBlink = 1;
          if (++DisplayState >  DISPLAY_TEMP) DisplayState = DISPLAY_TIME;
        }

        if (SwitchState == SWITCH_STATE_CLOSE) {
          if (++HeaterTimer >= ConfigData.Timer) {
            HeaterTimer = 0;
            DisplayBlink = 1;
            HeaterState = HEATER_STANDBY;
            DisplayState = DISPLAY_END_TIME;
          }
        }

      }
      else if (HeaterState == HEATER_STANDBY) {

        if (++HeaterTimer >= HEATER_STANDBY_TIME) {
          HeaterTurnOff();
        }

      }
    }
    else {
      if (--MenuTimer == 0) {
        HeaterTurnOff();
      }
    }
  }

  if (HeaterState != HEATER_OFF) HeaterPid();

}
//**************************************************************************
void HeaterSwitch(void) { // Update all time

  if ((PNP_SW_GPIO_Port->IDR & PNP_SW_Pin) == 0) { // switch closed

    if (HeaterState != HEATER_OFF) {

      if (SwitchState == SWITCH_STATE_CLOSE) {
        SwitchState = SWITCH_STATE_OPEN;
        //SwitchTimeOut = HAL_GetTick() + (HEATER_STANDBY_TIME * 1000);
        HeaterLastState = HeaterState;
        if (HeaterLastState == HEATER_ON) {
          HeaterTimer = 0;
          DisplayState = DISPLAY_TEMP;
        }
        BuzzerShortBeep();
      }
      /*else if(SwitchState == SWITCH_STATE_OPEN) {

      	if(HAL_GetTick() > SwitchTimeOut) {
      		SwitchState = SWITCH_STATE_LONG_OPENING;
      		HeaterTurnOff();
      	}

      }
      */
    }

  }
  else if (SwitchState != SWITCH_STATE_CLOSE) { // switch open
    SwitchState = SWITCH_STATE_CLOSE;
    BuzzerShortBeep();
    if (HeaterLastState == HEATER_STANDBY) {
      HeaterLastState = HEATER_ON;
      HeaterState = HEATER_ON;
      DisplayState = DISPLAY_TEMP;
      HeaterTimer = 0;

    }

  }

}
//**************************************************************************
//*********************** BUTTON FUNCTIONS *********************************
//**************************************************************************
void btnPowerPressed(void) {
  if (DisplayState == DISPLAY_OFF) {
    DisplayState = DISPLAY_TEMP;
    HeaterState  = HEATER_ON;
    // HeaterTimer  = 0;
    HeaterUpdate();
    DisplayUpdate();
  }
  else {
    HeaterTurnOff();
  }

  BuzzerShortBeep();
}

//**************************************************************************
void btnSelectHold(void) {
  if (DisplayState != DISPLAY_OFF) {

    if (MenuState == MENU_IDLE) {
      MenuState = MENU_TEMP;
      MenuTimer = MENU_TIMMING;
      DisplayState = DISPLAY_MENU;
      DisplayUpdate();
      BuzzerShortBeep();
    }
    else { // Save Parameters
      MenuState = MENU_SAVE;
      DisplayUpdate();
    }

  }
}
//**************************************************************************
void btnSelectPressed(void) {
  if (MenuState != MENU_IDLE) {
    if (++MenuState >= MENU_SAVE) MenuState = MENU_TEMP;
    MenuTimer = MENU_TIMMING;
    DisplayUpdate();
    BuzzerShortBeep();
  }
}
//**************************************************************************
void btnDownHold(void) {
  if (MenuState != MENU_IDLE) {
    if (*MenuParameters[MenuState - 1].Value > MenuParameters[MenuState - 1].min + (LONG_STEP - 1))
      *MenuParameters[MenuState - 1].Value -= LONG_STEP;
    else
      *MenuParameters[MenuState - 1].Value = MenuParameters[MenuState - 1].min;
    MenuTimer = MENU_TIMMING;
    DisplayUpdate();
    BuzzerShortBeep();
  }
}
//**************************************************************************
void btnDownPressed(void) {
  if (MenuState != MENU_IDLE) {
    if (*MenuParameters[MenuState - 1].Value > MenuParameters[MenuState - 1].min)
      *MenuParameters[MenuState - 1].Value -= SHORT_STEP;
    else
      *MenuParameters[MenuState - 1].Value = MenuParameters[MenuState - 1].min;
    MenuTimer = MENU_TIMMING;
    DisplayUpdate();
    BuzzerShortBeep();
  }
}
//**************************************************************************
void btnUpHold(void) {
  if (MenuState != MENU_IDLE) {
    if (*MenuParameters[MenuState - 1].Value < MenuParameters[MenuState - 1].max - (LONG_STEP - 1))
      *MenuParameters[MenuState - 1].Value += LONG_STEP;
    else
      *MenuParameters[MenuState - 1].Value = MenuParameters[MenuState - 1].max;
    MenuTimer = MENU_TIMMING;
    DisplayUpdate();
    BuzzerShortBeep();
  }
}
//**************************************************************************
void btnUpPressed(void) {
  if (MenuState != MENU_IDLE) {
    if (*MenuParameters[MenuState - 1].Value < MenuParameters[MenuState - 1].max)
      *MenuParameters[MenuState - 1].Value += SHORT_STEP;
    else
      *MenuParameters[MenuState - 1].Value = MenuParameters[MenuState - 1].max;
    MenuTimer = MENU_TIMMING;
    DisplayUpdate();
    BuzzerShortBeep();
  }
}
//**************************************************************************
void HeaterTurnOff(void) {
  TIM14->CCR1 = 0;
  HeaterTimer = 0;
  HeaterState = HEATER_OFF;
  HeaterLastState = HEATER_OFF;
  DisplayState = DISPLAY_OFF;
  MenuState = MENU_IDLE;
  memset(SevenSegDigit, 0x08, 4);    // ---
}
//**************************************************************************
void HeaterPid(void) {
  double PidError;
  if (HeaterState == HEATER_ON) {
    PidError  = ConfigData.Temperature - Temperature;
  }
  else { // HEATER_STANDBY
    PidError  = (ConfigData.Temperature / 2) - Temperature;
  }

  int32_t PidResult = PidKp * PidError;

  PidITerm += PidError * PidKi;

  if (PidITerm > iMax) PidITerm = iMax;
  else if (PidITerm < -iMax) PidITerm = -iMax;

  PidResult += PidITerm;

  PidResult += (PidLastError - PidError) * PidKd;
  PidLastError = PidError;

  if (PidResult < 0) PidResult = 0;
  else if (PidResult > 24999) PidResult = 24999;

  TIM14->CCR1 = PidResult; // SSR Pin(TIM14 CH1 PWM)
}

//**************************************************************************
void SevenSegDecode(uint16_t Value, uint8_t indicator) { // Update 500ms
  if (indicator == LED_TEMP_BIT) {
    SevenSegDigit[0] = SevenSegCode[Value / 100];
    SevenSegDigit[1] = SevenSegCode[(Value % 100) / 10];
    SevenSegDigit[2] = SevenSegCode[Value % 10];
    if (SevenSegDigit[0] == 0xD7) SevenSegDigit[0] = 0; // if 0
  }
  else {
    uint8_t Minutes = Value / 60;
    uint8_t Seconds = Value % 60;
    SevenSegDigit[0] = SevenSegCode[Minutes];
    SevenSegDigit[1] = SevenSegCode[Seconds / 10];
    SevenSegDigit[2] = SevenSegCode[Seconds % 10];
    if (OneSecond) SevenSegDigit[0] |= SEG_DP_BIT;
    //DisplayBlink ^= 1;
  }
  SevenSegDigit[3] = indicator;
}
//**************************************************************************
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim->Instance == TIM3) {

    DIG0_GPIO_Port->BRR = DIG0_Pin | DIG1_Pin | DIG2_Pin | DIG3_Pin; // Reset PINs
    uint8_t temp = 0;
    HAL_SPI_Transmit(&hspi1, &temp, 1, 1);
    LATCH_GPIO_Port->BSRR = LATCH_Pin; // Set HIGH
    LATCH_GPIO_Port->BRR  = LATCH_Pin; // Set LOW

    switch (selDigit) {
      case 0: DIG0_GPIO_Port->BSRR = DIG0_Pin; break;
      case 1: DIG1_GPIO_Port->BSRR = DIG1_Pin; break;
      case 2: DIG2_GPIO_Port->BSRR = DIG2_Pin; break;
      case 3: DIG3_GPIO_Port->BSRR = DIG3_Pin; break;
    }

    HAL_SPI_Transmit(&hspi1, &SevenSegDigit[selDigit], 1, 1);
    LATCH_GPIO_Port->BSRR = LATCH_Pin; // Set HIGH
    LATCH_GPIO_Port->BRR  = LATCH_Pin; // Set LOW

    if (++selDigit >= 4) selDigit = 0;

  }
}
//**************************************************************************
