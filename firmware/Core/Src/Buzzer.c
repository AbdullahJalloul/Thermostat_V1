#include "Buzzer.h"
#include "main.h"
#include "Config.h"

uint8_t  BuzzerState;
uint32_t BuzzerTimer;
//******************************************************************************
void BuzzerShortBeep(void) {
  BuzzerSingleBeep(50);
}
//******************************************************************************
void BuzzerLongBeep(void) {
  BuzzerSingleBeep(500);
}
//******************************************************************************
void BuzzerSingleBeep(uint16_t Duration) {
  BuzzerTimer = Duration  + HAL_GetTick();
  BUZZER_GPIO_Port->BSRR = (uint32_t)BUZZER_Pin;
  BuzzerState = 1;
}
//******************************************************************************
void BuzzerUpdate(void) {
  if (BuzzerState) {
    if (HAL_GetTick() >= BuzzerTimer) {
      BUZZER_GPIO_Port->BRR = (uint32_t)BUZZER_Pin;
      BuzzerState = 0;
    }
  }
}
//******************************************************************************
