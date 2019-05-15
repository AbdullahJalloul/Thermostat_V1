#include "Eeprom.h"
#include "main.h"
#include "Config.h"

EEPROM_Data_t ConfigData;

void EEPROM_Write(void) {
	uint32_t PageError = 0;
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = ADDR_FLASH_PAGE_15;
  EraseInitStruct.NbPages = 1;
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	
	uint8_t Checksum = 0x55;
	uint8_t *ptr = (uint8_t *)&ConfigData;
	
	for(uint8_t n = 0; n < sizeof(EEPROM_Data_t); n++)
		Checksum ^= *(ptr++);
	
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_USER_START_ADDR + 0, ConfigData.Temperature);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_USER_START_ADDR + 2, ConfigData.Timer);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_USER_START_ADDR + 4, Checksum);

	HAL_FLASH_Lock();
}

void EEPROM_Read (void) {

	ConfigData.Temperature = *(uint16_t *)(FLASH_USER_START_ADDR);
	ConfigData.Timer = *(uint16_t *)(FLASH_USER_START_ADDR + 2);
	uint8_t Checksum = *(uint16_t *)(FLASH_USER_START_ADDR + 4);
	
	uint8_t Sum = 0x55;
	uint8_t *ptr = (uint8_t *)&ConfigData;
	
	for(uint8_t n = 0; n < sizeof(EEPROM_Data_t); n++)
		Sum ^= *(ptr++);
	
	if(Sum != Checksum) {
		ConfigData.Temperature = MIN_TEMP;
		ConfigData.Timer = MIN_TIMER;
		EEPROM_Write();
	}
}
