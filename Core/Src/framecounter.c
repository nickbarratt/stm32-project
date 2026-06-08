#include "FreeRTOS.h" // Must always be included BEFORE task.h
#include "task.h"     // Contains definitions for vTaskSuspendAll and xTaskResumeAll
#include "stm32f4xx_hal.h"

// Define the exact hardware parameters for Sector 7 on your STM32F407VE
#define FLASH_STORAGE_ADDRESS   0x08080000 
#define FLASH_TARGET_SECTOR     FLASH_SECTOR_7
#define SECTOR_SIZE_BYTES       (128 * 1024) // 128 KB
#define MAX_SLOTS               (SECTOR_SIZE_BYTES / 2) // 65,536 2-byte slots

void Save_Frame_Counter_Wear_Leveled(uint16_t counter) {
    // CRITICAL FIX: Temporarily freeze the FreeRTOS scheduler core.
    // This guarantees no context switching can happen while we touch the flash registers!
    vTaskSuspendAll();

    uint32_t current_address = FLASH_STORAGE_ADDRESS;
    int32_t empty_slot_index = -1;

    for (uint32_t i = 0; i < MAX_SLOTS; i++) {
        uint16_t val = *(__IO uint16_t*)(FLASH_STORAGE_ADDRESS + (i * 2));
        if (val == 0xFFFF) {
            empty_slot_index = i;
            break;
        }
    }

    HAL_FLASH_Unlock();

    if (empty_slot_index == -1) {
        FLASH_EraseInitTypeDef EraseInitStruct;
        uint32_t SectorError = 0;
        EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
        EraseInitStruct.Sector       = FLASH_TARGET_SECTOR;
        EraseInitStruct.NbSectors    = 1;
        EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

        if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK) {
            current_address = FLASH_STORAGE_ADDRESS;
        }
    } else {
        current_address = FLASH_STORAGE_ADDRESS + (empty_slot_index * 2);
    }

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, current_address, counter);
    HAL_FLASH_Lock();

    // Re-enable the FreeRTOS scheduler core safely
    xTaskResumeAll();
}

uint16_t Load_Frame_Counter_Wear_Leveled(void) {
    uint16_t highest_counter = 0;
    
    // We are just reading standard memory addresses here, so no suspension is needed yet
    for (uint32_t i = 0; i < MAX_SLOTS; i++) {
        uint16_t val = *(__IO uint16_t*)(FLASH_STORAGE_ADDRESS + (i * 2));
        if (val == 0xFFFF) {
            break; 
        }
        highest_counter = val;
    }
    
    return highest_counter; 
}

