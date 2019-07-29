#include "bsp_flash.h"
#include "main.h"
#include "string.h"
#include "struct_typedef.h"

static uint32_t Get_Next_Flash_Address(uint32_t Address);
static uint32_t GetSector(uint32_t Address);

void flash_erase_address(uint32_t address, uint16_t len) {
  FLASH_EraseInitTypeDef flash_erase;
  uint32_t error;

  flash_erase.Sector = GetSector(address);
  flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  flash_erase.NbSectors = len;

  HAL_FLASH_Unlock();
  HAL_FLASHEx_Erase(&flash_erase, &error);
  HAL_FLASH_Lock();
}

uint8_t flash_write_single_address(uint32_t Address, uint32_t *buf,
                                   uint32_t len) {
  uint32_t uwAddress = 0;
  uint32_t uwEndAddress = 0;
  uint32_t *data_buf;
  uint32_t data_len;

  HAL_FLASH_Unlock();

  uwAddress = Address;
  uwEndAddress = Get_Next_Flash_Address(Address);
  data_buf = buf;
  data_len = 0;

  while (uwAddress <= uwEndAddress) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, uwAddress, *data_buf) !=
        HAL_OK) {
      HAL_FLASH_Lock();
      return 1;
    } else {
      uwAddress = uwAddress + 4;
      data_buf++;
      data_len++;
      if (data_len == len) {
        break;
      }
    }
  }

  HAL_FLASH_Lock();
  return 0;
}

void flash_read(uint32_t Address, uint32_t *buf, uint32_t len) {
  memcpy(buf, (void *)Address, len * 4);
}

static uint32_t GetSector(uint32_t Address) {
  uint32_t sector = 0;

  if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0)) {
    sector = FLASH_SECTOR_0;
  } else if ((Address < ADDR_FLASH_SECTOR_2) &&
             (Address >= ADDR_FLASH_SECTOR_1)) {
    sector = FLASH_SECTOR_1;
  } else if ((Address < ADDR_FLASH_SECTOR_3) &&
             (Address >= ADDR_FLASH_SECTOR_2)) {
    sector = FLASH_SECTOR_2;
  } else if ((Address < ADDR_FLASH_SECTOR_4) &&
             (Address >= ADDR_FLASH_SECTOR_3)) {
    sector = FLASH_SECTOR_3;
  } else if ((Address < ADDR_FLASH_SECTOR_5) &&
             (Address >= ADDR_FLASH_SECTOR_4)) {
    sector = FLASH_SECTOR_4;
  } else if ((Address < ADDR_FLASH_SECTOR_6) &&
             (Address >= ADDR_FLASH_SECTOR_5)) {
    sector = FLASH_SECTOR_5;
  } else if ((Address < ADDR_FLASH_SECTOR_7) &&
             (Address >= ADDR_FLASH_SECTOR_6)) {
    sector = FLASH_SECTOR_6;
  } else if ((Address < ADDR_FLASH_SECTOR_8) &&
             (Address >= ADDR_FLASH_SECTOR_7)) {
    sector = FLASH_SECTOR_7;
  } else if ((Address < ADDR_FLASH_SECTOR_9) &&
             (Address >= ADDR_FLASH_SECTOR_8)) {
    sector = FLASH_SECTOR_8;
  } else if ((Address < ADDR_FLASH_SECTOR_10) &&
             (Address >= ADDR_FLASH_SECTOR_9)) {
    sector = FLASH_SECTOR_9;
  } else if ((Address < ADDR_FLASH_SECTOR_11) &&
             (Address >= ADDR_FLASH_SECTOR_10)) {
    sector = FLASH_SECTOR_10;
  }

  else if ((Address < ADDR_FLASH_SECTOR_12) &&
           (Address >= ADDR_FLASH_SECTOR_11)) {
    sector = FLASH_SECTOR_11;
  }

  else if ((Address < ADDR_FLASH_SECTOR_13) &&
           (Address >= ADDR_FLASH_SECTOR_12)) {
    sector = FLASH_SECTOR_12;
  } else if ((Address < ADDR_FLASH_SECTOR_14) &&
             (Address >= ADDR_FLASH_SECTOR_13)) {
    sector = FLASH_SECTOR_13;
  } else if ((Address < ADDR_FLASH_SECTOR_15) &&
             (Address >= ADDR_FLASH_SECTOR_14)) {
    sector = FLASH_SECTOR_14;
  } else if ((Address < ADDR_FLASH_SECTOR_16) &&
             (Address >= ADDR_FLASH_SECTOR_15)) {
    sector = FLASH_SECTOR_15;
  } else if ((Address < ADDR_FLASH_SECTOR_17) &&
             (Address >= ADDR_FLASH_SECTOR_16)) {
    sector = FLASH_SECTOR_16;
  } else if ((Address < ADDR_FLASH_SECTOR_18) &&
             (Address >= ADDR_FLASH_SECTOR_17)) {
    sector = FLASH_SECTOR_17;
  } else if ((Address < ADDR_FLASH_SECTOR_19) &&
             (Address >= ADDR_FLASH_SECTOR_18)) {
    sector = FLASH_SECTOR_18;
  } else if ((Address < ADDR_FLASH_SECTOR_20) &&
             (Address >= ADDR_FLASH_SECTOR_19)) {
    sector = FLASH_SECTOR_19;
  } else if ((Address < ADDR_FLASH_SECTOR_21) &&
             (Address >= ADDR_FLASH_SECTOR_20)) {
    sector = FLASH_SECTOR_20;
  } else if ((Address < ADDR_FLASH_SECTOR_22) &&
             (Address >= ADDR_FLASH_SECTOR_21)) {
    sector = FLASH_SECTOR_21;
  } else if ((Address < ADDR_FLASH_SECTOR_23) &&
             (Address >= ADDR_FLASH_SECTOR_22)) {
    sector = FLASH_SECTOR_22;
  } else /*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_23))*/
  {
    sector = FLASH_SECTOR_23;
  }
  return sector;
}

static uint32_t Get_Next_Flash_Address(uint32_t Address) {
  uint32_t sector = 0;

  if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0)) {
    sector = ADDR_FLASH_SECTOR_1;
  } else if ((Address < ADDR_FLASH_SECTOR_2) &&
             (Address >= ADDR_FLASH_SECTOR_1)) {
    sector = ADDR_FLASH_SECTOR_2;
  } else if ((Address < ADDR_FLASH_SECTOR_3) &&
             (Address >= ADDR_FLASH_SECTOR_2)) {
    sector = ADDR_FLASH_SECTOR_3;
  } else if ((Address < ADDR_FLASH_SECTOR_4) &&
             (Address >= ADDR_FLASH_SECTOR_3)) {
    sector = ADDR_FLASH_SECTOR_4;
  } else if ((Address < ADDR_FLASH_SECTOR_5) &&
             (Address >= ADDR_FLASH_SECTOR_4)) {
    sector = ADDR_FLASH_SECTOR_5;
  } else if ((Address < ADDR_FLASH_SECTOR_6) &&
             (Address >= ADDR_FLASH_SECTOR_5)) {
    sector = ADDR_FLASH_SECTOR_6;
  } else if ((Address < ADDR_FLASH_SECTOR_7) &&
             (Address >= ADDR_FLASH_SECTOR_6)) {
    sector = ADDR_FLASH_SECTOR_7;
  } else if ((Address < ADDR_FLASH_SECTOR_8) &&
             (Address >= ADDR_FLASH_SECTOR_7)) {
    sector = ADDR_FLASH_SECTOR_8;
  } else if ((Address < ADDR_FLASH_SECTOR_9) &&
             (Address >= ADDR_FLASH_SECTOR_8)) {
    sector = ADDR_FLASH_SECTOR_9;
  } else if ((Address < ADDR_FLASH_SECTOR_10) &&
             (Address >= ADDR_FLASH_SECTOR_9)) {
    sector = ADDR_FLASH_SECTOR_10;
  } else if ((Address < ADDR_FLASH_SECTOR_11) &&
             (Address >= ADDR_FLASH_SECTOR_10)) {
    sector = ADDR_FLASH_SECTOR_11;
  }

  else if ((Address < ADDR_FLASH_SECTOR_12) &&
           (Address >= ADDR_FLASH_SECTOR_11)) {
    sector = ADDR_FLASH_SECTOR_12;
  }

  else if ((Address < ADDR_FLASH_SECTOR_13) &&
           (Address >= ADDR_FLASH_SECTOR_12)) {
    sector = ADDR_FLASH_SECTOR_13;
  } else if ((Address < ADDR_FLASH_SECTOR_14) &&
             (Address >= ADDR_FLASH_SECTOR_13)) {
    sector = ADDR_FLASH_SECTOR_14;
  } else if ((Address < ADDR_FLASH_SECTOR_15) &&
             (Address >= ADDR_FLASH_SECTOR_14)) {
    sector = ADDR_FLASH_SECTOR_15;
  } else if ((Address < ADDR_FLASH_SECTOR_16) &&
             (Address >= ADDR_FLASH_SECTOR_15)) {
    sector = ADDR_FLASH_SECTOR_16;
  } else if ((Address < ADDR_FLASH_SECTOR_17) &&
             (Address >= ADDR_FLASH_SECTOR_16)) {
    sector = ADDR_FLASH_SECTOR_17;
  } else if ((Address < ADDR_FLASH_SECTOR_18) &&
             (Address >= ADDR_FLASH_SECTOR_17)) {
    sector = ADDR_FLASH_SECTOR_18;
  } else if ((Address < ADDR_FLASH_SECTOR_19) &&
             (Address >= ADDR_FLASH_SECTOR_18)) {
    sector = ADDR_FLASH_SECTOR_19;
  } else if ((Address < ADDR_FLASH_SECTOR_20) &&
             (Address >= ADDR_FLASH_SECTOR_19)) {
    sector = ADDR_FLASH_SECTOR_20;
  } else if ((Address < ADDR_FLASH_SECTOR_21) &&
             (Address >= ADDR_FLASH_SECTOR_20)) {
    sector = ADDR_FLASH_SECTOR_21;
  } else if ((Address < ADDR_FLASH_SECTOR_22) &&
             (Address >= ADDR_FLASH_SECTOR_21)) {
    sector = ADDR_FLASH_SECTOR_22;
  } else if ((Address < ADDR_FLASH_SECTOR_23) &&
             (Address >= ADDR_FLASH_SECTOR_22)) {
    sector = ADDR_FLASH_SECTOR_23;
  } else /*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_23))*/
  {
    sector = FLASH_END_ADDR;
  }
  return sector;
}
