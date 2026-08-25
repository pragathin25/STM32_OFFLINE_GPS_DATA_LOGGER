#ifndef NAND_H
#define NAND_H

#include "main.h"

extern SPI_HandleTypeDef hspi1;

#define NAND_CS_PORT GPIOA
#define NAND_CS_PIN  nand_cs_Pin   // use your IOC name

void NAND_Init(void);
void NAND_Select(void);
void NAND_Deselect(void);

void NAND_Reset(void);
void NAND_ReadID(uint8_t *id);

void NAND_PageRead(uint32_t page);
void NAND_ReadFromCache(uint16_t column, uint8_t *buffer, uint16_t len);
void NAND_PageProgram(uint32_t page, uint8_t *data);

uint8_t NAND_ReadStatus(void);
int NAND_WaitBusy(void);
int NAND_BlockErase(uint32_t block);
void NAND_WriteEnable(void);
void NAND_SetFeature(uint8_t reg, uint8_t value);
void NAND_ProgramLoad(uint16_t column, uint8_t *data, uint16_t size);
void NAND_ProgramExecute(uint32_t page);


// LittleFS helper functions
int NAND_Read(uint32_t page, uint16_t column, uint8_t *buffer, uint32_t length);
int NAND_Write(uint32_t page, uint16_t column, const uint8_t *buffer, uint32_t length);
#endif
