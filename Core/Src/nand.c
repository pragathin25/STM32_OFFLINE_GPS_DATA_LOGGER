#include "nand.h"
#include <stdio.h>

// ---------------- CS ----------------
void NAND_Select() {
HAL_GPIO_WritePin(NAND_CS_PORT, NAND_CS_PIN, GPIO_PIN_RESET);//cs high,[nand chip active] CS =0 NAND listen to SPI commands
}


void NAND_Deselect() {
HAL_GPIO_WritePin(NAND_CS_PORT, NAND_CS_PIN, GPIO_PIN_SET);   //CS=1 Communication finished , cs high
}

// ---------------- WRITE ENABLE ----------------
void NAND_WriteEnable(void)
{
uint8_t cmd = 0x06;
NAND_Select();
HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY); // NAND sets WEL bit
NAND_Deselect();
HAL_Delay(1);

}

// ---------------- RESET ----------------

void NAND_Reset() {
uint8_t cmd = 0xFF;
NAND_Select();
HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY); //Reboot NAND internally
NAND_Deselect();
HAL_Delay(5);
}

// ---------------- INIT ----------------
void NAND_Init()
{
    NAND_Deselect();
    HAL_Delay(10);

    //printf("NAND RESET...\r\n");

    NAND_Reset();

    uint8_t status;//Variable to store the status register
    uint32_t timeout = HAL_GetTick();//Stores the current time in milliseconds(Used for timeout protection)

    while (1)
    {
        status = NAND_ReadStatus();//Reads the NAND status register.

        if ((status & 0x01) == 0)   // 0x01 mask - select bit 0 , if = 0 busy bit =0 nand is ready  or if = 1 nand is busy
            break;

        if (HAL_GetTick() - timeout > 2000)
        {
            printf("❌ NAND NOT READY AFTER RESET! STATUS=0x%02X\r\n", status);
            break;
        }
    }

   // printf("NAND READY AFTER RESET\r\n");

    // Unlock blocks (Unlocked means the NAND flash does not protect that block from being modified)
    NAND_SetFeature(0xA0, 0x00);//Controls block protection.

    // Enable ECC
    NAND_SetFeature(0xB0, 0x10);//ECC automatically detects and corrects bit errors during reads.
}
// ---------------- READ ID ----------------
void NAND_ReadID(uint8_t *id) //(EFAA21)
{
uint8_t cmd = 0x9F;  // Read JEDEC ID
uint8_t dummy;  //Used to discard an unwanted byte.

NAND_Select();

HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);//Give me your ID

// discard first byte
HAL_SPI_Receive(&hspi1, &dummy, 1, HAL_MAX_DELAY);

HAL_SPI_Receive(&hspi1, id, 3, HAL_MAX_DELAY);  // 3-reads 3 id bytes manufacturer memory device

NAND_Deselect();

}

// ---------------- READ STATUS ----------------
uint8_t NAND_ReadStatus(void)
{
uint8_t cmd[2] = {0x0F, 0xC0};  //Give me the contents of Status Register
uint8_t status;

NAND_Select();
HAL_SPI_Transmit(&hspi1, cmd, 2, HAL_MAX_DELAY);  //Read Status Register
uint8_t dummy = 0xFF;
HAL_SPI_TransmitReceive(&hspi1, &dummy, &status, 1, HAL_MAX_DELAY);
NAND_Deselect();

return status;

}

// ---------------- WAIT BUSY -----------------'
int NAND_WaitBusy(void)//This prevents the program from hanging indefinitely.
{
    uint32_t start = HAL_GetTick();

    while (1)
    {
        uint8_t status = NAND_ReadStatus();

        if ((status & 0x01) == 0)
        {
            return 0;   // READY
        }

        if (HAL_GetTick() - start > 2000)
        {
            printf("❌ NAND BUSY TIMEOUT STATUS=0x%02X\r\n", status);
            return -1;
        }
    }
}
// ---------------- PAGE READ ----------------
void NAND_PageRead(uint32_t page) //(uint32_t page) Takes the page number that needs to be read
{
    //printf("PAGE READ: %lu\r\n", page);

    uint8_t cmd[4];//4 bytes form the SPI command sent to NAND

    cmd[0] = 0x13;//Load the requested page from flash memory into the internal cache
    cmd[1] = (page >> 16) & 0xFF;//extracts the highest 8 bits (page address is 24bits)
    cmd[2] = (page >> 8) & 0xFF;//Gets the middle byte.
    cmd[3] = page & 0xFF;//Gets the lowest byte.

    NAND_Select();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    NAND_Deselect();

    NAND_WaitBusy();
}

// ---------------- READ FROM CACHE ----------------
void NAND_ReadFromCache(uint16_t column, uint8_t *buffer, uint16_t len)//Reads data from the NAND's internal cache into the STM32 RAM buffer.
{
    uint8_t cmd[4];

    cmd[0] = 0x03;
    cmd[1] = (column >> 8) & 0xFF;//Extracts the upper 8 bits of the column address.
    cmd[2] = column & 0xFF;//Gets the lower byte.
    cmd[3] = 0x00;//dummy byte

    NAND_Select();

    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);

    HAL_SPI_TransmitReceive(&hspi1, buffer, buffer, len, HAL_MAX_DELAY);

    NAND_Deselect();
}

// ---------------- PAGE PROGRAM ----------------

void NAND_PageProgram(uint32_t page, uint8_t *data) // (writes 2048 bytes from STM32 RAM into a NAND flash page)
{
    uint8_t cmd[3];

    //  STEP 1: Write Enable
    NAND_WriteEnable();

    //  DEBUG: check WEL immediately
    uint8_t status = NAND_ReadStatus();
   // printf("WEL STATUS: 0x%02X\r\n", status);


    // If WEL not set → STOP
    if (!(status & 0x02))
    {
        //printf("WEL NOT SET! \r\n");
        return;
    }

    // ✅ STEP 2: Load data into cache
    cmd[0] = 0x02; // Copy incoming data into NAND cache
    cmd[1] = 0x00;
    cmd[2] = 0x00;

    NAND_Select();
    HAL_SPI_Transmit(&hspi1, cmd, 3, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, data, 2048, HAL_MAX_DELAY);
    NAND_Deselect();

    // ✅ STEP 3: Write Enable AGAIN (VERY IMPORTANT)
    NAND_WriteEnable();

    // ✅ STEP 4: Execute program
    uint8_t exec[4];
    exec[0] = 0x10;  //Copies cache contents into flash memory
    exec[1] = (page >> 16) & 0xFF;
    exec[2] = (page >> 8) & 0xFF;
    exec[3] = page & 0xFF;         // Program Page 10

    NAND_Select();
    HAL_SPI_Transmit(&hspi1, exec, 4, HAL_MAX_DELAY);   //The NAND now physically programs the flash cells.
    NAND_Deselect();

    // ✅ WAIT
    NAND_WaitBusy();

    status = NAND_ReadStatus();

   // printf("FINAL STATUS: 0x%02X\r\n", status);

    if (status & 0x08)
    {
       // printf("PROGRAM FAIL\r\n");
    }
}

// ---------------- BLOCK ERASE ----------------
int NAND_BlockErase(uint32_t block)//Erases one NAND block.
{
   // printf("ERASING BLOCK %lu\r\n", block);

    uint32_t page = block * 64;
    uint8_t cmd[4];//Erase command.
    uint8_t status;

    NAND_WriteEnable();

    status = NAND_ReadStatus();

    cmd[0] = 0xD8;//Block Erase command.
    cmd[1] = (page >> 16) & 0xFF;
    cmd[2] = (page >> 8) & 0xFF;
    cmd[3] = page & 0xFF;

    NAND_Select();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    NAND_Deselect();

    //printf("ERASE CMD SENT\r\n");

    NAND_WaitBusy();

    status = NAND_ReadStatus();

    if (status & 0x04)
    {
        //printf("ERASE FAILED\r\n");
        return -1;
    }

   // printf("ERASE DONE\r\n");

    return 0;
}

// ---------------- SET FEATURE ----------------
void NAND_SetFeature(uint8_t reg, uint8_t value)//Changes a NAND configuration register.
{
    uint8_t cmd[3];

    NAND_WriteEnable();

    cmd[0] = 0x1F;
    cmd[1] = reg;//Feature register.
    cmd[2] = value;

    NAND_Select();
    HAL_SPI_Transmit(&hspi1, cmd, 3, HAL_MAX_DELAY);
    NAND_Deselect();

    NAND_WaitBusy();
}

void NAND_ProgramLoad(uint16_t column, uint8_t *data, uint16_t size)//Loads data into the NAND cache.
{
    uint8_t cmd[3];

    // ⭐ ALWAYS use 0x02 (simplify for LittleFS)
    cmd[0] = 0x02;

    cmd[1] = (column >> 8) & 0xFF;//Upper column address.
    cmd[2] = column & 0xFF;//lower column address.

    NAND_Select();

    HAL_SPI_Transmit(&hspi1, cmd, 3, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, data, size, HAL_MAX_DELAY);
    HAL_Delay(1);

    NAND_Deselect();
}

void NAND_ProgramExecute(uint32_t page)//Writes the cache contents into the specified flash page.
{
    uint8_t cmd[4];

    // ⭐ CRITICAL FIX
    NAND_WriteEnable();

    uint8_t status = NAND_ReadStatus();
   // printf("STATUS BEFORE EXEC=0x%02X\r\n", status);

    if(!(status & 0x02))
    {
       // printf("EXEC WEL FAIL\r\n");
        return;
    }

    cmd[0] = 0x10;
    cmd[1] = (page >> 16) & 0xFF;
    cmd[2] = (page >> 8) & 0xFF;
    cmd[3] = page & 0xFF;

    NAND_Select();
    HAL_SPI_Transmit(&hspi1, cmd, 4, HAL_MAX_DELAY);
    NAND_Deselect();

    NAND_WaitBusy();

    status = NAND_ReadStatus();
   // printf("EXEC STATUS=0x%02X\r\n", status);

    if(status & 0x08)
    {
        //printf("PROGRAM FAILED\r\n");
    }
}
int NAND_Read(uint32_t page,
              uint16_t column,
              uint8_t *buffer,
              uint32_t length)//reads any number of bytes, even if the data spans multiple pages
{
    while(length)//Continue until all requested bytes have been read.
    {
        uint32_t bytes = 2048 - column;//Calculates the remaining bytes available in the current page.

        if(bytes > length)//If the remaining request is smaller than the remaining page space, only read the requested amount.
            bytes = length;

        NAND_PageRead(page);   //Loads the page into the NAND's internal cache.

        uint8_t status = NAND_ReadStatus();

        if(status & 0x30)//Checks ECC status bits (bits 4 and 5). If these bits indicate an uncorrectable error, the function returns -1.
        {
            //printf("READ ECC ERROR STATUS=0x%02X\r\n", status);
            return -1;
        }

        NAND_ReadFromCache(column, buffer, bytes);//Copies the required bytes from the NAND cache into the STM32 RAM buffer.

        buffer += bytes;//Moves the buffer pointer forward by the number of bytes just read.
        length -= bytes;//Reduces the remaining number of bytes to read.

        page++;
        column = 0;//The next page read always starts at the beginning (column 0).
    }

    return 0;
}
int NAND_Write(uint32_t page,
               uint16_t column,
               const uint8_t *buffer,
               uint32_t length)
{
    uint32_t start_time = HAL_GetTick();

    while(length)
    {
        // ⭐ TIMEOUT PROTECTION
        if(HAL_GetTick() - start_time > 2000)
        {
            //printf("⚠ NAND WRITE TIMEOUT\r\n");
            return -1;
        }

        uint32_t chunk = length;//Assume the entire remaining data can be written.

        if( chunk > (2048 - column))//If it doesn't fit in the current page, limit the write to the remaining space in that page.
            chunk = 2048 - column;

        // STEP 1: Enable write
        NAND_WriteEnable();

        uint8_t status = NAND_ReadStatus();

        if(!(status & 0x02))//If the WEL bit is not set, writing is not permitted, so return -1.
        {
            //printf("WRITE WEL FAIL\r\n");
            return -1;
        }

        NAND_ProgramLoad(column, (uint8_t*)buffer, chunk);//Copies chunk bytes from the STM32 RAM buffer into the NAND's internal cache, starting at the specified column.

        NAND_ProgramExecute(page);//Programs the cached data into the selected flash page.

        status = NAND_ReadStatus();//Checks the result of the programming operation.

        if(status & 0x08)
        {
           // printf("WRITE FAIL\r\n");
            return -1;
        }

        buffer += chunk;//Move the RAM buffer pointer to the next unwritten data.
        length -= chunk;//Reduce the remaining number of bytes to write.

        page++;
        column = 0;
    }

    return 0;
}
