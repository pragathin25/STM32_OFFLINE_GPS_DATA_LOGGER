#include "lfs_port.h"
#include "nand.h"

#include <string.h>
#include <stdio.h>

lfs_t lfs; // LittleFS object (open,write,mount)

/*--------------------------------------------------------------------
    Buffers
--------------------------------------------------------------------*/

static uint8_t read_buffer[2048];
static uint8_t prog_buffer[2048];
static uint8_t lookahead_buffer[128];

static struct lfs_config cfg;//LittleFS configuration structure.

/*--------------------------------------------------------------------
    LittleFS READ
--------------------------------------------------------------------*/

static int lfs_read(const struct lfs_config *c,
                    lfs_block_t block,
                    lfs_off_t off,
                    void *buffer,
                    lfs_size_t size)
{
    uint32_t page = block * 64 + (off / 2048);//Converts the filesystem block and offset(start point) into a NAND page
    uint16_t column = off % 2048;//Calculates starting byte inside that page

    uint32_t start = HAL_GetTick();

    int res = NAND_Read(page, column, (uint8_t *)buffer, size);

    if(res != 0)
    {
        //printf("❌ READ FAIL block=%lu page=%lu\r\n", block, page);
        return LFS_ERR_IO;
    }

    if(HAL_GetTick() - start > 1000)
    {
        //printf("⚠ READ SLOW\r\n");
    }

    return 0;
}
/*--------------------------------------------------------------------
    LittleFS PROGRAM
--------------------------------------------------------------------*/
static int lfs_prog(const struct lfs_config *c,
                    lfs_block_t block,
                    lfs_off_t off,
                    const void *buffer,
                    lfs_size_t size)   //wants to write data
{
    (void)c; // avoids compiler warnings

    uint32_t page = block * 64 + (off / 2048);//Converts filesystem block into NAND page
    uint16_t column = off % 2048;//Calculates starting byte inside that page

    // printf("LFS PROG block=%lu off=%lu size=%lu\r\n", block, off, size);

    int res = NAND_Write(page, column, buffer, size);

    //printf("LFS PROG RES=%d\r\n", res);

    return res;
}
/*--------------------------------------------------------------------
    LittleFS ERASE
--------------------------------------------------------------------*/

static int lfs_erase(const struct lfs_config *c,
                     lfs_block_t block) //LittleFS calls this before writing to an erased block.
{
    int res = NAND_BlockErase(block);

    if (res != 0)
    {
        //printf("ERASE FAIL BLOCK=%lu\r\n", block);
        return LFS_ERR_IO;   // ⭐ IMPORTANT
    }

    return 0;
}
/*--------------------------------------------------------------------
    LittleFS SYNC
--------------------------------------------------------------------*/

static int lfs_sync(const struct lfs_config *c)//wants all writes safely stored
{
    (void)c;

    return 0;
}

/*--------------------------------------------------------------------
    LittleFS INIT
--------------------------------------------------------------------*/

int LittleFS_Init(void)
{
    memset(&cfg,0,sizeof(cfg));//Clears the configuration structure

    cfg.read  = lfs_read;//Registers the read function.
    cfg.prog  = lfs_prog;//Registers the write function.
    cfg.erase = lfs_erase;
    cfg.sync  = lfs_sync;

    /* NAND Geometry */

    cfg.read_size = 2048;

    cfg.prog_size = 2048;

    cfg.block_size = 131072;//(bytes 64*2048)

    cfg.block_count = 1024; //NAND blocks. (1024 × 131072 = 128 mb)

    cfg.cache_size = 2048;//One NAND page

    cfg.lookahead_size = 128;//free-block management.

    cfg.block_cycles = 500;//wear leveling

    cfg.read_buffer = read_buffer;

    cfg.prog_buffer = prog_buffer;

    cfg.lookahead_buffer = lookahead_buffer;

    //printf("MOUNT START\r\n");

    uint32_t t0 = HAL_GetTick();

    int err = lfs_mount(&lfs, &cfg);

    printf("Mount time = %lu ms\r\n", HAL_GetTick() - t0);

    //printf("MOUNT RES = %d\r\n", err);

    if(err)
    {
        //printf("LittleFS not found\r\n");

        //printf("Formatting...\r\n");

        err = lfs_format(&lfs,&cfg);

        //printf("FORMAT RES = %d\r\n", err);

        if(err)
        {
            printf("Format Failed\r\n");
            return err;
        }

        //printf("FORMAT DONE\r\n");

        err = lfs_mount(&lfs,&cfg);

       // printf("REMOUNT RES = %d\r\n", err);

        if(err)
        {
            printf("Mount Failed\r\n");
            return err;
        }
    }

   // printf("LittleFS Mounted\r\n");

    return 0;
}

/*--------------------------------------------------------------------
    FORMAT
--------------------------------------------------------------------*/

int LittleFS_Format(void)
{
    lfs_unmount(&lfs);

    int err = lfs_format(&lfs,&cfg);//Erases filesystem metadata and creates a fresh filesystem.

    if(err)
        return err;

    return lfs_mount(&lfs,&cfg);
}

/*--------------------------------------------------------------------
    UNMOUNT
--------------------------------------------------------------------*/

void LittleFS_DeInit(void)//Called before shutdown or reset.
{
    lfs_unmount(&lfs);
}
