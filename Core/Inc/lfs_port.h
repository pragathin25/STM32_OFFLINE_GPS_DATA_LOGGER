#ifndef LFS_PORT_H
#define LFS_PORT_H

#include "lfs.h"

extern lfs_t lfs;

int LittleFS_Init(void);
int LittleFS_Format(void);
void LittleFS_DeInit(void);

#endif
