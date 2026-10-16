#ifndef RADAR_MEMORY_H
#define RADAR_MEMORY_H

#include <stdint.h>

#define L3_MEM_SIZE    (0x40000U + (160U * 1024U) - (39U * 1024U))

#define MMWDEMO_OBJDET_CORE_LOCAL_MEM_SIZE   ((8U + 6U + 4U + 2U + 8U) * 1024U)

#define MMWDEMO_OBJDET_CORE_LOCAL_MEM2_SIZE     (25U * 1024U)

extern uint8_t gMmwL3[L3_MEM_SIZE];

extern uint8_t gMmwCoreLocMem[
    MMWDEMO_OBJDET_CORE_LOCAL_MEM_SIZE];

extern uint8_t gMmwCoreLocMem2[
    MMWDEMO_OBJDET_CORE_LOCAL_MEM2_SIZE];

#endif
