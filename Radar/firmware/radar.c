#include "radar_memory.h"

/*
 * Shared radar-processing memory.
 * TI reference places this in HWASS shared memory.
 */
uint8_t gMmwL3[L3_MEM_SIZE]
    __attribute__((section(".l3")));

/*
 * Core-local working memory used by Range/DoA/etc.
 */
uint8_t gMmwCoreLocMem[
    MMWDEMO_OBJDET_CORE_LOCAL_MEM_SIZE];

/*
 * Tracker memory.
 * Not required by RangeProc yet, but this matches
 * the TI reference memory layout.
 */
uint8_t gMmwCoreLocMem2[
    MMWDEMO_OBJDET_CORE_LOCAL_MEM2_SIZE];

 