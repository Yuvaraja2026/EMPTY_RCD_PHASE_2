#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <drivers/uart.h>
#include <drivers/edma.h>
#include <drivers/soc.h>
#include <control/mmwave/mmwave.h>
#include <control/mmwave/include/mmwave_internal.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "hwa.h"


/* Inform the compiler that the global Master Subsystem Control Block is defined in another file */
//extern MmwDemo_MSS_MCB gMmwMssMCB;

/* Declare a static global handle for the Hardware Accelerator (HWA) driver, initialized to NULL */
static HWA_Handle gHwaHandle = NULL;

/* Declare a volatile global status variable for HWA operations, initialized to a success code */
static volatile int32_t gHwaStatus = SystemP_SUCCESS;

/* Declare a volatile global variable to store TI mmWave SDK error codes, initialized to zero */
 int32_t gMmwaveErrCode;
static MMWave_Handle gMmwaveHandle = NULL;

static void MinimalRadar_sendText(const char *text)
{
    UART_Transaction transaction;
    UART_Transaction_init(&transaction);
    transaction.buf = (void *)text;
    transaction.count = (uint32_t)strlen(text);
    (void)UART_write(gUartHandle[0], &transaction);
}

void MinimalRadar_runStage(void)
{
    /* T006: generated EDMA instance must already be opened by Drivers_open(). */
    if (gEdmaHandle[0] == NULL)
    {
         while (1) { } 
    }

        /* Trigger hardware-based hardware memory initialization (zero-initialisation/BIST) for critical system 
        memory banks (HWA Shared RAM, TPCC A & B buffers, FECSS Shared RAM, and APPSS Shared RAM blocks 0 & 1) 
        to clear residual data and prevent memory corruption before peripheral startup */
    SOC_memoryInit(SOC_RCM_MEMINIT_HWA_SHRAM_INIT | SOC_RCM_MEMINIT_TPCCA_INIT | SOC_RCM_MEMINIT_TPCCB_INIT | SOC_RCM_MEMINIT_FECSS_SHRAM_INIT | SOC_RCM_MEMINIT_APPSS_SHRAM0_INIT | SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT);
    MinimalRadar_sendText("Radar memories initialized\r\n");
    
    /* T007: open only HWA instance 0, matching the supplied DPC_Init(). */
    gHwaStatus = SystemP_SUCCESS;
    gHwaHandle = HWA_open(0, NULL, (int32_t *)&gHwaStatus); //Open instance 0 of the Hardware Accelerator (HWA) peripheral using default configurations, and store the resulting driver handle and status code 
    if (gHwaHandle == NULL) 
    {
         while (1) { }
    }
     MinimalRadar_sendText("HWA OK\r\n");
    /* Exact memory-init mask from motion_detect.c. */
    /* T009: initialize mmWave control only. */

        MMWave_InitCfg initCfg;                 /* Declare a configuration structure for the mmWave control module framework */
       int32_t gMmwaveErrCode;
        memset((void*)&initCfg, 0, sizeof(initCfg));   /* Clear the structure memory to zero to avoid unpredictable behavior from random stack data */
        initCfg.iswarmstart = false;            /* Configure the driver to perform a clean, full cold boot instead of waking from a sleep state */
        gMmwaveHandle = MMWave_init(&initCfg, &gMmwaveErrCode);  /* Execute the TI SDK initialization routine and store the system control handle and any error codes */
        if (gMmwaveHandle == NULL)
        { 
            MinimalRadar_sendText("MMWave init FAILED\r\n");
            while (1) { } 
        }
    
    MinimalRadar_sendText("MMWave Init OK\r\n");
}