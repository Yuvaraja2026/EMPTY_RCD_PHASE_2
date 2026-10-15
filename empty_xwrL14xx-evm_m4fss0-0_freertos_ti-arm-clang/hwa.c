#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <drivers/uart.h>
#include <drivers/edma.h>
#include <drivers/soc.h>
#include <control/mmwave/mmwave.h>
#include "motion_detect.h"
#include "mmw_flash_cal.h"
#include "factory_cal.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "hwa.h"


/* Inform the compiler that the global Master Subsystem Control Block is defined in another file */
extern MmwDemo_MSS_MCB gMmwMssMCB;

extern void Mmwave_populateDefaultChirpControlCfg(MMWave_OpenCfg *ptrCtrlCfg);
/* Declare a static global handle for the Hardware Accelerator (HWA) driver, initialized to NULL */
static HWA_Handle gHwaHandle = NULL;

/* Declare a volatile global status variable for HWA operations, initialized to a success code */
static volatile int32_t gHwaStatus = SystemP_SUCCESS;

/* Declare a volatile global variable to store TI mmWave SDK error codes, initialized to zero */
static volatile int32_t gMmwaveErrCode = 0;

float gSocClk= 40000000;//hz reference from mmw_cli.c
/* Reference-backed fixed RF configuration.
 * These field names and conversions come from mmw_cli.c in the supplied demo.
 */
static void MinimalRadar_loadRfDefaults(void)
{
    gMmwMssMCB.channelCfg.h_RxChCtrlBitMask = 7U;
    gMmwMssMCB.channelCfg.h_TxChCtrlBitMask = 3U;
    gMmwMssMCB.channelCfg.c_MiscCtrl = 0U;
    gMmwMssMCB.numRxAntennas = 3U;
    gMmwMssMCB.numTxAntennas = 2U;
    gMmwMssMCB.rxAntOrder[0] = 0U;
    gMmwMssMCB.rxAntOrder[1] = 1U;
    gMmwMssMCB.rxAntOrder[2] = 2U;

    gMmwMssMCB.profileComCfg.c_DigOutputSampRate = 20U;
    gMmwMssMCB.profileComCfg.c_DigOutputBitsSel = 0U;
    gMmwMssMCB.profileComCfg.c_DfeFirSel = 0U;
    gMmwMssMCB.profileComCfg.h_NumOfAdcSamples = 128U;
    gMmwMssMCB.profileComCfg.c_ChirpTxMimoPatSel = 4U;
    gMmwMssMCB.profileComCfg.h_ChirpRampEndTime = 300U; /* CLI value 30.0 multiplied by 10. */
    gMmwMssMCB.profileComCfg.c_ChirpRxHpfSel = 0U;
    gMmwMssMCB.profileComCfg.c_MiscSettings = 0U;
    gMmwMssMCB.profileComCfg.c_HpfFastInitDuration = 15U;
    gMmwMssMCB.adcSamplingRate = 100.0f / 20.0f;
    gMmwMssMCB.numRangeBins = 64U; /* real-only: pow2(128)/2 */
    gMmwMssMCB.isBpmEnabled = 1U;

    gMmwMssMCB.profileTimeCfg.h_ChirpIdleTime = 60U; /* 6.00 us * 10 */
    gMmwMssMCB.profileTimeCfg.h_ChirpAdcStartTime = (28U << 10);
    gMmwMssMCB.profileTimeCfg.xh_ChirpTxStartTime = 0U;
    gMmwMssMCB.chirpSlope = 90.0f;
    gMmwMssMCB.startFreq = 77.0f;
    gMmwMssMCB.profileTimeCfg.xh_ChirpRfFreqSlope = (uint32_t)((90.0f * 1048576.0f) / (4.0f * 100.0f * 100.0f));
    gMmwMssMCB.profileTimeCfg.w_ChirpRfFreqStart = (uint32_t)((77.0f * 1000.0f * 256.0f) / 400.0f);

    gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst = 8U;
    gMmwMssMCB.frameCfg.c_NumOfChirpsAccum = 0U;
    gMmwMssMCB.burstPeriod = 403.0f;
    gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame = 1U;
    /* Frame-period tick conversion uses gSocClk exactly as the reference CLI parser does. */
    gMmwMssMCB.frameCfg.w_FramePeriodicity = (uint32_t)((250.0f * gSocClk) / 1000.0f);
    gMmwMssMCB.frameCfg.h_NumOfFrames = 0U;
    gMmwMssMCB.frameCfg.w_BurstPeriodicity = (uint32_t)(10.0f * 403.0f);

    memset(&gMmwMssMCB.sensorStart, 0, sizeof(gMmwMssMCB.sensorStart));
}

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
        memset((void*)&initCfg, 0, sizeof(initCfg));   /* Clear the structure memory to zero to avoid unpredictable behavior from random stack data */
        initCfg.iswarmstart = false;            /* Configure the driver to perform a clean, full cold boot instead of waking from a sleep state */
        gMmwMssMCB.ctrlHandle = MMWave_init(&initCfg, (int32_t *)&gMmwaveErrCode);  /* Execute the TI SDK initialization routine and store the system control handle and any error codes */
        if (gMmwMssMCB.ctrlHandle == NULL)
        { 
            MinimalRadar_sendText("MMWave init FAILED\r\n");
            while (1) { } 
        }
    
    MinimalRadar_sendText("MMWave Init OK\r\n");
      MinimalRadar_loadRfDefaults();
    /* Stop here: inspect channelCfg and antenna counts in CCS. */
    MinimalRadar_sendText("Stage reached - inspect CCS variables described in the design document.\r\n");
        /* Reference requires valid RF synth trim before calibration. */
    if (SOC_rcmReadSynthTrimValid() == RF_SYNTH_TRIM_VALID)
    {
        gMmwMssMCB.factoryCalCfg.atecalibinEfuse = true;
    }
    else { while (1) { } }
    MinimalRadar_sendText("RF trim OK\r\n");
        /*mmw_flash_cal_initializatin */
    if (mmwDemo_flashInit() < 0) { while (1) { } }
    MinimalRadar_sendText("Flash init OK\r\n");
        /* Factory-calibration implementation */
    if (mmwDemo_factoryCal() < 0) { while (1) { } }
    MinimalRadar_sendText("Factory calibration OK\r\n");
        /* T018: open RF module only after mmwOpenCfg is populated from the reference control code. */
    if (MMWave_open(gMmwMssMCB.ctrlHandle, &gMmwMssMCB.mmwOpenCfg, (int32_t *)&gMmwaveErrCode) < 0) { while (1) { } }
    MinimalRadar_sendText("MMWave open OK\r\n");
   // MinimalRadar_sendText("Stage reached - inspect CCS variables described in the design document.\r\n");
    /* T019-T021 use the exact reference helper; see ti_reference_extract.c. */
    Mmwave_populateDefaultChirpControlCfg(&gMmwMssMCB.mmwOpenCfg);
    /* Stop after profile creation and inspect frameCfg[0].profileHandle[0]. */
    MinimalRadar_sendText("populated default chirp control cfg.\r\n");
    
}