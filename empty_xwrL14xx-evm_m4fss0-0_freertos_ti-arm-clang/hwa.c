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
#include "mmwave_control/interrupts.h"
//#include <datapath/dpu/rangeproc/v0/rangeprochwa.h>
#include <datapath/dpu/doaproc/v0/doaproc.h>
#include "dpc/dpc.h"

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "hwa.h"
#include "radar_memory.h"


/* Inform the compiler that the global Master Subsystem Control Block is defined in another file */
extern MmwDemo_MSS_MCB gMmwMssMCB;
DPU_RangeProcHWA_Config rangeProcDpuCfg;
DPU_DoaProc_Config doaProcDpuCfg;
extern void Mmwave_populateDefaultChirpControlCfg(MMWave_CtrlCfg *ptrCtrlCfg);
extern void rangeProc_dpuInit();
extern void doaProc_dpuInit();
extern void MmwDemo_calcActiveAntennaGeometry();
extern int32_t DoaProc_configParser();
extern int32_t MmwDemo_cfgDopplerParamMapping(DPU_DoaProc_HWA_Option_Cfg *dopplerParamCfg, uint32_t mappingOption);
/* Declare a static global handle for the Hardware Accelerator (HWA) driver, initialized to NULL */
//static HWA_Handle gHwaHandle = NULL;
 HWA_Handle hwaHandle;

/* Declare a volatile global status variable for HWA operations, initialized to a success code */
static volatile int32_t gHwaStatus = SystemP_SUCCESS;

/* Declare a volatile global variable to store TI mmWave SDK error codes, initialized to zero */
static volatile int32_t gMmwaveErrCode = 0;

static volatile uint32_t gFrameCounter = 0U;

float gSocClk= 40000000;//hz reference from mmw_cli.c
/* Reference-backed fixed RF configuration.
 * These field names and conversions come from mmw_cli.c in the supplied demo.
 */

    /**
 *  @b Description
 *  @n
 *      The function resets HWA DMA source channel pool
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  @retval
 *      none
 */
void DPC_ObjDet_HwaDmaTrigSrcChanPoolReset(HwaDmaTrigChanPoolObj *pool)
{
    pool->dmaTrigSrcNextChan = 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function for reseting memory pool.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      none.
 */
void DPC_ObjDet_MemPoolReset(MemPoolObj *pool)
{
    pool->currAddr = (uintptr_t)pool->cfg.addr;
    pool->maxCurrAddr = pool->currAddr;
}

	static const MmwDemo_antennaGeometryCfg gMinimalAntGeometry =
{
    .ant =
    {
        {0, 0},
        {1, 1},
        {0, 2},
        {0, 1},
        {1, 2},
        {0, 3}
    }
};

static void MinimalRadar_loadRfDefaults(void)
{
    memcpy(&gMmwMssMCB.antennaGeometryCfg,&gMinimalAntGeometry,sizeof(MmwDemo_antennaGeometryCfg));
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
    hwaHandle = HWA_open(0, NULL, (int32_t *)&gHwaStatus); //Open instance 0 of the Hardware Accelerator (HWA) peripheral using default configurations, and store the resulting driver handle and status code 
    if (hwaHandle == NULL) 
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
    Mmwave_populateDefaultChirpControlCfg(&gMmwMssMCB.mmwCtrlCfg);
    /* Stop after profile creation and inspect frameCfg[0].profileHandle[0]. */
    MinimalRadar_sendText("populated default chirp control cfg.\r\n");
    if (MMWave_config(gMmwMssMCB.ctrlHandle, &gMmwMssMCB.mmwCtrlCfg, (int32_t *)&gMmwaveErrCode) < 0) { while (1) { } }
    MinimalRadar_sendText("RF configuration accepted\r\n");
        /* Register the reference frame-start interrupt;*/
    if (mmwDemo_registerFrameStartInterrupt() != 0) { while (1) { } }
    /* Stop before sensor start. */
    MinimalRadar_sendText("Frame start interrupt registered.\r\n");

    /* DPU init/config implementations are preserved in ti_reference_extract.c. */
    rangeProc_dpuInit();
    if (gMmwMssMCB.rangeProcDpuHandle == NULL)
    {
        while (1) {}
    }
    MinimalRadar_sendText("Range DPU_init ok\r\n");
    doaProc_dpuInit();
        if (gMmwMssMCB.doaProcDpuHandle == NULL)
    {
        while (1) {}
    }
    
    /* Stop: verify gMmwMssMCB.doaProcDpuHandle != NULL. */
    MinimalRadar_sendText("doaProc_dpuInit ok\r\n");

    /* RangeProc_configParser is reference-backed; preserve its exact negative return value during debug. */
    /* Initialize the backing memory pools. */
    gMmwMssMCB.L3RamObj.cfg.addr =
        (void *)&gMmwL3[0];

    gMmwMssMCB.L3RamObj.cfg.size =
        sizeof(gMmwL3);

    gMmwMssMCB.CoreLocalRamObj.cfg.addr =
        (void *)&gMmwCoreLocMem[0];

    gMmwMssMCB.CoreLocalRamObj.cfg.size =
        sizeof(gMmwCoreLocMem);


    /* Initialize current allocation pointers. */
    DPC_ObjDet_MemPoolReset(
        &gMmwMssMCB.L3RamObj);

    DPC_ObjDet_MemPoolReset(
        &gMmwMssMCB.CoreLocalRamObj);


    /* RangeProc parser also allocates HWA DMA trigger channels. */
    DPC_ObjDet_HwaDmaTrigSrcChanPoolReset(
        &gMmwMssMCB.HwaDmaChanPoolObj);

    if (RangeProc_configParser() < 0) { while (1) { } }
    MinimalRadar_sendText("RangeProc_configParser ok\r\n");
    if (DPU_RangeProcHWA_config(gMmwMssMCB.rangeProcDpuHandle, &rangeProcDpuCfg) < 0) { while (1) { } }
    MinimalRadar_sendText("DPU_RangeProcHWA_config ok\r\n");
     
    if (DPU_RangeProcHWA_control(gMmwMssMCB.rangeProcDpuHandle, DPU_RangeProcHWA_Cmd_triggerProc, NULL, 0U) < 0) { while (1) { } }
    MinimalRadar_sendText("DPU_RangeProcHWA_control ok\r\n");

    MmwDemo_calcActiveAntennaGeometry();
    MinimalRadar_sendText("ActiveAntennaGeometry ok\r\n");

    /* The mapping helper is copied exactly from dpc.c; inspect every srcBcnt before DoA config. */
    if (MmwDemo_cfgDopplerParamMapping(&doaProcDpuCfg.hwRes.hwaCfg.doaRngGateCfg, DOPPLER_OUTPUT_MAPPING_DOP_ROW_COL) < 0) { while (1) { } }
    MinimalRadar_sendText("MmwDemo_cfgDopplerParamMapping ok\r\n");
    
    if (MmwDemo_startSensor() < 0)
    {
        while (1) {}
    }

    MinimalRadar_sendText("Sensor Started\r\n");
    
        DPU_RangeProcHWA_OutParams rangeOut;
        memset((void*)&rangeOut, 0, sizeof(rangeOut));
        MinimalRadar_sendText("Before process\r\n");

    while(1)
    {
       int32_t retVal = DPU_RangeProcHWA_process(gMmwMssMCB.rangeProcDpuHandle,&rangeOut);

       // MinimalRadar_sendText("After process A\r\n");

        if (retVal < 0)
        {
            while (1)
            {
            }
        }
        gFrameCounter++;

        if (DPU_RangeProcHWA_control(gMmwMssMCB.rangeProcDpuHandle, DPU_RangeProcHWA_Cmd_triggerProc, NULL, 0U) < 0) { while (1) { } }
    }
}