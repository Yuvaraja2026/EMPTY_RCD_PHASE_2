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
#include "dpc/dpc.h"

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "hwa.h"
#include "radar_memory.h"
#include "tracker_utils.h"

#define MMW_DEMO_MINOR_MODE 0
#define MMW_DEMO_MAJOR_MODE 1
#define MAX_NUM_DETECTIONS 3
/* Inform the compiler that the global Master Subsystem Control Block is defined in another file */
extern MmwDemo_MSS_MCB gMmwMssMCB;
/** Declare RangeProc HWA DPU configuration */
DPU_RangeProcHWA_Config rangeProcDpuCfg;

/** Declare DoA DPU configuration */
DPU_DoaProc_Config doaProcDpuCfg;

/** Declare CFAR HWA DPU configuration */
DPU_CFARProcHWA_Config cfarProcDpuCfg;

/** Declare CFAR HWA DPU output parameters */
DPU_CFARProcHWA_OutParams outParmsCfar;

/** Declare function to populate default chirp control configuration */
extern void Mmwave_populateDefaultChirpControlCfg(MMWave_CtrlCfg *ptrCtrlCfg);

/** Declare function to initialize the RangeProc DPU */
extern void rangeProc_dpuInit();

/** Declare function to initialize the DoA DPU */
extern void doaProc_dpuInit();

/** Declare function to calculate active antenna geometry */
extern void MmwDemo_calcActiveAntennaGeometry();

/** Declare function to parse and configure DoA processing parameters */
extern int32_t DoaProc_configParser();

/** Declare function to configure Doppler parameter mapping */
extern int32_t MmwDemo_cfgDopplerParamMapping(DPU_DoaProc_HWA_Option_Cfg *dopplerParamCfg,uint32_t mappingOption);

extern void trackerProc_dpuInit();

extern void gtrack_free(void *pFree, unsigned int sizeInBytes);

extern void *gtrack_alloc(unsigned int numElements, unsigned int sizeInBytes);

/* Declare a static global handle for the Hardware Accelerator (HWA) driver, initialized to NULL */
 HWA_Handle hwaHandle;

/* Declare a volatile global status variable for HWA operations, initialized to a success code */
static volatile int32_t gHwaStatus = SystemP_SUCCESS;

/* Declare a volatile global variable to store TI mmWave SDK error codes, initialized to zero */
static volatile int32_t gMmwaveErrCode = 0;

static volatile uint32_t gFrameCounter = 0U;

//hz reference from mmw_cli.c
float gSocClk= 40000000;
uint32_t numDetectedPoints[2];
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
/** Reset the next available HWA DMA trigger source channel */
void DPC_ObjDet_HwaDmaTrigSrcChanPoolReset(HwaDmaTrigChanPoolObj *pool)
{
    /** Reset the DMA trigger source channel index */
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
/** Reset the memory pool current address to the configured start address */
void DPC_ObjDet_MemPoolReset(MemPoolObj *pool)
{
    /** Set the current allocation address to the pool start address */
    pool->currAddr = (uintptr_t)pool->cfg.addr;

    /** Initialize the maximum current address to the current address */
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
    /** Copy the antenna geometry configuration into the global configuration */
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

    gMmwMssMCB.numDopplerBins = 2U;

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

    gMmwMssMCB.sigProcChainCfg.azimuthFftSize = 64U;
    gMmwMssMCB.sigProcChainCfg.elevationFftSize = 8U;
    gMmwMssMCB.sigProcChainCfg.motDetMode = 2U;
    gMmwMssMCB.sigProcChainCfg.coherentDoppler = 0U;
    gMmwMssMCB.sigProcChainCfg.numFrmPerMinorMotProc = 4U;
    gMmwMssMCB.sigProcChainCfg.numMinorMotionChirpsPerFrame = 4U;
    gMmwMssMCB.sigProcChainCfg.forceMinorMotionVelocityToZero = 0U;
    gMmwMssMCB.sigProcChainCfg.minorMotionVelocityInclusionThr = 0.5f;
    gMmwMssMCB.sigProcChainCfg.dopElevDimReductOrder = 0U;

    
    /* Tracker Configuration
    *
    * trackingCfg 1 2 60 3 80 50 250
    */
    gMmwMssMCB.trackerCfg.staticCfg.trackerEnabled = 1U;

    /* Initial tracker parameter set */
    gMmwMssMCB.trackerCfg.staticCfg.trackingParamSet = 3U;

    /* GTRACK module configuration */
    gMmwMssMCB.trackerCfg.staticCfg.gtrackModuleConfig.maxNumPoints = 60U;
    gMmwMssMCB.trackerCfg.staticCfg.gtrackModuleConfig.maxNumTracks = 3U;

    /* 80: Maximum radial velocity
    * trackingCfg value is represented in 0.1 m/s
    * 80 -> 8.0 m/s
    */
    gMmwMssMCB.trackerCfg.staticCfg.gtrackModuleConfig.maxVelocitymps = 8.0f;

    /* 50: Radial velocity resolution
    * trackingCfg value is represented in 0.001 m/s
    * 50 -> 0.050 m/s
    */
    gMmwMssMCB.trackerCfg.staticCfg.gtrackModuleConfig.radialVelocityResolutionmps = 0.050f;
    gMmwMssMCB.trackerCfg.staticCfg.gtrackModuleConfig.stateVectorType =GTRACK_STATE_VECTORS_2DA;
    /* 250 ms frame period */
    gMmwMssMCB.trackerCfg.staticCfg.gtrackModuleConfig.deltaT =((float)gMmwMssMCB.frameCfg.w_FramePeriodicity) /gSocClk;

    
    gMmwMssMCB.enableMajorMotion = 1U;
    gMmwMssMCB.enableMinorMotion = 0U;

    memset(&gMmwMssMCB.sensorStart, 0, sizeof(gMmwMssMCB.sensorStart));
}

/** Define the UART text sending function */
static void MinimalRadar_sendText(const char *text)
{
    UART_Transaction transaction;            /** Declare a UART transaction object */


    UART_Transaction_init(&transaction);      /** Initialize the UART transaction */

    transaction.buf = (void *)text;             /** Set the text buffer for transmission */

    transaction.count = (uint32_t)strlen(text);    /** Set the number of characters to transmit */

    (void)UART_write(gUartHandle[0], &transaction);    /** Send the text through UART */

}

void MinimalRadar_runStage(void)
{
    int32_t retVal;
    /** Declare RangeProc output parameters */
    DPU_RangeProcHWA_OutParams rangeOut;
    /** Declare DoA processing output parameters */
    DPU_DoaProc_OutParams doaOut;

    DPU_CFARProcHWA_OutParams cfarOut;

    DPC_ObjectDetection_ExecuteResult *result = &gMmwMssMCB.dpcResult;
    
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

    /** Load RF default configuration parameters */
    MinimalRadar_loadRfDefaults();
    /* Stop here: inspect channelCfg and antenna counts in CCS. */
    MinimalRadar_sendText("load RF_config done.\r\n");

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

    /* T019-T021 use the exact reference helper; see ti_reference_extract.c. */
    Mmwave_populateDefaultChirpControlCfg(&gMmwMssMCB.mmwCtrlCfg);
    /* Stop after profile creation and inspect frameCfg[0].profileHandle[0]. */
    MinimalRadar_sendText("populated default chirp control cfg.\r\n");

    /*Configure MMWave control parameters and halt on configuration failure */
    if (MMWave_config(gMmwMssMCB.ctrlHandle, &gMmwMssMCB.mmwCtrlCfg, (int32_t *)&gMmwaveErrCode) < 0) { while (1) { } }
    MinimalRadar_sendText("RF configuration accepted\r\n");

        /* Register the reference frame-start interrupt;*/
    if (mmwDemo_registerFrameStartInterrupt() != 0) { while (1) { } }
    MinimalRadar_sendText("Frame start interrupt registered.\r\n");
    
    /* DPU init/config implementations are preserved in ti_reference_extract.c. */
    rangeProc_dpuInit();
    if (gMmwMssMCB.rangeProcDpuHandle == NULL)
    {
        while (1) {}
    }
    MinimalRadar_sendText("Range DPU_init ok\r\n");

    /******************************************************
    * 1. INITIALIZE MEMORY POOLS
    ******************************************************/

    /* Initialize the backing memory pools. */
    gMmwMssMCB.L3RamObj.cfg.addr = (void *)&gMmwL3[0];

    gMmwMssMCB.L3RamObj.cfg.size = sizeof(gMmwL3);

    gMmwMssMCB.CoreLocalRamObj.cfg.addr = (void *)&gMmwCoreLocMem[0];

    gMmwMssMCB.CoreLocalRamObj.cfg.size = sizeof(gMmwCoreLocMem);
    /*
     * IMPORTANT:
     * Reset BEFORE any DPC_ObjDet_MemPoolAlloc().
     */
    /* Initialize current allocation pointers. */
    DPC_ObjDet_MemPoolReset(&gMmwMssMCB.L3RamObj);

    /** Reset the Core Local RAM memory pool */
    DPC_ObjDet_MemPoolReset(&gMmwMssMCB.CoreLocalRamObj);

    /* RangeProc parser also allocates HWA DMA trigger channels. */
    DPC_ObjDet_HwaDmaTrigSrcChanPoolReset(&gMmwMssMCB.HwaDmaChanPoolObj);
    /******************************************************
     * 2. CONSTRUCT TRACKER HEAP
     ******************************************************/
    HeapP_construct(&gMmwMssMCB.CoreLocalTrackerHeapObj,(void *)gMmwCoreLocMem2,MMWDEMO_OBJDET_CORE_LOCAL_MEM2_SIZE);

    /** Parse and configure RangeProc parameters */
    if (RangeProc_configParser() < 0) { while (1) { } }
    MinimalRadar_sendText("RangeProc_configParser ok\r\n");

    /** Configure the RangeProc HWA DPU */
    if (DPU_RangeProcHWA_config(gMmwMssMCB.rangeProcDpuHandle, &rangeProcDpuCfg) < 0) { while (1) { } }
    MinimalRadar_sendText("DPU_RangeProcHWA_config ok\r\n");

    /** Calculate the active antenna geometry */
    MmwDemo_calcActiveAntennaGeometry();
    MinimalRadar_sendText("ActiveAntennaGeometry ok\r\n");
    
    /** Initialize the DoA processing DPU */
    doaProc_dpuInit();
    /** Check whether the DoA DPU handle was successfully created */
    if (gMmwMssMCB.doaProcDpuHandle == NULL)
    {
        while (1) {}
    }
    MinimalRadar_sendText("Doa init ok\r\n");

    /** Parse and configure DoA processing parameters */
    if (DoaProc_configParser() < 0) { while (1) { } }
    MinimalRadar_sendText("DoaProc_configParser ok\r\n");

    /******************************************************
     * 4. DoA CONFIGURATION
     ******************************************************/
    /** Configure the DoA processing DPU */
    if (DPU_DoaProc_config(gMmwMssMCB.doaProcDpuHandle, &doaProcDpuCfg) < 0) { while (1) { } }
    MinimalRadar_sendText("DPU_DoaProc_config ok\r\n");

    /******************************************************
     * 5. CFAR INITIALIZATION / CONFIG PARSER
     ******************************************************/
    /** Initialize the CFAR processing DPU */
    cfarProc_dpuInit();
    if (gMmwMssMCB.cfarProcDpuHandle == NULL)
    {
        while(1) {}
    }
    MinimalRadar_sendText("cfarProc_dpuInit ok\r\n");

    /** Parse and configure CFAR processing parameters */
    if (CfarProc_configParser() < 0) { while (1) { } }
    MinimalRadar_sendText("CfarProc_configParser ok\r\n");
    /******************************************************
     * 6. ALLOCATE CFAR CARTESIAN OUTPUT
     *
     * THIS BUFFER GOES TO TRACKER.
     ******************************************************/
    gMmwMssMCB.cfarDetObjOut =
        (DPIF_PointCloudCartesianExt *)
        DPC_ObjDet_MemPoolAlloc(
            &gMmwMssMCB.L3RamObj,
            MAX_NUM_DETECTIONS *
                sizeof(DPIF_PointCloudCartesianExt),
            sizeof(uint32_t));

    if (gMmwMssMCB.cfarDetObjOut == NULL)
    {
        while (1) {}
    }
    /******************************************************
     * 7. ALLOCATE CFAR INDEX OUTPUT
     ******************************************************/
        gMmwMssMCB.dpcObjIndOut =
        (DPIF_PointCloudRngAzimElevDopInd *)
        DPC_ObjDet_MemPoolAlloc(
            &gMmwMssMCB.L3RamObj,
            MAX_NUM_DETECTIONS *
                sizeof(DPIF_PointCloudRngAzimElevDopInd),
            sizeof(uint32_t));

    if (gMmwMssMCB.dpcObjIndOut == NULL)
    {
        while (1) {}
    }
    /******************************************************
     * 8. CFAR CONFIG API
     ******************************************************/
    /** Configure the CFAR processing HWA DPU */
    if (DPU_CFARProcHWA_config(gMmwMssMCB.cfarProcDpuHandle, &cfarProcDpuCfg)< 0){while(1) {}}   
    MinimalRadar_sendText("DPU_CFARProcHWA_config ok\r\n");

    trackerProc_dpuInit();
    if (gMmwMssMCB.trackerProcDpuHandle == NULL)
    {
        while(1){}
    }
    MinimalRadar_sendText("trackerProc_dpuInit ok\r\n");

    if(DPU_TrackerProc_config(gMmwMssMCB.trackerProcDpuHandle, &gMmwMssMCB.trackerCfg)< 0) { while (1) { } }
    MinimalRadar_sendText("mmwDemo_trackerConfig ok\r\n");
    
    /** Trigger RangeProc HWA processing */
    if (DPU_RangeProcHWA_control(gMmwMssMCB.rangeProcDpuHandle, DPU_RangeProcHWA_Cmd_triggerProc, NULL, 0U) < 0) { while (1) { } }
    MinimalRadar_sendText("DPU_RangeProcHWA_control and trigger ok\r\n");

    /** Start the radar sensor */
    if (MmwDemo_startSensor() < 0)
    {
        while (1) {}
    }

    MinimalRadar_sendText("Sensor Started\r\n");
    /** Initialize RangeProc output parameters to zero */
    memset((void*)&rangeOut, 0, sizeof(rangeOut));
    MinimalRadar_sendText("Before process\r\n");

while (1)
{
    uint16_t numDetectedPoints;
    /******************************************************
     * A. RANGE PROCESSING
     ******************************************************/
    memset(&rangeOut, 0, sizeof(rangeOut));

    retVal = DPU_RangeProcHWA_process(
        gMmwMssMCB.rangeProcDpuHandle,
        &rangeOut);

    if (retVal < 0)
    {
        while (1) {}
    }
    /******************************************************
     * B. DoA / DOPPLER PROCESSING
     ******************************************************/
    memset(&doaOut, 0, sizeof(doaOut));

    retVal = DPU_DoaProc_process(
        gMmwMssMCB.doaProcDpuHandle,
        &gMmwMssMCB.radarCubeSrc[MMW_DEMO_MAJOR_MODE],
        &gMmwMssMCB.detMatrix[MMW_DEMO_MAJOR_MODE],
        &doaOut);

    if (retVal < 0)
    {
        while (1) {}
    }
    /******************************************************
     * C. PREPARE CFAR OUTPUT POINTERS
     ******************************************************/
    memset(&cfarOut, 0, sizeof(cfarOut));
    /*
     * THIS is the Cartesian point cloud that Tracker needs.
     */
    cfarOut.detObjOut = gMmwMssMCB.cfarDetObjOut;
    /*
     * CFAR detection index information.
     */
    cfarOut.detObjIndOut = gMmwMssMCB.dpcObjIndOut;
    /*
     * Do NOT use maxNumTracks here.
     *
     * CFAR points != tracked objects.
     */
    cfarOut.detObjOutMaxSize = MAX_NUM_DETECTIONS;
    cfarOut.rangeProfile = gMmwMssMCB.rangeProfile[MMW_DEMO_MAJOR_MODE];
    /******************************************************
     * D. CFAR PROCESS
     ******************************************************/
    retVal = DPU_CFARProcHWA_process(
        gMmwMssMCB.cfarProcDpuHandle,
        &gMmwMssMCB.detMatrix[MMW_DEMO_MAJOR_MODE],
        &gMmwMssMCB.cfarRunTimeInputParams[
            MMW_DEMO_MAJOR_MODE],
        &cfarOut);

    if (retVal < 0)
    {
        while (1) {}
    }
    /******************************************************
     * E. NUMBER OF VALID CFAR POINTS
     ******************************************************/
    numDetectedPoints = (uint16_t)cfarOut.numCfarDetectedPoints;
    /******************************************************
     * F. TRACKER PROCESS
     ******************************************************/
    retVal = DPU_TrackerProc_process(
        gMmwMssMCB.trackerProcDpuHandle,

        (uint32_t)numDetectedPoints,

        /*
         * Pass exactly the pointer CFAR used.
         */
        cfarOut.detObjOut,

        &result->trackerOutParams);

    if (retVal < 0)
    {
        while (1)
        {
            /*
             * inspect retVal in CCS
             */
        }
    }
    /******************************************************
     * G. CURRENT FRAME FINISHED
     ******************************************************/
    gFrameCounter++;
    /******************************************************
     * H. TRIGGER RANGE FOR NEXT FRAME
     ******************************************************/
    retVal = DPU_RangeProcHWA_control(
        gMmwMssMCB.rangeProcDpuHandle,
        DPU_RangeProcHWA_Cmd_triggerProc,
        NULL,
        0U);

    if (retVal < 0)
    {
        while (1) {}
    }
}
}