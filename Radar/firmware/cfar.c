#include <stdint.h>
#include <stdbool.h>
#include <string.h>
//#include "dpc/dpc.h"
#include "motion_detect.h"
#include "mmw_res.h"

#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
/* MCU Plus Include Files. */
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include "FreeRTOS.h"
#include "task.h"
/* mmwave SDK files */
#include <control/mmwave/mmwave.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include <utils/mathutils/mathutils.h>
#include <FreeRTOS.h>
#include <task.h>
#include "hwa.h"

#define MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC (3e8)

extern MmwDemo_MSS_MCB gMmwMssMCB;
extern uint8_t DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(HwaDmaTrigChanPoolObj *pool);

/* Declare a static global handle for the Hardware Accelerator (HWA) driver, initialized to NULL */
HWA_Handle hwaHandle;
DPU_CFARProcHWA_Config cfarProcDpuCfg;
Edma_IntrObject intrObj_cfarProc;

/**
*  @b Description
*  @n
*    CFAR DPU Initialization
*/
void cfarProc_dpuInit()
{
    int32_t  errorCode = 0;
    DPU_CFARProcHWA_InitParams initParams;
    initParams.hwaHandle =  hwaHandle;
    /* generate the dpu handler*/
    gMmwMssMCB.cfarProcDpuHandle =  DPU_CFARProcHWA_init(&initParams, &errorCode);
    if (gMmwMssMCB.cfarProcDpuHandle == NULL)
    {
        while(1){}
    }
}

/**
*  @b Description
*  @n
*    Based on the configuration, set up the CFAR processing DPU configurations
*/
int32_t CfarProc_configParser()
{
    int32_t retVal = 0;
    float adcStart, startFreq, slope, bandwidth, centerFreq;
    DPU_CFARProcHWA_HW_Resources *pHwConfig;
    DPU_CFARProcHWA_StaticConfig  *params;

    /* CFARproc DPU */
    pHwConfig = &cfarProcDpuCfg.res;
    params = &cfarProcDpuCfg.staticCfg;

    memset((void *)&cfarProcDpuCfg, 0, sizeof(DPU_CFARProcHWA_Config));

    /* HWA configurations, not related to per test, common to all test */
    pHwConfig->hwaCfg.paramSetStartIdx = DPU_DOAPROC_MAX_NUM_HWA_PARAMSET + DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS;
    pHwConfig->hwaCfg.numParamSet = DPU_CFARPROCHWA_MAX_NUM_HWA_PARAMSET;
    pHwConfig->hwaCfg.dmaTrigSrcChan = DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(&gMmwMssMCB.HwaDmaChanPoolObj);

    /* edma configuration */
    pHwConfig->edmaHandle  = gEdmaHandle[0];

    /* Data Input EDMA */
    pHwConfig->edmaHwaIn.channel         = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_CH;
    pHwConfig->edmaHwaIn.channelShadow   = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SHADOW;
    pHwConfig->edmaHwaIn.eventQueue      = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_EVENT_QUE;
    pHwConfig->edmaHwaInSignature.channel         = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_CH;
    pHwConfig->edmaHwaInSignature.channelShadow   = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_SHADOW;
    pHwConfig->edmaHwaInSignature.eventQueue      = DPC_OBJDET_DPU_CFAR_PROC_EDMAIN_SIG_EVENT_QUE;
    pHwConfig->intrObj = &intrObj_cfarProc;

    /* Data Output EDMA */
    pHwConfig->edmaHwaOut.channel         = DPC_OBJDET_DPU_CFAR_PROC_EDMAOUT_RNG_PROFILE_CH;
    pHwConfig->edmaHwaOut.channelShadow   = DPC_OBJDET_DPU_CFAR_PROC_EDMAOUT_RNG_PROFILE_SHADOW;
    pHwConfig->edmaHwaOut.eventQueue      = DPC_OBJDET_DPU_CFAR_PROC_EDMAOUT_RNG_PROFILE_EVENT_QUE;

    /* Give M0 and M1 memory banks for detection matrix scratch. */
    pHwConfig->hwaMemInp = (uint16_t *) CSL_APP_HWA_DMA0_RAM_BANK0_BASE;
    pHwConfig->hwaMemInpSize = (CSL_APP_HWA_BANK_SIZE * 2) / sizeof(uint16_t);

    /* M2 bank: for CFAR detection list */
    pHwConfig->hwaMemOutDetList = (DPU_CFARProcHWA_CfarDetOutput *) CSL_APP_HWA_DMA0_RAM_BANK2_BASE;
    pHwConfig->hwaMemOutDetListSize = CSL_APP_HWA_BANK_SIZE /
                                sizeof(DPU_CFARProcHWA_CfarDetOutput);

    /* M3 bank: for maximum azimuth values per range bin  (range profile) */
    pHwConfig->hwaMemOutRangeProfile = (DPU_CFARProcHWA_HwaMaxOutput *) CSL_APP_HWA_DMA0_RAM_BANK3_BASE;

    /* dynamic config */
    cfarProcDpuCfg.dynCfg.cfarCfg   = &gMmwMssMCB.cfarCfg;
    cfarProcDpuCfg.dynCfg.fovRange  = &gMmwMssMCB.rangeSelCfg;
    cfarProcDpuCfg.dynCfg.fovAoaCfg    = &gMmwMssMCB.fovCfg;

    /* DPU Static config */
    params->detectionHeatmapType = DPU_CFAR_RANGE_AZIMUTH_HEATMAP;
    params->numRangeBins = gMmwMssMCB.numRangeBins;
    params->numDopplerBins     = gMmwMssMCB.numDopplerBins;
    params->log2NumDopplerBins = mathUtils_ceilLog2(params->numDopplerBins);

    params->selectCoherentPeakInDopplerDim = gMmwMssMCB.sigProcChainCfg.coherentDoppler;
    params->angleDimension        = gMmwMssMCB.angleDimension;
    params->isDetMatrixLogScale   = false;
    params->azimuthFftSize        = gMmwMssMCB.sigProcChainCfg.azimuthFftSize;
    params->elevationFftSize      = gMmwMssMCB.sigProcChainCfg.elevationFftSize;
    params->isStaticClutterRemovalEnabled = gMmwMssMCB.staticClutterRemovalEnable;
    params->dopElevDimReductOrder = gMmwMssMCB.sigProcChainCfg.dopElevDimReductOrder;

    if (params->isDetMatrixLogScale)
    {
        gMmwMssMCB.cfarCfg.thresholdScale    = (uint32_t) lroundf(gMmwMssMCB.cfarCfg.threshold_dB * 2048.0 / (log10(2)*20));
    }
    else
    {
        gMmwMssMCB.cfarCfg.thresholdScale = (uint32_t) lroundf(pow(10, gMmwMssMCB.cfarCfg.threshold_dB/20.0) * 16.0);
    }
    
    adcStart                        =   (gMmwMssMCB.adcStartTime * 1.e-6);
    startFreq                       =   (float)(gMmwMssMCB.startFreq * 1.e9);
    slope                           =   (float)(gMmwMssMCB.chirpSlope * 1.e12);
    bandwidth                       =   (slope * gMmwMssMCB.profileComCfg.h_NumOfAdcSamples)/(gMmwMssMCB.adcSamplingRate * 1.e6);
    centerFreq                      =   startFreq + bandwidth * 0.5f + adcStart * slope;

    params->rangeStep            =   (MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC * (gMmwMssMCB.adcSamplingRate * 1.e6)) /
                                        (2.f * slope * (2*params->numRangeBins));
    /*outParams->rangeResolution      =   (MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC * gMmwMssMCB.adcSamplingRate * 1e6) /
                                        (2.f * slope * gMmwMssMCB.profileComCfg.h_NumOfAdcSamples);*/

    gMmwMssMCB.rangeStep           = params->rangeStep;
    
    if (gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame > 1)
    {
        /* Burst mode: h_NumOfBurstsInFrame > 1, h_NumOfChirpsInBurst = 2 */
        params->dopplerStep          =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                            (2.f * params->numDopplerBins *
                                            centerFreq * (gMmwMssMCB.burstPeriod * 1e-6));
    }
    else
    {
        /* Normal mode: h_NumOfBurstsInFrame = 1, h_NumOfChirpsInBurst >= 2 */
        params->dopplerStep          =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                            (2.f * gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst *
                                            centerFreq * ((gMmwMssMCB.profileTimeCfg.h_ChirpIdleTime + gMmwMssMCB.profileComCfg.h_ChirpRampEndTime) * 1e-1 * 1e-6));                         
        if(gMmwMssMCB.frameCfg.c_NumOfChirpsAccum != 0)
        {
            /* When numOfChirpsAccum is greater than zero, the chirping window will increase acccording to numOfChirpsAccum selected. */ 
            params->dopplerStep = params->dopplerStep/gMmwMssMCB.frameCfg.c_NumOfChirpsAccum;
        }
    }
    /*outParams->dopplerResolution    =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                        (2.f * gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame * centerFreq * (gMmwMssMCB.frameCfg.w_BurstPeriodicity));*/

    if (gMmwMssMCB.antennaGeometryCfg.antDistanceXdim == 0.)
    {
        params->lambdaOverDistX = 2.0;
    }
    else
    {
        params->lambdaOverDistX = 3e8 / (centerFreq * gMmwMssMCB.antennaGeometryCfg.antDistanceXdim);
    }

    if (gMmwMssMCB.antennaGeometryCfg.antDistanceZdim == 0.)
    {
        params->lambdaOverDistZ = 2.0;
    }
    else
    {
        params->lambdaOverDistZ = 3e8 / (centerFreq * gMmwMssMCB.antennaGeometryCfg.antDistanceZdim);
    }

    /* Range bias (m) */
    params->rangeBias = gMmwMssMCB.compRxChannelBiasCfg.rangeBias;

    #if (CLI_REMOVAL == 0)
    if(gMmwMssMCB.adcDataSourceCfg.source == 1)
    {
        //ADC data from file, populate point cloud list with target indices (range/azimuth/elevation/doppler)
        params->enableCfarPointCloudListWithIndices = true;
    }
    else
    {
        params->enableCfarPointCloudListWithIndices = false;
    }
    #else
    params->enableCfarPointCloudListWithIndices = false;
    #endif

    params->enableCfarPointCloudListWithIndices = true;

    /* hwres config - Copy these structures */
    pHwConfig->dopplerIndexMatrix = gMmwMssMCB.dopplerIndexMatrix;
    pHwConfig->elevationIndexMatrix = gMmwMssMCB.elevationIndexMatrix;

    /* For point cloud compression over UART: set the cloud point units and reciprocal values */
    gMmwMssMCB.pointCloudToUart.pointUint.xyzUnit = (params->rangeStep * params->numRangeBins) / 32768.0;
    gMmwMssMCB.pointCloudToUart.pointUint.dopplerUnit = (params->dopplerStep * params->numDopplerBins/2) / 32768.0;
    gMmwMssMCB.pointCloudToUart.pointUint.snrUint = 0.25;
    gMmwMssMCB.pointCloudToUart.pointUint.noiseUint = 1.0;
    gMmwMssMCB.pointCloudUintRecip.xyzUnit = 1. / gMmwMssMCB.pointCloudToUart.pointUint.xyzUnit;
    gMmwMssMCB.pointCloudUintRecip.dopplerUnit = 1. / gMmwMssMCB.pointCloudToUart.pointUint.dopplerUnit;
    gMmwMssMCB.pointCloudUintRecip.snrUint = 1. / gMmwMssMCB.pointCloudToUart.pointUint.snrUint;  //scale 0.1 since in the CFAR DPU structure it is in 0.1 dB
    gMmwMssMCB.pointCloudUintRecip.noiseUint = 1. / gMmwMssMCB.pointCloudToUart.pointUint.noiseUint;  //scale 0.1 since in the CFAR DPU structure it is in 0.1 dB


    if(gMmwMssMCB.enableMajorMotion)
    {
        /* In major motion CFAR post processing, the velocity inclusion threshold is set to maximum, so all points are includeed and their detected velocity is not modified */
        gMmwMssMCB.cfarRunTimeInputParams[0].forceVelocityToZero = false;
        gMmwMssMCB.cfarRunTimeInputParams[0].velocityInclusionThr = params->dopplerStep * params->numDopplerBins/2; //Maximum velocity, include all
    }
    if(gMmwMssMCB.enableMinorMotion)
    {
        /* In minor motion CFAR post processing, the detected points with radial velocity greater than minorMotionVelocityInclusionThr are excluded from the list, 
           and then velocity  of included points is forced to zero if the flag forceMinorMotionVelocityToZero is set to true. */
        gMmwMssMCB.cfarRunTimeInputParams[1].forceVelocityToZero = gMmwMssMCB.sigProcChainCfg.forceMinorMotionVelocityToZero;
        gMmwMssMCB.cfarRunTimeInputParams[1].velocityInclusionThr = gMmwMssMCB.sigProcChainCfg.minorMotionVelocityInclusionThr;
    }

    if (rangeCompCfg.enabled == 1 && gMmwMssMCB.stats.frameStartIntCounter == 0)
    {
        // If no range/SNR values are specified, then default to the maximum range index and detection SNR
        if (rangeCompCfg.detectionRangeIdx < 0)
        {
            // max range index allowed is the number of samples / 2 * RANGE_FFT_ANTI_ALIAS_SAFETY_FACTOR
            rangeCompCfg.detectionRangeIdx = gMmwMssMCB.profileComCfg.h_NumOfAdcSamples * 0.5 * RANGE_FFT_ANTI_ALIAS_SAFETY_FACTOR;
            // inverse operation to get the detection SNR specified in the configuration file
            rangeCompCfg.detectionSNR = log10(((((float)gMmwMssMCB.cfarCfg.thresholdScale) - 0.5) / 16.0)) * 20;
        }
        // If the range/SNR are specified, then convert the range from meters to index. detection SNR is already set from mmw_cli.c
        else
        {
            rangeCompCfg.detectionRangeIdx = roundf(rangeCompCfg.detectionRangeIdx / params->rangeStep);
            rangeCompCfg.minCompRange      = roundf(rangeCompCfg.minCompRange / params->rangeStep);
            rangeCompCfg.maxCompRange      = roundf(rangeCompCfg.maxCompRange / params->rangeStep);
        }
    }

    return retVal;
}