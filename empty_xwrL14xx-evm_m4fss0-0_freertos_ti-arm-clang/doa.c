#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"

#include <kernel/dpl/DebugP.h>
#include "FreeRTOS.h"
#include "task.h"
#include <drivers/uart.h>
#include "motion_detect.h"
#include "mmw_flash_cal.h"
#include <mmwavelink/include/rl_device.h>
#include "hwa.h"
#include <utils/mathutils/mathutils.h>
#include"mmw_res.h"

#define DOPPLER_OUTPUT_MAPPING_DOP_ROW_COL   0
#define DOPPLER_OUTPUT_MAPPING_ROW_DOP_COL   1
#define DPC_DPU_DPIF_DETMATRIX_FORMAT_2 2
#define HWA_MAX_NUM_DMA_TRIG_CHANNELS 16
#define MMW_DEMO_MAJOR_MODE 0
#define MMW_DEMO_MINOR_MODE 1

extern MmwDemo_MSS_MCB gMmwMssMCB; 
extern void MmwDemo_calcActiveAntennaGeometry();
extern int32_t DoaProc_configParser();
//extern void mmwDemo_doaProcConfig();

HWA_Handle hwaHandle;
DPU_DoaProc_Config doaProcDpuCfg;
DPU_DoaProc_HW_Resources  *hwRes;
Edma_IntrObject intrObj_doaProc;
/**
*  @b Description
*  @n
*    DOA DPU Initialization
*/
void doaProc_dpuInit()
{
    int32_t  errorCode = 0;
    DPU_DoaProc_InitParams initParams;
    initParams.hwaHandle =  hwaHandle;
    /* generate the dpu handler*/
    gMmwMssMCB.doaProcDpuHandle =  DPU_DoaProc_init(&initParams, &errorCode);
    if (gMmwMssMCB.doaProcDpuHandle == NULL)
    {
        DebugP_log ("Error: DoaProc DPU initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}

/**
*  @b Description
*  @n
*    Based on the activeAntennaGeometryCfg configures the table which used to configure
*    Doppler FFT HWA param sets in DoA DPU. THese param sets perform Doppler FFT and
*    at the same time mapping of input antennas into 2D row-column antenna array where columns
*    are in  azimuth dimension, and rows in elevation dimension.
*    It also calculates the size of 2D antenna array, ie. number of rows and number of columns.
*/
int32_t MmwDemo_cfgDopplerParamMapping(DPU_DoaProc_HWA_Option_Cfg *dopplerParamCfg, uint32_t mappingOption)
{
    int32_t ind, indNext, indNextPrev;
    int32_t row, col;
    int32_t dopParamInd;
    int32_t state;
    int16_t BT[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
    int16_t DT[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
    int16_t SCAL[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
    int8_t  DONE[DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS];
    int32_t retVal = 0;
    int32_t rowOffset;

    if (gMmwMssMCB.numAntRow * gMmwMssMCB.numAntCol > DPU_DOA_PROC_MAX_2D_ANT_ARRAY_ELEMENTS)
    {
        retVal = DPC_OBJECTDETECTION_EANTENNA_GEOMETRY_CFG_FAILED;
        goto exit;
    }

    if (mappingOption == DOPPLER_OUTPUT_MAPPING_DOP_ROW_COL)
    {
        /*For AOA DPU, Output is */
        rowOffset =  gMmwMssMCB.numAntCol;
    }
    else if (mappingOption == DOPPLER_OUTPUT_MAPPING_ROW_DOP_COL)
    {
        rowOffset =  gMmwMssMCB.numDopplerBins * gMmwMssMCB.numAntCol;
    }
    else
    {
        retVal = DPC_OBJECTDETECTION_EANTENNA_GEOMETRY_CFG_FAILED;
        goto exit;
    }

    /* Initialize tables */
    for (ind = 0; ind < (gMmwMssMCB.numAntRow * gMmwMssMCB.numAntCol); ind++)
    {
        BT[ind] = 0;
        SCAL[ind] = 0;
        DONE[ind] = 0;
    }

    for (ind = 0; ind < (gMmwMssMCB.numTxAntennas * gMmwMssMCB.numRxAntennas); ind++)
    {
        row = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].row;
        col = gMmwMssMCB.activeAntennaGeometryCfg.ant[ind].col;
        BT[row * gMmwMssMCB.numAntCol + col] = ind;
        SCAL[row * gMmwMssMCB.numAntCol + col] = 1;
    }
    for (row = 0; row < gMmwMssMCB.numAntRow; row++)
    {
        for (col = 0; col < gMmwMssMCB.numAntCol; col++)
        {
            ind = row * gMmwMssMCB.numAntCol + col;
            DT[ind] = row * rowOffset + col;
        }
    }


    /* Configure Doppler HWA mapping params for antenna mapping */
    dopParamInd = 0;
    dopplerParamCfg->numDopFftParams = 0;
    for (ind = 0; ind < (gMmwMssMCB.numAntRow * gMmwMssMCB.numAntCol); ind++)
    {
        if (!DONE[ind])
        {
            if(dopParamInd < DPU_DOA_PROC_MAX_NUM_DOP_FFFT_PARAMS)
            {
                DONE[ind] = 1;
                dopplerParamCfg->numDopFftParams++;
                dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt = 1;
                dopplerParamCfg->dopFftCfg[dopParamInd].scale = SCAL[ind];
                if (dopplerParamCfg->dopFftCfg[dopParamInd].scale == 0)
                {
                    dopplerParamCfg->dopFftCfg[dopParamInd].srcAddrOffset = 0;
                }
                else
                {
                    dopplerParamCfg->dopFftCfg[dopParamInd].srcAddrOffset = BT[ind];
                }
                dopplerParamCfg->dopFftCfg[dopParamInd].dstAddrOffset = DT[ind];
                state = 1;//STATE_SECOND:
                for (indNext = ind+1; indNext < (gMmwMssMCB.numAntRow * gMmwMssMCB.numAntCol); indNext++)
                {

                    if (!DONE[indNext] && (dopplerParamCfg->dopFftCfg[dopParamInd].scale == SCAL[indNext]))
                    {
                        switch (state)
                        {
                            case 1://STATE_SECOND:
                                dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt++;
                                DONE[indNext] = 1;
                                if (SCAL[indNext] == 1)
                                {
                                    dopplerParamCfg->dopFftCfg[dopParamInd].srcBidx = BT[indNext] - dopplerParamCfg->dopFftCfg[dopParamInd].srcAddrOffset;
                                }
                                else
                                {
                                    dopplerParamCfg->dopFftCfg[dopParamInd].srcBidx = 0;
                                }
                                dopplerParamCfg->dopFftCfg[dopParamInd].dstBidx = DT[indNext] - DT[ind];
                                indNextPrev = indNext;
                                state = 2;//STATE_NEXT:
                                break;
                            case 2://STATE_NEXT:
                                if (SCAL[indNext] == 1)
                                {
                                    if ((dopplerParamCfg->dopFftCfg[dopParamInd].srcBidx == (BT[indNext] - BT[indNextPrev])) &&
                                        (dopplerParamCfg->dopFftCfg[dopParamInd].dstBidx == (DT[indNext] - DT[indNextPrev])))
                                    {
                                        DONE[indNext] = 1;
                                        dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt++;
                                        indNextPrev = indNext;
                                    }
                                }
                                else
                                {
                                    if (dopplerParamCfg->dopFftCfg[dopParamInd].dstBidx == (DT[indNext] - DT[indNextPrev]))
                                    {
                                        DONE[indNext] = 1;
                                        dopplerParamCfg->dopFftCfg[dopParamInd].srcBcnt++;
                                        indNextPrev = indNext;
                                    }
                                }
                                break;
                        }
                    }
                }
                dopParamInd++;
            }
            else
            {
                retVal = DPC_OBJECTDETECTION_EANTENNA_GEOMETRY_CFG_FAILED;
                goto exit;
            }
        }
    }

    dopplerParamCfg->numDopFftParams = dopParamInd;

exit:
    return retVal;
}
/**
 *  @b Description
 *  @n
 *      Utility function for allocating from a static memory pool.
 *
 *  @param[in]  pool Handle to pool object.
 *  @param[in]  size Size in bytes to be allocated.
 *  @param[in]  align Alignment in bytes
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      pointer to beginning of allocated block. NULL indicates could not
 *      allocate.
 */
void *DPC_ObjDet_MemPoolAlloc(MemPoolObj *pool,
                              uint32_t size,
                              uint8_t align)
{
    void *retAddr = NULL;
    uintptr_t addr;

    addr = MEM_ALIGN(pool->currAddr, align);
    if ((addr + size) <= ((uintptr_t)pool->cfg.addr + pool->cfg.size))
    {
        retAddr = (void *)addr;
        pool->currAddr = addr + size;
        pool->maxCurrAddr = MAX(pool->currAddr, pool->maxCurrAddr);
    }

    return(retAddr);
}
/**
 *  @b Description
 *  @n
 *      The function allocates HWA DMA source channel from the pool
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  @retval
 *      channel Allocated HWA trigger source channel
 */
uint8_t DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(HwaDmaTrigChanPoolObj *pool)
{
    uint8_t channel = 0xFF;
    if(pool->dmaTrigSrcNextChan < HWA_MAX_NUM_DMA_TRIG_CHANNELS)
    {
        channel = pool->dmaTrigSrcNextChan;
        pool->dmaTrigSrcNextChan++;
    }
    return channel;
}
/**
*  @b Description
*  @n
*        Function configuring DOA DPU
*/
// void mmwDemo_doaProcConfig()
// {
//     //uint32_t i;
//     int32_t retVal = 0;
//     uint32_t numSamples;
//     DPU_DoaProc_HW_Resources  *hwRes;

//     hwRes = &doaProcDpuCfg.hwRes;

//     retVal = DoaProc_configParser();
//     if (retVal < 0)
//     {
//         DebugP_log("Error: Error in setting up doa profile:%d \n", retVal);
//         DebugP_assert(0);
//     }

//     /* Note: chunk[1] is not used */
//     /* Major motion - set input source and destination address */
//     numSamples = doaProcDpuCfg.staticCfg.numVirtualAntennas * doaProcDpuCfg.staticCfg.numDopplerChirps;
//     gMmwMssMCB.radarCubeSrc[MMW_DEMO_MAJOR_MODE].chunk[0].srcAddress = (uint32_t) gMmwMssMCB.radarCube[0].data;
//     gMmwMssMCB.radarCubeSrc[MMW_DEMO_MAJOR_MODE].chunk[0].dstAddress = hwRes->hwaCfg.hwaMemInpAddr;
//     gMmwMssMCB.radarCubeSrc[MMW_DEMO_MAJOR_MODE].chunk[0].Bcnt_Acnt = (numSamples << 16)   | sizeof(cmplx16ImRe_t);

//     /* Minor Motion set input source and destination address */
//     numSamples = doaProcDpuCfg.staticCfg.numVirtualAntennas * doaProcDpuCfg.staticCfg.numDopplerChirps;
//     gMmwMssMCB.radarCubeSrc[MMW_DEMO_MINOR_MODE].chunk[0].srcAddress = (uint32_t) gMmwMssMCB.radarCube[1].data;
//     gMmwMssMCB.radarCubeSrc[MMW_DEMO_MINOR_MODE].chunk[0].dstAddress = hwRes->hwaCfg.hwaMemInpAddr;
//     gMmwMssMCB.radarCubeSrc[MMW_DEMO_MINOR_MODE].chunk[0].Bcnt_Acnt = (numSamples << 16)   | sizeof(cmplx16ImRe_t);

//     retVal = DPU_DoaProc_config (gMmwMssMCB.doaProcDpuHandle, &doaProcDpuCfg);
//     if (retVal < 0)
//     {
//         DebugP_log("DOA DPU config return error:%d \n", retVal);
//         DebugP_assert(0);
//     }

// }

/**
*  @b Description
*  @n
*    Based on the configuration, set up the doa processing DPU configurations
*/
int32_t DoaProc_configParser()
{
    DPIF_DetMatrix dopplerIndexMatrix;
    DPIF_DetMatrix elevationIndexMatrix;

    /* Doaproc DPU */
    DPU_DoaProc_EdmaCfg *edmaCfg;
    DPU_DoaProc_HwaCfg *hwaCfg;
    int32_t winGenLen, i;
    int32_t retVal = 0;
    DPU_DoaProc_StaticConfig  *doaStaticCfg;

    memset((void *)&doaProcDpuCfg, 0, sizeof(DPU_DoaProc_Config));

    hwRes = &doaProcDpuCfg.hwRes;
    doaStaticCfg = &doaProcDpuCfg.staticCfg;
    edmaCfg = &hwRes->edmaCfg;
    hwaCfg = &hwRes->hwaCfg;

    /* Select active antennas from available antennas and calculate number of antennas rows and columns */
    MmwDemo_calcActiveAntennaGeometry();

    doaStaticCfg->numAntRow = gMmwMssMCB.numAntRow;
    doaStaticCfg->numAntCol = gMmwMssMCB.numAntCol;


    /* Angle dimension */
    if ((gMmwMssMCB.numAntRow > 1) && (gMmwMssMCB.numAntCol > 1))
    {
        gMmwMssMCB.angleDimension = 2;
    }
    else if ((gMmwMssMCB.numAntRow == 1) && (gMmwMssMCB.numAntCol > 1))
    {
        gMmwMssMCB.angleDimension = 1;
    }
    else
    {
        gMmwMssMCB.angleDimension = 0;
    }


    doaStaticCfg->enableMajorMotion = gMmwMssMCB.enableMajorMotion;
    doaStaticCfg->enableMinorMotion = gMmwMssMCB.enableMinorMotion;

    doaStaticCfg->numTxAntennas = (uint8_t) gMmwMssMCB.numTxAntennas;
    doaStaticCfg->numRxAntennas = (uint8_t) gMmwMssMCB.numRxAntennas;
    doaStaticCfg->numVirtualAntennas = (uint8_t) (gMmwMssMCB.numTxAntennas * gMmwMssMCB.numRxAntennas);
    doaStaticCfg->numRangeBins = gMmwMssMCB.numRangeBins;


    doaStaticCfg->numMinorMotionChirpsPerFrame = gMmwMssMCB.sigProcChainCfg.numMinorMotionChirpsPerFrame;
    doaStaticCfg->numFrmPerMinorMotProc = gMmwMssMCB.sigProcChainCfg.numFrmPerMinorMotProc;
    if (doaStaticCfg->numTxAntennas > 1)
    {
        if (doaStaticCfg->enableMajorMotion)
        {
            if ((gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame > 1) && (gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst == 2))
            {
                /* Burst mode: h_NumOfBurstsInFrame > 1, h_NumOfChirpsInBurst = 2 */
                doaStaticCfg->numDopplerChirps   = gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame;
            }
            else if (gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame == 1)
            {
                /* Normal mode: h_NumOfBurstsInFrame = 1, h_NumOfChirpsInBurst >= doaStaticCfg->numTxAntennas */
                doaStaticCfg->numDopplerChirps   = gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst / doaStaticCfg->numTxAntennas;
            }
            else
            {
                retVal = DPU_DOAPROC_EINVAL;
                goto exit;
            }
        }
        else
        {
            /* Minor motion */
            doaStaticCfg->numDopplerChirps   = gMmwMssMCB.sigProcChainCfg.numFrmPerMinorMotProc * gMmwMssMCB.sigProcChainCfg.numMinorMotionChirpsPerFrame;
        }
    }
    else
    {
        /* 1Tx antenna */
        if (gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame != 1)
        {
            retVal = DPU_DOAPROC_EINVAL;
            goto exit;
        }

        if (doaStaticCfg->enableMajorMotion)
        {
            doaStaticCfg->numDopplerChirps = gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst;
        }
        else
        {
            /* This is to support 1Tx/1chirpPerFrame/NchirpsAcrossFrames in minor mode */
            doaStaticCfg->numDopplerChirps = gMmwMssMCB.sigProcChainCfg.numFrmPerMinorMotProc * gMmwMssMCB.sigProcChainCfg.numMinorMotionChirpsPerFrame; //These are now chirps, not chirp pairs
        }
    }


    doaStaticCfg->numDopplerBins     = mathUtils_pow2roundup(doaStaticCfg->numDopplerChirps);
    doaStaticCfg->log2NumDopplerBins = mathUtils_ceilLog2(doaStaticCfg->numDopplerBins);

    gMmwMssMCB.numDopplerBins = doaStaticCfg->numDopplerBins;

    doaStaticCfg->selectCoherentPeakInDopplerDim = gMmwMssMCB.sigProcChainCfg.coherentDoppler;
    doaStaticCfg->angleDimension        = gMmwMssMCB.angleDimension;
    doaStaticCfg->isDetMatrixLogScale   = false;
    doaStaticCfg->azimuthFftSize        = gMmwMssMCB.sigProcChainCfg.azimuthFftSize;
    doaStaticCfg->elevationFftSize      = gMmwMssMCB.sigProcChainCfg.elevationFftSize;
    doaStaticCfg->isStaticClutterRemovalEnabled = gMmwMssMCB.staticClutterRemovalEnable;
    doaStaticCfg->isRxChGainPhaseCompensationEnabled   = 1;
    doaStaticCfg->doaRangeLoopType = 0;
    doaStaticCfg->dopElevDimReductOrder = gMmwMssMCB.sigProcChainCfg.dopElevDimReductOrder;

    /* Configure Doppler fft param sets for mapping antennas into 1D/2D virtual antenna array */
    retVal = MmwDemo_cfgDopplerParamMapping(&hwaCfg->doaRngGateCfg, DOPPLER_OUTPUT_MAPPING_DOP_ROW_COL);
    if (retVal < 0)
    {
        goto exit;
    }

    /* L3 - Detection Matrix */
    for (i=0; i<2; i++)
    {
        if (((i==0) && doaStaticCfg->enableMajorMotion) || ((i==1) && (doaStaticCfg->enableMinorMotion)))
        {
            if(doaStaticCfg->isDetMatrixLogScale)
            {
                gMmwMssMCB.detMatrix[i].dataSize = doaStaticCfg->numRangeBins * doaStaticCfg->azimuthFftSize * sizeof(uint16_t);
            }
            else
            {
                gMmwMssMCB.detMatrix[i].dataSize = doaStaticCfg->numRangeBins * doaStaticCfg->azimuthFftSize * sizeof(uint32_t);
            }
            gMmwMssMCB.detMatrix[i].data = DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                        gMmwMssMCB.detMatrix[i].dataSize,
                                                        sizeof(uint32_t));
            if (gMmwMssMCB.detMatrix[i].data == NULL)
            {
                retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_DET_MATRIX;
                goto exit;
            }
            gMmwMssMCB.detMatrix[i].datafmt = DPC_DPU_DPIF_DETMATRIX_FORMAT_2;


            gMmwMssMCB.rangeProfile[i] = DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                 doaStaticCfg->numRangeBins * sizeof(uint32_t),
                                                                 sizeof(uint32_t));
            if (gMmwMssMCB.rangeProfile[i] == NULL)
            {
                retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_RANGE_PROFILE;
                goto exit;
            }
        }
        else
        {
            gMmwMssMCB.detMatrix[i].dataSize = 0;
            gMmwMssMCB.detMatrix[i].data = NULL;
            gMmwMssMCB.detMatrix[i].datafmt = 0;
            gMmwMssMCB.rangeProfile[i] = NULL;
        }
    }

    if (doaStaticCfg->dopElevDimReductOrder == 0)
    {
        /* V1: Dimensionality reduction order: Doppler then Elevation */
        /* L3 - Doppler Index Matrix */
        if ((doaStaticCfg->selectCoherentPeakInDopplerDim == 1) ||
            (doaStaticCfg->selectCoherentPeakInDopplerDim == 2))
        {
            if (doaStaticCfg->angleDimension == 2)
            {
                /* 2D-case -  with elevation */
                dopplerIndexMatrix.dataSize = doaStaticCfg->numRangeBins * doaStaticCfg->azimuthFftSize * doaStaticCfg->elevationFftSize * sizeof(uint8_t);
            }
            else
            {
                /* 1D-case -  no elevation */
                dopplerIndexMatrix.dataSize = doaStaticCfg->numRangeBins * doaStaticCfg->azimuthFftSize * sizeof(uint8_t);
            }
            dopplerIndexMatrix.data = DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                              dopplerIndexMatrix.dataSize,
                                                              sizeof(uint8_t));
            if (dopplerIndexMatrix.data == NULL)
            {
                retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_DET_MATRIX;
                goto exit;
            }
            dopplerIndexMatrix.datafmt = DPC_DPU_DPIF_DETMATRIX_FORMAT_2;
        }
        else
        {
            /* Non-coherent combining along Doppler dimension, Doppler output = 0 */
            dopplerIndexMatrix.dataSize = 0;
            dopplerIndexMatrix.data = NULL;
            dopplerIndexMatrix.datafmt = 0;
        }

        if (doaStaticCfg->angleDimension == 2)
        {
            /* L3 - Elevation Index Matrix */
            elevationIndexMatrix.dataSize = doaStaticCfg->numRangeBins * doaStaticCfg->azimuthFftSize * sizeof(uint8_t);
            elevationIndexMatrix.data = DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                elevationIndexMatrix.dataSize,
                                                                sizeof(uint8_t));
            if (elevationIndexMatrix.data == NULL)
            {
                retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_DET_MATRIX;
                goto exit;
            }
            elevationIndexMatrix.datafmt = DPC_DPU_DPIF_DETMATRIX_FORMAT_2;
        }
        else
        {
            elevationIndexMatrix.dataSize = 0;
            elevationIndexMatrix.data = NULL;
            elevationIndexMatrix.datafmt = 0;
        }
    }
    else
    {
        /* V2: Dimensionality reduction order: Elevation then Doppler */
        /* L3 - Doppler Index Matrix */
        if ((doaStaticCfg->selectCoherentPeakInDopplerDim == 1) ||
            (doaStaticCfg->selectCoherentPeakInDopplerDim == 2))
        {
            if (doaStaticCfg->angleDimension == 2)
            {
                /* 2D-case -  with elevation */
                dopplerIndexMatrix.dataSize = doaStaticCfg->numRangeBins * doaStaticCfg->azimuthFftSize * sizeof(uint8_t);
            }
            else
            {
                /* 1D-case -  no elevation */
                dopplerIndexMatrix.dataSize = doaStaticCfg->numRangeBins * doaStaticCfg->azimuthFftSize * sizeof(uint8_t);
            }
            dopplerIndexMatrix.data = DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                              dopplerIndexMatrix.dataSize,
                                                              sizeof(uint8_t));
            if (dopplerIndexMatrix.data == NULL)
            {
                retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_DET_MATRIX;
                goto exit;
            }
            dopplerIndexMatrix.datafmt = DPC_DPU_DPIF_DETMATRIX_FORMAT_2;
        }
        else
        {
            /* Non-coherent combining along Doppler dimension, Doppler output = 0 */
            dopplerIndexMatrix.dataSize = 0;
            dopplerIndexMatrix.data = NULL;
            dopplerIndexMatrix.datafmt = 0;
        }
        if (doaStaticCfg->angleDimension == 2)
        {
             /* L3 - Elevation Index Matrix */
            elevationIndexMatrix.dataSize = doaStaticCfg->numRangeBins * doaStaticCfg->azimuthFftSize * doaStaticCfg->numDopplerBins *sizeof(uint8_t);
            elevationIndexMatrix.data = DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                elevationIndexMatrix.dataSize,
                                                                sizeof(uint8_t));
            if (elevationIndexMatrix.data == NULL)
            {
                retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_DET_MATRIX;
                goto exit;
            }
            elevationIndexMatrix.datafmt = DPC_DPU_DPIF_DETMATRIX_FORMAT_2;
        }
        else
        {
            elevationIndexMatrix.dataSize = 0;
            elevationIndexMatrix.data = NULL;
            elevationIndexMatrix.datafmt = 0;
        }
    }

    /* hwRes - copy these structures */
    hwRes->radarCube = gMmwMssMCB.radarCube[0];
    hwRes->radarCubeMinMot = gMmwMssMCB.radarCube[1];

    hwRes->dopplerIndexMatrix = dopplerIndexMatrix;
    hwRes->elevationIndexMatrix = elevationIndexMatrix;

    gMmwMssMCB.dopplerIndexMatrix = dopplerIndexMatrix;
    gMmwMssMCB.elevationIndexMatrix = elevationIndexMatrix;

    /* hwRes - edmaCfg */
    edmaCfg->edmaHandle = gEdmaHandle[0];

    /* edmaIn - ping - minor motion*/
    edmaCfg->edmaIn.chunk[0].channel =            DPC_OBJDET_DPU_DOAPROC_EDMAIN_PING_CH;
    edmaCfg->edmaIn.chunk[0].channelShadow =      DPC_OBJDET_DPU_DOAPROC_EDMAIN_PING_SHADOW;
    edmaCfg->edmaIn.chunk[0].eventQueue =         DPC_OBJDET_DPU_DOAPROC_EDMAIN_PING_EVENT_QUE;

    /* edmaIn - pong - minor motion*/
    edmaCfg->edmaIn.chunk[1].channel =            DPC_OBJDET_DPU_DOAPROC_EDMAIN_PONG_CH;
    edmaCfg->edmaIn.chunk[1].channelShadow =      DPC_OBJDET_DPU_DOAPROC_EDMAIN_PONG_SHADOW;
    edmaCfg->edmaIn.chunk[1].eventQueue =         DPC_OBJDET_DPU_DOAPROC_EDMAIN_PONG_EVENT_QUE;

    /* edmaHotSig */
    edmaCfg->edmaHotSig.channel =             DPC_OBJDET_DPU_DOAPROC_EDMA_HOT_SIG_CH;
    edmaCfg->edmaHotSig.channelShadow =       DPC_OBJDET_DPU_DOAPROC_EDMA_HOT_SIG_SHADOW;
    edmaCfg->edmaHotSig.eventQueue =          DPC_OBJDET_DPU_DOAPROC_EDMA_HOT_SIG_EVENT_QUE;




    /* edmaOut - Detection Matrix */
    edmaCfg->edmaDetMatOut.channel =           DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DET_MATRIX_CH;
    edmaCfg->edmaDetMatOut.channelShadow =     DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DET_MATRIX_SHADOW;
    edmaCfg->edmaDetMatOut.eventQueue =        DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DET_MATRIX_EVENT_QUE;

    /* edmaOut - Elevation Index Matrix */
    edmaCfg->elevIndMatOut.channel =       DPC_OBJDET_DPU_DOAPROC_EDMAOUT_ELEVIND_MATRIX_CH;
    edmaCfg->elevIndMatOut.channelShadow = DPC_OBJDET_DPU_DOAPROC_EDMAOUT_ELEVIND_MATRIX_SHADOW;
    edmaCfg->elevIndMatOut.eventQueue =    DPC_OBJDET_DPU_DOAPROC_EDMAOUT_ELEVIND_MATRIX_EVENT_QUE;

    /* edmaOut - Doppler Index Matrix */
    edmaCfg->dopIndMatOut.channel =        DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DOPIND_MATRIX_CH;
    edmaCfg->dopIndMatOut.channelShadow =  DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DOPIND_MATRIX_SHADOW;
    edmaCfg->dopIndMatOut.eventQueue =     DPC_OBJDET_DPU_DOAPROC_EDMAOUT_DOPIND_MATRIX_EVENT_QUE;

    edmaCfg->edmaInterLoopOut.channel =       DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAOUT_DET_MATRIX_CH;
    edmaCfg->edmaInterLoopOut.channelShadow = DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAOUT_DET_MATRIX_SHADOW;
    edmaCfg->edmaInterLoopOut.eventQueue =    DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAOUT_DET_MATRIX_EVENT_QUE;

    edmaCfg->edmaInterLoopIn.channel =       DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAIN_CH;
    edmaCfg->edmaInterLoopIn.channelShadow = DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAIN_SHADOW;
    edmaCfg->edmaInterLoopIn.eventQueue =    DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMAIN_EVENT_QUE;

    edmaCfg->edmaInterLoopHotSig.channel =       DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_HOT_SIG_CH;
    edmaCfg->edmaInterLoopHotSig.channelShadow = DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_HOT_SIG_SHADOW;
    edmaCfg->edmaInterLoopHotSig.eventQueue =    DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_HOT_SIG_EVENT_QUE;

    edmaCfg->edmaInterLoopChainBack.channel =       DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_CHAIN_BACK_CH;
    edmaCfg->edmaInterLoopChainBack.channelShadow = DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_CHAIN_BACK_SHADOW;
    edmaCfg->edmaInterLoopChainBack.eventQueue =    DPC_OBJDET_DPU_DOAPROC_INTER_LOOP_EDMA_CHAIN_BACK_EVENT_QUE;
    edmaCfg->intrObj = &intrObj_doaProc;

    /* hwaCfg */
    hwaCfg->hwaMemInpAddr = CSL_APP_HWA_DMA0_RAM_BANK3_BASE;
    hwaCfg->numParamSets = 0;  //The number depends on the configuration, will be populated by DPU_DoaProc_config()
    hwaCfg->paramSetStartIdx = DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS;

    hwaCfg->dmaTrigSrcChan = DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(&gMmwMssMCB.HwaDmaChanPoolObj);

    /* hwaCfg - window */
    //Share FFT window between azimuth and elevation FFT, window = [+1 -1 +1 -1 ... ]
    winGenLen = (doaStaticCfg->azimuthFftSize > doaStaticCfg->elevationFftSize) ? doaStaticCfg->azimuthFftSize : doaStaticCfg->elevationFftSize;
    hwaCfg->windowSize = winGenLen * sizeof(int32_t);

    hwaCfg->window = (int32_t *)DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.CoreLocalRamObj,
                                                        hwaCfg->windowSize,
                                                        sizeof(uint32_t));
    if (hwaCfg->window == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_DOA_HWA_WINDOW;
        goto exit;
    }
    /*Alternate 1,-1,...*/
    for (i=0; i<winGenLen; i++)
    {
        hwaCfg->window[i] = (1 - 2 * (i & 0x1)) * ((1<<17) - 1);
    }

    hwaCfg->winRamOffset = doaStaticCfg->numRangeBins;
    hwaCfg->winSym = HWA_FFT_WINDOW_NONSYMMETRIC;

    /* Rx compensation coefficients */
    {
        int32_t rxInd, txInd;
        int32_t ind = 0;
        doaStaticCfg->compRxChanCfg.rangeBias = gMmwMssMCB.compRxChannelBiasCfg.rangeBias;
        for (txInd = 0; txInd < doaStaticCfg->numTxAntennas; txInd++)
        {
            for (rxInd = 0; rxInd < doaStaticCfg->numRxAntennas; rxInd++)
            {
                doaStaticCfg->compRxChanCfg.rxChPhaseComp[ind++] = gMmwMssMCB.compRxChannelBiasCfg.rxChPhaseComp[gMmwMssMCB.rxAntOrder[rxInd] + (txInd * SYS_COMMON_NUM_RX_CHANNEL)];
            }
        }
    }

    hwRes->interLoopDataBuffer = NULL;

exit:
return retVal;
}
