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
#include "dpc/dpc.h"
#include "mmw_res.h"
#include <utils/mathutils/mathutils.h>

/* Data Path Include files */
#include <datapath/dpu/rangeproc/v0/rangeprochwa.h>
/* Internal include Files */
#include <datapath/dpu/rangeproc/v0/rangeprochwa_internal.h>
/* User defined heap memory and handle */
#define RANGEPROCHWA_HEAP_MEM_SIZE  (sizeof(rangeProcHWAObj))

#define HWA_MAX_NUM_DMA_TRIG_CHANNELS 16

#define DPC_OBJDET_HWA_WINDOW_RAM_OFFSET 0
#define DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE MATHUTILS_WIN_BLACKMAN
#define DPC_OBJDET_QFORMAT_RANGE_FFT 17
#define MMW_DEMO_TEST_ADC_BUFF_SIZE 1024  //maximum 128 real samples (int16_t), 3 Rx channels
Edma_IntrObject     intrObj_rangeProc[2];
extern MmwDemo_MSS_MCB gMmwMssMCB; 

static uint8_t gRangeProcHeapMem[RANGEPROCHWA_HEAP_MEM_SIZE] __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));
HWA_Handle hwaHandle;
DPU_RangeProcHWA_Config rangeProcDpuCfg;


/**
 *  @b Description
 *  @n
 *      The function is rangeProc DPU init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]  initParams              Pointer to DPU init parameters
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_RANGEPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid rangeProc handle
 *  @retval
 *      Error       - NULL
 */
DPU_RangeProcHWA_Handle DPU_RangeProcHWA_init
(
    DPU_RangeProcHWA_InitParams     *initParams,
    int32_t*                        errCode
)
{
    rangeProcHWAObj     *rangeProcObj = NULL;
    // SemaphoreP_Params   semParams;
    HWA_MemInfo         hwaMemInfo;
    uint8_t             index;
    int32_t             status = SystemP_SUCCESS;

    *errCode = 0;

    if( (initParams == NULL) ||
       (initParams->hwaHandle == NULL) )
    {
        *errCode = DPU_RANGEPROCHWA_EINVAL;
        goto exit;
    }

    /* Allocate Memory for rangeProc */
    rangeProcObj = (rangeProcHWAObj*)&gRangeProcHeapMem;
    if(rangeProcObj == NULL)
    {
        *errCode = DPU_RANGEPROCHWA_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)rangeProcObj, 0, sizeof(rangeProcHWAObj));

    memcpy((void *)&rangeProcObj->initParms, initParams, sizeof(DPU_RangeProcHWA_InitParams));

    /* Set HWA bank memory address */
    *errCode =  HWA_getHWAMemInfo(initParams->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {
        goto exit;
    }

    for (index = 0; index < hwaMemInfo.numBanks; index++)
    {
        rangeProcObj->hwaMemBankAddr[index] = hwaMemInfo.baseAddress + index * hwaMemInfo.bankSize;
    }

    /* Create semaphore for EDMA done */
    status = SemaphoreP_constructBinary(&rangeProcObj->edmaDoneSemaHandle, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_RANGEPROCHWA_ESEMA;
        goto exit;
    }

    /* Create semaphore for HWA done */
    status = SemaphoreP_constructBinary(&rangeProcObj->hwaDoneSemaHandle, 0);
    if(status != SystemP_SUCCESS)
    {
        *errCode = DPU_RANGEPROCHWA_ESEMA;
        goto exit;
    }

exit:
    if(*errCode < 0)
    {
        rangeProcObj = (DPU_RangeProcHWA_Handle)NULL;
    }
    else
    {
        /* Fall through */
    }
    return ((DPU_RangeProcHWA_Handle)rangeProcObj);

}


/**
*  @b Description
*  @n
*    Range processing DPU Initialization
*/
void rangeProc_dpuInit()
{
    int32_t errorCode = 0;
    DPU_RangeProcHWA_InitParams initParams;
    initParams.hwaHandle = hwaHandle;

    /* generate the dpu handler*/
    gMmwMssMCB.rangeProcDpuHandle = DPU_RangeProcHWA_init(&initParams, &errorCode);
    if (gMmwMssMCB.rangeProcDpuHandle == NULL)
    {
        DebugP_log("Error: RangeProc DPU initialization returned error %d\n", errorCode);
        DebugP_assert(0);
        return;
    }
}




/**
*  @b Description
*  @n
*    Based on the configuration, set up the range processing DPU configurations
*/
int32_t RangeProc_configParser()
{

    int32_t retVal = 0;
    DPU_RangeProcHWA_HW_Resources *pHwConfig = &rangeProcDpuCfg.hwRes;
    DPU_RangeProcHWA_StaticConfig  * params;
    uint32_t index;
    uint32_t bytesPerRxChan;

    /* Rangeproc DPU */
    pHwConfig = &rangeProcDpuCfg.hwRes;
    params = &rangeProcDpuCfg.staticCfg;

    memset((void *)&rangeProcDpuCfg, 0, sizeof(DPU_RangeProcHWA_Config));

    params->enableMajorMotion = gMmwMssMCB.enableMajorMotion;
    params->enableMinorMotion = gMmwMssMCB.enableMinorMotion;

    params->numFramesPerMinorMotProc = gMmwMssMCB.sigProcChainCfg.numFrmPerMinorMotProc;
    params->numMinorMotionChirpsPerFrame = gMmwMssMCB.sigProcChainCfg.numMinorMotionChirpsPerFrame;
    params->frmCntrModNumFramesPerMinorMot = gMmwMssMCB.frmCntrModNumFramesPerMinorMot;
    params->lowPowerMode = gMmwMssMCB.lowPowerMode;

    gMmwMssMCB.frmCntrModNumFramesPerMinorMot++;
    if(gMmwMssMCB.frmCntrModNumFramesPerMinorMot == gMmwMssMCB.sigProcChainCfg.numFrmPerMinorMotProc)
    {
        gMmwMssMCB.frmCntrModNumFramesPerMinorMot = 0;
    }

    /* hwi configuration */
    pHwConfig = &rangeProcDpuCfg.hwRes;

    /* HWA configurations, not related to per test, common to all test */
    pHwConfig->hwaCfg.paramSetStartIdx = 0;
    pHwConfig->hwaCfg.numParamSet = DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS;
    pHwConfig->hwaCfg.hwaWinRamOffset  = DPC_OBJDET_HWA_WINDOW_RAM_OFFSET;
    pHwConfig->hwaCfg.hwaWinSym = 1;
    pHwConfig->hwaCfg.dataInputMode = DPU_RangeProcHWA_InputMode_ISOLATED;
    pHwConfig->hwaCfg.dmaTrigSrcChan[0] = DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(&gMmwMssMCB.HwaDmaChanPoolObj);
    pHwConfig->hwaCfg.dmaTrigSrcChan[1] = DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(&gMmwMssMCB.HwaDmaChanPoolObj);


    /* edma configuration */
    pHwConfig->edmaHandle  = gEdmaHandle[0];
    /* edma configuration depends on the interleave or non-interleave */

    /* windowing buffer is fixed, size will change*/
    params->windowSize = sizeof(uint32_t) * ((gMmwMssMCB.profileComCfg.h_NumOfAdcSamples +1 ) / 2); //symmetric window, for real samples
    params->window =  (int32_t *)DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.CoreLocalRamObj,
                                                         params->windowSize,
                                                         sizeof(uint32_t));
    if (params->window == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__CORE_LOCAL_RAM_RANGE_HWA_WINDOW;
        goto exit;
    }

    /* adc buffer buffer, format fixed, interleave, size will change */
    params->ADCBufData.dataProperty.dataFmt = DPIF_DATAFORMAT_REAL16;
    params->ADCBufData.dataProperty.adcBits = 2U; // 12-bit only
    params->ADCBufData.dataProperty.numChirpsPerChirpEvent = 1U;

    #if (CLI_REMOVAL == 0)
    if(gMmwMssMCB.adcDataSourceCfg.source == 0)
    {
        params->ADCBufData.data = (void *)CSL_APP_HWA_ADCBUF_RD_U_BASE;
    }
    else
    {
        gMmwMssMCB.adcTestBuff  = (uint8_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                            MMW_DEMO_TEST_ADC_BUFF_SIZE,
                                                                            sizeof(uint32_t));
        if(gMmwMssMCB.adcTestBuff == NULL)
        {
            retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_ADC_TEST_BUFF;
            goto exit;
        }
        params->ADCBufData.data = (void *)gMmwMssMCB.adcTestBuff;

    }
    #else
    params->ADCBufData.data = (void *)CSL_APP_HWA_ADCBUF_RD_U_BASE;
    #endif

    params->numTxAntennas = (uint8_t) gMmwMssMCB.numTxAntennas;
    params->numVirtualAntennas = (uint8_t) (gMmwMssMCB.numTxAntennas * gMmwMssMCB.numRxAntennas);
    params->numRangeBins = gMmwMssMCB.numRangeBins;
    params->numChirpsPerFrame = gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame * gMmwMssMCB.frameCfg.h_NumOfChirpsInBurst;
    params->numDopplerChirpsPerFrame = params->numChirpsPerFrame/params->numTxAntennas;

    if ((params->numTxAntennas == 1) && (gMmwMssMCB.frameCfg.h_NumOfBurstsInFrame !=1))
    {
        retVal = DPC_OBJECTDETECTION_EINVAL_CFG;
        goto exit;
    }
    if ((params->numTxAntennas == 1) && (gMmwMssMCB.isBpmEnabled))
    {
        retVal = DPC_OBJECTDETECTION_EINVAL_CFG;
        goto exit;
    }

    if (params->enableMajorMotion)
    {
        params->numDopplerChirpsPerProc = params->numDopplerChirpsPerFrame;
    }
    else
    {
        params->numDopplerChirpsPerProc = params->numFramesPerMinorMotProc * params->numMinorMotionChirpsPerFrame;
    }

    params->isBpmEnabled = gMmwMssMCB.isBpmEnabled;
    /* windowing */
    params->ADCBufData.dataProperty.numRxAntennas = (uint8_t) gMmwMssMCB.numRxAntennas;
    params->ADCBufData.dataSize = gMmwMssMCB.profileComCfg.h_NumOfAdcSamples * params->ADCBufData.dataProperty.numRxAntennas * 4 ;
    params->ADCBufData.dataProperty.numAdcSamples = gMmwMssMCB.profileComCfg.h_NumOfAdcSamples;

    if (!gMmwMssMCB.oneTimeConfigDone)
    {
        mathUtils_genWindow((uint32_t *)params->window,
                            (uint32_t) params->ADCBufData.dataProperty.numAdcSamples,
                            params->windowSize/sizeof(uint32_t),
                            DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE,
                            DPC_OBJDET_QFORMAT_RANGE_FFT);
    }
    params->rangeFFTtuning.fftOutputDivShift = 2;
    params->rangeFFTtuning.numLastButterflyStagesToScale = 0; /* no scaling needed as ADC is 16-bit and we have 8 bits to grow */

    params->rangeFftSize = mathUtils_pow2roundup(params->ADCBufData.dataProperty.numAdcSamples);

    bytesPerRxChan = params->ADCBufData.dataProperty.numAdcSamples * sizeof(uint16_t);
    bytesPerRxChan = (bytesPerRxChan + 15) / 16 * 16;

    for (index = 0; index < SYS_COMMON_NUM_RX_CHANNEL; index++)
    {
        params->ADCBufData.dataProperty.rxChanOffset[index] = index * bytesPerRxChan;
    }

    params->ADCBufData.dataProperty.interleave = DPIF_RXCHAN_NON_INTERLEAVE_MODE;
    /* Data Input EDMA */
    pHwConfig->edmaInCfg.dataIn.channel         = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_CH;
    pHwConfig->edmaInCfg.dataIn.channelShadow[0]   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SHADOW_PING;
    pHwConfig->edmaInCfg.dataIn.channelShadow[1]   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SHADOW_PONG;
    pHwConfig->edmaInCfg.dataIn.eventQueue      = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_EVENT_QUE;
    pHwConfig->edmaInCfg.dataInSignature.channel         = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_CH;
    pHwConfig->edmaInCfg.dataInSignature.channelShadow   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_SHADOW;
    pHwConfig->edmaInCfg.dataInSignature.eventQueue      = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_EVENT_QUE;
    pHwConfig->intrObj = intrObj_rangeProc;

    /* Data Output EDMA */
    pHwConfig->edmaOutCfg.path[0].evtDecim.channel = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_CH;
    pHwConfig->edmaOutCfg.path[0].evtDecim.channelShadow[0] = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_SHADOW_0;
    pHwConfig->edmaOutCfg.path[0].evtDecim.channelShadow[1] = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_SHADOW_1;
    pHwConfig->edmaOutCfg.path[0].evtDecim.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PING_EVENT_QUE;

    pHwConfig->edmaOutCfg.path[1].evtDecim.channel = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_CH;
    pHwConfig->edmaOutCfg.path[1].evtDecim.channelShadow[0] = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_SHADOW_0;
    pHwConfig->edmaOutCfg.path[1].evtDecim.channelShadow[1] = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_SHADOW_1;
    pHwConfig->edmaOutCfg.path[1].evtDecim.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EVT_DECIM_PONG_EVENT_QUE;

    pHwConfig->edmaOutCfg.path[0].dataOutMinor.channel = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PING_CH;
    pHwConfig->edmaOutCfg.path[0].dataOutMinor.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PING_SHADOW;
    pHwConfig->edmaOutCfg.path[0].dataOutMinor.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PING_EVENT_QUE;

    pHwConfig->edmaOutCfg.path[1].dataOutMinor.channel = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PONG_CH;
    pHwConfig->edmaOutCfg.path[1].dataOutMinor.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PONG_SHADOW;
    pHwConfig->edmaOutCfg.path[1].dataOutMinor.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MINOR_PONG_EVENT_QUE;

    pHwConfig->edmaOutCfg.path[0].dataOutMajor.channel = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PING_CH;
    pHwConfig->edmaOutCfg.path[0].dataOutMajor.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PING_SHADOW;
    pHwConfig->edmaOutCfg.path[0].dataOutMajor.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PING_EVENT_QUE;

    pHwConfig->edmaOutCfg.path[1].dataOutMajor.channel = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PONG_CH;
    pHwConfig->edmaOutCfg.path[1].dataOutMajor.channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PONG_SHADOW;
    pHwConfig->edmaOutCfg.path[1].dataOutMajor.eventQueue = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_MAJOR_PONG_EVENT_QUE;

    /* Radar cube Minor Motion*/
    if (params->enableMinorMotion)
    {
        gMmwMssMCB.radarCube[1].dataSize = params->numRangeBins * params->numVirtualAntennas * sizeof(cmplx16ReIm_t) * params->numDopplerChirpsPerProc;
        gMmwMssMCB.radarCube[1].data  = (cmplx16ImRe_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                                     gMmwMssMCB.radarCube[1].dataSize,
                                                                                     sizeof(uint32_t));
        if(gMmwMssMCB.radarCube[1].data == NULL)
        {
            retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE;
            goto exit;
        }

    }
    else
    {
        gMmwMssMCB.radarCube[1].data  = NULL;
        gMmwMssMCB.radarCube[1].dataSize = 0;
    }
    gMmwMssMCB.radarCube[1].datafmt = DPIF_RADARCUBE_FORMAT_6;
    rangeProcDpuCfg.hwRes.radarCubeMinMot = gMmwMssMCB.radarCube[1];

    /* Radar cube Major Motion*/
    if (params->enableMajorMotion)
    {

        gMmwMssMCB.radarCube[0].dataSize = params->numRangeBins * params->numVirtualAntennas * sizeof(cmplx16ReIm_t) * params->numDopplerChirpsPerProc;
        gMmwMssMCB.radarCube[0].data  = (cmplx16ImRe_t *) DPC_ObjDet_MemPoolAlloc(&gMmwMssMCB.L3RamObj,
                                                                               gMmwMssMCB.radarCube[0].dataSize,
                                                                               sizeof(uint32_t));
        if(gMmwMssMCB.radarCube[0].data == NULL)
        {
            retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE;
            goto exit;
        }


    }
    else
    {
        gMmwMssMCB.radarCube[0].data  = NULL;
        gMmwMssMCB.radarCube[0].dataSize = 0;
    }
    gMmwMssMCB.radarCube[0].datafmt = DPIF_RADARCUBE_FORMAT_6;
    rangeProcDpuCfg.hwRes.radarCube = gMmwMssMCB.radarCube[0];

exit:
    return retVal;
}
