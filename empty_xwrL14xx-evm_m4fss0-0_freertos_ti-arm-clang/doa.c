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

#define DOPPLER_OUTPUT_MAPPING_DOP_ROW_COL   0
#define DOPPLER_OUTPUT_MAPPING_ROW_DOP_COL   1

extern MmwDemo_MSS_MCB gMmwMssMCB; 

HWA_Handle hwaHandle;
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
