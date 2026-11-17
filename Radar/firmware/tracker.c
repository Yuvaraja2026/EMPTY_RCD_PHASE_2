#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/uart.h>
#include <drivers/edma.h>
#include <drivers/soc.h>
#include "motion_detect.h"

extern MmwDemo_MSS_MCB gMmwMssMCB;
extern uint8_t DPC_ObjDet_HwaDmaTrigSrcChanPoolAlloc(HwaDmaTrigChanPoolObj *pool);

unsigned int gGtrackMemoryUsed = 0;

/* These functions are abstracted to the DPC */
void *gtrack_alloc(unsigned int numElements, unsigned int sizeInBytes)
{
	gGtrackMemoryUsed += numElements*sizeInBytes;
	return HeapP_alloc(&gMmwMssMCB.CoreLocalTrackerHeapObj, numElements*sizeInBytes);
}
void gtrack_free(void *pFree, unsigned int sizeInBytes)
{
	gGtrackMemoryUsed -= sizeInBytes;
	HeapP_free(&gMmwMssMCB.CoreLocalTrackerHeapObj, pFree);
}
/**
*  @b Description
*  @n
*    Processing chain initialization
*/
void trackerProc_dpuInit()
{
    int32_t  errorCode = 0;
    /* generate the dpu handler*/
    gMmwMssMCB.trackerProcDpuHandle =  DPU_TrackerProc_init(&errorCode);
    if (gMmwMssMCB.trackerProcDpuHandle == NULL)
    {
        DebugP_log ("Error: Tracker Proc DPU initialization returned error %d\n", errorCode);
        DebugP_assert (0);
        return;
    }
}