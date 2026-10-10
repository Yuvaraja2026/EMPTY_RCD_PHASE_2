#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <drivers/uart.h>
#include <drivers/edma.h>
#include <drivers/soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "hwa.h"

static HWA_Handle gHwaHandle = NULL;
static volatile int32_t gHwaStatus = SystemP_SUCCESS;

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
    if (gEdmaHandle[0] == NULL) { while (1) { } }
    /* T007: open only HWA instance 0, matching the supplied DPC_Init(). */
    gHwaStatus = SystemP_SUCCESS;
    gHwaHandle = HWA_open(0, NULL, (int32_t *)&gHwaStatus);
    if (gHwaHandle == NULL) { while (1) { } }
     MinimalRadar_sendText("HWA OK\r\n");
         /* Exact memory-init mask from motion_detect.c. */
    SOC_memoryInit(SOC_RCM_MEMINIT_HWA_SHRAM_INIT | SOC_RCM_MEMINIT_TPCCA_INIT | SOC_RCM_MEMINIT_TPCCB_INIT | SOC_RCM_MEMINIT_FECSS_SHRAM_INIT | SOC_RCM_MEMINIT_APPSS_SHRAM0_INIT | SOC_RCM_MEMINIT_APPSS_SHRAM1_INIT);
    MinimalRadar_sendText("Radar memories initialized\r\n");
}