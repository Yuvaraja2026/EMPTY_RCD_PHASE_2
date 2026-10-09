#include "ti_drivers_config.h"     
#include "ti_board_config.h"   
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/DebugP.h>
#include <drivers/uart.h>
#include "FreeRTOS.h"
#include "task.h"

#define MAIN_TASK_PRI (configMAX_PRIORITIES - 1U)

#define MAIN_TASK_SIZE (8192U / sizeof(configSTACK_DEPTH_TYPE))

StackType_t gMainTaskStack[MAIN_TASK_SIZE]__attribute__((aligned(32)));
StaticTask_t gMainTaskObj;
TaskHandle_t gMainTask;

static void MinimalApp_task(void *args)
{
    UART_Transaction transaction;
    uint8_t message[] ="AWRL1432 Minimal Application\r\n";
    Drivers_open();         /* Open and initialize the required drivers so that the software can communicate */
    Board_driversOpen();    /* Open the board-specific drivers */
    DebugP_log("Drivers_open and Board_driversOpen!!\r\n");
    UART_Transaction_init(&transaction);
    transaction.buf=message;
    transaction.count=sizeof(message)-1;
    UART_write(gUartHandle[0], &transaction);
}
int main(void)
{
    System_init();          /* Initialize the system by preparing the required system resources for operation.*/
    Board_init();           /* Initialize the board by preparing the hardware peripherals for use.*/

    gMainTask =
        xTaskCreateStatic(
            MinimalApp_task,
            "minimal_app",
            MAIN_TASK_SIZE,
            NULL,
            MAIN_TASK_PRI,
            gMainTaskStack,
            &gMainTaskObj);

    while (1)
    {
        //task stays here forever
    }
}