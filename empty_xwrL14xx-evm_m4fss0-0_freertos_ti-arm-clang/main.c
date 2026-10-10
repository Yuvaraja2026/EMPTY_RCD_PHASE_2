#include "ti_drivers_config.h"     
#include "ti_board_config.h"   
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/DebugP.h>
#include <drivers/uart.h>
#include "FreeRTOS.h"
#include "task.h"

#define MAIN_TASK_PRI (configMAX_PRIORITIES - 1U)                   /* Sets the main task priority to one less than the maximum allowed priority */

#define MAIN_TASK_SIZE (8192U / sizeof(configSTACK_DEPTH_TYPE))     /* Calculates the main task stack size based on 8192 bytes of memory */

StackType_t gMainTaskStack[MAIN_TASK_SIZE]__attribute__((aligned(32))); /* Allocates the main task stack with 32-byte memory alignment */
StaticTask_t gMainTaskObj;  /* Declares a static task control block to store the main task's information */
TaskHandle_t gMainTask;     /* Declares a task handle used to reference and manage the main task */

static void MinimalApp_task(void *args)
{
    UART_Transaction transaction;       /* Declares a UART transaction structure to store the transmission details */
    uint8_t message[] ="FreeRTOS task is running\r\n";
    Drivers_open();         /* Open and initialize the required drivers so that the software can communicate */
    Board_driversOpen();    /* Open the board-specific drivers */
    DebugP_log("Drivers_open and Board_driversOpen!!\r\n");
    UART_Transaction_init(&transaction);    /* Initializes the UART transaction structure */
    transaction.buf=message;                /* Assigns the message buffer to the UART transaction */
    transaction.count=sizeof(message)-1;    /* Sets the number of bytes to transmit, excluding the null terminator */
    UART_write(gUartHandle[0], &transaction);/* Transmits the message through the UART interface using UART handle 0 */
}
int main(void)
{
    System_init();          /* Initialize the system by preparing the required system resources for operation.*/
    Board_init();           /* Initialize the board by preparing the hardware peripherals for use.*/

    gMainTask =
        xTaskCreateStatic(       /* Creates a FreeRTOS task using statically allocated memory */
            MinimalApp_task,    /* Specifies the function that the task will execute */
            "minimal_app",      /* Sets the name of the task */
            MAIN_TASK_SIZE,     /* Specifies the stack size allocated for the task */
            NULL,
            MAIN_TASK_PRI,      /* Sets the priority of the task */
            gMainTaskStack,      /* Provides the memory allocated for the task stack */
            &gMainTaskObj);     /* Provides the task control block for static task allocation */

    while (1)
    {
        //task stays here forever
    }
}
