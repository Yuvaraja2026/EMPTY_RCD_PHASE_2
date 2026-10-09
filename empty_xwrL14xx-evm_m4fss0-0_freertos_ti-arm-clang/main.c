#include "ti_drivers_config.h"     
#include "ti_board_config.h"   
<<<<<<< HEAD
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/DebugP.h>

int main(void)
{
    System_init();          /* Initialize the system by preparing the required system resources for operation.*/
    Board_init();           /* Initialize the board by preparing the hardware peripherals for use.*/

    Drivers_open();         /* Open and initialize the required drivers so that the software can communicate */
    Board_driversOpen();    /* Open the board-specific drivers */
    
    DebugP_log("Drivers_open and Board_driversOpen!!\r\n");
=======

int main(void)
{
    System_init();
    Board_init();
>>>>>>> 384dbf58d685f1104c5529d026482b7f4d8de3e0
    while (1)
    {
        //task stays here forever
    }
}