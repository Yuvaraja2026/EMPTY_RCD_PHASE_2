#include "ti_drivers_config.h"     
#include "ti_board_config.h"   
<<<<<<< HEAD
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
=======
>>>>>>> 9a53e54c3529d50eb1ce4fb9934b17e6a50afc90

int main(void)
{
    System_init();
    Board_init();
<<<<<<< HEAD
>>>>>>> 384dbf58d685f1104c5529d026482b7f4d8de3e0
=======
>>>>>>> 9a53e54c3529d50eb1ce4fb9934b17e6a50afc90
    while (1)
    {
        //task stays here forever
    }
}