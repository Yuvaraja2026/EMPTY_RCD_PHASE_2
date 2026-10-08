#include "ti_drivers_config.h"     
#include "ti_board_config.h"   

int main(void)
{
    System_init();
    Board_init();
    while (1)
    {
        //task stays here forever
    }
}