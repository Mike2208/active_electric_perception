/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes

#include "sensor_api/cap_sensor.h"


volatile bool led_stat = false;

static void tmr2EventHandler (uint32_t intCause, uintptr_t context)
{
    led_stat = !led_stat;
    
    if(led_stat)
        SLED1_Set();
    else
        SLED1_Clear();
}


int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    CapSensor cap_sensor;
    
    // Setup ADC DMA buffer
    AmpAdc::DisableDma();
    ADCHS_DMACallbackRegister(CapSensor::AdcDmaEventHandler, (uintptr_t)&cap_sensor);
    ADCHS_DMAResultBaseAddrSet(KVA_TO_PA(ADC_DMA));

    // Enable UART DMA
    DMAC_ChannelCallbackRegister(DMAC_CHANNEL_0, CapSensor::UartDmaChannelHandler, (uintptr_t)&cap_sensor);
    
    // Start timers
    TMR2_CallbackRegister(tmr2EventHandler, 0);
    TMR2_Start();

    SLED1_Clear();
    SLED2_Clear();
    
    PwmCtrl::StartAllPWM();
    AmpAdc::EnableDma();
    
    while ( true )
    {        
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
        
        if(!cap_sensor.ReadUARTComm())
            SLED2_Set();
    }

    /* Execution should not come here during normal operation */
    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/
