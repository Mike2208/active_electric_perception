/*******************************************************************************
  ADCHS Peripheral Library Interface Source File

  Company
    Microchip Technology Inc.

  File Name
    plib_adchs.c

  Summary
    ADCHS peripheral library source.

  Description
    This file implements the ADCHS peripheral library.

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/
// DOM-IGNORE-END
#include "device.h"
#include "plib_adchs.h"

#define ADCHS_CHANNEL_32  (32U)

// *****************************************************************************
// *****************************************************************************
// Section: ADCHS Implementation
// *****************************************************************************
// *****************************************************************************


/* Object to hold callback function and context for ADC DMA interrupt*/
ADCHS_DMA_CALLBACK_OBJECT ADCHS_DMACallbackObj;


void ADCHS_Initialize()
{
    ADCCON1bits.ON = 0;
ADC0CFG = DEVADC0;
ADC0TIME = 0x3810064;
ADC1CFG = DEVADC1;
ADC1TIME = 0x3810064;
ADC2CFG = DEVADC2;
ADC2TIME = 0x3810064;
ADC3CFG = DEVADC3;
ADC3TIME = 0x3810064;
ADC4CFG = DEVADC4;
ADC4TIME = 0x3810064;
ADC5CFG = DEVADC5;
ADC5TIME = 0x3810064;

    ADCCON1 = 0x600007;
    ADCCON2 = 0x0;
    ADCCON3 = 0xc0000000;

    ADCTRGMODE = 0xea00000;

    ADCTRG1 = 0x15151515; 
    ADCTRG2 = 0x1515; 
    ADCTRG3 = 0x0; 
    ADCTRG4 = 0x0; 
    ADCTRG5 = 0x0; 
    
    ADCTRG7 = 0x0; 

    ADCTRGSNS = 0x0;

    ADCIMCON1 = 0x0;
    ADCIMCON2 = 0x0; 
    ADCIMCON3 = 0x0; 
    ADCIMCON4 = 0x0; 

    /* Input scan */
    ADCCSS1 = 0x0;
    ADCCSS2 = 0x0; 

ADCDSTAT = 0x90009000;



    IEC3SET = _IEC3_AD1FCBTIE_MASK;



    /* Turn ON ADC */
    ADCCON1bits.ON = 1;
    while(!ADCCON2bits.BGVRRDY); // Wait until the reference voltage is ready
    while(ADCCON2bits.REFFLT); // Wait if there is a fault with the reference voltage

    /* ADC 0 */
    ADCANCONbits.ANEN0 = 1;      // Enable the clock to analog bias
    while(!ADCANCONbits.WKRDY0); // Wait until ADC is ready
    ADCCON3bits.DIGEN0 = 1;      // Enable ADC

    /* ADC 1 */
    ADCANCONbits.ANEN1 = 1;      // Enable the clock to analog bias
    while(!ADCANCONbits.WKRDY1); // Wait until ADC is ready
    ADCCON3bits.DIGEN1 = 1;      // Enable ADC

    /* ADC 2 */
    ADCANCONbits.ANEN2 = 1;      // Enable the clock to analog bias
    while(!ADCANCONbits.WKRDY2); // Wait until ADC is ready
    ADCCON3bits.DIGEN2 = 1;      // Enable ADC

    /* ADC 3 */
    ADCANCONbits.ANEN3 = 1;      // Enable the clock to analog bias
    while(!ADCANCONbits.WKRDY3); // Wait until ADC is ready
    ADCCON3bits.DIGEN3 = 1;      // Enable ADC

    /* ADC 4 */
    ADCANCONbits.ANEN4 = 1;      // Enable the clock to analog bias
    while(!ADCANCONbits.WKRDY4); // Wait until ADC is ready
    ADCCON3bits.DIGEN4 = 1;      // Enable ADC

    /* ADC 5 */
    ADCANCONbits.ANEN5 = 1;      // Enable the clock to analog bias
    while(!ADCANCONbits.WKRDY5); // Wait until ADC is ready
    ADCCON3bits.DIGEN5 = 1;      // Enable ADC



}


/* Enable ADCHS channels */
void ADCHS_ModulesEnable (ADCHS_MODULE_MASK moduleMask)
{
    ADCCON3 |= (moduleMask << 16);
}

/* Disable ADCHS channels */
void ADCHS_ModulesDisable (ADCHS_MODULE_MASK moduleMask)
{
    ADCCON3 &= ~(moduleMask << 16);
}


void ADCHS_ChannelResultInterruptEnable (ADCHS_CHANNEL_NUM channel)
{
    if (channel < ADCHS_CHANNEL_32)
    {
        ADCGIRQEN1 |= 0x01 << channel;
    }
    else
    {
        ADCGIRQEN2 |= 0x01 << (channel - 32);
    }
}

void ADCHS_ChannelResultInterruptDisable (ADCHS_CHANNEL_NUM channel)
{
    if (channel < ADCHS_CHANNEL_32)
    {
        ADCGIRQEN1 &= ~(0x01 << channel);
    }
    else
    {
        ADCGIRQEN2 &= ~(0x01 << (channel - 32));
    }
}

void ADCHS_ChannelEarlyInterruptEnable (ADCHS_CHANNEL_NUM channel)
{
    if (channel < ADCHS_CHANNEL_32)
    {
        ADCEIEN1 |= (0x01 << channel);
    }
    else
    {
        ADCEIEN2 |= (0x01 << (channel - 32));
    }
}

void ADCHS_ChannelEarlyInterruptDisable (ADCHS_CHANNEL_NUM channel)
{
    if (channel < ADCHS_CHANNEL_32)
    {
        ADCEIEN1 &= ~(0x01 << channel);
    }
    else
    {
        ADCEIEN2 &= ~(0x01 << (channel - 32));
    }
}

void ADCHS_GlobalEdgeConversionStart(void)
{
    ADCCON3bits.GSWTRG = 1;
}

void ADCHS_GlobalLevelConversionStart(void)
{
    ADCCON3bits.GLSWTRG = 1;
}

void ADCHS_GlobalLevelConversionStop(void)
{
    ADCCON3bits.GLSWTRG = 0;
}

void ADCHS_ChannelConversionStart(ADCHS_CHANNEL_NUM channel)
{
    ADCCON3bits.ADINSEL = channel;
    ADCCON3bits.RQCNVRT = 1;
}


/*Check if conversion result is available */
bool ADCHS_ChannelResultIsReady(ADCHS_CHANNEL_NUM channel)
{
    bool status = false;
    if (channel < ADCHS_CHANNEL_32)
    {
        status = (ADCDSTAT1 >> channel) & 0x01;
    }
    else
    {
        status = (ADCDSTAT2 >> (channel - 32)) & 0x01;
    }
    return status;
}

/* Read the conversion result */
uint16_t ADCHS_ChannelResultGet(ADCHS_CHANNEL_NUM channel)
{
    return (uint16_t) (*((&ADCDATA0) + (channel << 2)));

}


void ADCHS_DMASampleCountBaseAddrSet(uint32_t baseAddr)
{
    ADCCNTB = baseAddr;
}

void ADCHS_DMAResultBaseAddrSet(uint32_t baseAddr)
{
    ADCDMAB = baseAddr;
}


void ADCHS_DMACallbackRegister(ADCHS_DMA_CALLBACK callback, uintptr_t context)
{
    ADCHS_DMACallbackObj.callback_fn = callback;
    ADCHS_DMACallbackObj.context = context;
}

void ADC_DMA_InterruptHandler(void)
{
    ADCHS_DMA_STATUS dmaStatus = ADCDSTAT & 0xBF003F;

    IFS3CLR = _IFS3_AD1FCBTIF_MASK;

    if (ADCHS_DMACallbackObj.callback_fn != NULL)
    {
      ADCHS_DMACallbackObj.callback_fn(dmaStatus, ADCHS_DMACallbackObj.context);
    }
}




bool ADCHS_EOSStatusGet(void)
{
    return (bool)(ADCCON2bits.EOSRDY);
}


