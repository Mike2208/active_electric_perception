#include "amp_adc.h"


volatile uint16_t ADC_DMA[DMA_SEG_COUNT][2][DMA_SEG_LEN] __attribute__((coherent, aligned(2048)));        //IMPORTANT: align data to a power of 2 >= DMA_TOTAL_LEN*2 
                                                                                                                //see http://ww1.microchip.com/downloads/en/DeviceDoc/70182b.pdf, page 22-31


AmpAdc::AmpAdc(PwmCtrl *pwm, uint8_t channel_num)
    : _pwm(pwm),
      _channel_num(channel_num)
{}

void AmpAdc::CopyAdcdstat()
{
    AmpAdc::_adc_stat = ADCDSTAT;
}

void AmpAdc::UpdateAdcdstatInterruptbits()
{
    // Mask to only update buffer A and B full interrupts
    constexpr uint32_t mask = (((uint32_t)0b111111) << 24) | (((uint32_t)0b111111) << 8);
    ADCDSTAT |= (this->_adc_stat & mask);
}

void AmpAdc::EnableDma()
{
    ADCDSTATbits.DMAEN = 1;
}

void AmpAdc::DisableDma()
{
    ADCDSTATbits.DMAEN = 0;
}

bool AmpAdc::IsBufferAFull(ADCHS_DMA_STATUS dma_status) const
{
    return dma_status & ((uint32_t)1 << this->_channel_num);
}

volatile uint16_t *AmpAdc::GetBufferA() const
{
    return ADC_DMA[this->_channel_num][0];
}

bool AmpAdc::IsBufferBFull(ADCHS_DMA_STATUS dma_status) const
{
    return dma_status & ((uint32_t)1 << (16 + this->_channel_num));
}

volatile uint16_t *AmpAdc::GetBufferB() const
{
    return ADC_DMA[this->_channel_num][1];
}

void AmpAdc::UnpauseAdcDma()
{
    // Enable DMA buffer A and B interrupts
    this->_adc_stat |= (uint32_t)1 << ( 8 + this->_channel_num);
    this->_adc_stat |= (uint32_t)1 << (24 + this->_channel_num);
}

void AmpAdc::PauseAdcDma()
{
    // Disable DMA buffer A and B interrupts
    this->_adc_stat &= (uint32_t)0 << ( 8 + this->_channel_num);
    this->_adc_stat &= (uint32_t)0 << (24 + this->_channel_num);
}

void AmpAdc::SetSamplingFrequency(float freq)
{
    this->_pwm->SetFrequency(freq);
}

float AmpAdc::GetSamplingFrequency() const
{
    return this->_pwm->GetFrequency();
}

uint8_t AmpAdc::ChannelNum() const
{
    return this->_channel_num;
}