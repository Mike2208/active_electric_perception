#pragma once


#include "pwm_ctrl.h"
#include "definitions.h"


/*! \brief ADC PWM 7 Trigger */
#define AMP_ADC_PWM_TRIG_7  b10101

//DMA Buffer is split in segments, each segments contains sample from one the channels , AN0..AN5
//[AN0 SAMPLE 1,AN0 SAMPLE 2, ... ] ,  [AN1 SAMPLE 1,AN1 SAMPLE 2, ... ] , ...

#define DMA_SEG_COUNT 6		//Number of segments in DMA buffer (one for each channel)
#define DMA_SEG_LEN	  128	//Lenght of each segments in words (number of samples per channel, stored in DMA buffer at one time)
#define DMA_TOTAL_LEN (DMA_SEG_COUNT*DMA_SEG_LEN)	//=112 Total length of DMA buffer

// Declare extern here and define actual memory in amp_adc.cpp. Prevents multiple .cpp files from   allocating separate memory?
extern volatile uint16_t ADC_DMA[DMA_SEG_COUNT][2][DMA_SEG_LEN] __attribute__((coherent, aligned(2048)));

class AmpAdc
{
    public:
        /*!
            \brief Control ADC and corresponding triggering PWM
            \param pwm PWM module that triggers ADC conversion. Trigger and PWM should be set via Harmony
        */
        AmpAdc(PwmCtrl *pwm, uint8_t _channel_num);

        /*! \brief Copy ADCDSTAT to local memory. WARNING: This will clear error and warning bits stored in ADCSTAT. */
        static void CopyAdcdstat();

        /*! \brief Update DMA interrupt bits from local memory to ADCDSTAT. WARNING: This will clear error and warning bits stored in ADCSTAT. */
        void UpdateAdcdstatInterruptbits();

        static void EnableDma();
        static void DisableDma();

        bool IsBufferAFull(ADCHS_DMA_STATUS dma_status) const;
        volatile uint16_t *GetBufferA() const;

        bool IsBufferBFull(ADCHS_DMA_STATUS dma_status) const;
        volatile uint16_t *GetBufferB() const;

        void UnpauseAdcDma();
        void PauseAdcDma();

        void SetSamplingFrequency(float freq);
        float GetSamplingFrequency() const;

        uint8_t ChannelNum() const;

    private:
        /*! \brief Triggering PWM */
        PwmCtrl *const _pwm;

        /*! \brief ADC Channel number. Used to enable/disable adc interrupt. */
        const uint8_t _channel_num;

        /*! \brief Used to update ADCSTAT   */
        static uint32_t _adc_stat;
};