#pragma once


#include "amp_adc.h"
#include "avg_sampling.h"
#include "lc_tank.h"


#include <array>
#include <vector>
#include <stdint.h>

/*! \brief Cap Sensor measurements. Records and sends sum of sensor measurements from all 6 ADC channels to PC via UART.
 *  NOTE: If MCU is too slow, increase AvgSampling::DATA_STEP to decrease processing time (I estimate that DATA_STEP = 16 should work well)
 */
class CapSensor
{
        struct UartRxCmdData
        {
            enum CmdType
            {
                ERROR = -1,
                SEND_MEASURES = 0,
                SET_FREQ,
                SET_FREQ_2,
                CALIBRATE
            };
            
            int32_t     Cmd = (int32_t)SEND_MEASURES;
            uint32_t    MeasuresToSend = DMA_SEG_LEN;
            float       ElectrodeFrequency = 100000;
            float       SampleFrequency = 4*100000;
            
            inline enum CmdType GetType() const
            {   return (CmdType)this->Cmd;   }
        };

        static constexpr uint8_t DATA_SAMPLES_PER_CYCLE_OFFSET = 0;
        static constexpr uint8_t DATA_PADDING0_OFFSET = DATA_SAMPLES_PER_CYCLE_OFFSET + sizeof(uint8_t);
        static constexpr uint8_t DATA_MEASURES_OFFSET = DATA_PADDING0_OFFSET + sizeof(uint8_t);
        static constexpr uint8_t DATA_PADDING1_OFFSET = DATA_MEASURES_OFFSET + sizeof(uint16_t);
        static constexpr uint8_t DATA_SAMPLES_OFFSET = DATA_PADDING1_OFFSET + sizeof(uint16_t)*6;

    public:
        static constexpr float   DEFAULT_ELECTRODE_FREQ    = 100000.0;
        static constexpr uint8_t DEFAULT_SAMPLES_PER_CYCLE = 4;

        CapSensor();
        //~CapSensor();

        void UnpauseSensing();
        void PauseSensing();

        /*! \brief Set Electrode PWM frequency.  */
        void SetElectrodeFrequency(float freq, float pdc = 0.5);

        /*! \brief Get Electrode frequency. */
        float GetElectrodeFrequency();

        /*! \brief Set all adc sampling frequencies. */
        void SetSamplingFrequencies(float freq, float pdc = 0.5);

        /*! \brief Get sampling frequency. */
        float GetSamplingFrequency();

        /*! \brief Set electrode and sampling frequencies correctly */
        float SetFrequenciesSampled(float electrode_freq, uint8_t samples_per_cycle, float pdc = 0.5);

        static void AdcDmaEventHandler(ADCHS_DMA_STATUS dmaStatus, uintptr_t context);
        static void UartDmaChannelHandler(DMAC_TRANSFER_EVENT event, uintptr_t context);

        bool ReadUARTComm();

    private:
        LcTank _lc_tank;
        PwmCtrl _adc_pwm;
        std::array<AmpAdc, 6> _amp_adcs;

        std::vector<uint8_t> _uart_comm_data;
        std::array<AvgSampling, 6> _avg_sampling;

        uint8_t _samples_per_cycle = DEFAULT_SAMPLES_PER_CYCLE;

        std::array<uint32_t, 6> _num_measures = { 0, 0, 0, 0, 0, 0 };

        UartRxCmdData _uart_rx_cmd;
        volatile bool _uart_tx_ready = true;

        volatile bool _adc_sending = false;

        void UpdateAvgSamplingSetup(uint8_t samples_per_cycle);
        
        static uint8_t UartDataLength(uint8_t samples_per_cycle)
        {   return DATA_SAMPLES_OFFSET + sizeof(uint32_t)*samples_per_cycle*6;  }
};