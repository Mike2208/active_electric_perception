#include "cap_sensor.h"


#include <iostream>


CapSensor::CapSensor()
    : _lc_tank(&PHASE2, &PDC2, &IOCON2),
      _adc_pwm(&PHASE7, &PDC7, &IOCON7),
      _amp_adcs({
        AmpAdc(&_adc_pwm, 0),
        AmpAdc(&_adc_pwm, 1),
        AmpAdc(&_adc_pwm, 2),
        AmpAdc(&_adc_pwm, 3),
        AmpAdc(&_adc_pwm, 4),
        AmpAdc(&_adc_pwm, 5)})
{
    this->SetFrequenciesSampled(DEFAULT_ELECTRODE_FREQ, DEFAULT_SAMPLES_PER_CYCLE);
    this->UpdateAvgSamplingSetup(DEFAULT_SAMPLES_PER_CYCLE);
}

//CapSensor::~CapSensor()
//{
//    if(this->_uart_comm_data)
//    {
//        free(this->_uart_comm_data);
//        this->_uart_comm_data = nullptr;
//    }
//}

void CapSensor::UnpauseSensing()
{
    for(auto &adc : this->_amp_adcs)
    {
        adc.UnpauseAdcDma();
    }
}

void CapSensor::PauseSensing()
{
    for(auto &adc : this->_amp_adcs)
    {
        adc.PauseAdcDma();
    }
}

void CapSensor::SetElectrodeFrequency(float freq, float pdc)
{
    this->_lc_tank.SetFrequency(freq, pdc);
}

float CapSensor::GetElectrodeFrequency()
{
    return this->_lc_tank.GetFrequency();
}

void CapSensor::SetSamplingFrequencies(float freq, float pdc)
{
    this->_adc_pwm.SetFrequency(freq, pdc);
}

float CapSensor::GetSamplingFrequency()
{
    return this->_adc_pwm.GetFrequency();
}

float CapSensor::SetFrequenciesSampled(float electrode_freq, uint8_t samples_per_cycle, float pdc)
{
    this->_lc_tank.SetFrequency(electrode_freq, pdc);

    // Ensure freq is divisible by samples_per_cycle
    uint16_t phase = this->_lc_tank.GetPhaseRegister();
    const uint8_t delta_phase = phase % samples_per_cycle;
    if(delta_phase <= samples_per_cycle / 2)
        phase -= delta_phase;
    else
        phase += (samples_per_cycle-delta_phase);

    this->_lc_tank.SetPhaseRegister(phase);
    this->_lc_tank.SetPDCRegister(std::round(pdc*phase));

    this->_adc_pwm.SetPhaseRegister(phase/samples_per_cycle);
    this->_adc_pwm.SetPDCRegister(std::round(pdc*this->_adc_pwm.GetPhaseRegister()));
    
    this->_samples_per_cycle = samples_per_cycle;
    this->UpdateAvgSamplingSetup(this->_samples_per_cycle);

    return this->_lc_tank.GetFrequency();
}

void CapSensor::AdcDmaEventHandler(ADCHS_DMA_STATUS dmaStatus, uintptr_t context)
{
    CapSensor *const psensors = reinterpret_cast<CapSensor*>(context);

    if(dmaStatus & ADCHS_DMA_STATUS_WROVERR)
        std::cout << "Error\r\n";
    
    bool requested = false;
    uint8_t completed = true;

    for(uint8_t i=0; i < psensors->_amp_adcs.size(); ++i)
    {
        if(psensors->_num_measures[i] == 0)
            continue;
        
        requested = true;

        if(psensors->_avg_sampling[i].SampleCount() >= psensors->_num_measures[i])
            continue;
        
        completed = false;

        AmpAdc &amp_adc = psensors->_amp_adcs[i];

        if(amp_adc.IsBufferAFull(dmaStatus))
            psensors->_avg_sampling[i].UpdateSum((uint16_t*)amp_adc.GetBufferA());
        else if(amp_adc.IsBufferBFull(dmaStatus))
            psensors->_avg_sampling[i].UpdateSum((uint16_t*)amp_adc.GetBufferB());
    }

    if(psensors->_uart_tx_ready && 
       requested && 
       completed &&
       !psensors->_adc_sending)
    {
        psensors->_adc_sending = true;

        psensors->_uart_comm_data[DATA_SAMPLES_PER_CYCLE_OFFSET] = psensors->_samples_per_cycle;
        
        for(uint8_t i=0; i < 6; ++i)
        {
            *(uint16_t*)(&psensors->_uart_comm_data[DATA_MEASURES_OFFSET + i*sizeof(uint16_t)]) = psensors->_avg_sampling[i].SampleCount();
        }
        
        psensors->_uart_tx_ready = false;

        DMAC_ChannelTransfer(DMAC_CHANNEL_0, 
                            (const void *)psensors->_uart_comm_data.data(), psensors->_uart_comm_data.size(), 
                            (const void *)&U1TXREG, 1, 1);

        // Reset num measures
        for(uint8_t i=0; i < 6; ++i)
        {
            psensors->_num_measures[i] = 0;
        }
    }
}

void CapSensor::UartDmaChannelHandler(DMAC_TRANSFER_EVENT event, uintptr_t context)
{
    if (event == DMAC_TRANSFER_EVENT_COMPLETE)
    {
        CapSensor *const psensors = reinterpret_cast<CapSensor*>(context);

        for(auto &avg_sampling : psensors->_avg_sampling)
        {
            avg_sampling.ResetSum();
        }

        psensors->_uart_tx_ready = true;
        psensors->_adc_sending = false;
    }
}


bool CapSensor::ReadUARTComm()
{
    if(UART1_Read((void*)&this->_uart_rx_cmd, sizeof(this->_uart_rx_cmd)))
    {
        //this->_uart_tx_ready = true;
        
        switch(((const UartRxCmdData&)this->_uart_rx_cmd).GetType())
        {
            case UartRxCmdData::SEND_MEASURES:
                for(auto &num_measures : this->_num_measures)
                {   num_measures = this->_uart_rx_cmd.MeasuresToSend;    }
                break;
                
            case UartRxCmdData::SET_FREQ:
                while(!this->_uart_tx_ready){};
                
                AmpAdc::DisableDma();
                PwmCtrl::StopAllPWM();
                
                if(this->_uart_rx_cmd.ElectrodeFrequency >= 0.f)
                    this->SetElectrodeFrequency(this->_uart_rx_cmd.ElectrodeFrequency, 0.5);
                
                if(this->_uart_rx_cmd.SampleFrequency >= 0.f)
                    this->SetSamplingFrequencies(this->_uart_rx_cmd.SampleFrequency, 0.5);
                
                this->_uart_rx_cmd.ElectrodeFrequency = this->GetElectrodeFrequency();
                this->_uart_rx_cmd.SampleFrequency    = this->GetSamplingFrequency();
                
                UART1_Write((void*)&this->_uart_rx_cmd, sizeof(this->_uart_rx_cmd));
                
                PwmCtrl::StartAllPWM();
                AmpAdc::EnableDma();
                
                break;
                
            case UartRxCmdData::SET_FREQ_2:
                while(!this->_uart_tx_ready){};
                
                AmpAdc::DisableDma(); 
                PwmCtrl::StopAllPWM();
                
                if(this->_uart_rx_cmd.ElectrodeFrequency < 0)
                    this->_uart_rx_cmd.ElectrodeFrequency = this->_lc_tank.GetFrequency();

                const uint32_t samples_per_cycle = std::max((uint32_t)std::round(this->_uart_rx_cmd.SampleFrequency), (uint32_t)2);
                
                this->SetFrequenciesSampled(this->_uart_rx_cmd.ElectrodeFrequency, samples_per_cycle, 0.5);

                this->_uart_rx_cmd.ElectrodeFrequency = this->GetElectrodeFrequency();
                this->_uart_rx_cmd.SampleFrequency    = this->GetSamplingFrequency();
                
                UART1_Write((void*)&this->_uart_rx_cmd, sizeof(this->_uart_rx_cmd));
                
                PwmCtrl::StartAllPWM();
                AmpAdc::EnableDma();
                
                break;
        }

        return true;
    }

    return false;
}

void CapSensor::UpdateAvgSamplingSetup(uint8_t samples_per_cycle)
{
    const uint32_t new_size = UartDataLength(samples_per_cycle);
    this->_uart_comm_data.resize(new_size);
    //this->_uart_comm_data = (uint8_t*)malloc(this->_uart_comm_data_length);
    
    for(uint8_t i=0; i < this->_avg_sampling.size(); ++i)
    {
        uint8_t *const pavg_data = &(this->_uart_comm_data.at(DATA_SAMPLES_OFFSET + i*samples_per_cycle*sizeof(uint32_t)));
        this->_avg_sampling[i] = AvgSampling(DMA_SEG_LEN, (uint32_t*)pavg_data, samples_per_cycle);
        this->_avg_sampling[i].ResetSum();
    }
}