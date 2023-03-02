#include "avg_sampling.h"


#include <cmath>
#include <cstring>
#include <memory>


AvgSampling::AvgSampling(uint8_t data_length, 
                         uint32_t *sample_data, uint8_t samples_per_cycle)
    : _data_length(data_length),
      _sample_data(sample_data),
      _samples_per_cycle(samples_per_cycle),
      _cycles_per_data(std::round((float)_data_length/(DATA_STEP == 1 ? _samples_per_cycle : DATA_STEP))),
      _sample_count(0)
{}

AvgSampling::AvgSampling()
    : AvgSampling(0, nullptr, 0)
{}

void AvgSampling::ResetSum()
{
    this->_sample_count = 0;
    std::memset(this->_sample_data, 0, sizeof(uint32_t)*this->_samples_per_cycle);
}

void AvgSampling::UpdateSum(uint16_t *data)
{
    this->_sample_count += this->_cycles_per_data;

    if constexpr (DATA_STEP == 1)
    {
        for(uint8_t i = 0; i < this->_data_length; ++i)
        {
            this->_sample_data[i%this->_samples_per_cycle] += data[i];  
        }
    }
    else
    {
        for(uint8_t i = 0; i < this->_data_length; i+=DATA_STEP)
        {
            for(uint8_t j=0; j < this->_samples_per_cycle; ++j)
            {
                this->_sample_data[j] += data[i+j];
            }
        }
    }
}

const uint32_t *AvgSampling::SampleData() const
{
    return this->_sample_data;
}

uint8_t AvgSampling::SamplesPerCycle() const
{
    return this->_samples_per_cycle;
}

uint16_t AvgSampling::SampleCount() const
{
    return this->_sample_count;
}