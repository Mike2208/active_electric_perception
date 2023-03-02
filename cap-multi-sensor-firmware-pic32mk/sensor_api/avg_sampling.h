#pragma once


#include <cmath>
#include <stdint.h>


class AvgSampling
{
    /*! \brief Only evaluate every DATA_STEP times _samples_per_cycle of _data on update. Ensure that _data_length is divisible by DATA_STEP. Set to 1 to disable and use all available data*/
    static constexpr uint8_t DATA_STEP = 1;
    
    public:
        AvgSampling(uint8_t data_length, 
                    uint32_t *sample_data, uint8_t samples_per_cycle);

        AvgSampling();

        /*! \brief Reset sum. Clear _sample_data and set _sample_count to 0 */
        void ResetSum();

        /*! \brief Update _sample_data by adding from _data */
        void UpdateSum(uint16_t *data);

        const uint32_t *SampleData() const;
        uint8_t         SamplesPerCycle() const;

        uint16_t SampleCount() const;

    private:
        uint8_t _data_length;
        uint32_t *_sample_data;
        uint8_t _samples_per_cycle;

        uint8_t _cycles_per_data;
        
        /*! \brief Number of samples taken */
        uint16_t _sample_count = 0;
};