#pragma once


#include <cmath>
#include <stdint.h>
#include "definitions.h"


#define PWM_PHASE   PHASE2
#define PWM_PDC     PDC2
#define PWM_IOCON   IOCON2

#define PWM_MAX_FREQ    120000000
#define PWM_PRESCALER   1


class PwmCtrl
{
        /*! \brief PWM Multiplier to convert frequency to PHASE register value. The x2 multiplier is required. */
        static constexpr float    PWM_MULT = ((float)PWM_MAX_FREQ / PWM_PRESCALER);
        static constexpr uint16_t PWM_DEFAULT_PHASE = std::round(PWM_MULT/100000);

        /*! \brief  IOCON register, taken from documentation (https://ww1.microchip.com/downloads/en/DeviceDoc/Section-44-Motor-Control-PWM-(MCPWM)-DS60001393B.pdf#G1.1115424). */
        struct IOCON_t 
        {
            uint32_t OSYNC:1;
            uint32_t SWAP:1;
            uint32_t CLDAT:2;
            uint32_t FLTDAT:2;
            uint32_t OVRDAT:2;
            uint32_t OVRENL:1;
            uint32_t OVRENH:1;
            uint32_t PMOD:2;
            uint32_t POLL:1;
            uint32_t POLH:1;
            uint32_t PENL:1;
            uint32_t PENH:1;
            uint32_t FLTMOD:2;
            uint32_t FLTPOL:1;
            uint32_t FLTSRC:4;
            uint32_t :1;
            uint32_t CLMOD:1;
            uint32_t CLPOL:1;
            uint32_t CLSRC:4;
        };

    public:
        PwmCtrl(volatile uint32_t *phase, 
                volatile uint32_t *pdc,
                volatile uint32_t *iocon);
    
        static void StartAllPWM();
        static void StopAllPWM();

        void UnpausePWM();
        void PausePWM(bool active_out = true);

        void SetFrequency(float freq, float pdc = 0.5);
        float GetFrequency();

        void SetPhaseRegister(uint16_t phase);
        void SetPDCRegister(uint16_t pdc);

        uint16_t GetPhaseRegister() const;
        uint16_t GetPDCRegister() const;

    private:
        /*! \brief  Phase register, set to control frequency.   */
        volatile uint32_t *const _pphase;

        /*! \brief  Primary duty cycle register   */
        volatile uint32_t *const _ppdc;

        /*! \brief  IOCON register. Used to enable/disable PWM output.   */
        volatile IOCON_t *const _piocon;
};