#include "pwm_ctrl.h"


#include <algorithm>
#include <iostream>
#include <cmath>


PwmCtrl::PwmCtrl(volatile uint32_t *phase, 
                 volatile uint32_t *pdc,
                 volatile uint32_t *iocon)
    : _pphase(phase),
      _ppdc(pdc),
      _piocon(reinterpret_cast<volatile IOCON_t*>(iocon))
{
    this->SetFrequency(PWM_DEFAULT_PHASE, 0.5);
}

void PwmCtrl::StartAllPWM()
{
    MCPWM_Start();
}

void PwmCtrl::StopAllPWM()
{
    MCPWM_Stop();
}

void PwmCtrl::PausePWM(bool active_out)
{
    this->_piocon->OVRDAT = active_out ? 0b11 : 0b00;

    this->_piocon->OVRENL = 1;
    this->_piocon->OVRENH = 1;
}

void PwmCtrl::UnpausePWM()
{
    this->_piocon->OVRENL = 0;
    this->_piocon->OVRENH = 0;
}

void PwmCtrl::SetFrequency(float freq, float pdc)
{
    // Calculate frequency. 
    // We've set up the PWM to use the PHASEx buffer to modifiy the frequency instead of the phase
    uint16_t phase = std::round(PWM_MULT / freq);
    phase = std::max((uint16_t)1, phase);

    // Set duty cycle
    uint16_t pdc_reg = std::min((uint16_t)std::round(pdc * phase), phase);
    
    *this->_pphase = phase;
    *this->_ppdc = pdc_reg;
}

float PwmCtrl::GetFrequency()
{
    return PWM_MULT / *this->_pphase;
}

void PwmCtrl::SetPhaseRegister(uint16_t phase)
{
    *this->_pphase = phase;
}

void PwmCtrl::SetPDCRegister(uint16_t pdc)
{
    *this->_ppdc = pdc;
}

uint16_t PwmCtrl::GetPhaseRegister() const
{
    return *this->_pphase;
}

uint16_t PwmCtrl::GetPDCRegister() const
{
    return *this->_ppdc;
}