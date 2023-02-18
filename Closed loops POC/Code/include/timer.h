#ifndef TIMER_H
#define TIMER_H

#include "stm32f303xe.h"

class Timer
{
private:
    uint32_t PSC = 0; 
    uint32_t ARR = 0;
    TIM_TypeDef *TIM = nullptr;

public:
    Timer(TIM_TypeDef *timer, uint32_t PSC, uint32_t ARR);
    ~Timer() = default;

    void InitialiseTimerHighRegister(uint8_t RCC_bits);
    void InitialiseTimerLowRegister(uint8_t RCC_bits);
    void SetTimerInterrupt(IRQn_Type interruptIRQ);
    void ResetTimerInterrupt();
    void StartTimer();
    void StopTimer();
};

#endif