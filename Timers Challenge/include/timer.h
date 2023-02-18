#ifndef TIMER_H
#define TIMER_H

#include "stm32f303xe.h"

class Timer
{
private:
    TIM_TypeDef *TIM = nullptr;

public:
    Timer(TIM_TypeDef *timer, uint32_t RCC_mask, uint32_t RCC_pos, uint8_t RCC_bits, uint32_t PSC, uint32_t ARR);
    ~Timer() = default;
    void StartTimer();
    void StopTimer();
};

#endif