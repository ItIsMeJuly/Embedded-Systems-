#ifndef PWMINPUT_H
#define PWMINPUT_H

#define SMCR_TS_FILTERED_TIMER_INPUT_1 0b0101

#include "timer.h"
#include "stm32f303xe.h"

class PwmInput
{
private:
    Timer *timerPtr = nullptr;
    TIM_TypeDef *TIM = nullptr;

public:
    PwmInput(TIM_TypeDef *timer, uint8_t RCC_bits, uint32_t PSC);
    ~PwmInput();
    void StartReading();
    void StopReading();
};

#endif