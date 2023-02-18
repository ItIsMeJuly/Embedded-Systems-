#include "pwmInput.h"

PwmInput::PwmInput(TIM_TypeDef *timer, uint8_t RCC_bits, uint32_t PSC)
    : TIM(timer)
{
    timerPtr = new Timer(timer, PSC, 0xFFFF);
    timerPtr->InitialiseTimerLowRegister(RCC_bits);

    TIM->CCMR1 |= TIM_CCMR1_CC1S_0;
    TIM->CCER &= ~TIM_CCER_CC1NP;
    TIM->CCER &= ~TIM_CCER_CC1P;
    TIM->CCER |= TIM_CCER_CC1E;
    TIM->CCMR1 |= TIM_CCMR1_CC2S_1;
    TIM->CCER &= ~TIM_CCER_CC2NP;
    TIM->CCER |= TIM_CCER_CC2P;
    TIM->CCER |= TIM_CCER_CC2E;
    TIM->SMCR = (TIM->SMCR & ~TIM_SMCR_TS_Msk) | (SMCR_TS_FILTERED_TIMER_INPUT_1 << TIM_SMCR_TS_Pos);
    TIM->SMCR |= TIM_SMCR_SMS_2;
}

PwmInput::~PwmInput()
{
    delete timerPtr;
    timerPtr = nullptr;
}

void PwmInput::StartReading()
{
    timerPtr->StartTimer();
}

void PwmInput::StopReading()
{
    timerPtr->StopTimer();
}