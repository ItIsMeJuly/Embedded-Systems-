#include "timer.h"

Timer::Timer(TIM_TypeDef *timer, uint32_t PSC, uint32_t ARR)
    : TIM(timer), PSC(PSC), ARR(ARR)
{
}

void Timer::InitialiseTimerHighRegister(uint8_t RCC_bits)
{
    RCC->APB2ENR |= RCC_bits;
    TIM->PSC = PSC;
    TIM->ARR = ARR;
    TIM->BDTR |= TIM_BDTR_MOE;
}

void Timer::InitialiseTimerLowRegister(uint8_t RCC_bits)
{
    RCC->APB1ENR |= RCC_bits;
    TIM->PSC = PSC;
    TIM->ARR = ARR;
}

void Timer::SetTimerInterrupt(IRQn_Type interruptIRQ)
{
    NVIC_EnableIRQ(interruptIRQ);
    TIM->DIER |= TIM_DIER_UIE;
}

void Timer::ResetTimerInterrupt()
{
    TIM->SR = ~TIM_SR_UIF;
}

void Timer::StartTimer()
{
    TIM->CR1 |= TIM_CR1_CEN;
}

void Timer::StopTimer()
{
    TIM->CR1 &= ~TIM_CR1_CEN;
}