#include "timer.h"

Timer::Timer(TIM_TypeDef *timer, uint32_t RCC_mask, uint32_t RCC_pos, uint8_t RCC_bits, uint32_t PSC, uint32_t ARR)
    : TIM(timer)
{
    RCC->APB1ENR = (RCC->APB1ENR & ~RCC_mask) | (RCC_bits << RCC_pos);
    TIM->PSC = PSC;
    TIM->ARR = ARR;
}

void Timer::StartTimer()
{
    TIM->CR1 = (TIM->CR1 & ~TIM_CR1_CEN_Msk) | (TIM_CR1_CEN << TIM_CR1_CEN_Pos);
}

void Timer::StopTimer()
{
    TIM->CR1 = (TIM->CR1 & ~TIM_CR1_CEN_Msk) | (0x0 << TIM_CR1_CEN_Pos);
}