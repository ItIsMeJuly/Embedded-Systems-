#include "servo.h"

Servo::Servo(TIM_TypeDef *timer, uint32_t RCC_mask, uint32_t RCC_pos, uint8_t RCC_bits, uint32_t PSC, uint32_t ARR, int maxRotation)
    : TIM(timer), maxRotation(maxRotation)
{
    timerPtr = new Timer(timer, RCC_mask, RCC_pos, RCC_bits, PSC, ARR);

    TIM->CCMR1 = (TIM->CCMR1 & ~TIM_CCMR1_CC1S_Msk) | (0b00 << TIM_CCMR1_CC1S_Pos);
    TIM->CCMR1 = (TIM->CCMR1 & ~TIM_CCMR1_OC1M_Msk) | (0b0110 << TIM_CCMR1_OC1M_Pos);
    TIM->CCER = (TIM->CCER & ~TIM_CCER_CC1E_Msk) | (TIM_CCER_CC1E << TIM_CCER_CC1E_Pos);
}

Servo::~Servo()
{
    delete timerPtr;
    timerPtr = nullptr;
}

void Servo::StartServo()
{
    timerPtr->StartTimer();
}

void Servo::StopServo()
{
    timerPtr->StopTimer();
}

void Servo::SetPosition(float angle)
{
    if (angle > maxRotation)
    {
        angle = maxRotation;
    }
    float value = (20 / maxRotation) * angle + 4.5;
    TIM->CCR1 = (int)value;
}