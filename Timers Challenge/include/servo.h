#ifndef SERVO_H
#define SERVO_H

#include "stm32f303xe.h"
#include "timer.h"

class Servo
{
private:
    Timer *timerPtr = nullptr;
    TIM_TypeDef *TIM = nullptr;
    float maxRotation = 0;
    uint32_t PSC;
    uint32_t ARR;

public:
    Servo(TIM_TypeDef *timer, uint32_t RCC_mask, uint32_t RCC_pos, uint8_t RCC_bits, uint32_t PSC, uint32_t ARR, int maxRotation);
    ~Servo();
    void StartServo();
    void StopServo();
    void SetPosition(float angle);
};

#endif