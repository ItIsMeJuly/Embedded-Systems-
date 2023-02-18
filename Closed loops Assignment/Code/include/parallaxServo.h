#ifndef PARALLAXSERVO_H
#define PARALLAXSERVO_H

#include "timer.h"
#include "pwmInput.h"
#include "stm32f303xe.h"

class ParallaxServo
{
public:
    ParallaxServo(TIM_TypeDef *outTimer, uint32_t PSC, uint32_t ARR, int maxRotation, int tMin, int tMax);
    ~ParallaxServo();

    void InitialisePwmOutputLowerRegister(uint8_t outputRccBits);
    void InitialisePwmOutputHighRegister(uint8_t outputRccBits);
    void InitialisePwmInput(TIM_TypeDef *inTimer, uint8_t inputRccBits);
    int CalculateAngle(float minValue, int maxValue);
    void StartServo();
    void StopServo();
    void SetPosition(float angle);
    void SetKp(float p);
    void SetKi(float i);
    int GetError();
    int GetIntegral();

private:
    uint32_t PSC = 0; 
    uint32_t ARR = 0;
    int tMin = 0;
    int tMax = 0;
    int error = 0;
    int prevError = 0;
    int integral = 0;
    int power = 0;
    int derivative = 0;

    float kP = 1.526;
    float kI = 0.275;
    float kD = 0;

    int maxRotation = 0;

    Timer *outputPwm = nullptr;
    PwmInput *inputPwm = nullptr;
    TIM_TypeDef *outputTimer = nullptr;
    TIM_TypeDef *inputTimer = nullptr;

    int MapAngle(int value);
    float CalculateDutyCycle(float tCycle, float value);
};

#endif