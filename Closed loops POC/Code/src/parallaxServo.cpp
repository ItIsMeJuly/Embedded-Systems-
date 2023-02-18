#include "parallaxServo.h"

#define SCALING 10

ParallaxServo::ParallaxServo(TIM_TypeDef *outTimer, uint32_t PSC, uint32_t ARR, int maxRotation, int tMin, int tMax)
    : outputTimer(outTimer), PSC(PSC), ARR(ARR), maxRotation(maxRotation), tMin(tMin), tMax(tMax)
{
}

ParallaxServo::~ParallaxServo()
{
  delete outputTimer;
  outputTimer = nullptr;

  delete inputTimer;
  inputTimer = nullptr;
}

void ParallaxServo::StartServo()
{
  outputPwm->StartTimer();
  inputPwm->StartReading();
}

void ParallaxServo::StopServo()
{
  outputPwm->StopTimer();
  inputPwm->StopReading();
}

void ParallaxServo::InitialisePwmOutputLowerRegister(uint8_t outputRccBits)
{
  outputPwm = new Timer(outputTimer, PSC, ARR);
  outputPwm->InitialiseTimerLowRegister(outputRccBits);

  outputTimer->CCMR1 = (outputTimer->CCMR1 & ~TIM_CCMR1_CC1S_Msk) | (0b00 << TIM_CCMR1_CC1S_Pos);
  outputTimer->CCMR1 = (outputTimer->CCMR1 & ~TIM_CCMR1_OC1M_Msk) | (0b0110 << TIM_CCMR1_OC1M_Pos);
  outputTimer->CCER |= TIM_CCER_CC1E;
}

void ParallaxServo::InitialisePwmOutputHighRegister(uint8_t outputRccBits)
{
  outputPwm = new Timer(outputTimer, PSC, ARR);
  outputPwm->InitialiseTimerHighRegister(outputRccBits);

  outputTimer->CCMR1 = (outputTimer->CCMR1 & ~TIM_CCMR1_CC1S_Msk) | (0b00 << TIM_CCMR1_CC1S_Pos);
  outputTimer->CCMR1 = (outputTimer->CCMR1 & ~TIM_CCMR1_OC1M_Msk) | (0b0110 << TIM_CCMR1_OC1M_Pos);
  outputTimer->CCER |= TIM_CCER_CC1E;
}

void ParallaxServo::InitialisePwmInput(TIM_TypeDef *inTimer, uint8_t inputRccBits)
{
  this->inputTimer = inTimer;
  inputPwm = new PwmInput(inTimer, inputRccBits, PSC);
}

int ParallaxServo::CalculateAngle(float minValue, int maxValue)
{
  float tHigh = inputTimer->CCR2;
  float tCycle = inputTimer->CCR1;

  float min = CalculateDutyCycle(tCycle, minValue) * 10u;
  float max = CalculateDutyCycle(tCycle, maxValue) * 10u;
  float dc = CalculateDutyCycle(tCycle, tHigh) * 10u;

  float theta = (maxRotation - 1) - ((dc - min) * maxRotation) / (max - min + 1);

  if (theta < 0)
  {
    theta = 0;
  }
  else if (theta > (maxRotation - 1))
  {
    theta = maxRotation - 1;
  }

  return theta;
}

void ParallaxServo::SetPosition(float angle)
{
  int sensorValue = CalculateAngle(tMin, tMax);
  int value = MapAngle(angle);
  error = value - sensorValue;
  integral = integral + error;
  if (error == 0 || error > angle)
  {
    integral = 0;
  }

  if (error <= 20)
  {
    integral = 0;
  }
  derivative = error - prevError;
  prevError = error;
  power = error * kP + integral * kI + derivative * kD;

  if (power > 220)
  {
    power = 220;
  }

  if (power < -220)
  {
    power = -220;
  }

  int offset = 0;
  if (power > 0)
  {
    offset = 45;
  }
  else if (power < 0)
  {
    offset = -45;
  }

  outputTimer->CCR1 = 1500 + power + offset;
}

int ParallaxServo::MapAngle(int value)
{
  int angle = 180 + -value;
  return angle;
}

float ParallaxServo::CalculateDutyCycle(float tCycle, float value)
{
  float dutyCycle = (value / tCycle) * 100;
  return dutyCycle;
}