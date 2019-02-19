#include "Drivetrain.h"

void drive(int leftPWM, int rightPWM, int duration_ms, Servo& left, Servo& right)
{
  left.write(leftPWM);
  right.write(rightPWM);
  delay(duration_ms);
  left.write(90);
  right.write(90);
  return;
}
