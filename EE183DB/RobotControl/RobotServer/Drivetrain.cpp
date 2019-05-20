#include "Drivetrain.h"

void drive(int leftPWM, int rightPWM, Servo& left, Servo& right)
{
  left.write(leftPWM);
  right.write(rightPWM);
  return;
}
