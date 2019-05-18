#include <Servo.h>

const int SERVO_LEFT = D1;
const int SERVO_RIGHT = D2;

void drive(int leftPWM, int rightPWM, Servo& left, Servo& right);
