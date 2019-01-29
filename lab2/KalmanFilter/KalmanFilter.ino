#include <BasicLinearAlgebra.h>
#include <math.h>
#include "Sensor.h"
VL53L0X sensor, sensor2;

//State, Output Vectors
BLA::Matrix<3> q = {0,250,375};
BLA::Matrix<3> z = {350,200,0};
BLA::Matrix<3> q_est;
BLA::Matrix<3> z_est;

//Noise Vectors
BLA::Matrix<3> w;

//Kalman Filter Matrices
BLA::Matrix<3,3> F;
BLA::Matrix<3,3> S;
BLA::Matrix<3,3> Q;
BLA::Matrix<3,3> P;
BLA::Matrix<3,3> K;
BLA::Matrix<3,3> R;
BLA::Matrix<3,3> H;

BLA::Matrix<3,3> I3 = {1,0,0,0,1,0,0,0,1};

const float L = 750;
const float W = 500;
const float sigmaR = .000115;//m/s
const float sigmaL = .0000538;//m/s
const float sumVar = sigmaR * sigmaR + sigmaL * sigmaL;
const float diffVar = sigmaR * sigmaR - sigmaL * sigmaL;
const float sigmaAngle = 15.3;//deg
const float sigmaFLaser =  3.91;//mm
const float sigmaSLaser =  5.68;//mm
const float b = 0.094; //meters
const int FRONT = 0;
const int SIDE = 1;
float vL, vR, vT, wAng;

//Sensor Readings, temporary
float dF, dL, hMag;

// Maze: LB, LT, RT, RBottom;
const float px[4] = {0,0,W,W};
const float py[4] = {0,L,L,0};
int wall_f = - 1; 
int wall_s = -1;

// Miscellaneous equations for finding H i.e. linearized equations
float eq(int num) 
{
  float theta = q(0);
  float x = q(1);
  float y = q(2);
  // Adjust theta depending on which wall facing:
  wall_f = det_wall(FRONT);
  switch(wall_f)
  {
    case 0: break;
    case 1: theta = theta - 90; break;
    case 2: theta = theta - 180; break;
    case 3: theta = theta - 270; break;
  }
  switch(num)
  {
    case 1:
      return (L-y)* sin(theta * (180 / PI))/pow(cos(theta* (180 / PI)),2); break;
    case 2:
      return -1/(cos(theta* (180 / PI))); break;
    case 3:
      return (-y)* cos(theta* (180 / PI))/pow(sin(theta* (180 / PI)),2); break;
    case 4:
      return 1/(sin(theta)); break;
    case 5:
      return (W - x)*sin(theta* (180 / PI))/pow(cos(theta* (180 / PI)),2); break;
    case 6:
      return (y-L)*cos(theta* (180 / PI))/pow(sin(theta* (180 / PI)),2);  break;
    case 7:
      return -1/sin(theta* (180 / PI)); break;
    case 8:
      return -x * cos(theta* (180 / PI))/pow(sin(theta* (180 / PI)),2); break;
    case 9:
      return sin(theta* (180 / PI))/pow(cos(theta* (180 / PI)),2) * y; break;
    case 10:
      return 1/(cos(theta* (180 / PI))); break;
    case 11:
      return cos(theta* (180 / PI))/pow(sin(theta* (180 / PI)),2)*(x - W); break;
    case 12:
      return -x*sin(theta* (180 / PI))/pow((cos(theta* (180 / PI))),2); break;
  }
}

void update_H(float dt)
{
  // Determine walls:
  wall_f = det_wall(FRONT);
  wall_s = det_wall(SIDE);
  // 
  if (wall_f == 0 && wall_s == 2)
  {
    H  << 1,0,0,
           eq(1), 0, eq(2),
           eq(3), 0, eq(4);}
  else if (wall_f == 0 && wall_s == 1){
    H  << 1,0,0,
           eq(1), 0, eq(2),
           eq(5), eq(2), 0;}
  else if (wall_f == 0 && wall_s == 0){
    H  << 1,0,0,
           eq(1), 0, eq(2),
           eq(6), 0, eq(7);}
  else if (wall_f == 1 && wall_s == 3){
    H  << 1,0,0,
           eq(5), eq(2), 0,
           eq(8), eq(4), 0;}
  else if (wall_f == 1 && wall_s == 2){
    H  << 1,0,0,
           eq(5), eq(2), 0,
           eq(9), 0, eq(10);}
  else if (wall_f == 1 && wall_s == 1){
    H  << 1,0,0,
           eq(5), eq(2), 0,
           eq(11), eq(7), 0;}
  else if (wall_f == 2 && wall_s == 0){
    H  << 1,0,0,
           eq(9), 0, eq(10),
           eq(6), 0, eq(7);}
  else if (wall_f == 2 && wall_s == 3){
    H  << 1,0,0,
           eq(9), 0, eq(10),
           eq(11), eq(10), 0;}
  else if (wall_f == 2 && wall_s == 2){
    H  << 1,0,0,
           eq(9), 0, eq(10),
           eq(3), 0, eq(4);}
  else if (wall_f == 3 && wall_s == 1){
    H  << 1,0,0,
           eq(12), eq(10), 0,
           eq(11), eq(7), 0;}
  else if (wall_f == 3 && wall_s == 0){
    H  << 1,0,0,
           eq(12), eq(10), 0,
           eq(1), 0, eq(2);}
  else if (wall_f == 3 && wall_s == 3){
    H  << 1,0,0,
           eq(12), eq(10), 0,
           eq(8), eq(4), 0;}
  return;
}

// Input: given state
// Output: determine wall wall_f, wall_s
// Argument: 
//   int sensorType: 0 - front sensor
//      1 - right sensor
int det_wall(int sensorType)
{
  // TODO: measure sensors from middle point of vehicle to account for displacement
  float theta = q(0);
  float x = q(1);
  float y = q(2);

  if (sensorType == 1)
    theta += 90;
  if (theta > 360)
    theta -= 360;

  // N = 0, E = 1, S = 3, W = 4;
  // Find distances to each corner:
  float z[4];
  // distances to corners, px's are coordinates of corners
  for (int i =0; i <4; i++)
  {
    z[i] = sqrt(pow((px[i] - x),2) + pow((py[i] - y),2));
  }
  // Angles for possible 8 triangular view:
  // theta with respect to true North
  float thetas[8];
  // starting from North line rotating clockwise, total of eight divisions
  thetas[0]= acos((L-y)/z[2]) * 180 / PI;
  thetas[1]= acos((W-x)/z[2]) * 180 / PI;
  thetas[2]= acos((W-x)/z[3]) * 180 / PI;
  thetas[3]= acos(y/z[3]) * 180 / PI;
  thetas[4]= acos(y/z[0]) * 180 / PI;
  thetas[5]= acos(x/z[0]) * 180 / PI;
  thetas[6]= acos(x/z[1]) * 180 / PI;
  thetas[7]= acos((L-y)/z[1]) * 180 / PI;

  int n_sections = 8;
  // Range should be between 0-360
  // And check which walls the robot is pointing at:
  // cumulative theta 
  for (int i = 0; i < n_sections; i++) {
  if (i == 0) {
    if (theta < thetas[i]) {
      return 0;
    }
    continue;
  }
  thetas[i] = thetas[i-1] + thetas[i];
  if (theta < thetas[i]) {
    int w = (i+1)/2;
    if (w == 4)
            return 0;
    return w;
  }
  }
}

void getVelocities(float pwmR, float pwmL, float& vR, float& vL, float &vT, float& wAng)
{
  vR = -.140 * tanh(-0.048 * (pwmR - 91.8));
  vL = .139 * tanh(-0.047 * (pwmL - 92.6));
  vT = .5 * (vL + vR);
  wAng = 1/b * (vR-vL);
}

void update_F(float dt)
{
  F << 1, 0, 0,
       -vT * sin(q(0)) * dt, 1, 0,
       vT * cos(q(0)) * dt, 1, 0;
}

void update_Q(float dt)
{
  Q << pow((dt/b),2)*sumVar, pow(dt,2)/(2*b)*cos(q_est(0))*diffVar, pow(dt,2)/(2*b)*sin(q_est(0))*diffVar,
       pow(dt,2)/(2*b)*cos(q_est(0))*diffVar, pow(cos(q_est(0)),2) * pow(dt,2)/4*sumVar, sin(q_est(0))* cos(q_est(0)) * pow(dt,2)/4*sumVar,
       pow(dt,2)/(2*b)*sin(q_est(0))*diffVar, sin(q_est(0))* cos(q_est(0)) * pow(dt,2)/4*sumVar, pow(sin(q_est(0)),2) * pow(dt,2)/4*sumVar;
  return;
}

void aPrioriUpdate(float dt)
{
  //get q^ estimate
  q_est(0) = q_est(0) + (wAng * dt);
  q_est(1) = vT * cos(q(0));
  q_est(2) = vT * sin(q(0));

  //P update
  update_F(dt);
  update_Q(dt);
  P = ((F * P) * (~F)) + Q;
  
}

void aPosterioriUpdate(float dt)
{
  outputEstimate(z_est, q_est);
  update_H(dt);
  BLA::Matrix<3> innovation = z - z_est;
  S = ((H * P) * (~H)) + R; //innovation covariance
  K = (P * (~H))*(S.Inverse()); //Kalman Gain
  q_est += (K * innovation); //A Posteriori State Estimate
  P = (I3 - (K*H))*P; //Update Covariance Estimate  
  
}

void outputEstimate(BLA::Matrix<3>& z_est, BLA::Matrix<3>& q_est)
{
    // Determine walls:
  int wall_f = det_wall(0);
  int wall_s = det_wall(1);
  // 
  float theta = q_est(0);
  float x = q_est(1);
  float y = q_est(2);
  // Adjust theta depending on which wall facing:
  switch (wall_f)
  {
    case 0: break;
    case 1: theta = theta - 90; break;
    case 2: theta = theta - 180; break;
    case 3: theta = theta - 270; break;
  }

  if (wall_f == 0 && wall_s == 2) {
    z_est  << q_est(0),(L - y)/cos(theta*(180/PI)),
           y/(sin(theta*(180/PI)));}
  else if (wall_f == 0 && wall_s == 1){
    z_est << q_est(0),(L - y)/cos(theta*(180/PI)),
           (W-x)/(cos(theta*(180/PI)));}
  else if (wall_f == 0 && wall_s == 0) {
    z_est << q_est(0),(L - y)/cos(theta*(180/PI)),
           (L - y)/(sin(theta*(180/PI)));}
  else if (wall_f == 1 && wall_s == 3) {
    z_est << q_est(0),(W - x)/cos(theta*(180/PI)),
           x/(sin(theta*(180/PI)));}
  else if (wall_f == 1 && wall_s == 2) {
    z_est << q_est(0),(W - x)/cos(theta*(180/PI)),
           y/(cos(theta*(180/PI)));}
  else if (wall_f == 1 && wall_s == 1) {
    z_est << q_est(0),(W - x)/cos(theta*(180/PI)),
           (W-x)/(sin(theta*(180/PI)));}
  else if (wall_f == 2 && wall_s == 0) {
    z_est << q_est(0), y/cos(theta*(180/PI)),
           (L - y)/(sin(theta*(180/PI)));}
  else if (wall_f == 2 && wall_s == 3) {
    z_est << q_est(0),y/cos(theta*(180/PI)),
           x/(cos(theta*(180/PI)));}
  else if (wall_f == 2 && wall_s == 2) {
    z_est << q_est(0),y/cos(theta*(180/PI)),
           y/(sin(theta*(180/PI)));}
  else if (wall_f == 3 && wall_s == 1) {
    z_est << q_est(0), x/cos(theta*(180/PI)),
           (W-x)/(sin(theta*(180/PI)));}
  else if (wall_f == 3 && wall_s == 0) {
    z_est << q_est(0),x/cos(theta*(180/PI)),
           (L -y)/(cos(theta*(180/PI)));}
  else if (wall_f == 3 && wall_s == 3){
    z_est << q_est(0),x/cos(theta*(180/PI)),
           x/(sin(theta*(180/PI)));}
  BLA::Matrix <3> offset;
  offset<< 0,25,30;
  z_est -= offset;
  return;
}

void setup() {
  // put your setup code here, to run once:

  //Initialize P to I
  P << 1,0,0,
       0,1,0,
       0,0,1;
       
  //Need to initialize Matrices here (Q, R)
  R << pow(sigmaAngle, 2), 0, 0,
       0, pow(sigmaFLaser, 2), 0,
       0, 0, pow(sigmaSLaser, 2);

  Q << 1,0,0,0,1,0,0,0,1;

  q_est = q;
  z_est = z;
  
  Serial.begin (115200);

  //Setup Distance Sensors
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  digitalWrite(D7, LOW);
  digitalWrite(D8, LOW);

  delay(500);
  Wire.begin(SDA_PORT,SCL_PORT);

  digitalWrite(D3, HIGH);
  delay(150);
  Serial.println("00");
  
  sensor.init(true);
  Serial.println("01");
  delay(100);
  sensor.setAddress((uint8_t)22);

  digitalWrite(D4, HIGH);
  delay(150);
  sensor2.init(true);
  Serial.println("03");
  delay(100);
  sensor2.setAddress((uint8_t)25);
  Serial.println("04");

  Serial.println("addresses set");
  
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);

  delay(3000);
  
  setupIMU();
}

int t0 = 0;

void loop() {
  int tF = millis()/1000;
  float dt = tF - t0;
  t0 = tF;
  // put your main code here, to run repeatedly:
  int pwmR = 90;
  int pwmL = 90 ;
  float gz, head, fDist, sDist;
  ReadIMU(gz, head);
  wall_f = det_wall(FRONT);
  wall_s = det_wall(SIDE);
  update_H(dt);
  outputEstimate(z_est, q_est);
  q_est(0) += 1;
  
  Serial.print(" Z: ");
  Serial.print(z_est(0)); Serial.print(" ");
  Serial.print(z_est(1)); Serial.print(" ");
  Serial.print(z_est(2)); Serial.print("\n");

  /*
  Serial.print("Heading: ");
  Serial.print(head);
  Serial.print(" Distance: ");
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.print(" ");
  Serial.print(sensor2.readRangeSingleMillimeters());
  if (sensor2.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  */
  getVelocities(pwmR, pwmL, vL, vR, vT, wAng);
  delay(10);
}
