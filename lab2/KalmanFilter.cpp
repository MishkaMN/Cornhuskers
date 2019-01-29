#include <BasicLinearAlgebra.h>
#include <math.h>

//State, Output Vectors
BLA::Matrix<3> q = {1,1,1};
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

const float sigmaR = .000115;//m/s
const float sigmaL = .0000538;//m/s
const float sumVar = sigmaR * sigmaR + sigmaL * sigmaL;
const float diffVar = sigmaR * sigmaR - sigmaL * sigmaL;
const float sigmaAngle = 15.3;//deg
const float sigmaFLaser =  3.91;//mm
const float sigmaSLaser =  5.68;//mm
const float b = 0.094; //meters

float vL, vR, vT, wAng;

//Sensor Readings, temporary
float dF, dL, hMag;

// Constants:
const float L = 1; 
const float W =1;
// Maze: LB, LT, RT, RBottom;
const float px[4] = {0,0,W,W};
const float py[4] = {0,L,L,0};
int wall_f = - 1; 
int wall_s = -1;

int NORTH_WALL = 0;
int EAST_WALL = 1;
int SOUTH_WALL = 2;
int WEST_WALL = 3;
// Miscellaneous equations for finding H i.e. linearized equations
float eq(int num) 
{
  float theta = q(1);
  float x = q(2);
  float y = q(3);
  // Adjust theta depending on which wall facing:
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
      return (L-y)* sin(theta)/pow(cos(theta),2); break;
    case 2:
      return -1/(cos(theta)); break;
    case 3:
      return (-y)* cos(theta)/pow(sin(theta),2); break;
    case 4:
      return 1/(sin(theta)); break;
    case 5:
      return (W - x)*sin(theta)/pow(cos(theta),2); break;
    case 6:
      return (y-L)*cos(theta)/pow(sin(theta),2);  break;
    case 7:
      return -1/sin(theta); break;
    case 8:
      return -x * cos(theta)/pow(sin(theta),2); break;
    case 9:
      return sin(theta)/pow(cos(theta),2) * y; break;
    case 10:
      return 1/(cos(theta)); break;
    case 11:
      return cos(theta)/pow(sin(theta),2)*(x - W); break;
    case 12:
      return -x*sin(theta)/pow((cos(theta)),2); break;
  }
}
void update_H(float dt)
{
  // Determine walls:
  // 0: wall_north
  // 1: wall_east
  // 2: wall_south
  // 3: wall_west
  wall_f = det_wall(0);
  wall_s = det_wall(1);

  if (wall_f == NORTH_WALL && wall_s == SOUTH_WALL)
  {
    H  << 1,0,0,
           eq(1), 0, eq(2),
           eq(3), 0, eq(4);}
  else if (wall_f == NORTH_WALL && wall_s == EAST_WALL){
    H  << 1,0,0,
           eq(1), 0, eq(2),
           eq(5), eq(2), 0;}
  else if (wall_f == NORTH_WALL && wall_s == NORTH_WALL){
    H  << 1,0,0,
           eq(1), 0, eq(2),
           eq(6), 0, eq(7);}
  else if (wall_f == EAST_WALL && wall_s == WEST_WALL){
    H  << 1,0,0,
           eq(5), eq(2), 0,
           eq(8), eq(4), 0;}
  else if (wall_f == EAST_WALL && wall_s == SOUTH_WALL){
    H  << 1,0,0,
           eq(5), eq(2), 0,
           eq(9), 0, eq(10);}
  else if (wall_f == EAST_WALL && wall_s == EAST_WALL){
    H  << 1,0,0,
           eq(5), eq(2), 0,
           eq(11), eq(7), 0;}
  else if (wall_f == SOUTH_WALL && wall_s == NORTH_WALL){
    H  << 1,0,0,
           eq(9), 0, eq(10),
           eq(6), 0, eq(7);}
  else if (wall_f == SOUTH_WALL && wall_s == WEST_WALL){
    H  << 1,0,0,
           eq(9), 0, eq(10),
           eq(11), eq(10), 0;}
  else if (wall_f == SOUTH_WALL && wall_s == SOUTH_WALL){
    H  << 1,0,0,
           eq(9), 0, eq(10),
           eq(3), 0, eq(4);}
  else if (wall_f == WEST_WALL && wall_s == EAST_WALL){
    H  << 1,0,0,
           eq(12), eq(10), 0,
           eq(11), eq(7), 0;}
  else if (wall_f == WEST_WALL && wall_s == NORTH_WALL){
    H  << 1,0,0,
           eq(12), eq(10), 0,
           eq(1), 0, eq(2);}
  else if (wall_f == WEST_WALL && wall_s == WEST_WALL){
    H  << 1,0,0,
           eq(12), eq(10), 0,
           eq(8), eq(4), 0;}
  return;
}

// Input: given state
// Output: determine wall wall_f, wall_s
// Argument: 
// 	int sensorType: 0 - front sensor
// 			1 - right sensor
int det_wall(int sensorType)
{
  // TODO: measure sensors from middle point of vehicle to account for displacement
  float theta = q(1);
  float x = q(2);
  float y = q(3);

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
  thetas[0]= acos((L-y)/z[2]);
  thetas[1]= acos((W-x)/z[2]);
  thetas[2]= acos((W-x)/z[3]);
  thetas[3]= acos(y/z[0]);
  thetas[4]= acos(y/z[3]);
  thetas[5]= acos(x/z[0]);
  thetas[6]= acos(x/z[1]);
  thetas[7]= acos((L-y)/z[1]);

  int n_sections = 8
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
       -vT * sin(q(1)) * dt, 1, 0,
       vT * cos(q(1)) * dt, 1, 0;
}

void update_Q(float dt)
{
  Q << pow((dt/b),2)*sumVar, pow(dt,2)/(2*b)*cos(q_est(1))*diffVar, pow(dt,2)/(2*b)*sin(q_est(1))*diffVar,
       pow(dt,2)/(2*b)*cos(q_est(1))*diffVar, pow(cos(q_est(1)),2) * pow(dt,2)/4*sumVar, sin(q_est(1))* cos(q_est(1)) * pow(dt,2)/4*sumVar,
       pow(dt,2)/(2*b)*sin(q_est(1))*diffVar, sin(q_est(1))* cos(q_est(1)) * pow(dt,2)/4*sumVar, pow(sin(q_est(1)),2) * pow(dt,2)/4*sumVar;
  return;
}

void aPrioriUpdate(float dt)
{
  //get q^ estimate
  q_est(1) = q_est(1) + (wAng * dt);
  q_est(2) = vT * cos(q(1));
  q_est(3) = vT * sin(q(1));

  //P update
  update_F(dt);
  update_Q(dt);
  P = ((F * P) * (~F)) + Q;
  
}

void aPosterioriUpdate(float dt)
{
  outputEstimate();
  BLA::Matrix<3> innovation = z - z_est;
  S = ((H * P) * (~H)) + R; //innovation covariance
  K = (P * (~H))*(S.Inverse()); //Kalman Gain
  q_est += (K * innovation); //A Posteriori State Estimate
  P = (I3 - (K*H))*P; //Update Covariance Estimate  
  
}

void outputEstimate()
{
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

  Serial.begin (115200);

  setupDistanceSensors();
  setupIMU();
}

void loop() {
  
  // put your main code here, to run repeatedly:
  int pwmR = 90;
  int pwmL = 90 ;
  float gz, head, fDist, sDist;
  ReadDistSensors(fDist, sDist);
  //ReadIMU(gz, head);
  /*
  Serial.print("Heading: ");
  Serial.print(head);
  Serial.print(" Wall: ");
  Serial.print(wall_f);
  Serial.print(" ");
  Serial.print(wall_s);
  Serial.print("\n");*/
  getVelocities(pwmR, pwmL, vL, vR, vT, wAng);
}
