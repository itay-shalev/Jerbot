#include <Wire.h>
#include <MPU6050.h>

// ***** Side Definitions ****** //
#define BACKWARD 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

// ***** Original Connections ****** //
#define O_M_FL 4
#define O_M_FR 2
#define O_M_BL 8
#define O_M_BR 6
#define O_US_FL 46
#define O_US_FR 48
#define O_US_RL 44
#define O_US_RR 42
#define O_US_BL 30
#define O_US_LL 38
#define O_US_LR 36
#define O_US_BR 40
#define UV 33
#define P_R 28
#define P_L 29
#define F_LED 27
#define MIC_LED 26

#define SPEED_LIMIT(x) (80 - x) // Speed Macro

// ***** Pathfinding Definitions ****** //
#define X 12
#define Y 25
#define PATH_SIZE 50
#define HOME_1 5, 1
#define ROOM_1_1 originalMap[2][8] == '0' ? 8 : 5, originalMap[2][8] == '0' ? 1 : 8
#define ROOM_2_1 1, 6
#define ROOM_3_1 5, 11
#define ROOM_4_1 5, originalMap[8][6] == '1' ? 11 : 8
#define HOME_2 5, 13
#define ROOM_1_2 8, 13
#define ROOM_2_2 1, 18
#define ROOM_3_2 5, 23
#define ROOM_4_2 5, 23

// ***** Pathfinding Variables ****** //
char path[PATH_SIZE] = {0};
char _map[Y][X] = {
    {'1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '1', '0', '1', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '1', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '1', '0', '1'},
    {'1', '0', '1', '1', '1', '0', '1', '1', '1', '1', '0', '1'},
    {'1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1'},
    {'1', '1', '1', '1', '1', '0', '1', '1', '1', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1'},
    {'1', '1', '1', '1', '1', '0', '1', '1', '1', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '0', '0', '0', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '1', '0', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '1', '1', '1'},
    {'1', '0', '1', '1', '1', '0', '1', '0', '1', '1', '1', '1'},
    {'1', '0', '0', '0', '0', '0', '0', '0', '1', '1', '1', '1'},
    {'1', '1', '1', '1', '1', '0', '1', '1', '1', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1'},
    {'1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1'}};
char originalMap[Y][X] = {
    {'1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '0', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '1', '0', '1', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '1', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '1', '0', '1'},
    {'1', '0', '1', '1', '1', '0', '1', '1', '1', '1', '0', '1'},
    {'1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1'},
    {'1', '1', '1', '1', '1', '0', '1', '1', '1', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1'},
    {'1', '1', '1', '1', '1', '0', '1', '1', '1', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '0', '0', '0', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '1', '0', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '1', '1', '1'},
    {'1', '0', '1', '1', '1', '0', '1', '0', '1', '1', '1', '1'},
    {'1', '0', '0', '0', '0', '0', '0', '0', '1', '1', '1', '1'},
    {'1', '1', '1', '1', '1', '0', '1', '1', '1', '1', '1', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '1', '0', '1', '0', '0', '0', '0', '1'},
    {'1', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '1'},
    {'1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1'}};

// ***** Motor Connections ****** //
int M_FL = 4;
int M_FR = 2;
int M_BL = 8;
int M_BR = 6;
const int M_FAN = 32;

// ***** IR Connections ****** //
const int IR_LR = A2;
const int IR_LL = A3;
const int IR_FR = A0;
const int IR_FL = A1;

// ***** US Connections ****** //
int US_FL = O_US_FL;
int US_FR = O_US_FR;
int US_RL = O_US_RL;
int US_RR = O_US_RR;
int US_BL = O_US_BL;
int US_LL = O_US_LL;
int US_LR = O_US_LR;
int US_BR = O_US_BR;

// ***** Map Variables ***** //
bool isRoom4_1Checked = false;
bool isRoom1_1Checked = false;
bool isRoom1_1Updated = false;
bool isRoom1_1Default = true;
bool isDog = false;

// ***** Variables ****** //
unsigned long timer = 0;
int yaw = 0;
float timeStep = 0.01;
int facingDeg = 0;
int currFace = FORWARD;

MPU6050 mpu;

void setup()
{
  Serial.begin(9600);
  mpu.begin();
  mpu.calibrateGyro();
  initMotor(M_FL);
  initMotor(M_FR);
  initMotor(M_BL);
  initMotor(M_BR);
  pinMode(IR_LR, INPUT);
  pinMode(IR_LL, INPUT);
  pinMode(IR_FR, INPUT);
  pinMode(IR_FL, INPUT);
  initUS(US_FL);
  initUS(US_BR);
  initUS(US_LR);
  initUS(US_LL);
  initUS(US_BL);
  initUS(US_RR);
  initUS(US_RL);
  initUS(US_FR);
  pinMode(UV, INPUT);
  pinMode(F_LED, OUTPUT);
  pinMode(MIC_LED, OUTPUT);
  pyroSetup();
}

void loop()
{
  digitalWrite(MIC_LED, HIGH);
  delay(100);
  digitalWrite(MIC_LED, LOW);
  startAlign();
  checkDog();
  if (!isDog)
  {
    mapDrive(100, HOME_1, ROOM_1_1);
    align(BACKWARD);
    checkRoom1(false);
    if (isRoom1_1Default)
    {
      scanRoom1(false);
    }
    mapDrive(100, ROOM_1_1, ROOM_2_1);
  }
  else
  {
    mapDrive(100, HOME_1, ROOM_2_1);
  }
  scanRoom2();
  align(FORWARD);
  mapDrive(100, ROOM_2_1, ROOM_3_1);
  scanRoom3();
  align(FORWARD);
  mapDrive(100, ROOM_3_1, ROOM_4_1);
  scanRoom4();
  if (isDog || !isRoom1_1Default)
  {
    checkRoom1(true);
    scanRoom1(true);
    mapDrive(100, ROOM_1_1, HOME_1);
  }
  else
  {
    mapDrive(100, ROOM_4_1, HOME_1);
  }
  stopProgram();
}

/*
This function controls a single motor with a given direction and speed
Input:
  motor - First motor pin
  speed - The speed to move the motor
  dir - The direction to move
Output:
  None
*/
void motorControl(int motor, int speed, int dir)
{
  if (dir) // Check if backwards or forwards
  {
    analogWrite(motor, map(speed, 0, 100, 0, 255)); // Set first pin to the speed
    digitalWrite(motor + 1, LOW);                   // Set second pin to low
  }
  else
  {
    analogWrite(motor + 1, map(speed, 0, 100, 0, 255)); // Set the second pin to the speed
    digitalWrite(motor, LOW);                           // Set first pin to low
  }
}

/*
This function drives the robot in a given direction
Input:
  speed - The speed to move the robot
  dir - The direction to move
Output:
  None
*/
void drive(int speed, int dir)
{
  switch (dir) // Check which direction and move the robot according to the holonomic drive system
  {
  case BACKWARD:
    motorControl(M_FL, speed, BACKWARD);
    motorControl(M_FR, speed, FORWARD);
    motorControl(M_BL, speed, BACKWARD);
    motorControl(M_BR, speed, FORWARD);
    break;
  case FORWARD:
    motorControl(M_FL, speed, FORWARD);
    motorControl(M_FR, speed, BACKWARD);
    motorControl(M_BL, speed, FORWARD);
    motorControl(M_BR, speed, BACKWARD);
    break;
  case LEFT:
    motorControl(M_FL, speed, BACKWARD);
    motorControl(M_FR, speed, BACKWARD);
    motorControl(M_BL, speed, FORWARD);
    motorControl(M_BR, speed, FORWARD);
    break;
  case RIGHT:
    motorControl(M_FL, speed, FORWARD);
    motorControl(M_FR, speed, FORWARD);
    motorControl(M_BL, speed, BACKWARD);
    motorControl(M_BR, speed, BACKWARD);
    break;
  }
}

/*
This function stop the robot in place
Input:
  None
Output:
  None
*/
void stopRobot()
{
  // Sets all of the motor pins to 0 (so the robot stops in place)
  for (int i = O_M_FR; i <= O_M_BL + 1; i++)
  {
    digitalWrite(i, LOW);
  }
}

/*
This function setups the motor pins
Input:
  motor - The first pin of the motor
Output:
  None
*/
void initMotor(int motor)
{
  pinMode(motor, OUTPUT);
  pinMode(motor + 1, OUTPUT);
}

/*
This function updates the yaw of the robot
Input:
  None
Output:
  The current yaw
*/
double getYaw()
{
  timer = millis();                      // Get the current time
  Vector norm = mpu.readNormalizeGyro(); // Read the gyro input

  yaw += abs(norm.ZAxis * timeStep) > 0.5 ? norm.ZAxis * timeStep : 0; // Add the turn to the yaw
  delay(timeStep * 1000 - (millis() - timer));                         // Add delay
  return yaw;
}

/*
This function turns the robot in place
Input:
  speed - The speed to move at
  dir - The direction of movement
Output:
  None
*/
void turn(int speed, int dir)
{
  if (dir == RIGHT) // Check the direction
  {
    motorControl(M_FL, speed, FORWARD);
    motorControl(M_FR, speed, FORWARD);
    motorControl(M_BL, speed, FORWARD);
    motorControl(M_BR, speed, FORWARD);
  }
  else if (dir == LEFT)
  {
    motorControl(M_FL, speed, BACKWARD);
    motorControl(M_FR, speed, BACKWARD);
    motorControl(M_BL, speed, BACKWARD);
    motorControl(M_BR, speed, BACKWARD);
  }
}

/*
This function turns the robot by a certain amount of degrees
Input:
  angle - The angle to turn to
  speed - The speed to move at
Output:
  None
*/
void gyroTurn(int angle, int speed)
{
  yaw = 0;                                                    // Resets the yaw of the robot.
  int newAngle = angle > 0 ? angle / 2 + 23 : angle / 2 - 23; // A fix for the angle to counter the offset caused by the speed
  if (newAngle > 0)                                           // Turn according to the direction
  {
    turn(speed, RIGHT);
  }
  else if (newAngle < 0)
  {
    turn(speed, LEFT);
  }
  while (abs(abs(newAngle) - abs(yaw)) >= 3) // Turn until angle is reached
  {
    yaw = getYaw();
  }
  stopRobot();
}

/*
This function read the value from a ultrasonic sensor
Input:
  US - The sensor to read
Output:
  The distance in cm
*/
double readUS(int US)
{
  double ans = 0;
  int i = 0;
  int Echo = US;
  int trig = US + 1;

  for (i = 0; i < 3; i++) // Get the average of 3 reads
  {
    // Send the signal
    digitalWrite(trig, LOW);
    delayMicroseconds(2);

    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    ans += (pulseIn(Echo, HIGH) * 0.034) / 2; // Get the result
    delayMicroseconds(10);
  }
  return ans / 3.0;
}

/*
This function initializes the pins for an ultrasonic sensor
Input:
  US - The first pin of the ultrasonic
Output:
  None
*/
void initUS(int US)
{
  pinMode(US, INPUT);
  pinMode(US + 1, OUTPUT);
}

/*
This function reads the infrared sensor
Input:
  ir - The sensor to read
Output:
  The result from the sensor
*/
int readIR(int ir)
{
  return analogRead(ir);
}

/*
This function aligns the robot
Input:
  face - The face to align
Output:
  None
*/
void align(int face)
{
  // See the full explaination on the documents
  double p = 7;
  int err = 0;
  int left_us = 0;
  int right_us = 0;
  int turnDir = FORWARD;
  faceCycle(FORWARD);
  switch (face)
  {
  case FORWARD:
    left_us = US_FL;
    right_us = US_FR;
    break;
  case RIGHT:
    left_us = US_RL;
    right_us = US_RR;
    break;
  case BACKWARD:
    left_us = US_BL;
    right_us = US_BR;
    break;
  case LEFT:
    left_us = US_LL;
    right_us = US_LR;
    break;
  }
  for (int i = 0; i < 2; i++)
  {
    err = (int)readUS(left_us) - (int)readUS(right_us);
    while (abs(err) != 0)
    {
      if (err > 0)
      {
        turn((p * abs(err)) > 60 ? 60 : (p * abs(err)) < 20 ? 20 : (p * abs(err)), RIGHT);
      }
      else
      {
        turn((p * abs(err)) > 60 ? 60 : (p * abs(err)) < 20 ? 20 : (p * abs(err)), LEFT);
      }
      err = (int)readUS(left_us) - (int)readUS(right_us);
    }
    stopRobot();
  }
}

/*
This function drives the robot in a direction, aligning it along the way
Input:
  None
Output:
  None
*/
void alignedDrive(int speed)
{
  // See the full explaination in the documents
  double pYaw = 4, pUS = 3; // The fix of the driving, p_gyro for gyro, p_us for ultrasonic.
  double yawFix = 0;
  int wallDist = 9; // The distance that the robot keeps from te selected wall.
  int usFix = 0;
  int currDistUS = US_LR; // The currently used us sensor
  int currRYawUS = US_LR;
  int currLYawUS = US_LL;
  int negativeUS = 1; // This value controles he negetivity of the us values.
  bool flagUS_RR = false;
  bool flagUS_LL = false;
  bool foundWall = false;
  int minDist = 20;
  int negativeFace = 1;

  //Find on what wall to follow, drives forward untill finds wall if starts with no walls around him.
  while (!foundWall && (readUS(US_FR) > minDist) && (readUS(US_FL) > minDist))
  {
    if (readUS(US_RL) < 30 && readUS(US_RR) < 30)
    {
      currDistUS = US_RL;
      currLYawUS = US_RL;
      currRYawUS = US_RR;
      negativeUS = 1;
      foundWall = true;
    }
    else if (readUS(US_LR) < 30 && readUS(US_LL) < 30)
    {
      currDistUS = US_LR;
      currLYawUS = US_LL;
      currRYawUS = US_LR;
      negativeUS = -1;
      foundWall = true;
    }
    else
    {
      drive(speed, FORWARD);
    }
  }

  if (currFace == FORWARD || currFace == RIGHT || currFace == LEFT)
  {
    negativeFace = negativeFace * 1;
  }
  else
  {
    negativeFace = negativeFace * -1;
  }

  while (!checkHole(US_LL, flagUS_LL) && !checkHole(US_RR, flagUS_RR) && (readUS(US_FR) > minDist) && (readUS(US_FL) > minDist))
  {
    if (readUS(US_RR) < 30 && !flagUS_RR)
    {
      flagUS_RR = true;
    }

    if (readUS(US_LL) < 30 && !flagUS_LL)
    {
      flagUS_LL = true;
    }

    if (readUS(currDistUS) > 30)
    {
      drive(speed, FORWARD);
    }
    else
    {
      yawFix = (((readUS(currRYawUS) - readUS(currLYawUS)) * pYaw > SPEED_LIMIT(speed) ? SPEED_LIMIT(speed) : (readUS(currRYawUS) - readUS(currLYawUS)) * pYaw));
      usFix = negativeUS * (((readUS(currDistUS) - wallDist) * pUS) > SPEED_LIMIT(speed) ? SPEED_LIMIT(speed) : ((readUS(currDistUS) - wallDist) * pUS));

      motorControl(M_FR, speed - usFix, BACKWARD);
      motorControl(M_FL, readUS(currRYawUS) > 30 || readUS(currLYawUS) > 30 ? speed : speed - yawFix, FORWARD);  // affected by gyro fix
      motorControl(M_BR, readUS(currRYawUS) > 30 || readUS(currLYawUS) > 30 ? speed : speed + yawFix, BACKWARD); // affected by gyro fix
      motorControl(M_BL, speed - usFix, FORWARD);
    }
  }
  stopRobot();
  delay(400);
}

/*
This function checks if theres a hole
Input:
  us - The sensor to check
  flag - Whether a wall was seen already
Output:
  None
*/
bool checkHole(int us, bool flag)
{
  int holeDist = 30;
  return readUS(us) >= holeDist && flag;
}

/*
This function changes the sensor and motor locations of the robot
Input:
  face - The face to change to
Output:
  None
*/
void faceCycle(int face)
{
  currFace = face;
  switch (face)
  {
  case FORWARD:
    M_BL = O_M_BL;
    M_BR = O_M_BR;
    M_FL = O_M_FL;
    M_FR = O_M_FR;
    US_FL = O_US_FL;
    US_FR = O_US_FR;
    US_RL = O_US_RL;
    US_RR = O_US_RR;
    US_BL = O_US_BL;
    US_LL = O_US_LL;
    US_LR = O_US_LR;
    US_BR = O_US_BR;
    break;
  case LEFT:
    M_BL = O_M_BR;
    M_BR = O_M_FR;
    M_FL = O_M_BL;
    M_FR = O_M_FL;
    US_FL = O_US_LL;
    US_FR = O_US_LR;
    US_RL = O_US_FL;
    US_RR = O_US_FR;
    US_BL = O_US_RL;
    US_BR = O_US_RR;
    US_LL = O_US_BL;
    US_LR = O_US_BR;
    break;
  case BACKWARD:
    M_BL = O_M_FR;
    M_BR = O_M_FL;
    M_FL = O_M_BR;
    M_FR = O_M_BL;
    US_FL = O_US_BL;
    US_FR = O_US_BR;
    US_RL = O_US_LL;
    US_RR = O_US_LR;
    US_BL = O_US_FL;
    US_BR = O_US_FR;
    US_LL = O_US_RL;
    US_LR = O_US_RR;
    break;
  case RIGHT:
    M_BL = O_M_FL;
    M_BR = O_M_BL;
    M_FL = O_M_FR;
    M_FR = O_M_BR;
    US_FL = O_US_RL;
    US_FR = O_US_RR;
    US_RL = O_US_BL;
    US_RR = O_US_BR;
    US_BL = O_US_LL;
    US_BR = O_US_LR;
    US_LL = O_US_FL;
    US_LR = O_US_FR;
    break;
  }
}

/*
This function finds the path from two points
Input:
  x1 - The start x
  y1 - The start y
  x2 - The target x
  y2 - The target y
Output:
  Return if ended successfully
*/
int pathfind(int x1, int y1, int x2, int y2)
{
  // See full explaination on the documents
  int count = 0;
  int flag = 1;
  int i = 0;
  int x = x1;
  int y = y1;
  int counter = 0;
  char check[4] = {'U', 'R', 'D', 'L'};

  if (_map[y2][x2] == '1' || _map[y1][x1] == '1') // If starting or ending locations are a wall
  {
    return 0;
  }

  for (i = 0; i < PATH_SIZE; i++) // Clear the path
  {
    path[i] = 0;
  }

  while (x != x2 || y != y2)
  {
    if (_map[y][x] == '+')
    {
      counter = 0;
      x = x1;
      y = y1;
      blockPath(x1, y1);
    }

    _map[y][x] = '+';
    flag = 1;
    count = 0;

    if (_map[y][x + 1] == '0' || _map[y][x + 1] == '+')
    {
      count++;
    }

    if (_map[y][x - 1] == '0' || _map[y][x - 1] == '+')
    {
      count++;
    }

    if (_map[y + 1][x] == '0' || _map[y + 1][x] == '+')
    {
      count++;
    }

    if (_map[y - 1][x] == '0' || _map[y - 1][x] == '+')
    {
      count++;
    }

    if (count > 2)
    {
      path[counter++] = '*';
    }

    if (y <= y2 && x <= x2)
    {
      if (y2 - y > x2 - x)
      {
        check[0] = 'U';
        check[1] = 'L';
        check[2] = 'D';
        check[3] = 'R';
      }
      else
      {
        check[0] = 'L';
        check[1] = 'U';
        check[2] = 'R';
        check[3] = 'D';
      }
    }
    else if (y <= y2 && x >= x2)
    {
      if (y2 - y > x - x2)
      {
        check[0] = 'U';
        check[1] = 'R';
        check[2] = 'D';
        check[3] = 'L';
      }
      else
      {
        check[0] = 'R';
        check[1] = 'U';
        check[2] = 'L';
        check[3] = 'D';
      }
    }
    else if (y >= y2 && x <= x2)
    {
      if (y - y2 > x2 - x)
      {
        check[0] = 'D';
        check[1] = 'L';
        check[2] = 'U';
        check[3] = 'R';
      }
      else
      {
        check[0] = 'L';
        check[1] = 'D';
        check[2] = 'R';
        check[3] = 'U';
      }
    }
    else if (y >= y2 && x >= x2)
    {
      if (y - y2 > x - x2)
      {
        check[0] = 'D';
        check[1] = 'R';
        check[2] = 'U';
        check[3] = 'L';
      }
      else
      {
        check[0] = 'R';
        check[1] = 'D';
        check[2] = 'L';
        check[3] = 'U';
      }
    }
    for (i = 0; i < 4 && flag; i++)
    {
      switch (check[i])
      {
      case 'U':
        if (_map[y + 1][x] == '0')
        {
          path[counter++] = check[i];
          y++;
          flag = 0;
        }
        break;
      case 'R':
        if (_map[y][x - 1] == '0')
        {
          path[counter++] = check[i];
          x--;
          flag = 0;
        }
        break;
      case 'D':
        if (_map[y - 1][x] == '0')
        {
          path[counter++] = check[i];
          y--;
          flag = 0;
        }
        break;
      case 'L':
        if (_map[y][x + 1] == '0')
        {
          path[counter++] = check[i];
          x++;
          flag = 0;
        }
        break;
      default:
        break;
      }
    }
  }
  _map[y][x] = 'O';
  _map[y1][x1] = 'X';
  return 1;
}

/*
This function clears the path
Input:
  None
Output:
  None
*/
void clearPath()
{
  int x = 0;
  int y = 0;
  int i = 0;
  for (y = 0; y < Y; y++)
  {
    for (x = 0; x < X; x++)
    {
      _map[y][x] = originalMap[y][x]; // Set each value to the original map
    }
  }
  for (i = 0; i < PATH_SIZE; i++)
  {
    path[i] = 0; // Clear the path
  }
}

/*
This function minimizes the path
Input:
  None
Output:
  None
*/
void minimizePath()
{
  // See full explaination on the documents
  char newPath[PATH_SIZE] = {0};
  char last = 0;
  int i = 0;
  int counter = 0;
  for (i = 0; path[i]; i++)
  {
    if (path[i] != last && path[i] != '*')
    {
      newPath[counter++] = path[i];
      last = path[i];
    }
    else if (path[i] == '*')
    {
      last = path[i];
    }
  }

  for (i = 0; i < PATH_SIZE; i++)
  {
    path[i] = 0;
  }

  for (i = 0; i < counter; i++)
  {
    path[i] = newPath[i];
  }
}

void blockPath(int x1, int y1)
{
  int x = 0;
  int y = 0;
  int i = 0;

  for (y = 0; y < Y; y++)
  {
    for (x = 0; x < X; x++)
    {
      if (x == x1 && y == y1)
      {
        _map[y][x] = '0';
      }
      else if (_map[y][x] == '+')
      {
        _map[y][x] = '1';
      }
    }
  }

  for (i = 0; i < PATH_SIZE; i++)
  {
    path[i] = 0;
  }
}

/*
This function drives along the generated path
Input:
  speed - The speed to move at
Output:
  None
*/
void translateDrive(int speed)
{
  int i = 0;
  for (i = 0; path[i] != 0; i++) // Follow the path
  {
    switch (path[i]) // Drive in the path's direction
    {
    case 'U':
      faceCycle(FORWARD);
      break;

    case 'L':
      faceCycle(LEFT);
      break;

    case 'R':
      faceCycle(RIGHT);
      break;

    case 'D':
      faceCycle(BACKWARD);
      break;

    default:
      stopRobot();
      break;
    }
    alignedDrive(speed);
  }
  stopRobot();
  delay(200);
  faceCycle(FORWARD);
}

/*
This function drives from point A to B
Input:
  speed - The speed to move at
  x1 - The start x
  y1 - The start y
  x2 - The target x
  y2 - The target y
Output:
  None
*/
void mapDrive(int speed, int x1, int y1, int x2, int y2)
{
  if (checkIfRoom(x1, y1, ROOM_1_1) && !isRoom1_1Updated && isRoom1_1Checked) // Generate the path
  {
    isRoom1_1Updated = true;
    pathfind(8, 1, x2, y2);
  }
  else
  {
    pathfind(x1, y1, x2, y2);
  }
  minimizePath();                    // Minimize it
  translateDrive(speed);             // Drive along the path
  clearPath();                       // Clear the path
  if (checkIfRoom(x2, y2, ROOM_3_1)) // Room specific check
  {
    if (readUS(US_RR) < 20)
    {
      faceCycle(FORWARD);
      alignedDrive(100);
      delay(100);
    }
    align(FORWARD);
  }
}

/*
This function checks if the coordinates are a room
Input:
  x1 - First x value
  y1 - First y value
  x2 - Second x value
  y2 - Second y value
Output:
  Whether they match
*/
bool checkIfRoom(int x1, int y1, int x2, int y2)
{
  return x1 == x2 && y1 == y2;
}

/*
This function reads the data from the UV sensor
Input:
  None
Output:
  Returns if there's fire
*/
bool readUV()
{
  return !digitalRead(UV);
}

/*
This function checks room 4
Input:
  None
Output:
  None
*/
void checkRoom4()
{
  isRoom4_1Checked = true;
  faceCycle(FORWARD);
  if (readUS(US_LL) > 30 && readUS(US_LR) > 30)
  {
    originalMap[8][6] = '1';
    originalMap[9][6] = '1';
    originalMap[10][6] = '1';
    originalMap[11][6] = '0';
  }
  else
  {
    originalMap[8][6] = '0';
    originalMap[9][6] = '1';
    originalMap[10][6] = '1';
    originalMap[11][6] = '1';
  }
  clearPath();
}

/*
This function checks room 1
Input:
  None
Output:
  None
*/
void checkRoom1(bool reversed)
{
  isRoom1_1Checked = true;
  faceCycle(FORWARD);
  if (reversed)
  {
    stopRobot();
    while (readUS(US_BL) > 80 || readUS(US_BR) > 80 || readUS(US_FL) > 20 || readUS(US_FR) > 20)
    {
      drive(100, LEFT);
    }
    stopRobot();
    if (readUS(US_BL) < 20 && readUS(US_BR) < 20)
    {
      originalMap[2][8] = '0';
      originalMap[5][7] = '1';
      isRoom1_1Default = true;
    }
    else
    {
      originalMap[2][8] = '1';
      originalMap[5][7] = '0';
      isRoom1_1Default = false;
    }
  }
  else
  {
    if (readUS(US_LL) > 20 && readUS(US_LR) > 20)
    {
      originalMap[2][8] = '0';
      originalMap[5][7] = '1';
      isRoom1_1Default = true;
    }
    else
    {
      originalMap[2][8] = '1';
      originalMap[5][7] = '0';
      isRoom1_1Default = false;
    }
  }
  clearPath();
}

/*
This function scans room 1
Input:
  None
Output:
  None
*/
void scanRoom1(bool reversed)
{
  faceCycle(FORWARD);
  delay(50);
  if (!isRoom1_1Default)
  {
    align(FORWARD);
    delay(50);
    gyroTurn(180, 50);
    if (readUV())
    {
      align(BACKWARD);
      delay(10);
      while (readUS(US_BR) < 48)
      {
        drive(100, FORWARD);
      }
      stopRobot();
      delay(50);
      pyroDetect();
      while (readUS(US_BR) > 15)
      {
        drive(100, BACKWARD);
      }
      stopRobot();
      delay(50);
    }
    gyroTurn(190, 50);
    delay(50);
    align(FORWARD);
    delay(100);
    faceCycle(RIGHT);
    alignedDrive(100);
    faceCycle(FORWARD);
  }
  else
  {
    if (reversed)
    {
      mapDrive(100, 8, 6, ROOM_1_1);
    }
    if (readUV())
    {
      align(BACKWARD);
      delay(10);
      while (readUS(US_BR) < 48)
      {
        drive(100, FORWARD);
      }
      stopRobot();
      delay(50);
      pyroDetect();
      while (readUS(US_BR) > 15)
      {
        drive(100, BACKWARD);
      }
      stopRobot();
      delay(50);
      mapDrive(100, ROOM_1_1, HOME_1);
      stopProgram();
    }
  }
}

/*
This function scans room 2
Input:
  None
Output:
  None
*/
void scanRoom2()
{
  faceCycle(FORWARD);
  align(FORWARD);
  delay(100);
  gyroTurn(-70, 50);
  gyroTurn(-70, 50);
  delay(300);
  if (readUV())
  {
    while ((readUS(US_BR) + readUS(US_BL)) / 2 < 48)
    {
      drive(100, FORWARD);
    }
    stopRobot();
    delay(50);
    pyroDetect();
    delay(100);
    align(LEFT);
    delay(50);
    while (readUS(US_LL) > 15 || readUS(US_LR) > 15)
    {
      drive(100, LEFT);
    }
    stopRobot();
    while (readUS(US_BR) > 15)
    {
      drive(100, BACKWARD);
    }
    stopRobot();
    delay(50);
    gyroTurn(180, 50);
    mapDrive(100, ROOM_2_1, HOME_1);
    stopProgram();
  }
  gyroTurn(80, 50);
  gyroTurn(80, 50);
  delay(100);
  align(FORWARD);
  delay(100);
}

/*
This function scans room 3
Input:
  None
Output:
  None
*/
void scanRoom3()
{
  bool candle_detected = false;
  faceCycle(FORWARD);
  while (readUS(US_FR) > 15 && readUS(US_FL) > 15) //If both don't see a wall
  {
    drive(100, FORWARD);
  }
  delay(100);
  stopRobot();
  align(FORWARD);
  if ((readUS(US_FR) > 15 && readUS(US_FL) < 15) || (readUS(US_FR) < 15 && readUS(US_FL) > 15)) //Only if one of them sees a wall in front
  {
    align(FORWARD);
  }
  delay(100);
  checkRoom4();
  while (readUS(US_BL) > 80 || readUS(US_BR) > 80 || (readUS(US_RL) + readUS(US_RR)) / 2.0 > 50) // If one of the back sensors don't see a wall and if the avg of the right sensors doesn't get too close
  {
    drive(100, RIGHT);
  }
  stopRobot();
  gyroTurn(135, 50);
  delay(200);
  if (readUV())
  {
    candle_detected = true;
    pyroDetect();
    delay(100);
  }
  gyroTurn(-135, 50);
  delay(50);
  align(FORWARD);
  delay(50);
  while (readUS(US_FR) > 15 || readUS(US_FL) > 15) // If one of the front sensors is nt close to a wall
  {
    drive(40, FORWARD);
  }
  while (readUS(US_BL) < 80 && ((readUS(US_LL) + readUS(US_LR)) / 2 > 12)) // If the back left sensor doesn't see a hole and the avg of the left sensors is not a wall
  {
    drive(100, LEFT);
  }
  stopRobot();
  delay(50);
  align(FORWARD);
  if (candle_detected)
  {
    if (readUS(US_LL) < 30 || readUS(US_LR) < 30)
    {
      faceCycle(BACKWARD);
      alignedDrive(100);
      faceCycle(FORWARD);
    }
    mapDrive(100, ROOM_3_1, HOME_1);
    stopProgram();
  }
}

/*
This function scans room 4
Input:
  None
Output:
  None
*/
void scanRoom4()
{
  bool isDefault = !(originalMap[8][6] == '0');
  bool candle_detected = false;
  align(FORWARD);
  delay(50);
  while (readUS(US_FL) < 10 || readUS(US_FR) < 10 || (!isDefault ? readUS(US_FL) < 52 || readUS(US_FR) < 52 : 0))
  {
    drive(!isDefault ? 50 : 30, BACKWARD);
  }
  stopRobot();
  delay(50);
  align(FORWARD);
  delay(50);
  while (readUS(US_BL) > 80 || readUS(US_BR) > 80 || (!isDefault ? readUS(US_RR) < 30 || readUS(US_RL) < 30 : 0) || (isDefault ? readUS(US_LR) > 100 && readUS(US_LL) > 100 : 0))
  {
    drive(100, LEFT);
  }
  delay(150);
  stopRobot();
  delay(50);
  gyroTurn(isDefault ? -135 : -45, 50);
  delay(200);
  if (readUV())
  {
    candle_detected = true;
    pyroDetect();
  }
  gyroTurn(isDefault ? 135 : 45, 50);
  if (!isDefault)
  {
    delay(200);
    align(RIGHT);
    delay(100);
    faceCycle(RIGHT);
    while (readUS(US_FR) > 20 && readUS(US_FL) > 20)
    {
      alignedDrive(100);
    }
    faceCycle(FORWARD);
    align(RIGHT);
  }
  else
  {
    align(FORWARD);
    delay(100);
    while (readUS(US_FL) < 10 || readUS(US_FR) < 10)
    {
      drive(30, BACKWARD);
    }
    stopRobot();
    delay(100);
    align(FORWARD);
    delay(100);
    while ((readUS(US_BL) < 80 || readUS(US_BR) < 80) && readUS(US_RL) < 100)
    {
      drive(100, RIGHT);
    }
    stopRobot();
    delay(100);
    align(FORWARD);
    delay(100);
    motorControl(M_FL, 10, BACKWARD);
    motorControl(M_FR, 50, FORWARD);
    motorControl(M_BL, 10, BACKWARD);
    motorControl(M_BR, 50, FORWARD);
    delay(50);
  }
  faceCycle(BACKWARD);
  alignedDrive(100);
}

/*
This function fixes the robot orientation on the start of the match
Input:
  None
Output:
  None
*/
void startAlign()
{
  // If the robot is not faced correctly turn the robot to the correct orientation
  if ((readUS(US_RL) + readUS(US_RR)) / 2.0 > 40)
  {
    gyroTurn(80, 50);
    delay(10);
    align(RIGHT);
    delay(100);
  }
}

/*
This function checks if a dog is blocking the hallway
Input:
  None
Output:
  None
*/
void checkDog()
{
  // If the hallway to the right is open
  if (readIR(IR_LL) < 100 && readIR(IR_LR) < 100)
  {
    isDog = false;
  }
  else
  {
    isDog = true;
    originalMap[1][6] = '1';
    clearPath();
  }
}

/*
This function stops the program
Input:
  None
Output:
  None
*/
void stopProgram()
{
  delay(100000000000); // Waits forever
}

/*
This function sets the pyro sensors pins
Input:
  None
Outpu:
  None
*/
void pyroSetup()
{
  pinMode(P_R, INPUT);
  pinMode(P_L, INPUT);
}

/*
This function reads the data from the pyro
Input:
  pyro - The pyro pin
Output:
  The data from the sensor
*/
int pyroRead(int pyro)
{
  return digitalRead(pyro);
}

/*
This function detects where the flame is
Input:
  None
Output:
  None
*/
void pyroDetect()
{
  yaw = 0;
  digitalWrite(F_LED, HIGH);               // Notify that theres fire
  while (!pyroRead(P_R) || !pyroRead(P_L)) // While the flame is not found
  {
    // Rotate to the flame and update the yaw
    yaw = getYaw();
    if (pyroRead(P_L))
    {
      turn(20, LEFT);
    }
    else
    {
      turn(20, RIGHT);
    }
  }
  stopRobot();
  delay(100);
  digitalWrite(M_FAN, HIGH); // Activate the fan until the flame is extinguished
  while (readUV())
  {
  }
  digitalWrite(M_FAN, LOW);
  delay(200);
  yaw = yaw > 0 ? (abs(yaw) % 360) : ((abs(yaw) % 360) * -1); // Turn back to the starting location
  gyroTurn((yaw > 0 ? yaw : yaw), 20);
  faceCycle(FORWARD);
  digitalWrite(F_LED, LOW);
}