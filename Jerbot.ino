#include <Wire.h>
#include <MPU6050.h>

#define BACKWARD 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3
#define BOTH 4
#define FR 0
#define FL 1
#define BR 2
#define BL 3

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

#define SPEED_LIMIT(x) (80 - x)
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
// ***** Pathfinder Variables ***** //

//6, 8

char path[PATH_SIZE] = { 0 };
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
  {'1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1'}
};
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
  {'1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1'}
};

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

// ***** Function declares ***** //
void mapDrive(int speed, int x1, int y1, int x2, int y2);
void motorControl(int motor, int speed, int dir);

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
int swapDir = 1; // For the gyro, if driving right or backward the gyro fix should be minus (multiplied by -1 or 1).
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
  pyroSetup();
}

void loop()
{
  startAlign();
  checkDog();
  if(!isDog)
  {
    mapDrive(50, HOME_1, ROOM_1_1);
    align(BACKWARD);
    checkRoom1(false);
    if(isRoom1_1Default)
    {
      scanRoom1(false);
    }
    mapDrive(50, ROOM_1_1, ROOM_2_1);
  }
  else
  {
    mapDrive(50, HOME_1, ROOM_2_1);
  }
  scanRoom2();
  align(FORWARD);
  mapDrive(50, ROOM_2_1, ROOM_3_1);
  scanRoom3();
  align(FORWARD);
  mapDrive(50, ROOM_3_1, ROOM_4_1);
  scanRoom4();
  if(isDog || !isRoom1_1Default)
  {
    checkRoom1(true);
    scanRoom1(true);
    mapDrive(50, ROOM_1_1, HOME_1);
  }
  else
  {
    mapDrive(50, ROOM_4_1, HOME_1);
  }
  stopProgram();
}


void motorControl(int motor, int speed, int dir)
{
  if (dir)
  {
    analogWrite(motor, map(speed, 0, 100, 0, 255));
    digitalWrite(motor + 1, LOW);
  }
  else
  {
    analogWrite(motor + 1, map(speed, 0, 100, 0, 255));
    digitalWrite(motor, LOW);
  }
}


void drive(int speed, int dir)
{
  switch(dir)
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


void stopRobot()
{
  for(int i = O_M_FR; i <= O_M_BL + 1; i++)
  {
    digitalWrite(i, LOW);
  }
}


void initMotor(int motor)
{
  pinMode(motor, OUTPUT);
  pinMode(motor + 1, OUTPUT);
}


double getYaw()
{
  timer = millis();
  Vector norm = mpu.readNormalizeGyro();
  
  yaw += norm.ZAxis * timeStep;
  delay(timeStep * 1000 - (millis() - timer));
  return yaw;
}


void turn(int speed, int dir)
{
  if (dir == RIGHT)
  {
    motorControl(M_FL, speed, FORWARD);
    motorControl(M_FR, speed, FORWARD);
    motorControl(M_BL, speed, FORWARD);
    motorControl(M_BR, speed, FORWARD);
  }
  else if(dir == LEFT)
  {
    motorControl(M_FL, speed, BACKWARD);
    motorControl(M_FR, speed, BACKWARD);
    motorControl(M_BL, speed, BACKWARD);
    motorControl(M_BR, speed, BACKWARD);
  }
}


void gyroTurn(int angle, int speed)
{
  yaw = 0; //Resets the yaw of the robot.
  // angle - (speed / 2); //A fix for the degrees based on the offset caused by the speed.
  int newAngle = angle > 0 ? angle / 2 + 23: angle / 2 - 23;
  if (newAngle > 0)
  {
    turn(speed, RIGHT);
  }
  else if (newAngle < 0)
  {
    turn(speed, LEFT);
  }
  while (abs(abs(newAngle) - abs(yaw)) >= 3)
  {
    yaw = getYaw();
  }
  stopRobot();
}


void gyroDrive(int speed, int dir, int angle)
{
  double p = 0.34;
  double t = 0;
  int limit = 100;
  while(true) {
    t = angle - getYaw();
    switch(dir)
    {
      case BACKWARD:
        motorControl(M_FL, speed, BACKWARD);
        motorControl(M_FR, speed, FORWARD);
        motorControl(M_BL, speed, BACKWARD);
        motorControl(M_BR, speed, FORWARD);
        break;
      case FORWARD:
        motorControl(M_FL, (speed - t * p) > limit ? limit : (speed - t * p), FORWARD);
        motorControl(M_FR, (speed + t * p) > limit ? limit : (speed + t * p), BACKWARD);
        motorControl(M_BL, (speed - t * p) > limit ? limit : (speed - t * p), FORWARD);
        motorControl(M_BR, (speed + t * p) > limit ? limit : (speed + t * p), BACKWARD);
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
}


void checkUS()
{
  double currVal = 0;
  for (int i = 34; i <= 46; i += 2)
  {
    currVal = readUS(i);
    Serial.print("US connected in ");
    Serial.print(i);
    Serial.print("  ");
    Serial.print(currVal);
    Serial.print("\n");
    delay(3000);
  }

}


double readUS(int US)
{
  double ans = 0;
  int i = 0;
  int Echo = US;
  int trig = US + 1;

  for(i = 0; i < 3; i++)
  {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    ans += (pulseIn(Echo, HIGH) * 0.034) / 2;
    delayMicroseconds(10);
  }
  return ans / 3.0;
}


void initUS(int US)
{
  pinMode(US, INPUT);
  pinMode(US + 1, OUTPUT);
}


int readIR(int ir)
{
  return analogRead(ir);
}

void align(int face)
{
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
  err = (int)readUS(left_us) - (int)readUS(right_us);
  while (abs(err) != 0)
  {
    if (err > 0)
    {
      turn((p * abs(err)) > 60 ? 60 : (p*abs(err)) < 20 ? 20 : (p*abs(err)), RIGHT);
    }
    else
    {
      turn((p * abs(err)) > 60 ? 60 : (p*abs(err)) < 20 ? 20 : (p*abs(err)), LEFT);
    }
    err = (int)readUS(left_us) - (int)readUS(right_us);
  }
  stopRobot();
  err = (int)readUS(left_us) - (int)readUS(right_us);
  while (abs(err) != 0)
  {
    if (err > 0)
    {
      turn((p * abs(err)) > 60 ? 60 : (p*abs(err)) < 15 ? 15 : (p*abs(err)), RIGHT);
    }
    else
    {
      turn((p * abs(err)) > 60 ? 60 : (p*abs(err)) < 15 ? 15 : (p*abs(err)), LEFT);
    }
    err = (int)readUS(left_us) - (int)readUS(right_us);
  }
  stopRobot();
}


void gyroUSDrive(int speed)
{
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
  while(!foundWall && (readUS(US_FR) > minDist) && (readUS(US_FL) > minDist))
  {
    if (readUS(US_RL) <  30 && readUS(US_RR) <  30)
    {
      currDistUS = US_RL;
      currLYawUS = US_RL;
      currRYawUS = US_RR;
      negativeUS = 1;
      foundWall = true;
    }
    else if (readUS(US_LR) <  30 && readUS(US_LL) <  30)
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

  if(currFace == FORWARD || currFace == RIGHT || currFace == LEFT)
  {
    negativeFace = negativeFace * 1;
  }
  else
  {
    negativeFace = negativeFace * -1;
  }
  
  while (!checkHole(US_LL, flagUS_LL) && !checkHole(US_RR, flagUS_RR) && (readUS(US_FR) > minDist) && (readUS(US_FL) > minDist))
  {
    if(readUS(US_RR) < 30 && !flagUS_RR)
    {
      flagUS_RR = true;
    }
    if(readUS(US_LL) < 30 && !flagUS_LL)
    {
      flagUS_LL = true;
    }
    
    if(readUS(currDistUS) > 30)
    {
      drive(speed, FORWARD);
    }
    else
    {
      yawFix = (((readUS(currRYawUS) - readUS(currLYawUS)) * pYaw > SPEED_LIMIT(speed) ? SPEED_LIMIT(speed) : (readUS(currRYawUS) - readUS(currLYawUS)) * pYaw));
      usFix = negativeUS * (((readUS(currDistUS) - wallDist) * pUS) > SPEED_LIMIT(speed) ? SPEED_LIMIT(speed) : ((readUS(currDistUS) - wallDist) * pUS));
  
      motorControl(M_FR, speed - usFix, BACKWARD);
      motorControl(M_FL, readUS(currRYawUS) > 30 || readUS(currLYawUS) > 30 ? speed : speed - yawFix, FORWARD); // affected by gyro fix
      motorControl(M_BR, readUS(currRYawUS) > 30 || readUS(currLYawUS) > 30 ? speed : speed + yawFix, BACKWARD); // affected by gyro fix
      motorControl(M_BL, speed - usFix, FORWARD);
    }
  }
  stopRobot();
  delay(400);
}

bool checkHole(int us, bool flag)
{
  int holeDist = 30;
  return readUS(us) >= holeDist && flag;
}


void faceCycle(int face)
{
  currFace = face;
  switch(face)
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
      //facingDeg = 0;
      swapDir = 1;
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
      //facingDeg = 90;
      swapDir = 1;
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
      //facingDeg = 180;
      //swapDir = -1;
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
      //facingDeg = -90;
      //swapDir = -1;
      break;
  }
}


int pathfind(int x1, int y1, int x2, int y2)
{
  int count = 0;
  int flag = 1;
  int i = 0;
  int x = x1;
  int y = y1;
  int counter = 0;
  char check[4] = { 'U', 'R', 'D', 'L' };

  if (_map[y2][x2] == '1')
  {
    //printf("(x2,y2) is a wall.\n");
    return 0;
  }
  if (_map[y1][x1] == '1')
  {
    //printf("(x1,y1) is a wall.\n");
    return 0;
  }

  for (i = 0; i < PATH_SIZE; i++)
  {
    path[i] = 0;
  }

  while (x != x2 || y != y2)
  {
    if (_map[y][x] == '+')
    {
      //printf("Stuck, Could not find a suitable path!\n");
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
      //printf("*");
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
          //putchar(check[i]);
          path[counter++] = check[i];
          y++;
          flag = 0;
        }
        break;
      case 'R':
        if (_map[y][x - 1] == '0')
        {
          //putchar(check[i]);
          path[counter++] = check[i];
          x--;
          flag = 0;
        }
        break;
      case 'D':
        if (_map[y - 1][x] == '0')
        {
          //putchar(check[i]);
          path[counter++] = check[i];
          y--;
          flag = 0;
        }
        break;
      case 'L':
        if (_map[y][x + 1] == '0')
        {
          //putchar(check[i]);
          path[counter++] = check[i];
          x++;
          flag = 0;
        }
        break;
      default:
        //printf("Stuck, Could not find a suitable path!");
        break;
      }
    }
    //getchar();
    //printMap();
  }
  //printf("\n");
  _map[y][x] = 'O';
  _map[y1][x1] = 'X';
  return 1;
}

void clearPath()
{
  int x = 0;
  int y = 0;
  int i = 0;
  for (y = 0; y < Y; y++)
  {
    for (x = 0; x < X; x++)
    {
      _map[y][x] = originalMap[y][x];
    }
  }
  for (i = 0; i < PATH_SIZE; i++)
  {
    path[i] = 0;
  }
}

void minimizePath()
{
  char newPath[PATH_SIZE] = { 0 };
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

void translateDrive(int speed)
{
  int i = 0;
  for (i = 0; path[i] != 0; i++)
  {
    switch (path[i])
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
    gyroUSDrive(speed);
  }
  stopRobot();
  delay(200);
  faceCycle(FORWARD);
}


void mapDrive(int speed, int x1, int y1, int x2, int y2)
{
  if(checkIfRoom(x1, y1, ROOM_1_1) && !isRoom1_1Updated && isRoom1_1Checked)
  {
    isRoom1_1Updated = true;
    pathfind(8, 1, x2, y2);
  }
  else
  {
    pathfind(x1, y1, x2, y2);
  }
  minimizePath();
  translateDrive(speed);
  clearPath();
  if (checkIfRoom(x2, y2, ROOM_3_1))
  {
    if(readUS(US_RR) < 20)
    {
      faceCycle(FORWARD);
      gyroUSDrive(50);
      delay(100);
    }
    align(FORWARD);
  }
}

bool checkIfRoom(int x1, int y1, int x2, int y2)
{
  return x1 == x2 && y1 == y2;
}

bool readUV()
{
  return !digitalRead(UV);
}

void checkRoom4()
{
  isRoom4_1Checked = true;
  faceCycle(FORWARD);
  if(readUS(US_LL) > 30 && readUS(US_LR) > 30)
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

void checkRoom1(bool reversed)
{
  isRoom1_1Checked = true;
  faceCycle(FORWARD);
  if(reversed)
  {
    while(readUS(US_RR) < 15 || readUS(US_RL) < 15)
    {
      if (readUS(US_RR) < 15)
      {
        drive(30, FORWARD);
      }
      else if (readUS(US_RL) < 15)
      {
        drive(30, BACKWARD);
      }
      else
      {
        stopRobot();
      }
    }
    stopRobot();
    while(readUS(US_BL) > 80 || readUS(US_BR) > 80 || readUS(US_FL) > 20 || readUS(US_FR) > 20)
    {
      drive(50, LEFT);
    }
    stopRobot();
    if(readUS(US_BL) < 20 && readUS(US_BR) < 20)
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
    if(readUS(US_LL) > 20 && readUS(US_LR) > 20)
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


void scanRoom1(bool reversed)
{
  faceCycle(FORWARD);
  delay(50);
  if(!isRoom1_1Default)
  {
    align(FORWARD);
    delay(50);
    gyroTurn(180, 50);
    if (readUV())
    {
      align(BACKWARD);
      delay(10);
      while(readUS(US_BR) < 48)
      {
        drive(50, FORWARD);
      }
      stopRobot();
      delay(50);
      pyroDetect();
      while(readUS(US_BR) > 15)
      {
        drive(50, BACKWARD);
      }
      stopRobot();
      delay(50);
    }
    gyroTurn(190, 50);
    delay(50);
    align(FORWARD);
    delay(100);
    faceCycle(RIGHT);
    gyroUSDrive(50);
    faceCycle(FORWARD);
  }
  else
  {
    if (reversed)
    {
      mapDrive(50, 8, 6, ROOM_1_1);
    }
    if (readUV())
    {
      align(BACKWARD);
      delay(10);
      while(readUS(US_BR) < 48)
      {
        drive(50, FORWARD);
      }
      stopRobot();
      delay(50);
      pyroDetect();
      while(readUS(US_BR) > 15)
      {
        drive(50, BACKWARD);
      }
      stopRobot();
      delay(50);
      mapDrive(50, ROOM_1_1, HOME_1);
      stopProgram();
    }
  }
}

void scanRoom2()
{
  faceCycle(FORWARD);
  align(FORWARD);
  delay(100);
  gyroTurn(180, 50);
  delay(300);
  if(readUV())
  {
    while((readUS(US_BR)+readUS(US_BL)) / 2 < 48)
    {
      drive(50, FORWARD);
    }
    stopRobot();
    delay(50);
    pyroDetect();
    delay(100);
    align(LEFT);
    delay(50);
    while(readUS(US_LL) > 15 || readUS(US_LR) > 15)
    {
      drive(50, LEFT);
    }
    stopRobot();
    while(readUS(US_BR) > 15)
    {
      drive(50, BACKWARD);
    }
    stopRobot();
    delay(50);
    gyroTurn(180, 50);
    mapDrive(50, ROOM_2_1, HOME_1);
    stopProgram();
  }
  gyroTurn(180, 50);
  delay(100);
  align(FORWARD);
  delay(100);
}

void scanRoom3()
{
  bool candle_detected = false;
  faceCycle(FORWARD);
  while (readUS(US_FR) > 15 && readUS(US_FL) > 15) //If both don't see a wall
  {
    drive(50, FORWARD);
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
    drive(50, RIGHT);
  }
  stopRobot();
  gyroTurn(135, 50);
  stopRobot();
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
  while(readUS(US_FR) > 15 || readUS(US_FL) > 15) // If one of the front sensors is nt close to a wall
  {
    drive(40, FORWARD);
  }
  while(readUS(US_BL) < 80 && ((readUS(US_LL) + readUS(US_LR)) / 2 > 12)) // If the back left sensor doesn't see a hole and the avg of the left sensors is not a wall
  {
    drive(50, LEFT);
  }
  stopRobot();
  delay(50);
  align(FORWARD);
  if(candle_detected)
  {
    if(readUS(US_LL) < 30 || readUS(US_LR) < 30)
    {
      faceCycle(BACKWARD);
      gyroUSDrive(50);
      faceCycle(FORWARD);
    }
    mapDrive(50, ROOM_3_1, HOME_1);
    stopProgram();
  }
}


void scanRoom4()
{
  bool isDefault = !(originalMap[8][6] == '0');
  bool candle_detected = false;
  while(readUS(US_FL) < 10 || readUS(US_FR) < 10)
  {
    drive(30, BACKWARD);
  }
  stopRobot();
  delay(100);
  while(readUS(US_BL) > 80 || readUS(US_BR) > 80 || (!isDefault ? readUS(US_RR) < 30 || readUS(US_RL) < 30 : 0))
  {
    drive(50, LEFT);
  }
  delay(150);
  stopRobot();
  delay(50);
  gyroTurn(isDefault ? -135 : -45, 50);
  delay(200);
  if(readUV())
  {
    candle_detected = true;
    // TODO: Add scan and extinguish
  }
  gyroTurn(isDefault ? 135 : 45, 50);
  if(!isDefault)
  {
    delay(200);
    align(RIGHT);
    delay(100);
    faceCycle(RIGHT);
    while(readUS(US_FR) > 20 && readUS(US_FL) > 20)
    {
      gyroUSDrive(50);
    }
    faceCycle(FORWARD);
    align(RIGHT);
  }
  else
  {
    while(readUS(US_BL) < 80 || readUS(US_BR) < 80)
    {
      drive(50, RIGHT);
    }
    stopRobot();
    faceCycle(BACKWARD);
    gyroUSDrive(50);
  }
  faceCycle(BACKWARD);
  gyroUSDrive(50);
}

// void scanRoom4()
// {
//   bool isDefault = originalMap[8][6] == '0';
//   bool isCandle = false;
//   faceCycle(FORWARD);
//   delay(50);
//   if(isDefault)
//   {
//     while((readUS(US_FR) + readUS(US_FL)) / 2.0 < 52)
//     {
//       drive(50, BACKWARD);
//     }
//     stopRobot();
//     delay(50);
//     align(RIGHT);
//     delay(300);
//     while(readUS(US_BL) > 30)
//     {
//       drive(50, LEFT);
//     }
//     stopRobot();
//     delay(20);
//   }
//   else
//   {
//     align(FORWARD);
//     wallAlign(FORWARD, 8);
//     delay(300);
//     while(readUS(US_BL) > 80)
//     {
//       drive(50, LEFT);
//     }
//     delay(150);
//     stopRobot();
//     delay(100);
//     align(FORWARD);
//     delay(50);
//   }
//   gyroTurn(isDefault ? -15 : 225, 50);
//   delay(300);
//   if(readUV())
//   {
//     isCandle = true;
//     delay(50);
//     pyroDetect();
//     delay(100);
//   }
//   gyroTurn(isDefault ? 25 : 135, 50);
//   delay(100);
//   if(!isDefault)
//   {
//     align(FORWARD);
//     delay(100);
//     while((readUS(US_FL) + readUS(US_FR) / 2) > 15)
//     {
//       drive(550, FORWARD);
//     }
//     delay(100);
//     align(FORWARD);
//     delay(100);
//     while(readUS(US_BL) < 75)
//     {
//       drive(40, RIGHT);
//     }
//     delay(250);
//     stopRobot();
//     delay(100);
//     align(FORWARD);
//     delay(20);
//     while(readUS(US_RL) > 30 && readUS(US_LR) > 30)
//     {
//       drive(42, BACKWARD);
//       delay(50);
//       drive(35, BACKWARD);
//       delay(10);
//     }
//     stopRobot();
//     delay(100);
//     faceCycle(BACKWARD);
//     gyroUSDrive(50);
//     delay(200);
//   }
//   else
//   {
//     faceCycle(RIGHT);
//     gyroUSDrive(50);
//     faceCycle(FORWARD);
//     delay(100);
//     align(RIGHT);
//     if(isDog)
//     {
//       faceCycle(BACKWARD);
//       gyroUSDrive(50);
//       faceCycle(FORWARD);
//     }
//   }
//   delay(300);
//   if (isCandle)
//   {
//     mapDrive(50, ROOM_4_1, HOME_1);
//     stopProgram();
//   }
// }

void startAlign()
{
  if((readUS(US_RL) + readUS(US_RR)) / 2.0 > 40)
  {
    gyroTurn(80, 50);
    delay(10);
    align(RIGHT);
    delay(100);
  }
}

void checkDog()
{
  if(readIR(IR_LL) < 100 && readIR(IR_LR) < 100)
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

void stopProgram()
{
  delay(100000000000);
}

void pyroSetup()
{
  pinMode(P_R, INPUT);
  pinMode(P_L, INPUT);
}

int pyroRead(int pyro)
{
  return digitalRead(pyro);
}

void pyroDetect()
{
  yaw = 0;
  while(!pyroRead(P_R) || !pyroRead(P_L))
  {
    yaw = getYaw();
    if(pyroRead(P_L))
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
  digitalWrite(M_FAN, HIGH);
  while(readUV())
  {
    delay(0.5);
  }
  digitalWrite(M_FAN, LOW);
  delay(200);
  yaw = yaw > 0 ? (abs(yaw) % 360) : ((abs(yaw) % 360) * -1);
  gyroTurn((yaw > 0 ? yaw + 5 : yaw), 20);
  faceCycle(FORWARD);
}


// void scanRoom3()
// {
//   faceCycle(FORWARD);
//   align(FORWARD);
//   delay(300);
//   while((readUS(US_FR) + readUS(US_FL)) / 2.0 > 13)///////
//   {
//     drive(50, FORWARD);
//   }
//   stopRobot();
//   delay(100);
//   checkRoom4();
//   while(readUS(US_BR) > 90 || readUS(US_BL) > 90 || readUS(US_RR) > 60)  
//   {
//     drive(50, RIGHT);
//   }
//   delay(120); /////////
//   stopRobot();
//   delay(100);
//   gyroTurn(135, 50);
//   delay(300);
//   if(readUV())
//   {
//     stopRobot();
//     pyroDetect();
//     stopRobot();
//     delay(50);
//     gyroTurn(-135, 50);  //////////
//     align(FORWARD);
//     delay(100);
//     while((readUS(US_FR) + readUS(US_FL)) / 2.0 > 12)
//     {
//      drive(50, FORWARD);
//     }
//     stopRobot();
//     delay(100);
//     align(FORWARD);
//     delay(50);
//     while((readUS(US_RL) + readUS(US_RR)) / 2 < 70)
//     {
//       drive(50, LEFT);
//     }
//     stopRobot();
//     delay(300);
//     align(FORWARD);
//     mapDrive(50, ROOM_3_1, HOME_1);
//     stopProgram();
//   }  
//   gyroTurn(-135, 50);
//   delay(100);
//   align(FORWARD);
//   delay(100);
//   wallAlign(FORWARD, 7);
//   delay(100);
//   while(readUS(US_RR) < 75) //////////
//   {
//     drive(50, LEFT);
//   }
//   //delay(50);  /////
//   stopRobot();
//   delay(100);
//   align(FORWARD);
// }


// void wallDrive(int speed, int dir, int face, bool both)
// {
//     int val = 7;
//     int dist = 30;
//     int curr = 0;
//     int limit = 80;
//     double p = 3;
  
//     drive(speed, dir);
    
//     switch(dir)
//     {
//       case FORWARD:
//         switch(face)
//         {
//           case RIGHT:
//             while(!both ? readUS(US_RR) < dist : readUS(US_RR) < dist && readUS(US_LL) < dist)
//             {
//               curr = val - (int)(((int)readUS(US_RR) + (int)readUS(US_RL)) / 2);
//               if(curr < 0)
//               {
//                 motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p),BACKWARD);
//                 motorControl(M_FL, speed, FORWARD);
//                 motorControl(M_BR, speed, BACKWARD);
//                 motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
               
//               }
//               else
//               {
//                 motorControl(M_FR, speed, BACKWARD);
//                 motorControl(M_FL, (speed + curr * p * -1) > limit ? limit : (speed + curr *p* -1),FORWARD);
//                 motorControl(M_BR, (speed + curr * p * -1) > limit ? limit : (speed + curr *p* -1),BACKWARD);
//                 motorControl(M_BL, speed, FORWARD);
//               }
//               gyroTurn(0, speed);
//             }
//             break;
//           case LEFT:
//             while(!both ? readUS(US_RR) < dist : readUS(US_RR) < dist && readUS(US_LL) < dist)
//             {
//               curr = val - (int)(((int)readUS(US_LR) + (int)readUS(US_LL)) / 2);
//               if(curr < 0)
//               {
//                 motorControl(M_FR, speed,  BACKWARD);
//                 motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_BL, speed, FORWARD);
//               }
//               else
//               {
//                 motorControl(M_FR, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1), BACKWARD);
//                 motorControl(M_FL, speed, FORWARD);
//                 motorControl(M_BR, speed, BACKWARD);
//                 motorControl(M_BL, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1), FORWARD);
//               }
//               gyroTurn(0, speed);
//             }

//             break;
//         }
//         break;
//       case BACKWARD:
//         switch(face)
//         {
//           case RIGHT:
//             while(!both ? readUS(US_RL) < dist : readUS(US_RL) < dist && readUS(US_LR) < dist)
//             {
//               curr = val - (int)(((int)readUS(US_RR) + (int)readUS(US_RL)) / 2);
//               if(curr < 0)
//               {
//                 motorControl(M_FR, speed,FORWARD);
//                 motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_BL, speed, BACKWARD);
               
//               }
//               else
//               {
//                 motorControl(M_FR, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1), FORWARD);
//                 motorControl(M_FL, speed,BACKWARD);
//                 motorControl(M_BR, speed,FORWARD);
//                 motorControl(M_BL, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1), BACKWARD);
//               }
//               gyroTurn(0, speed);
//             }
//             break;
//           case LEFT:
//             while(!both ? readUS(US_RL) < dist : readUS(US_LR) < dist && readUS(US_RL) < dist)
//             {
//               curr = val - (int)(((int)readUS(US_LR) + (int)readUS(US_LL)) / 2);
//               if(curr < 0)
//               {
//                 motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p),  FORWARD);
//                 motorControl(M_FL, speed, BACKWARD);
//                 motorControl(M_BR, speed, FORWARD);
//                 motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//               }
//               else
//               {
//                 motorControl(M_FR, speed, FORWARD);
//                 motorControl(M_FL, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1), BACKWARD);
//                 motorControl(M_BR, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1), FORWARD);
//                 motorControl(M_BL, speed, BACKWARD);
//               }
//               gyroTurn(0, speed);
//             }
//             break;
//         }
//         break;
//       case RIGHT:
//         switch(face)
//         {
//           case FORWARD:
//             while(!both ? readUS(US_FR) < dist : readUS(US_FR) < dist && readUS(US_BL) < dist)
//             {
//               curr = val - (int)(((int)readUS(US_FR) + (int)readUS(US_FL)) / 2);
//               if(curr < 0)
//               {
//                 motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_FL, speed, FORWARD);
//                 motorControl(M_BR, speed, BACKWARD);
//                 motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
               
//               }
//               else
//               {
//                 motorControl(M_FR, speed, FORWARD);
//                 motorControl(M_FL, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1), FORWARD);
//                 motorControl(M_BR, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1),BACKWARD);
//                 motorControl(M_BL, speed, BACKWARD);
//               }
//               gyroTurn(0, speed);
//             }
//             break;
//           case BACKWARD:
//             while(!both ? readUS(US_BL) < dist : readUS(US_BL) < dist && readUS(US_FR) < dist)
//             {
//               curr = val - (int)(((int)readUS(US_BR) + (int)readUS(US_BL)) / 2);
//               if(curr < 0)
//               {
//                 motorControl(M_FR, speed, FORWARD);
//                 motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_BL, speed, BACKWARD);
               
//               }
//               else
//               {
//                 motorControl(M_FR, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1), FORWARD);
//                 motorControl(M_FL, speed, FORWARD);
//                 motorControl(M_BR, speed, BACKWARD);
//                 motorControl(M_BL, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1), BACKWARD);
//               }
//               gyroTurn(0, speed);
//             }
//             break;
//         }
//         break;
//       case LEFT:
//         switch(face)
//         {
//           case FORWARD:
//             while(!both ? readUS(US_FL) < dist : readUS(US_FL) < dist && readUS(US_BR) < dist)
//             {
//               curr = val - (int)(((int)readUS(US_FR) + (int)readUS(US_FL)) / 2);
//               if(curr < 0)
//               {
//                 motorControl(M_FR, speed, BACKWARD);
//                 motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_BL, speed, FORWARD);
               
//               }
//               else
//               {
//                 motorControl(M_FR, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1), BACKWARD);
//                 motorControl(M_FL, speed, BACKWARD);
//                 motorControl(M_BR, speed, FORWARD);
//                 motorControl(M_BL, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1), FORWARD);
//               }
//               gyroTurn(0, speed);
//             }
//             break;
//           case BACKWARD:
//             while(!both ? readUS(US_BR) < dist : readUS(US_BR) < dist && readUS(US_FL) < dist)
//             {
//               curr = val - (int)(((int)readUS(US_BR) + (int)readUS(US_BL)) / 2);
//               if(curr < 0)
//               {
//                 motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_FL, speed, BACKWARD);
//                 motorControl(M_BR, speed, FORWARD);
//                 motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
               
//               }
//               else
//               {
//                 motorControl(M_FR, speed, BACKWARD);
//                 motorControl(M_FL, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1), BACKWARD);
//                 motorControl(M_BR, (speed + curr *p* -1) > limit ? limit : (speed + curr *p* -1), FORWARD);
//                 motorControl(M_BL, speed, FORWARD);
//               }
//               gyroTurn(0, speed);
//             }
//             break;
//         }
//         break;
//     }
//     stopRobot();
// }

// void centeredDrive(int speed, int dir, int us, char greaterLess, int val)
// {
//     drive(speed, dir);
    
//     int dev = 0; // Deviation in short
//     int speedDiff = 7;
//     int max = 30;
//     int us_rr = 0;
//     int us_rl = 0;
//     int us_lr = 0;
//     int us_ll = 0;
//     int m_fr = 0;
//     int m_br = 0;
//     int m_bl = 0;
//     int m_fl = 0;
    
//     switch(dir)
//     {
//         case FORWARD:
//             us_rr = US_RR;
//             us_rl = US_RL;
//             us_lr = US_LR;
//             us_ll = US_LL;
//             m_fr = M_FR;
//             m_br = M_BR;
//             m_bl = M_BL;
//             m_fl = M_FL;
//             break;
//         case BACKWARD:
//             us_rr = US_LR;
//             us_rl = US_LL;
//             us_lr = US_RR;
//             us_ll = US_RL;
//             m_fr = M_BL;
//             m_br = M_FL;
//             m_bl = M_FR;
//             m_fl = M_BR;
//             break;
//         case LEFT:
//             us_rr = US_FR;
//             us_rl = US_FL;
//             us_lr = US_BR;
//             us_ll = US_BL;
//             m_fr = M_FL;
//             m_br = M_FR;
//             m_bl = M_BR;
//             m_fl = M_BL;
//             break;
//         case RIGHT:
//             us_rr = US_BR;
//             us_rl = US_BL;
//             us_lr = US_FR;
//             us_ll = US_FL;
//             m_fr = M_BR;
//             m_br = M_BL;
//             m_bl = M_FL;
//             m_fl = M_FR;
//             break;
//     }
    
//     while (greaterLess == '<' ? readUS(us) >= val : greaterLess == '>' ? readUS(us) <= val : false)
//     {
//         if (readUS(us_rr) < max && readUS(us_rl) < max && readUS(us_rr) - dev < readUS(us_rl))
//         {
//             motorControl(m_fl, speed + speedDiff, FORWARD);
//             motorControl(m_bl, speed + speedDiff, FORWARD);
//             motorControl(m_br, speed, BACKWARD);
//             motorControl(m_fr, speed,  BACKWARD);
//         }
//         else if (readUS(us_rr) < max && readUS(us_rl) < max && readUS(us_rr) > readUS(us_rl) - dev)
//         {
//             motorControl(m_br, speed + speedDiff, BACKWARD);
//             motorControl(m_fr, speed + speedDiff,  BACKWARD);
//             motorControl(m_fl, speed,  FORWARD);
//             motorControl(m_bl, speed, FORWARD);
//         }
//         else
//         {
//             motorControl(m_br, speed, BACKWARD);
//             motorControl(m_fr, speed, BACKWARD);
//             motorControl(m_fl, speed, FORWARD);
//             motorControl(m_bl, speed, FORWARD);
//         }
        
//         if (readUS(us_ll) < max && readUS(us_lr) < max && readUS(us_ll) - dev < readUS(us_lr))
//         {
//             motorControl(m_fr, speed + speedDiff, BACKWARD);
//             motorControl(m_br, speed + speedDiff, BACKWARD);
//             motorControl(m_bl, speed, FORWARD);
//             motorControl(m_fl, speed, FORWARD);
//         }
//         else if (readUS(us_ll) < max && readUS(us_lr) < max && readUS(us_ll) > readUS(us_lr) - dev)
//         {
//             motorControl(m_bl, speed + speedDiff, FORWARD);
//             motorControl(m_fl, speed + speedDiff, FORWARD);
//             motorControl(m_fr, speed, BACKWARD);
//             motorControl(m_br, speed, BACKWARD);
//         }
//         else
//         {
//             motorControl(m_br, speed, BACKWARD);
//             motorControl(m_fr, speed, BACKWARD);
//             motorControl(m_fl, speed, FORWARD);
//             motorControl(m_bl, speed, FORWARD);
//         }
//     }
    
//     stopRobot();
// }

// void smartDrive(int speed, int dir)
// {
//   int dist = 30;
//   int flag = 1;
//   bool both = false;
//   while(flag)
//   {
//     if(dir == FORWARD || dir == BACKWARD)
//     {
//       if(readUS(US_RR) < dist || readUS(US_RL) < dist)
//       {
//         alignedDrive(speed, dir, RIGHT, both);
//         flag = 0;
//       }
//       else if(readUS(US_LR) < dist || readUS(US_LL) < dist)
//       {
//         alignedDrive(speed, dir, LEFT, both);
//         flag = 0;
//       }
//       else
//       {
//         while(dir == BACKWARD ? (readUS(US_RL) > dist && readUS(US_LR) > dist) : (readUS(US_RR) > dist && readUS(US_LL) > dist))
//         {
//           drive(speed, dir);
//         }
//         delay(100);
//         stopRobot();
//         both = true;
//       }
//     }
//     else if(dir == LEFT || dir == RIGHT)
//     {
//       if(readUS(US_FR) < dist || readUS(US_FL) < dist)
//       {
//         alignedDrive(speed, dir, FORWARD, both);
//         flag = 0;
//       }
//       else if(readUS(US_BR) < dist || readUS(US_BL) < dist)
//       {
//         alignedDrive(speed, dir, BACKWARD, both);
//         flag = 0;
//       }
//       else
//       {
//         while(dir == RIGHT ? (readUS(US_BR) > dist && readUS(US_FL) > dist) : (readUS(US_FR) > dist && readUS(US_BL) > dist))
//         {
//           drive(speed, dir);
//         }
//         delay(100);
//         stopRobot();
//         both = true;
//       }
//     }
//     delay(100);
//   }
//   drive(speed, dir);
//   delay(-2.5*speed + 215);
//   stopRobot();
//   digitalWrite(M_FAN, LOW);
// }

// void cornerAlign(int speed, int corner, int dist)
// {
  
//   switch(corner)
//   {
//     case FR:
//       align(FORWARD);
//       delay(100);
//       while((int)((readUS(US_FR) + readUS(US_FL)) / 2) < dist - 1 || (int)((readUS(US_FR) + readUS(US_FL)) / 2) > dist + 1)
//       {
//         if((int)((readUS(US_FR) + readUS(US_FL)) / 2) < dist)
//         {
//           drive(speed, BACKWARD);
//         }
//         else
//         {
//           drive(speed, FORWARD);
//         }
//       }
//       stopRobot();
//       delay(100);
//       while((int)((readUS(US_RR) + readUS(US_RL)) / 2) < dist - 1 || (int)((readUS(US_RR) + readUS(US_RL)) / 2) > dist + 1)
//       {
//         if((int)((readUS(US_RR) + readUS(US_RL)) / 2) < dist)
//         {
//           drive(speed, LEFT);
//         }
//         else
//         {
//           drive(speed, RIGHT);
//         }
//       }
//       stopRobot();
//       delay(100);
//       align(RIGHT);
//       break;
//     case FL:
//       align(FORWARD);
//       delay(100);
//       while((int)((readUS(US_FR) + readUS(US_FL)) / 2) < dist - 1 || (int)((readUS(US_FR) + readUS(US_FL)) / 2) > dist + 1)
//       {
//         if((int)((readUS(US_FR) + readUS(US_FL)) / 2) < dist)
//         {
//           drive(speed, BACKWARD);
//         }
//         else
//         {
//           drive(speed, FORWARD);
//         }
//       }
//       stopRobot();
//       delay(100);
//       while((int)((readUS(US_LR) + readUS(US_LL)) / 2) < dist - 1 || (int)((readUS(US_LR) + readUS(US_LL)) / 2) > dist + 1)
//       {
//         if((int)((readUS(US_LR) + readUS(US_LL)) / 2) < dist)
//         {
//           drive(speed, RIGHT);
//         }
//         else
//         {
//           drive(speed, LEFT);
//         }
//       }
//       stopRobot();
//       delay(100);
//       align(LEFT);
//       break;
//     case BR:
//       align(BACKWARD);
//       delay(100);
//       while((int)((readUS(US_BR) + readUS(US_BL)) / 2) < dist - 1 || (int)((readUS(US_BR) + readUS(US_BL)) / 2) > dist + 1)
//       {
//         if((int)((readUS(US_BR) + readUS(US_BL)) / 2) < dist)
//         {
//           drive(speed, FORWARD);
//         }
//         else
//         {
//           drive(speed, BACKWARD);
//         }
//       }
//       stopRobot();
//       delay(100);
//       while((int)((readUS(US_RR) + readUS(US_RL)) / 2) < dist - 1 || (int)((readUS(US_RR) + readUS(US_RL)) / 2) > dist + 1)
//       {
//         if((int)((readUS(US_RR) + readUS(US_RL)) / 2) < dist)
//         {
//           drive(speed, LEFT);
//         }
//         else
//         {
//           drive(speed, RIGHT);
//         }
//       }
//       stopRobot();
//       delay(100);
//       align(RIGHT);
//       break;
//     case BL:
//       align(BACKWARD);
//       delay(100);
//       while((int)((readUS(US_BR) + readUS(US_BL)) / 2) < dist - 1 || (int)((readUS(US_BR) + readUS(US_BL)) / 2) > dist + 1)
//       {
//         if((int)((readUS(US_BR) + readUS(US_BL)) / 2) < dist)
//         {
//           drive(speed, FORWARD);
//         }
//         else
//         {
//           drive(speed, BACKWARD);
//         }
//       }
//       stopRobot();
//       delay(100);
//       while((int)((readUS(US_LR) + readUS(US_LL)) / 2) < dist - 1 || (int)((readUS(US_LR) + readUS(US_LL)) / 2) > dist + 1)
//       {
//         if((int)((readUS(US_LR) + readUS(US_LL)) / 2) < dist)
//         {
//           drive(speed, RIGHT);
//         }
//         else
//         {
//           drive(speed, LEFT);
//         }
//       }
//       stopRobot();
//       delay(100);
//       align(LEFT);
//       break;
//   }
// }

// void alignedDrive(int speed, int dir, int face, bool both)
// {
//     int dist = 30;
//     int curr = 0;
//     int limit = 80;
//     double p = 5;

//     drive(speed, dir);
    
//     switch(dir)
//     {
//       case FORWARD:
//         switch(face)
//         {
//           case RIGHT:
//             while(!both ? readUS(US_RR) < dist : readUS(US_RR) < dist && readUS(US_LL) < dist)
//             {
//               curr = (int)readUS(US_RR) - (int)readUS(US_RL);
//               if(curr < -1)
//               {
//                 motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_FL, speed, FORWARD);
//                 motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_BL, speed, FORWARD);
               
//               }
//               else if(curr > 1)
//               {
//                 motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_FL, speed,FORWARD);
//                 motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_BL, speed, FORWARD);
//               }
//               else
//               {
//                 wallDrive(speed, dir, face, both);
//               }
//             }
//             break;
//           case LEFT:
//             while(!both ? readUS(US_LL) < dist : readUS(US_RR) < dist && readUS(US_LL) < dist)
//             {
//               curr = (int)readUS(US_LL) - (int)readUS(US_LR);
//               if(curr < -1)
//               {
//                 motorControl(M_FR, speed,  BACKWARD);
//                 motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_BR, speed, BACKWARD);
//                 motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//               }
//               else if(curr > 1)
//               {
//                 motorControl(M_FR, speed,  BACKWARD);
//                 motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_BR, speed, BACKWARD);
//                 motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//               }
//               else
//               {
//                 wallDrive(speed, dir, face, both);
//               }
//             }
//             break;
//         }
//         break;
//       case BACKWARD:
//         switch(face)
//         {
//           case RIGHT:
//             while(!both ? readUS(US_RL) < dist : readUS(US_RL) < dist && readUS(US_LR) < dist)
//             {
//               curr = (int)readUS(US_RL) - (int)readUS(US_RR);
//               if(curr < -1)
//               {
//                 motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_FL, speed, BACKWARD);
//                 motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_BL, speed, BACKWARD);
               
//               }
//               else if(curr > 1)
//               {
//                 motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_FL, speed,BACKWARD);
//                 motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p),FORWARD);
//                 motorControl(M_BL, speed, BACKWARD);
//               }
//               else
//               {
//                 wallDrive(speed, dir, face, both);
//               }
              
//             }
//             break;
//           case LEFT:
//             while(!both ? readUS(US_LR) < dist : readUS(US_LR) < dist && readUS(US_RL) < dist)
//             {
//               curr = (int)readUS(US_LR) - (int)readUS(US_LL);
//               if(curr < -1)
//               {
//                 motorControl(M_FR, speed,  FORWARD);
//                 motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_BR, speed, FORWARD);
//                 motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//               }
//               else if(curr > 1)
//               {
//                 motorControl(M_FR, speed, FORWARD);
//                 motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_BR, speed, FORWARD);
//                 motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//               }
//               else
//               {
//                 wallDrive(speed, dir, face, both);
//               }
//             }
//             break;
//         }
//         break;
//       case RIGHT:
//         switch(face)
//         {
//           case FORWARD:
//             while(!both ? readUS(US_FR) < dist : readUS(US_FR) < dist && readUS(US_BL) < dist)
//             {
//               curr = (int)readUS(US_FL) - (int)readUS(US_FR);
//               if(curr < -1)
//               {
//                 motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_BR, speed, BACKWARD);
//                 motorControl(M_BL, speed, BACKWARD);
               
//               }
//               else if(curr > 1)
//               {
//                 motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_BR, speed, BACKWARD);
//                 motorControl(M_BL, speed, BACKWARD);
//               }
//               else
//               {
//                 wallDrive(speed, dir, face, both);
//               }
//             }
//             break;
//           case BACKWARD:
//             while(!both ? readUS(US_BL) < dist : readUS(US_BL) < dist && readUS(US_FR) < dist)
//             {
//               curr = (int)readUS(US_BR) - (int)readUS(US_BL);
//               if(curr < -1)
//               {
//                 motorControl(M_FR, speed, FORWARD);
//                 motorControl(M_FL, speed, FORWARD);
//                 motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
               
//               }
//               else if(curr > 1)
//               {
//                 motorControl(M_FR, speed, FORWARD);
//                 motorControl(M_FL, speed, FORWARD);
//                 motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//               }
//               else
//               {
//                 wallDrive(speed, dir, face, both);
//               }
//             }
//             break;
//         }
//         break;
//       case LEFT:
//         switch(face)
//         {
//           case FORWARD:
//             while(!both ? readUS(US_FL) < dist : readUS(US_FL) < dist && readUS(US_BR) < dist)
//             {
//               curr = (int)readUS(US_FR) - (int)readUS(US_FL);
//               if(curr < -1)
//               {
//                 motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_BR, speed, FORWARD);
//                 motorControl(M_BL, speed, FORWARD);
               
//               }
//               else if(curr > 1)
//               {
//                 motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
//                 motorControl(M_BR, speed, FORWARD);
//                 motorControl(M_BL, speed, FORWARD);
//               }
//               else
//               {
//                 wallDrive(speed, dir, face, both);
//               }
//             }
//             break;
//           case BACKWARD:
//             while(!both ? readUS(US_BR) < dist : readUS(US_BR) < dist && readUS(US_FL) < dist)
//             {
//               curr = (int)readUS(US_BL) - (int)readUS(US_BR);
//               if(curr < -1)
//               {
//                 motorControl(M_FR, speed, BACKWARD);
//                 motorControl(M_FL, speed, BACKWARD);
//                 motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
               
//               }
//               else if(curr > 1)
//               {
//                 motorControl(M_FR, speed, BACKWARD);
//                 motorControl(M_FL, speed, BACKWARD);
//                 motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//                 motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
//               }
//               else
//               {
//                 wallDrive(speed, dir, face, both);
//               }
//             }
//             break;
//         }
//         break;
//     }
//     stopRobot();
// }

void wallAlign(int face, int dist)
{
  int speed = 25;
  switch(face)
  {
    case FORWARD:
      while(dist != (int)((readUS(US_FL) + readUS(US_FR)) / 2))
      {
        if(dist + 1 > (int)((readUS(US_FL) + readUS(US_FR)) / 2))
        {
          drive(speed, BACKWARD);
        }
        else if(dist - 1 < (int)((readUS(US_FL) + readUS(US_FR)) / 2))
        {
          drive(speed, FORWARD);
        }
        else
        {
          stopRobot();
        }
      }
      stopRobot();
      break;
    case BACKWARD:
      while(dist != (int)((readUS(US_BL) + readUS(US_BR)) / 2))
      {
        if(dist + 1 > (int)((readUS(US_BL) + readUS(US_BR)) / 2))
        {
          drive(speed, FORWARD);
        }
        else if(dist - 1 < (int)((readUS(US_BL) + readUS(US_BR)) / 2))
        {
          drive(speed, BACKWARD);
        }
        else
        {
          stopRobot();
        }
      }
      stopRobot();
      break;
    case RIGHT:
      while(dist != (int)((readUS(US_RL) + readUS(US_RR)) / 2))
      {
        if(dist + 1 > (int)((readUS(US_RL) + readUS(US_RR)) / 2))
        {
          drive(speed, LEFT);
        }
        else if(dist - 1 < (int)((readUS(US_RL) + readUS(US_RR)) / 2))
        {
          drive(speed, RIGHT);
        }
        else
        {
          stopRobot();
        }
      }
      stopRobot();
      break;
    case LEFT:
      while(dist != (int)((readUS(US_LL) + readUS(US_LR)) / 2))
      {
        if(dist + 1 > (int)((readUS(US_LL) + readUS(US_LR)) / 2))
        {
          drive(speed, RIGHT);
        }
        else if(dist - 1 < (int)((readUS(US_LL) + readUS(US_LR)) / 2))
        {
          drive(speed, LEFT);
        }
        else
        {
          stopRobot();
        }
      }
      stopRobot();
      break;
  }
}
