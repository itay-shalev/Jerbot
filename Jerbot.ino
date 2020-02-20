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
#define F_LED 27
#define MIC_LED 26

#define SPEED_LIMIT(min, x, max) (x > max ? max : (x > min ? x : min))
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
    analogWrite(motor, (int)(map(speed, 0, 100, 0, 255)));
    digitalWrite(motor + 1, LOW);
  }
  else
  {
    analogWrite(motor + 1, (int)(map(speed, 0, 100, 0, 255)));
    digitalWrite(motor, LOW);
  }
}


void drive(int speed, int dir)
{
  int _speed = speed;
  switch(dir)
  {
    case BACKWARD:
      motorControl(M_FL, _speed, BACKWARD);
      motorControl(M_FR, _speed, FORWARD);
      motorControl(M_BL, _speed, BACKWARD);
      motorControl(M_BR, _speed, FORWARD);
      break;
    case FORWARD:
      motorControl(M_FL, _speed, FORWARD);
      motorControl(M_FR, _speed, BACKWARD);
      motorControl(M_BL, _speed, FORWARD);
      motorControl(M_BR, _speed, BACKWARD);
      break;
   case LEFT:
      motorControl(M_FL, _speed, BACKWARD);
      motorControl(M_FR, _speed, BACKWARD);
      motorControl(M_BL, _speed, FORWARD);
      motorControl(M_BR, _speed, FORWARD);
      break;
    case RIGHT:
      motorControl(M_FL, _speed, FORWARD);
      motorControl(M_FR, _speed, FORWARD);
      motorControl(M_BL, _speed, BACKWARD);
      motorControl(M_BR, _speed, BACKWARD);
      break;
  }
}


void stopRobot()
{
  drive(50, BACKWARD);
  delay(0.0001);
  drive(50, FORWARD);
  delay(0.0001);
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
  
  yaw += abs(norm.ZAxis * timeStep) > 0.5 ? norm.ZAxis * timeStep : 0;
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

int readIR(int ir)
{
  return analogRead(ir);
}

int limitSpeed(int speed, int min, int max)
{
  return speed > max ? max : (speed < min ? min : speed);
}

void align(int face)
{
  double p = 7;
  int err = 1; 
  int prevFace = currFace;
  faceCycle(face);
  while (abs(err) != 0)
  {
    err = (int)readUS(US_FL) - (int)readUS(US_FR);
    if (err > 0)
    {
      turn(limitSpeed(p * abs(err), 30, 60), RIGHT);
    }
    else if (err < 0)
    {
      turn(limitSpeed(p * abs(err), 30, 60), LEFT);
    }
  }
  stopRobot();
  faceCycle(prevFace);
}

// This function goes in an outer loop
void wallDrive(int speed, int face)
{
  double pYaw = 5; // The p for the yaw alignment
  int yawDiff = 0; // The differance in the sensors
  int yawFix = 0; // The speed fix for the yaw
  double pDist = 3; // The p for keeping distance from the wall
  int distDiff = 0; // The differance between the wanted distance the current
  int distFix = 0; // The speed fix for the distance
  int distanceFromWall = 10; // The distance to keep from the wall
  int rightSensor = O_US_FR; // The right sensor on the face
  int leftSensor = O_US_FL; // The left sensor of the face
  int prevFace = currFace;

  faceCycle(face); // change the face of the robot

  if (readUS(rightSensor) < 30 && readUS(leftSensor) < 30) // If one of the sensors doesn't see a wall
  {
    yawDiff = readUS(rightSensor) - readUS(leftSensor); // The difference between the left and right sensors
    distDiff = distanceFromWall - ((readUS(rightSensor) + readUS(leftSensor)) / 2); // The differance between the average of the sensors and the target distance
  }
  else
  {
    yawDiff = 0; // Ignores the difference
    distDiff = 0; // Ignores the difference
  }
  
   // If the robot is not straight it ignores the distance speed fix, doesn't fix the distance
  if (abs(yawDiff) > 1)
  {
    distFix = 0; // Ignores the distance fix
  }
  else
  {
    distFix = distDiff * pDist; // Sets teh fix to the differance times p
  }
  
  yawFix = yawDiff * pYaw; // The fix is the differance times p
  motorControl(M_FR, SPEED_LIMIT(20, speed + distFix + yawFix, 80), BACKWARD);
  motorControl(M_FL, SPEED_LIMIT(20, speed - distFix - yawFix, 80), FORWARD);
  motorControl(M_BR, SPEED_LIMIT(20, speed - distFix + yawFix, 80), BACKWARD);
  motorControl(M_BL, SPEED_LIMIT(20, speed + distFix - yawFix, 80), FORWARD);

  faceCycle(prevFace); // Return to the previous face

}

void gyroUSDrive(int speed)
{
  double pYaw = 5; // The p for the yaw alignment
  int yawDiff = 0; // The differance in the sensors
  int yawFix = 0; // The speed fix for the yaw
  double pDist = 3; // The p for keeping distance from the wall
  int distDiff = 0; // The differance between the wanted distance the current
  int distFix = 0; // The speed fix for the distance
  int distanceFromWall = 15; // The distance to keep from the wall
  int rightSensor = US_RR; // The right sensor on the face
  int leftSensor = US_RL; // The left sensor of the face
  bool isRightSeenWall = false; // If the right sensor saw a wall
  bool isLeftSeenWall = false; // If the left sensor saw a wall

  drive(speed, FORWARD);
  while (readUS(US_RR) > 30 && readUS(US_LL) > 30); // Drive until a wall is found

  if (readUS(US_RR) > 30) // If it stopped on the left wall
  {
    pDist *= -1; // Reversing the p
    rightSensor = US_LR; // Setting the left sensor
    leftSensor = US_LL; // Setting the right sensor
  }
  
  
  // Drives until a sensor sees a hole
  while (!checkHole(US_RR, isRightSeenWall) && !checkHole(US_LL, isLeftSeenWall))
  {
    if (readUS(rightSensor) < 30 && readUS(leftSensor) < 30) // If one of the sensors doesn't see a wall
    {
      yawDiff = readUS(rightSensor) - readUS(leftSensor); // The difference between the left and right sensors
      distDiff = distanceFromWall - ((readUS(rightSensor) + readUS(leftSensor)) / 2); // The differance between the average of the sensors and the target distance
    }
    else
    {
      yawDiff = 0; // Ignores the difference
      distDiff = 0; // Ignores the difference
    }
    

    // If the robot is not straight it ignores the distance speed fix, doesn't fix the distance
    if (abs(yawDiff) > 1)
    {
      distFix = 0; // Ignores the distance fix
    }
    else
    {
      distFix = distDiff * pDist; // Sets teh fix to the differance times p
    }
    
    yawFix = yawDiff * pYaw; // The fix is the differance times p

    motorControl(M_FR, SPEED_LIMIT(20, speed + distFix + yawFix, 80), BACKWARD);
    motorControl(M_FL, SPEED_LIMIT(20, speed - distFix - yawFix, 80), FORWARD);
    motorControl(M_BR, SPEED_LIMIT(20, speed - distFix + yawFix, 80), BACKWARD);
    motorControl(M_BL, SPEED_LIMIT(20, speed + distFix - yawFix, 80), FORWARD);

    // If the sensor hasn't seen a wall and it sees a wall
    if (!isRightSeenWall && readUS(US_RR) < 30)
    {
      isRightSeenWall = true;
    }

    // If the sensor hasn't seen a wall and it sees a wall
    if (!isLeftSeenWall && readUS(US_LL) < 30)
    {
      isLeftSeenWall = true;
    }
  }
  stopRobot();
}

bool checkHole(int us, bool flag)
{
  int holeDist = 30; // The max distance to see a wall 
  return readUS(us) >= holeDist && flag; // If the sensor already saw a wall and now sees a hole
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
    return 0;
  }
  if (_map[y1][x1] == '1')
  {
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
    delay(400);
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
    delay(200);
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
  gyroTurn(-70, 50);
  gyroTurn(-70, 50);
  delay(300);
  if(readUV())
  {
    while((readUS(US_BR) + readUS(US_BL)) / 2 < 48)
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
  gyroTurn(80, 50);
  gyroTurn(80, 50);
  delay(100);
  align(FORWARD);
  delay(100);
}

void scanRoom3()
{
  bool candle_detected = false;
  faceCycle(FORWARD);
  // if ((readUS(US_FR) > 15 && readUS(US_FL) < 15) || (readUS(US_FR) < 15 && readUS(US_FL) > 15)) //Only if one of them sees a wall in front
  // {
  //   align(FORWARD);
  // }
  delay(100);
  checkRoom4();
  while (readUS(US_BL) > 80 || readUS(US_BR) > 80 || (readUS(US_RL) + readUS(US_RR)) / 2.0 > 50) // If one of the back sensors don't see a wall and if the avg of the right sensors doesn't get too close
  {
    wallDrive(50, RIGHT);
  }
  stopRobot();
  gyroTurn(100, 50);
  delay(200);
  if (readUV())
  {
    candle_detected = true;
    pyroDetect();
    delay(100);
  }
  gyroTurn(-105, 50);
  delay(50);
  align(FORWARD);
  delay(50);
  // while(readUS(US_FR) > 15 || readUS(US_FL) > 15) // If one of the front sensors is not close to a wall
  // {
  //   drive(40, FORWARD);
  // }
  while(readUS(US_BL) < 80 && ((readUS(US_LL) + readUS(US_LR)) / 2 > 12)) // If the back left sensor doesn't see a hole and the avg of the left sensors is not a wall
  {
    wallDrive(50, LEFT);
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
  // Get back from the wall
  align(FORWARD);
  delay(50);
  while(readUS(US_FL) < 10 || readUS(US_FR) < 10 || (!isDefault ? readUS(US_FL) < 52 || readUS(US_FR) < 52 : 0))
  {
    drive(!isDefault ? 50: 30, BACKWARD);
  }
  stopRobot();
  delay(50);

  // If the room is default it goes backwards
  if (isDefault)
  {
    align(FORWARD);
    delay(50);
  }
  while(isDefault && (readUS(US_FR) > 15 || readUS(US_FL) > 15))
  {
    drive(40, FORWARD);
  }
  stopRobot();
  delay(50);

  // Goes left into the room
  while(readUS(US_BL) > 80 || readUS(US_BR) > 80 || (!isDefault ? readUS(US_RR) < 30 || readUS(US_RL) < 30 : 0) || (isDefault ? readUS(US_LR) > 100 && readUS(US_LL) > 100 : 0))
  {
    drive(50, LEFT);
  }
  delay(150);
  stopRobot();
  delay(50);

  // Turns to face the candle
  gyroTurn(isDefault ? -135 : -45, 50);
  delay(200);

  // If there is fire it detects it and extinguishes it
  if(readUV())
  {
    candle_detected = true;
    pyroDetect();
  }

  // Turn back to the hallway
  gyroTurn(isDefault ? 135 : 45, 50);


  if(!isDefault)
  {
    // Goes out of the room
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
    // Goes away from the front wall
    align(FORWARD);
    delay(100);
    while(readUS(US_FL) < 10 || readUS(US_FR) < 10)
    {
      drive(30, BACKWARD);
    }
    stopRobot();
    delay(50);
    align(FORWARD);
    delay(50);

    // Goes to the wall if it is too far
    while(readUS(US_FR) > 15 || readUS(US_FL) > 15) // If one of the front sensors is not close to a wall
    {
      drive(40, FORWARD);
    }
    stopRobot();
    delay(100);

    // Goes out of the room
    while((readUS(US_BL) < 80 || readUS(US_BR) < 80) && readUS(US_RL) < 100)
    {
      drive(50, RIGHT);
    }
    stopRobot();
    delay(100);
    align(FORWARD);
    delay(100);

    // Starts driving diagonaly
    motorControl(M_FL, 10, BACKWARD);
    motorControl(M_FR, 50, FORWARD);
    motorControl(M_BL, 10, BACKWARD);
    motorControl(M_BR, 50, FORWARD);
    delay(50);
  }
  // Goes back out of the room to the junction
  faceCycle(BACKWARD);
  gyroUSDrive(50);
}


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

void pyroDetect()
{
  yaw = 0;
  digitalWrite(F_LED, HIGH);
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
  }
  digitalWrite(M_FAN, LOW);
  delay(200);
  yaw = yaw > 0 ? (abs(yaw) % 360) : ((abs(yaw) % 360) * -1);
  gyroTurn((yaw > 0 ? yaw : yaw), 20);
  faceCycle(FORWARD);
  digitalWrite(F_LED, LOW);
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

void pyroSetup()
{
  pinMode(P_R, INPUT);
  pinMode(P_L, INPUT);
}

int pyroRead(int pyro)
{
  return digitalRead(pyro);
}