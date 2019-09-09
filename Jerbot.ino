#include <Wire.h>
#include <MPU6050.h>

#define BACKWARD 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3
#define FR 0
#define FL 1
#define BR 2
#define BL 3


#define X 12
#define Y 25
#define PATH_SIZE 50

#define HOME_1 5, 1
#define ROOM_1_1 8, 1
#define ROOM_2_1 1, 6
#define ROOM_3_1 5, 11
#define ROOM_4_1 5, 11
#define HOME_2 5, 13
#define ROOM_1_2 8, 13
#define ROOM_2_2 1, 18
#define ROOM_3_2 5, 23
#define ROOM_4_2 5, 23
// ***** Pathfinder Variables ***** //

char path[PATH_SIZE] = { 0 };
char _map[Y][X] = {
  {'1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1'},
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
const char originalMap[Y][X] = {
  {'1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1', '1'},
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
const int M_FL = 4;
const int M_FR = 2;
const int M_BL = 8;
const int M_BR = 6;

// ***** IR Connections ****** //
const int IR_LR = A2;
const int IR_LL = A3;
const int IR_FR = A0;
const int IR_FL = A1;

// ***** US Connections ****** //
const int US_FL = 46;
const int US_FR = 48;
const int US_RL = 44;
const int US_RR = 42;
const int US_BL = 40;
const int US_LL = 38;
const int US_LR = 36;
const int US_BR = 34;

// ***** Function declares ***** //
void mapDrive(int speed, int x1, int y1, int x2, int y2);
void motorControl(int motor, int speed, int dir);



// ***** Variables ****** //
unsigned long timer = 0;
double yaw = 0;
float timeStep = 0.01;
int ir = IR_FR;

MPU6050 mpu;

void setup() 
{
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
  Serial.begin(9600);
  }

void loop() 
{
  delay(100);
  cornerAlign(30, BR, 8);
  delay(200);
  mapDrive(60, HOME_1, ROOM_2_1);
  delay(200);
  cornerAlign(30, FR, 8);
  delay(200);
  align(FORWARD);
  delay(200);
  mapDrive(60, ROOM_2_1, ROOM_3_1);
  delay(200);
  align(FORWARD);
  delay(500);
  mapDrive(60, ROOM_3_1, HOME_1);
  delay(200);
  align(RIGHT);

  delay(1000000);
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
  for(int i = M_FR; i <= M_BL + 1; i++)
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
  angle = angle - (speed / 2); //A fix for the degrees based on the offset caused by the speed.
  if (angle > 0)
  {
    turn(speed, RIGHT);
  }
  else if (angle < 0)
  {
    turn(speed, LEFT);
  }
  while (abs(yaw) <= angle)
  {
    yaw = getYaw();
  }
  stopRobot();
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
  for(int i = 0; i < 3; i++)
  {
    int Echo = US;
    int trig = US + 1;
    
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    
    ans += pulseIn(Echo, HIGH) * 0.034 / 2;
    delay(10);
  }
  return ans / 3;
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
  double p = 9;
  int err = 0;
  int left_us = 0; 
  int right_us = 0;
  int turnDir = FORWARD;

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
  Serial.print(err);
  Serial.print("\n\n");
  while (abs(err) != 0)
  {
    Serial.print(p * abs(err)) > 100 ? 100 : (p*abs(err));
    Serial.print("\n\n");
    if (err > 0)
    {
      turn((p * abs(err)) > 100 ? 100 : (p*abs(err)) < 15 ? 15 : (p*abs(err)), RIGHT);
    }
    else
    {
      turn((p * abs(err)) > 100 ? 100 : (p*abs(err)) < 15 ? 15 : (p*abs(err)), LEFT);
    }
    err = (int)readUS(left_us) - (int)readUS(right_us);
    Serial.print(abs(err));
    Serial.print("\n");
  }
  stopRobot();
}


void alignedDrive(int speed, int dir, int us, char greaterLess, int val)
{
    drive(speed, dir);
    
    int dev = 0; //Deviation in short
    int speedDiff = 7;
    int max = 30;
    int us_rr = 0;
    int us_rl = 0;
    int us_lr = 0;
    int us_ll = 0;
    int m_fr = 0;
    int m_br = 0;
    int m_bl = 0;
    int m_fl = 0;
    
    switch(dir)
    {
        case FORWARD:
            us_rr = US_RR;
            us_rl = US_RL;
            us_lr = US_LR;
            us_ll = US_LL;
            m_fr = M_FR;
            m_br = M_BR;
            m_bl = M_BL;
            m_fl = M_FL;
            break;
        case BACKWARD:
            us_rr = US_LR;
            us_rl = US_LL;
            us_lr = US_RR;
            us_ll = US_RL;
            m_fr = M_BL;
            m_br = M_FL;
            m_bl = M_FR;
            m_fl = M_BR;
            break;
        case LEFT:
            us_rr = US_FR;
            us_rl = US_FL;
            us_lr = US_BR;
            us_ll = US_BL;
            m_fr = M_FL;
            m_br = M_FR;
            m_bl = M_BR;
            m_fl = M_BL;
            break;
        case RIGHT:
            us_rr = US_BR;
            us_rl = US_BL;
            us_lr = US_FR;
            us_ll = US_FL;
            m_fr = M_BR;
            m_br = M_BL;
            m_bl = M_FL;
            m_fl = M_FR;
            break;
    }
    
    while (greaterLess == '<' ? readUS(us) >= val : greaterLess == '>' ? readUS(us) <= val : false)
    {
        if (readUS(us_rr) < max && readUS(us_rl) < max && readUS(us_rr) - dev < readUS(us_rl))
        {
            motorControl(m_fl, speed + speedDiff, FORWARD);
            motorControl(m_bl, speed + speedDiff, FORWARD);
            motorControl(m_br, speed, BACKWARD);
            motorControl(m_fr, speed,  BACKWARD);
        }
        else if (readUS(us_rr) < max && readUS(us_rl) < max && readUS(us_rr) > readUS(us_rl) - dev)
        {
            motorControl(m_br, speed + speedDiff, BACKWARD);
            motorControl(m_fr, speed + speedDiff,  BACKWARD);
            motorControl(m_fl, speed,  FORWARD);
            motorControl(m_bl, speed, FORWARD);
        }
        else
        {
            motorControl(m_br, speed, BACKWARD);
            motorControl(m_fr, speed, BACKWARD);
            motorControl(m_fl, speed, FORWARD);
            motorControl(m_bl, speed, FORWARD);
        }
        
        if (readUS(us_ll) < max && readUS(us_lr) < max && readUS(us_ll) - dev < readUS(us_lr))
        {
            motorControl(m_fr, speed + speedDiff, BACKWARD);
            motorControl(m_br, speed + speedDiff, BACKWARD);
            motorControl(m_bl, speed, FORWARD);
            motorControl(m_fl, speed, FORWARD);
        }
        else if (readUS(us_ll) < max && readUS(us_lr) < max && readUS(us_ll) > readUS(us_lr) - dev)
        {
            motorControl(m_bl, speed + speedDiff, FORWARD);
            motorControl(m_fl, speed + speedDiff, FORWARD);
            motorControl(m_fr, speed, BACKWARD);
            motorControl(m_br, speed, BACKWARD);
        }
        else
        {
            motorControl(m_br, speed, BACKWARD);
            motorControl(m_fr, speed, BACKWARD);
            motorControl(m_fl, speed, FORWARD);
            motorControl(m_bl, speed, FORWARD);
        }
    }
    
    stopRobot();
}

void smartDrive(int speed, int dir)
{
  int dist = 30;
  int flag = 1;
  bool both = false;
  while(flag)
  {
    if(dir == FORWARD || dir == BACKWARD)
    {
      if(readUS(US_RR) < dist || readUS(US_RL) < dist)
      {
        wallDrive(speed, dir, RIGHT, both);
        flag = 0;
      }
      else if(readUS(US_LR) < dist || readUS(US_LL) < dist)
      {
        wallDrive(speed, dir, LEFT, both);
        flag = 0;
      }
      else
      {
        while(dir == BACKWARD ? (readUS(US_RL) > dist && readUS(US_LR) > dist) : (readUS(US_RR) > dist && readUS(US_LL) > dist))
        {
          drive(speed, dir);
        }
        stopRobot();
        both = true;
      }
    }
    else if(dir == LEFT || dir == RIGHT)
    {
      if(readUS(US_FR) < dist || readUS(US_FL) < dist)
      {
        wallDrive(speed, dir, FORWARD, both);
        flag = 0;
      }
      else if(readUS(US_BR) < dist || readUS(US_BL) < dist)
      {
        wallDrive(speed, dir, BACKWARD, both);
        flag = 0;
      }
      else
      {
        while(dir == RIGHT ? readUS(US_BR) > dist && readUS(US_FL) > dist : readUS(US_FR) > dist && readUS(US_BL) > dist)
        {
          drive(speed, dir);
        }
        stopRobot();
        both = true;
      }
    }
    delay(100);
  }
  drive(speed, dir);
  delay(-2.5*speed + 230);
  stopRobot();
}

void wallDrive(int speed, int dir, int face, bool both)
{
    int val = 8;
    int dist = 30;
    int curr = 0;
    int limit = 80;
    double p = 2;

    drive(speed, dir);
    
    switch(dir)
    {
      case FORWARD:
        switch(face)
        {
          case RIGHT:
            while(!both ? readUS(US_RL) < dist : readUS(US_RL) < dist && readUS(US_LR) < dist)
            {
              curr = val - (int)(((int)readUS(US_RR) + (int)readUS(US_RL)) / 2);
              if(curr < 0)
              {
                motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p),BACKWARD);
                motorControl(M_FL, speed, FORWARD);
                motorControl(M_BR, speed, BACKWARD);
                motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
               
              }
              else
              {
                motorControl(M_FR, speed, BACKWARD);
                motorControl(M_FL, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1),FORWARD);
                motorControl(M_BR, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1),BACKWARD);
                motorControl(M_BL, speed, FORWARD);
              }
            }
            break;
          case LEFT:
            while(!both ? readUS(US_LR) < dist : readUS(US_LR) < dist && readUS(US_RL) < dist)
            {
              curr = val - (int)(((int)readUS(US_LR) + (int)readUS(US_LL)) / 2);
              if(curr < 0)
              {
                motorControl(M_FR, speed,  BACKWARD);
                motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
                motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
                motorControl(M_BL, speed, FORWARD);
              }
              else
              {
                motorControl(M_FR, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), BACKWARD);
                motorControl(M_FL, speed, FORWARD);
                motorControl(M_BR, speed, BACKWARD);
                motorControl(M_BL, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), FORWARD);
              }
            }
            break;
        }
        break;
      case BACKWARD:
        switch(face)
        {
          case RIGHT:
            while(!both ? readUS(US_RR) < dist : readUS(US_RR) < dist && readUS(US_LL) < dist)
            {
              curr = val - (int)(((int)readUS(US_RR) + (int)readUS(US_RL)) / 2);
              if(curr < 0)
              {
                motorControl(M_FR, speed,FORWARD);
                motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
                motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
                motorControl(M_BL, speed, BACKWARD);
               
              }
              else
              {
                motorControl(M_FR, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), FORWARD);
                motorControl(M_FL, speed,BACKWARD);
                motorControl(M_BR, speed,FORWARD);
                motorControl(M_BL, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), BACKWARD);
              }
            }
            break;
          case LEFT:
            while(!both ? readUS(US_LL) < dist : readUS(US_LL) < dist && readUS(US_RR) < dist)
            {
              curr = val - (int)(((int)readUS(US_LR) + (int)readUS(US_LL)) / 2);
              if(curr < 0)
              {
                motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p),  FORWARD);
                motorControl(M_FL, speed, BACKWARD);
                motorControl(M_BR, speed, FORWARD);
                motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
              }
              else
              {
                motorControl(M_FR, speed, FORWARD);
                motorControl(M_FL, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), BACKWARD);
                motorControl(M_BR, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), FORWARD);
                motorControl(M_BL, speed, BACKWARD);
              }
            }
            break;
        }
        break;
      case RIGHT:
        switch(face)
        {
          case FORWARD:
            while(!both ? readUS(US_FR) < dist : readUS(US_FR) < dist && readUS(US_BL) < dist)
            {
              curr = val - (int)(((int)readUS(US_FR) + (int)readUS(US_FL)) / 2);
              if(curr < 0)
              {
                motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
                motorControl(M_FL, speed, FORWARD);
                motorControl(M_BR, speed, BACKWARD);
                motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
               
              }
              else
              {
                motorControl(M_FR, speed, FORWARD);
                motorControl(M_FL, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), FORWARD);
                motorControl(M_BR, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1),BACKWARD);
                motorControl(M_BL, speed, BACKWARD);
              }
            }
            break;
          case BACKWARD:
            while(!both ? readUS(US_BL) < dist : readUS(US_BL) < dist && readUS(US_FR) < dist)
            {
              curr = val - (int)(((int)readUS(US_BR) + (int)readUS(US_BL)) / 2);
              if(curr < 0)
              {
                motorControl(M_FR, speed, FORWARD);
                motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
                motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
                motorControl(M_BL, speed, BACKWARD);
               
              }
              else
              {
                motorControl(M_FR, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), FORWARD);
                motorControl(M_FL, speed, FORWARD);
                motorControl(M_BR, speed, BACKWARD);
                motorControl(M_BL, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), BACKWARD);
              }
            }
            break;
        }
        break;
      case LEFT:
        switch(face)
        {
          case FORWARD:
            while(!both ? readUS(US_FL) < dist : readUS(US_FL) < dist && readUS(US_BR) < dist)
            {
              curr = val - (int)(((int)readUS(US_FR) + (int)readUS(US_FL)) / 2);
              if(curr < 0)
              {
                motorControl(M_FR, speed, BACKWARD);
                motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
                motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
                motorControl(M_BL, speed, FORWARD);
               
              }
              else
              {
                motorControl(M_FR, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), BACKWARD);
                motorControl(M_FL, speed, BACKWARD);
                motorControl(M_BR, speed, FORWARD);
                motorControl(M_BL, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), FORWARD);
              }
            }
            break;
          case BACKWARD:
            while(!both ? readUS(US_BR) < dist : readUS(US_BR) < dist && readUS(US_FL) < dist)
            {
              curr = val - (int)(((int)readUS(US_BR) + (int)readUS(US_BL)) / 2);
              if(curr < 0)
              {
                motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
                motorControl(M_FL, speed, BACKWARD);
                motorControl(M_BR, speed, FORWARD);
                motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
               
              }
              else
              {
                motorControl(M_FR, speed, BACKWARD);
                motorControl(M_FL, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), BACKWARD);
                motorControl(M_BR, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), FORWARD);
                motorControl(M_BL, speed, FORWARD);
              }
            }
            break;
        }
        break;
    }
    stopRobot();
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
        smartDrive(speed, FORWARD);
        break;

      case 'L':
        smartDrive(speed, LEFT);
        break;

      case 'R':
        smartDrive(speed, RIGHT);
        break;

      case 'D':
        smartDrive(speed, BACKWARD);
        break;

      default:
        stopRobot();
        break;
    }
  }
  stopRobot();
}

void cornerAlign(int speed, int corner, int dist)
{
  
  switch(corner)
  {
    case FR:
      align(FORWARD);
      delay(100);
      while((int)((readUS(US_FR) + readUS(US_FL)) / 2) < dist - 1 || (int)((readUS(US_FR) + readUS(US_FL)) / 2) > dist + 1)
      {
        if((int)((readUS(US_FR) + readUS(US_FL)) / 2) < dist)
        {
          drive(speed, BACKWARD);
        }
        else
        {
          drive(speed, FORWARD);
        }
      }
      stopRobot();
      delay(100);
      while((int)((readUS(US_RR) + readUS(US_RL)) / 2) < dist - 1 || (int)((readUS(US_RR) + readUS(US_RL)) / 2) > dist + 1)
      {
        if((int)((readUS(US_RR) + readUS(US_RL)) / 2) < dist)
        {
          drive(speed, LEFT);
        }
        else
        {
          drive(speed, RIGHT);
        }
      }
      stopRobot();
      delay(100);
      align(RIGHT);
      break;
    case FL:
      align(FORWARD);
      delay(100);
      while((int)((readUS(US_FR) + readUS(US_FL)) / 2) < dist - 1 || (int)((readUS(US_FR) + readUS(US_FL)) / 2) > dist + 1)
      {
        if((int)((readUS(US_FR) + readUS(US_FL)) / 2) < dist)
        {
          drive(speed, BACKWARD);
        }
        else
        {
          drive(speed, FORWARD);
        }
      }
      stopRobot();
      delay(100);
      while((int)((readUS(US_LR) + readUS(US_LL)) / 2) < dist - 1 || (int)((readUS(US_LR) + readUS(US_LL)) / 2) > dist + 1)
      {
        if((int)((readUS(US_LR) + readUS(US_LL)) / 2) < dist)
        {
          drive(speed, RIGHT);
        }
        else
        {
          drive(speed, LEFT);
        }
      }
      stopRobot();
      delay(100);
      align(LEFT);
      break;
    case BR:
      align(BACKWARD);
      delay(100);
      while((int)((readUS(US_BR) + readUS(US_BL)) / 2) < dist - 1 || (int)((readUS(US_BR) + readUS(US_BL)) / 2) > dist + 1)
      {
        if((int)((readUS(US_BR) + readUS(US_BL)) / 2) < dist)
        {
          drive(speed, FORWARD);
        }
        else
        {
          drive(speed, BACKWARD);
        }
      }
      stopRobot();
      delay(100);
      while((int)((readUS(US_RR) + readUS(US_RL)) / 2) < dist - 1 || (int)((readUS(US_RR) + readUS(US_RL)) / 2) > dist + 1)
      {
        if((int)((readUS(US_RR) + readUS(US_RL)) / 2) < dist)
        {
          drive(speed, LEFT);
        }
        else
        {
          drive(speed, RIGHT);
        }
      }
      stopRobot();
      delay(100);
      align(RIGHT);
      break;
    case BL:
      align(BACKWARD);
      delay(100);
      while((int)((readUS(US_BR) + readUS(US_BL)) / 2) < dist - 1 || (int)((readUS(US_BR) + readUS(US_BL)) / 2) > dist + 1)
      {
        if((int)((readUS(US_BR) + readUS(US_BL)) / 2) < dist)
        {
          drive(speed, FORWARD);
        }
        else
        {
          drive(speed, BACKWARD);
        }
      }
      stopRobot();
      delay(100);
      while((int)((readUS(US_LR) + readUS(US_LL)) / 2) < dist - 1 || (int)((readUS(US_LR) + readUS(US_LL)) / 2) > dist + 1)
      {
        if((int)((readUS(US_LR) + readUS(US_LL)) / 2) < dist)
        {
          drive(speed, RIGHT);
        }
        else
        {
          drive(speed, LEFT);
        }
      }
      stopRobot();
      delay(100);
      align(LEFT);
      break;
  }
}

// void mapDrive(int speed, int x1, int y1, int x2, int y2)
// {
//   pathfind(x1, y1, x2, y2);
//   minimizePath();
//   translateDrive(speed);
//   clearPath();
// }