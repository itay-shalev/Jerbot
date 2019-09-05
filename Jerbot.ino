#include <Wire.h>
#include <MPU6050.h>

#define BACKWARD 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3

// ***** Motor Connections ****** //
const int M_FL = 4;
const int M_FR = 2;
const int M_BL = 8;
const int M_BR = 6;
const int M_WIND = 0;

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

// ***** UV connections ***** //
const int UV = 0;

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
  //initMotor(M_WIND);
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
  align(RIGHT);
  delay(200);
  wallDrive(50, BACKWARD, RIGHT);
  
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



int searchAndExtinguish()
{
    if (avgUV(UV) > 500)
    {
        alignOnCandle();
        digitalWrite(M_WIND, HIGH);
        return true;
    }
    else
    {
        return false;
    }
    
}

void alignOnCandle()
{
    while (readIR(IR_FL) < 100) // Need to check for exact value.
    {
        turn(80, RIGHT);
    }
    stopRobot();
    while (readIR(IR_FR) < 100)
    {
        turn(60, LEFT);
    }
    stopRobot();
}

int avgUV(int pin)
{
    int readings = 8;
    unsigned int avg = 0;
    int i = 0;
    for (i = 0; i < readings; i++)
    {
        avg += analogRead(pin);
    }
    return avg/readings;
}

void initUV(int UV)
{
    pinmode(UV, INPUT);
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
      turn((p * abs(err)) > 100 ? 100 : (p*abs(err)),RIGHT );
    }
    else
    {
      turn((p * abs(err)) > 100 ? 100 : (p*abs(err)),LEFT );
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

void wallDrive(int speed, int dir, int face)
{
    int val = 8;
    int dist = 30;
    int curr = 0;
    int limit = 80;
    double p = 1.7;

    drive(speed, dir);
    
    switch(dir)
    {
      case FORWARD:
        switch(face)
        {
          case RIGHT:
            while(readUS(US_RL) < dist)
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
                motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p),FORWARD);
                motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p),BACKWARD);
                motorControl(M_BL, speed, FORWARD);
              }
            }
            break;
          case LEFT:
            while(readUS(US_LR) < dist)
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
            while(readUS(US_RR) < dist)
            {
              curr = val - (int)(((int)readUS(US_RR) + (int)readUS(US_RL)) / 2);
              if(curr < 0)
              {
                motorControl(M_FR, (speed + curr * p) > limit ? limit : (speed + curr * p),FORWARD);
                motorControl(M_FL, speed, BACKWARD);
                motorControl(M_BR, speed, FORWARD);
                motorControl(M_BL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
               
              }
              else
              {
                motorControl(M_FR, speed, FORWARD);
                motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p),BACKWARD);
                motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p),FORWARD);
                motorControl(M_BL, speed, BACKWARD);
              }
            }
            break;
          case LEFT:
            while(readUS(US_LL) < dist)
            {
              curr = val - (int)(((int)readUS(US_LR) + (int)readUS(US_LL)) / 2);
              if(curr < 0)
              {
                motorControl(M_FR, speed,  FORWARD);
                motorControl(M_FL, (speed + curr * p) > limit ? limit : (speed + curr * p), BACKWARD);
                motorControl(M_BR, (speed + curr * p) > limit ? limit : (speed + curr * p), FORWARD);
                motorControl(M_BL, speed, BACKWARD);
              }
              else
              {
                motorControl(M_FR, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), FORWARD);
                motorControl(M_FL, speed, BACKWARD);
                motorControl(M_BR, speed, FORWARD);
                motorControl(M_BL, (speed + curr * p * -1) > limit ? limit : (speed + curr * p * -1), BACKWARD);
              }
            }
            break;
        }
        break;
      case RIGHT:
        switch(face)
        {
          case FORWARD:
            break;
          case BACKWARD:
            break;
        }
        break;
      case LEFT:
        switch(face)
        {
          case FORWARD:
            break;
          case BACKWARD:
            break;
        }
        break;
    }
    stopRobot();
}