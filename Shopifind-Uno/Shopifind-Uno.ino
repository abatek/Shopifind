#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

bool objectHeld = false;

//current values
float pitch;
float roll;
float yaw;
//target values
float tar_pitch = 0;
float tar_roll = 0;
float tar_yaw = 0;

float rangeThresh = 0.5;

//modes: 0 = tracking, 1 = go to object
int mode = 0;

//button
bool curPressed = false;
bool prevPressed = false;

float maxFrequency = 0.01; // blinks/ms
float threshold = 0.001;

int leftLedState = LOW;
int downLedState = LOW;
int upLedState = LOW;
int rightLedState = LOW;

unsigned long previousMillisPitch = 0;        // will store last time LED was updated
unsigned long previousMillisYaw = 0;

float leftFrequency = 0;
float downFrequency = 0;
float upFrequency = 0;
float rightFrequency = 0;

void checkSerial()
{
  int data;
  while (Serial.available())                                //check if data is available
  {
    Serial.println("receiving data");
    data = Serial.read();                                       //while data is available read the data
  }
  Serial.print("data received ");
  Serial.println(data);
  if (data == '1')
  {
      objectHeld = true;
  }else if (data == '0')
  {
      objectHeld = false;
  }

}

float getFrequency(float angle){
  float freq = maxFrequency * angle / 180;
  if (freq < threshold){
    return 0;
  }
  return freq;
}

void killLEDs()
{
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
}

 void setLEDFrequency()
{
  timer = millis();
  
  int normPitch = min(abs(tar_pitch - pitch), 360-abs(tar_pitch - pitch));
  int normRoll = min(abs(tar_roll - roll), 360-abs(tar_roll - roll));
  int normYaw = min(abs(tar_yaw - yaw), 360-abs(tar_yaw - yaw));
  
  leftFrequency = 0;
  downFrequency = 0;
  upFrequency = 0;
  rightFrequency = 0;
  
  if (pitch < 0){
    upFrequency = getFrequency(normPitch);
    downFrequency = 0;
  } else if (pitch > 0){
    downFrequency = getFrequency(normPitch);
    upFrequency = 0;
  }

  if (yaw < 0){
    leftFrequency = getFrequency(normYaw);
    rightFrequency = 0;
  } else if (yaw > 0){
    rightFrequency = getFrequency(normYaw);
    leftFrequency = 0;
  }

  if (upFrequency != 0){
    downLedState = LOW;
    float upPeriod = 1/upFrequency;
    if (millis() - previousMillisPitch >= upPeriod){
      if (upLedState == LOW){
        upLedState = HIGH;
      } else {
        upLedState = LOW;
      }
      previousMillisPitch = millis();
    }
  } else if (downFrequency != 0){
    upLedState = LOW;
    float downPeriod = 1/downFrequency;
    if (millis() - previousMillisPitch >= downPeriod){
      if (downLedState == LOW){
        downLedState = HIGH;
      } else {
        downLedState = LOW;
      }
      previousMillisPitch = millis();
    } 
  } else {
    downLedState = LOW;
    upLedState = LOW;
  }

  if (rightFrequency != 0){
    leftLedState = LOW;
    float rightPeriod = 1/rightFrequency;
    if (millis() - previousMillisYaw >= rightPeriod){
      if (rightLedState == LOW){
        rightLedState = HIGH;
      } else {
        rightLedState = LOW;
      }
      previousMillisYaw = millis();
    }
  } else if (leftFrequency != 0){
    rightLedState = LOW;
    float leftPeriod = 1/leftFrequency;
    if (millis() - previousMillisYaw >= leftPeriod){
      if (leftLedState == LOW){
        leftLedState = HIGH;
      } else {
        leftLedState = LOW;
      }
      previousMillisYaw = millis();
    } 
  } else {
    leftLedState = LOW;
    rightLedState = LOW;
  }

  
  digitalWrite(8, leftLedState);
  digitalWrite(9, downLedState);
  digitalWrite(10, upLedState);
  digitalWrite(11, rightLedState);
  
  
  //Serial.print(" Right Freq = ");
  Serial.println(rightFrequency*1000);
  //Serial.print(" Left Freq = ");
  Serial.println(leftFrequency*1000);
  //Serial.print(" Up Freq = ");
  Serial.println(upFrequency*1000);
  //Serial.print(" Down Freq = ");
  Serial.println(downFrequency*1000);
  
  

  delay((timeStep*1000) - (millis() - timer));
}

bool withinRange()
{
  return (rightFrequency == 0) && (leftFrequency == 0) && (upFrequency == 0) && (downFrequency == 0);
}

 void updateGyro()
{
  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.XAxis * timeStep;
  roll = roll + norm.YAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  if (pitch < -360) {
    pitch = pitch + 360;
  } else if (pitch > 360) {
    pitch = pitch - 360;
  }
  if (roll < -360) {
    roll = roll + 360;
  } else if (roll > 360) {
    roll = roll - 360;
  }
  if (yaw < -360) {
    yaw = yaw + 360;
  } else if (yaw > 360) {
    yaw = yaw - 360;
  }

}

void printGyro()
{
  Serial.print("p");
  Serial.print(pitch);
  Serial.print(" r");
  Serial.print(roll);
  Serial.print(" y");
  Serial.println(yaw);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  mpu.calibrateGyro();
  mpu.setThreshold(1);

  pinMode(2, INPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
}

void loop() {
  if (objectHeld)
  {
    digitalWrite(7, HIGH); 
  }else
  {
    digitalWrite(7, LOW);
  }

  
  curPressed = digitalRead(2);
  
  updateGyro();

  if (curPressed && !prevPressed) //catch rising edge
  {
    Serial.println(mode);
    killLEDs();
    if (mode == 1)
    {
      mode = 0;  
    }else 
    {
      mode = 1; 
    }
    
  }

  if (mode == 0)
  {
    checkSerial();
    
    if(objectHeld)
    {
      tar_pitch = pitch;
      tar_roll = roll;
      tar_yaw = yaw;
    }
    
  } else if (mode == 1)
  {
    setLEDFrequency();
    if (withinRange())
    {
      mode = 1;  
    }
    // when in range switch mode back to tracking
  }
  //printGyro();

  prevPressed = curPressed;
}

