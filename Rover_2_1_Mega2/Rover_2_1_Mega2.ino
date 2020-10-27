#include "CytronMotorDriver.h" //motor controler driver
#include <Servo.h> //servo driver
#include "SR04.h" //ultra sonic sensor driver
#include "Wire.h" //for i2c communication

//Pins for each motor for its pwm and dir
#define motor_fl_pwm  4
#define motor_fl_dir  24
#define motor_fr_pwm  5
#define motor_fr_dir  25
#define motor_bl_pwm  6
#define motor_bl_dir  26
#define motor_br_pwm  7
#define motor_br_dir  27

//Pins for encoders
#define encoder_bra 2
#define encoder_bla 3

//Pins for Ultrasonic Sensor
#define trig 28
#define echo 29
SR04 sr04=SR04(echo,trig);

#define SlaveAddr 5
int bcount=0;
int JS_Inputs[6]={128,128,128,128,0,0};
int Buts=0, Trigs=0;

//Configure the motor driver.
CytronMD motor_fl(PWM_DIR, motor_fl_pwm, motor_fl_dir);  
CytronMD motor_br(PWM_DIR, motor_br_pwm, motor_br_dir); 
CytronMD motor_fr(PWM_DIR, motor_fr_pwm, motor_fr_dir); 
CytronMD motor_bl(PWM_DIR, motor_bl_pwm, motor_bl_dir); 

//Create servo instances
Servo s0; Servo s1; Servo s2; Servo s3; Servo s4; Servo s5;
#define servo1 34  //Pin for each servo on Mega
#define servo2 35
#define servo3 36
#define servo4 37
#define servo5 38
#define servo6 39


// Sensor Slider
#define RailPWM 10
#define RailDir 23
#define RailOther 11

//LEDs
int tDelay = 100;
int latchPin = 30; //11;  // (11) ST_CP [RCK] on 74HC595
int clockPin = 31; // 9;      // (9) SH_CP [SCK] on 74HC595
int dataPin = 32; //12;     // (12) DS [S1] on 74HC595
byte leds = 0;

//Variables for moving rover
int max_speed = 30; //for rover car
int JS_adj[4]={0,0,0,0}; //init joy stick positions,offsets, and adj speed values after normalizing
int ds1 = 3; //delay speed for motor loops
int speed2 = 0, speed1 = 0;  //s1=max speed, s2=speed adj for turning

//Wheel movement
int interval=1000, ppr=723, fps=0;//vars for all dc motor encoders
long prevMillis=0, currentMillis=0;  //vars for all dc motor encoders
volatile int pulses[2]={0,0}; //var for each dc motor encoders
int rpm[2]={0,0}; //var for each dc motor encoders
int controlMode = 1, speedMode=2;  //change from wheels to arms, speed0=stop,1=50%,2=100%
int rtjs=0, j=0; //if 0 or L3 pressed uses left joystick else r3 pressed right joystick 
int prevspeed=0;

//Variable for ultra sonic distance sensor
long usDist;

//Servos 
int ssf=2, ssf1[6]={3,3,3,3,3,3}; //counter for servo
int stepp[6]={90,90,90,90,90,90}; //counter for servo1
int servoio=0;

//Gyroscope
const int MPU_ADDR =  0x68; //i2c addresss for MPU-6050
int16_t accelerometer_x, accelerometer_y, accelerometer_z; //var for raw accelerometer data
int16_t gyro_x, gyro_y, gyro_z; //vars for gyro raw data
char tmp_str[7]; //temp strin array
int16_t temperature; //temp data

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(300);
  
  roverstop();  //makes sure rover starts stationary

  //Put in in intercept ports 2,3,18,19,20,21 on Mega or 2,3 w/ Uno
  pinMode(encoder_bra,INPUT_PULLUP);
  pinMode(encoder_bla,INPUT_PULLUP);
  
  //Allow signal from encoder to be counted for finding DC motor position
  attachInterrupt(digitalPinToInterrupt(encoder_bra), updateEncoder_br, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_bla), updateEncoder_bl, CHANGE);
  
  prevMillis=millis(); //gets the time the device has been on

  // initialize servos
  servoinit();

  //i2c communication: gyro
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40
  Wire.write(0);  //wakes upMPU-6050
  Wire.endTransmission(true); //param indication that arduino will dend a restart, keeps connection active

  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
}

void loop () {

  // put your main code here, to run repeatedly:
  while (readI2C(SlaveAddr) < 255) {}
  for (bcount=0; bcount < 6; bcount++) {
    JS_Inputs[bcount]=readI2C(SlaveAddr);
  }
  
  for (int i=0; i<4; i++) { 
    JS_adj[i] = JS_Inputs[i]-128; 
    constrain(JS_adj[i],-128,128);
  }
  
  Buts=JS_Inputs[4];
  Trigs=JS_Inputs[5];

  //turn on/off servos
  if (Buts==7) {
    servoio++;
    if (servoio>1) { servoio=0; } 
  }    
  
  if (servoio==0) { bitSet(leds, 5); updateShiftRegister(); }
  else if (servoio==1) { bitSet(leds, 6); updateShiftRegister(); }

  //Rover controled by D-Pad and Triggers
  if (( (Buts==1) | (Buts==2) ) | ( (Buts==3) | (Buts==4) )) {
    speed1=max_speed*speedMode/2;  //Uses half max speed by default
    
    if ((speed1-prevspeed)>15) {speed1=prevspeed+15; }
    
    //D-pad directions make robot move in that direction
    if (Buts==3)
    {      roverbk(speed1);    }
    
    if (Buts==1)
    {      roverfwd(speed1);    }
    
    if (Buts==4)
    {      roverleft(speed1);    }
    
    if (Buts==2)
    {      roverrt(speed1);      }
  }
  
  //Pressing triangles toggles between controling the rover and the arm
  if (Buts==5) {
    controlMode++;
    if (controlMode>1) { 
      controlMode=0;
      } //only 2 control modes
  }

  //Pressing X changes the speed settings from 0, half, and full
  if (Buts==8) {
    speedMode++;
    if (speedMode=0) { bitSet(leds, 4); updateShiftRegister(); }
    if (speedMode=1) { bitSet(leds, 4); updateShiftRegister(); }
    if (speedMode>2) { bitSet(leds, 7); updateShiftRegister(); speedMode=0; } //3 speed settings
  }

  //Toggels between rover and arm controls when triangle is toggled
  switch (controlMode) {
    case 0: //Case for rover control
      //Only trigger L joystick if both X and Y > 5 or if either are > 15 
      
      if (Trigs==6) { rtjs=0; }
      else if (Trigs==3) { rtjs=1; }

      if (rtjs==1) { j=2; }
      else if (rtjs==0) { j=0; }
        
      if (((abs(JS_adj[j+1]) >= 5 && abs(JS_adj[j]) >= 5)) || (abs(JS_adj[j+1]) >= 15 || abs(JS_adj[j]) >= 15)) {
        speed1 = max(JS_adj[j+1], JS_adj[j])*speedMode/2; //rover speed scales w/ max X or Y vector
        if ((speed1-prevspeed)>15) {speed1=prevspeed+15; }
        constrain(speed1,-max_speed,max_speed); //Constrains max speed

        //Allows the diaganol wheels to change speed and direction, avgs X and Y magnitude
        speed2 = (int)speed1 * 2 * (((1 - abs(JS_adj[j+1]) / sqrt(sq(JS_adj[j+1]) + sq(JS_adj[j]))) + (abs(JS_adj[j]) / sqrt(sq(JS_adj[j+1]) + sq(JS_adj[j])))) / 2 - .5);
        constrain(speed2,-max_speed,max_speed); //Constrains max speed
               
        //Foward Right 
        if (JS_adj[j] >= 0 && JS_adj[j+1] >= 0) {
          rovermv(speed1,speed2);
        }
        //Foward Left
        else if (JS_adj[j] >= 0 && JS_adj[j+1] <= 0) {
          rovermv(speed2,speed1);
        }
        //Back Right
        else if (JS_adj[j] <= 0 && JS_adj[j+1] >= 0) {
          rovermv(speed2,-speed1);
        }
        //Back Left
        else if (JS_adj[j] <= 0 && JS_adj[j+1] <= 0) {
          rovermv(-speed1,speed2);
        }
      }
            
      //Triggers cause the rover to spin
      if ((Trigs==2) || (Trigs==1)) { roverspinrt(max_speed); }
  
      if ((Trigs==5) || (Trigs==4)) { roverspinleft(max_speed); }
      break;
      
    //Case for arm control, js control 4 servos 0-3, R/L2 moves claws, R/L1 rotates claw
    case 1: 
      if (abs(JS_adj[1]) >15) {
        stepp[0] = constrain(mvservo1(JS_adj[1],stepp[0]),10,170);
        s0.write(stepp[0]);
        delay(1);
        }
      else if (abs(JS_adj[0]) >15) {
        stepp[1] = constrain(mvservo1(JS_adj[0],stepp[1]),10,170);
        s1.write(stepp[1]);
        delay(1);
        }
      else if (abs(JS_adj[3]) >15) {
        stepp[2] = constrain(mvservo1(JS_adj[3],stepp[2]),10,170);
        s2.write(stepp[2]);
        delay(1);
        }
      else if (abs(JS_adj[2]) >15) {
        stepp[3] = constrain(mvservo1(-JS_adj[2],stepp[3]),10,170);
        s3.write(stepp[3]);
        delay(1);
        }
        
      if (Trigs==2) {
        stepp[5]=stepp[5]+ssf1[5];
        constrain(stepp[5],10,170);
        s5.write(stepp[5]);
        delay(1);
      }
      else if (Trigs==5) {
        stepp[5]=stepp[5]-ssf1[5];
        constrain(stepp[5],10,170);
        s5.write(stepp[5]);
        delay(1);
      }
      else if (Trigs==1) {
        stepp[4]=stepp[4]+ssf1[4];
        constrain(stepp[4],10,170);
        s4.write(stepp[4]);
        delay(1);
      }
      else if (Trigs==4) {
        stepp[4]=stepp[4]-ssf1[4];
        constrain(stepp[4],10,170);
        s4.write(stepp[4]);
        delay(1);
      }
      break;
  }

  //Counts RMP, and feet/sec for each motor every interval
  currentMillis=millis();
  if (currentMillis - prevMillis > interval) {
    prevMillis=currentMillis;
    for (int i=0; i<2; i++) {
      rpm[i]=(float)(pulses[i]); //*60000/interval/ppr/2);
      pulses[i]=0;  //resets counters
    }
    
    fps=(int)(rpm[0]+rpm[1])/2*12.36/60; //feet/sec w/ 100 mm diameter wheel
    
    usDist=sr04.Distance();

    //i2c communication: gyro
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); //start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); //idicaed arduino will send restart
    Wire.requestFrom(MPU_ADDR, 7*2,true); //register with 14
    
    // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
    accelerometer_x=Wire.read()<<8 | Wire.read(); //reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
    accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
    accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
    temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
    gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
    gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
    gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  }

//  leds = 0;
//  updateShiftRegister();
//  delay(tDelay);
//  for (int i = 0; i < 8; i++)
//  {
//    bitSet(leds, i);
//    updateShiftRegister();
//    delay(tDelay);
//  }
  
  //Print to screen
  Serial.print("LX: ");  Serial.print(JS_adj[1]); 
  Serial.print("\tLY: ");  Serial.print(JS_adj[0]); 
  Serial.print("\tRX: ");  Serial.print(JS_adj[3]); 
  Serial.print("\tRY: ");  Serial.print(JS_adj[2]); 
  Serial.print("\tButs: ");  Serial.print(Buts);
  Serial.print("\tTrigs: ");  Serial.print(Trigs);
  Serial.println(); 
//  Serial.print("\tS1: ");  Serial.print(speed1);
//  Serial.print("\tS2: ");  Serial.print(speed2);
//  Serial.print("\trpm: ");  
//  for (int i=0; i<2; i++) { Serial.print(rpm[i]); Serial.print(","); }  
//  Serial.print("\tspd: ");  Serial.print(fps);
//  Serial.print("\tsvo: ");
//  for (int i=0; i<6; i++) { Serial.print(stepp[i]); Serial.print(","); }  
//  Serial.print("\tDist: ");  Serial.print(usDist);
//  
//  Serial.print("\tPOS "); Serial.print(convert_int16_to_str(accelerometer_x));
//  Serial.print(","); Serial.print(convert_int16_to_str(accelerometer_y));
//  Serial.print(","); Serial.print(convert_int16_to_str(accelerometer_z));
//  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
//  Serial.print("\ttmp "); Serial.print(temperature/340.00+36.53);
//  Serial.print("\tGyo "); Serial.print(convert_int16_to_str(gyro_x));
//  Serial.print(","); Serial.print(convert_int16_to_str(gyro_y));
//  Serial.print(","); Serial.print(convert_int16_to_str(gyro_z));
//  
//  Serial.print("\tmd: ");  Serial.print(controlMode);
//  Serial.print("\tsm: ");  Serial.print(speedMode);
//  Serial.print("\trtjs: ");  Serial.print(rtjs);
//  Serial.print("\tservo: ");  Serial.print(servoio);
//  
//  Serial.println(); 
  
  prevspeed=speed1; speed1=0; speed2=0; //reset speeds
  roverstop(); //halts rover
}


//encoder fxs for counting
void updateEncoder_br() { pulses[0]++; }
void updateEncoder_bl() { pulses[1]++; }

//turns off servos
void servooff() {s0.detach(); s1.detach(); s2.detach(); s3.detach(); s4.detach(); s5.detach(); }

void servoinit() {
  //Assign arm servos to digital pin #
  s0.attach(servo1); s1.attach(servo2); s2.attach(servo3);
  s3.attach(servo4); s4.attach(servo5); s5.attach(servo6);

  s0.write(stepp[0]); s1.write(stepp[1]); s2.write(stepp[2]); 
  s3.write(stepp[3]); s4.write(stepp[4]); s5.write(stepp[5]); 
}

//Halts the rover
void roverstop() { motor_fl.setSpeed(0); motor_fr.setSpeed(0); motor_bl.setSpeed(0); motor_br.setSpeed(0); }

//Rover movements
void rovermv4(int speeda, int speedb, int speedc, int speedd) {
  motor_fl.setSpeed(speeda);
  motor_br.setSpeed(speedb);
  motor_fr.setSpeed(speedc);
  motor_bl.setSpeed(speedd);
  delay(3);
}

void rovermv(int speeda, int speedb) {  rovermv4(speeda,speeda,speedb,speedb);  }

void roverfwd(int speeda) {  rovermv(speeda,speeda); }

void roverbk(int speeda) {   roverfwd(-speeda);  }

void roverrt(int speeda) {  rovermv(speeda,-speeda);  }

void roverleft(int speeda) {  roverrt(-speeda);  }

void roverspinrt(int speeda) {  rovermv4(speeda,-speeda,-speeda,speeda); }

void roverspinleft(int speeda) {  roverspinrt(-speeda); }

int mvservo1(int js_val,int steppa) {
  int assf=map(js_val,-128,128,-ssf,ssf);
  steppa=steppa+assf;
  constrain(steppa,0,180);
  return steppa;
}

byte readI2C(int address) {
  byte bval;
  long entry=millis();
  Wire.requestFrom(address,1);
  while (Wire.available()==0 && (millis()-entry) <100) {Serial.print("waiting");}
  if (millis() - entry <100 ) { bval=Wire.read(); } 
  return bval;
}

void updateShiftRegister()
{
   digitalWrite(latchPin, LOW);
   shiftOut(dataPin, clockPin, LSBFIRST, leds);
   digitalWrite(latchPin, HIGH);
}
