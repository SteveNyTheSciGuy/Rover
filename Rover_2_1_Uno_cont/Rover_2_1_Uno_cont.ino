#include "CytronMotorDriver.h" //motor controler driver
#include <PS2X_lib.h>  //ps2controller driver
#include <Servo.h> //servo driver
#include "SR04.h" //ultra sonic sensor driver
#include "Wire.h" //for i2c communication

//Pins ps2 Controller
#define PS2_DAT        4  //Pin Data is plugged into - IO
#define PS2_CMD        7  //Pin Command is plugged into - IO
#define PS2_SEL        6  //Pin Attention is plugged into - PWM
#define PS2_CLK        5  //Pin Clock is plugged into - PWM
PS2X ps2x;  //create instance for ps2 controller

////Pins for each motor for its pwm and dir
//#define motor_fl_pwm  4
//#define motor_fl_dir  24
//#define motor_fr_pwm  5
//#define motor_fr_dir  25
//#define motor_bl_pwm  6
//#define motor_bl_dir  26
//#define motor_br_pwm  7
//#define motor_br_dir  27
//
////Pins for encoders
//#define encoder_fla 18
//#define encoder_bra 19
//#define encoder_fra 0
//#define encoder_bla 1
//
////Pins for Ultrasonic Sensor
//#define trig 28
//#define echo 11
//SR04 sr04=SR04(echo,trig);
//
////Configure the motor driver.
//CytronMD motor_fl(PWM_DIR, motor_fl_pwm, motor_fl_dir);  
//CytronMD motor_br(PWM_DIR, motor_br_pwm, motor_br_dir); 
//CytronMD motor_fr(PWM_DIR, motor_fr_pwm, motor_fr_dir); 
//CytronMD motor_bl(PWM_DIR, motor_bl_pwm, motor_bl_dir); 
//
////Create servo instances
//Servo s0; Servo s1; Servo s2; Servo s3; Servo s4; Servo s5;
//#define servo1 30  //Pin for each servo on Mega
//#define servo2 31
//#define servo3 35
//#define servo4 32
//#define servo5 34
//#define servo6 33

//Variables
int max_speed = 30; //for rover car
//int error = 0;  //for controler fx
int JS[4]={0,0,0,0}, JS_offset[4]={0,0,0,0}, JS_adj[4]={0,0,0,0}; //init joy stick positions,offsets, and adj speed values after normalizing
int ds1 = 3; //delay speed for motor loops
int speed2 = 0, speed1 = 0;  //s1=max speed, s2=speed adj for turning
//Wheel movement
int interval=1000, ppr=723, fps=0;//vars for all dc motor encoders
long prevMillis=0, currentMillis=0;  //vars for all dc motor encoders
volatile int pulses[4]={0,0,0,0}; //var for each dc motor encoders
int rpm[4]={0,0,0,0}; //var for each dc motor encoders
int controlMode = 1, speedMode=2;  //change from wheels to arms, speed0=stop,1=50%,2=100%
int rtjs=0, j=0; //if 0 or L3 pressed uses left joystick else r3 pressed right joystick 
int prevspeed=0;

////Variable for ultra sonic distance sensor
//long usDist;

////Servos 
//int ssf=2, ssf1[6]={3,3,3,3,3,3}; //counter for servo
//int stepp[6]={90,90,90,90,90,90}; //counter for servo1
//int servoio=0;

////Leds
//#define ledgreen 36 
//#define ledyellow 37 
//#define ledred 38
//#define ledblue 39

////Gyroscope
//const int MPU_ADDR =  0x68; //i2c addresss for MPU-6050
//int16_t accelerometer_x, accelerometer_y, accelerometer_z; //var for raw accelerometer data
//int16_t gyro_x, gyro_y, gyro_z; //vars for gyro raw data
//char tmp_str[7]; //temp strin array
//int16_t temperature; //temp data
//
//char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
//  sprintf(tmp_str, "%6d", i);
//  return tmp_str;
//}

void setup() {

  Serial.begin(115200);
  delay(300);
  //added delay to give wireless ps2 module some time to startup, before configuring it

  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  int error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);

  ps2x.read_gamepad(); //read contoler outputs
  //delay(600);
  JS[0] = -ps2x.Analog(PSS_LY) + 128; //joystick position converter to speed
  JS[1] = ps2x.Analog(PSS_LX) - 128;
  JS[2] = -ps2x.Analog(PSS_RY) + 128; //joystick position converter to speed
  JS[3] = ps2x.Analog(PSS_RX) - 128;
  for (int i=0; i<4; i++) { JS_offset[i] = JS[i]; } //speed_i - speed_i_offset;
  
//  roverstop();  //makes sure rover starts stationary

//  //Put in in intercept ports 2,3,18,19,20,21 on Mega or 2,3 w/ Uno
//  pinMode(encoder_fla,INPUT_PULLUP); //Use pull up to reduce noise
//  pinMode(encoder_bra,INPUT_PULLUP);
//  pinMode(encoder_fra,INPUT_PULLUP);
//  pinMode(encoder_bla,INPUT_PULLUP);
//  
//  //Allow signal from encoder to be counted for finding DC motor position
//  attachInterrupt(digitalPinToInterrupt(encoder_fla), updateEncoder_fl, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(encoder_bra), updateEncoder_br, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(encoder_fra), updateEncoder_fr, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(encoder_bla), updateEncoder_bl, CHANGE);
  
  prevMillis=millis(); //gets the time the device has been on

  // initialize servos
//  servoinit();

//  //i2c communication: gyro
//  Wire.begin();
//  Wire.beginTransmission(MPU_ADDR);
//  Wire.write(0x6B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40
//  Wire.write(0);  //wakes upMPU-6050
//  Wire.endTransmission(true); //param indication that arduino will dend a restart, keeps connection active
//
//  //leds
//  pinMode(ledgreen,OUTPUT);
//  pinMode(ledyellow,OUTPUT);
//  pinMode(ledred,OUTPUT);
//  pinMode(ledblue,OUTPUT);
//
}

void loop () {

  ps2x.read_gamepad(); //read contoler outputs
  JS[0] = -ps2x.Analog(PSS_LY) + 128; //joystick position, centered at 0 w/ a +/- 128 range
  JS[1] = ps2x.Analog(PSS_LX) - 128;
  JS[2] = -ps2x.Analog(PSS_RY) + 128; 
  JS[3] = ps2x.Analog(PSS_RX) - 128;

  // press select to re-zero the joy sticks
//  if (ps2x.ButtonPressed(PSB_SELECT))   {    buttonreset();  }

//  //turn on/off servos
//  if (ps2x.ButtonPressed(PSB_BLUE)) {
//    servoio++;
//    if (servoio>1) { servoio=0; } 
//  }    
//  
//  if (servoio=0) { digitalWrite(ledyellow,LOW); }
//  else if (servoio=1) { digitalWrite(ledyellow,HIGH); }

  //Uses joy stick offests to zero the reading from the controler
  for (int i=0; i<4; i++) { 
    JS_adj[i] = JS[i] - JS_offset[i]; 
    constrain(JS_adj[i],-128,128);
  }    

//  //Rover controled by D-Pad and Triggers
//  if (((ps2x.Button(PSB_PAD_DOWN) || ps2x.Button(PSB_PAD_UP)) || (ps2x.Button(PSB_PAD_LEFT) || ps2x.Button(PSB_PAD_RIGHT))) ) //|| ((ps2x.Button(PSB_L2) || ps2x.Button(PSB_R2) || (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1))))) {
//    speed1=max_speed*speedMode/2;  //Uses half max speed by default
//    
//    if ((speed1-prevspeed)>15) {speed1=prevspeed+15; }
//    
//    //D-pad directions make robot move in that direction
//    if (ps2x.Button(PSB_PAD_DOWN))
//    {      roverbk(speed1);    }
//    
//    if (ps2x.Button(PSB_PAD_UP))
//    {      roverfwd(speed1);    }
//    
//    if (ps2x.Button(PSB_PAD_LEFT))
//    {      roverleft(speed1);    }
//    
//    if (ps2x.Button(PSB_PAD_RIGHT))
//    {      roverrt(speed1);      }
//
//  //Pressing triangles toggles between controling the rover and the arm
//  if (ps2x.ButtonPressed(PSB_GREEN)) {
//    controlMode++;
//    if (controlMode>1) { 
//      controlMode=0;
//      } //only 2 control modes
//  }
//
//  //Pressing X changes the speed settings from 0, half, and full
//  if (ps2x.ButtonPressed(PSB_PINK)) {
//    speedMode++;
//    if (speedMode>2) { speedMode=0; } //3 speed settings
//  }
//
//  //Toggels between rover and arm controls when triangle is toggled
//  switch (controlMode) {
//    case 0: //Case for rover control
//      //Only trigger L joystick if both X and Y > 5 or if either are > 15 
//      
//      if (ps2x.Button(PSB_L3)) { rtjs=0; }
//      else if (ps2x.Button(PSB_R3)) { rtjs=1; }
//
//      if (rtjs=1) { j=2; }
//      else if (rtjs=0) { j=0; }
//        
//      if (((abs(JS_adj[j+1]) >= 5 && abs(JS_adj[j]) >= 5)) || (abs(JS_adj[j+1]) >= 15 || abs(JS_adj[j]) >= 15)) {
//        speed1 = max(JS_adj[j+1], JS_adj[j])*speedMode/2; //rover speed scales w/ max X or Y vector
//        if ((speed1-prevspeed)>15) {speed1=prevspeed+15; }
//        constrain(speed1,-max_speed,max_speed); //Constrains max speed
//
//        //Allows the diaganol wheels to change speed and direction, avgs X and Y magnitude
//        speed2 = (int)speed1 * 2 * (((1 - abs(JS_adj[j+1]) / sqrt(sq(JS_adj[j+1]) + sq(JS_adj[j]))) + (abs(JS_adj[j]) / sqrt(sq(JS_adj[j+1]) + sq(JS_adj[j])))) / 2 - .5);
//        constrain(speed2,-max_speed,max_speed); //Constrains max speed
//               
//        //Foward Right 
//        if (JS_adj[j] >= 0 && JS_adj[j+1] >= 0) {
//          rovermv(speed1,speed2);
//        }
//        //Foward Left
//        else if (JS_adj[j] >= 0 && JS_adj[j+1] <= 0) {
//          rovermv(speed2,speed1);
//        }
//        //Back Right
//        else if (JS_adj[j] <= 0 && JS_adj[j+1] >= 0) {
//          rovermv(speed2,-speed1);
//        }
//        //Back Left
//        else if (JS_adj[j] <= 0 && JS_adj[j+1] <= 0) {
//          rovermv(-speed1,speed2);
//        }
//      }
//            
//      //Triggers cause the rover to spin
//      if (ps2x.Button(PSB_R2) || ps2x.Button(PSB_R1))
//      {      roverspinrt(max_speed);
//              Serial.print("RRR"); }
//  
//      if (ps2x.Button(PSB_L2) || ps2x.Button(PSB_L1))
//      {      roverspinleft(max_speed);
//                    Serial.print("LLL"); }
//      break;
//      
//    //Case for arm control, js control 4 servos 0-3, R/L2 moves claws, R/L1 rotates claw
//    case 1: 
//      if (abs(JS_adj[1]) >15) {
//        stepp[0] = mvservo1(JS_adj[1],stepp[0]);
//        s0.write(stepp[0]);
//        //delay(3);
//        }
//      else if (abs(JS_adj[0]) >15) {
//        stepp[1] = mvservo1(JS_adj[0],stepp[1]);
//        s1.write(stepp[1]);
//        //delay(3);
//        }
//      else if (abs(JS_adj[3]) >15) {
//        stepp[2] = mvservo1(JS_adj[3],stepp[2]);
//        s2.write(stepp[2]);
//        delay(3);
//        }
//      else if (abs(JS_adj[2]) >15) {
//        stepp[3] = mvservo1(-JS_adj[2],stepp[3]);
//        s3.write(stepp[3]);
//        //delay(3);
//        }
//        
//      if (ps2x.Button(PSB_R2)) {
//        stepp[5]=stepp[5]+ssf1[5];
//        constrain(stepp[5],0,180);
//        s5.write(stepp[5]);
//        //delay(3);
//      }
//      else if (ps2x.Button(PSB_L2)) {
//        stepp[5]=stepp[5]-ssf1[5];
//        constrain(stepp[5],0,180);
//        s5.write(stepp[5]);
//        //delay(3);
//      }
//      else if (ps2x.Button(PSB_R1)) {
//        stepp[4]=stepp[4]+ssf1[4];
//        constrain(stepp[4],0,180);
//        s4.write(stepp[4]);
//        //delay(3);
//      }
//      else if (ps2x.Button(PSB_L1)) {
//        stepp[4]=stepp[4]-ssf1[4];
//        constrain(stepp[4],0,180);
//        s4.write(stepp[4]);
//        //delay(3);
//      }
//      break;
//  }

//  //Counts RMP, and feet/sec for each motor every interval
//  currentMillis=millis();
//  if (currentMillis - prevMillis > interval) {
//    prevMillis=currentMillis;
//    for (int i=0; i<4; i++) {
//      rpm[i]=(float)(pulses[i]*60000/interval/ppr/2);
//      pulses[i]=0;  //resets counters
//    }
//    
//    fps=(int)(rpm[0]+rpm[1]+rpm[2]+rpm[3])/4*12.36/60; //feet/sec w/ 100 mm diameter wheel
//    usDist=sr04.Distance();

//    //i2c communication: gyro
//    Wire.beginTransmission(MPU_ADDR);
//    Wire.write(0x3B); //start with register 0x3B (ACCEL_XOUT_H)
//    Wire.endTransmission(false); //idicaed arduino will send restart
//    Wire.requestFrom(MPU_ADDR, 7*2,true); //register with 14
    
//    // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
//    accelerometer_x=Wire.read()<<8 | Wire.read(); //reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
//    accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
//    accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
//    temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
//    gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
//    gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
//    gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
//  }

//  //leds
//  digitalWrite(ledgreen,controlMode); 
//  digitalWrite(ledyellow,servoio);
//  digitalWrite(ledred,speedMode);
//  digitalWrite(ledblue,0);
//  
  //Print to screen
  Serial.print("LX: ");  Serial.print(JS_adj[1]); 
  Serial.print("\tLY: ");  Serial.print(JS_adj[0]); 
  Serial.print("\tRX: ");  Serial.print(JS_adj[3]); 
  Serial.print("\tRY: ");  Serial.print(JS_adj[2]); 
//  Serial.print("\tS1: ");  Serial.print(speed1);
//  Serial.print("\tS2: ");  Serial.print(speed2);
//  Serial.print("\trpm: ");  
//  for (int i=0; i<4; i++) { Serial.print(rpm[i]); Serial.print(","); }  
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
  Serial.println(); 
  
//  prevspeed=speed1; speed1=0; speed2=0; //reset speeds
//  roverstop(); //halts rover
}

////encoder fxs for counting
//void updateEncoder_fl() { pulses[0]++; }
//
//void updateEncoder_br() { pulses[1]++; }
//
//void updateEncoder_fr() { pulses[2]++; }
//
//void updateEncoder_bl() { pulses[3]++; }
//
////Zeros the joy sticks
//void buttonreset() {JS_offset[0] = JS[0]; JS_offset[1] = JS[1]; JS_offset[2] = JS[2]; JS_offset[3] = JS[3]; }
//
////turns off servos
//void servooff() {s0.detach(); s1.detach(); s2.detach(); s3.detach(); s4.detach(); s5.detach(); }
//
//void servoinit() {
//  //Assign arm servos to digital pin #
//  s0.attach(servo1); s1.attach(servo2); s2.attach(servo3);
//  s3.attach(servo4); s4.attach(servo5); s5.attach(servo6);
//
//  s0.write(stepp[0]); s1.write(stepp[1]); s2.write(stepp[2]); 
//  s3.write(stepp[3]); s4.write(stepp[4]); s5.write(stepp[5]); 
//}
//
////Halts the rover
//void roverstop() { motor_fl.setSpeed(0); motor_fr.setSpeed(0); motor_bl.setSpeed(0); motor_br.setSpeed(0); }
//
////Rover movements
//void rovermv4(int speeda, int speedb, int speedc, int speedd) {
//  motor_fl.setSpeed(speeda);
//  motor_br.setSpeed(speedb);
//  motor_fr.setSpeed(speedc);
//  motor_bl.setSpeed(speedd);
//  delay(3);
//}
//
//void rovermv(int speeda, int speedb) {  rovermv4(speeda,speeda,speedb,speedb);  }
//
//void roverfwd(int speeda) {  rovermv(speeda,speeda); }
//
//void roverbk(int speeda) {   roverfwd(-speeda);  }
//
//void roverrt(int speeda) {  rovermv(speeda,-speeda);  }
//
//void roverleft(int speeda) {  roverrt(-speeda);  }
//
//void roverspinrt(int speeda) {  rovermv4(speeda,-speeda,-speeda,speeda); }
//
//void roverspinleft(int speeda) {  roverspinrt(-speeda); }
//
//int mvservo1(int js_val,int steppa) {
//  int assf=map(js_val,-128,128,-ssf,ssf);
//  steppa=steppa+assf;
//  constrain(steppa,0,180);
//  return steppa;
//}
