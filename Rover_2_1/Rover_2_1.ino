#include "CytronMotorDriver.h" //motor controler driver
#include <PS2X_lib.h>  //ps2controller driver

#define PS2_DAT        2  //Pin Data is plugged into
#define PS2_CMD        3  //Pin Command is plugged into
#define PS2_SEL        4  //Pin Attention is plugged into
#define PS2_CLK        5  //Pin Clock is plugged into

//#define pressures   true  //Allows pressures to be read (analog)
//#define rumble      true  //Allows rumbling

//Pins for each motor for its pwm and dir
#define motor_fl_pwm  6
#define motor_fl_dir  7
#define motor_fr_pwm  9
#define motor_fr_dir  8
#define motor_bl_pwm  10
#define motor_bl_dir  12
#define motor_br_pwm  11
#define motor_br_dir  13

// Configure the motor driver.
CytronMD motor_fl(PWM_DIR, motor_fl_pwm, motor_fl_dir);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor_fr(PWM_DIR, motor_fr_pwm, motor_fr_dir); // PWM 2 = Pin 9, DIR 2 = Pin 10.
CytronMD motor_bl(PWM_DIR, motor_bl_pwm, motor_bl_dir); // PWM 2 = Pin 9, DIR 2 = Pin 10.
CytronMD motor_br(PWM_DIR, motor_br_pwm, motor_br_dir); // PWM 2 = Pin 9, DIR 2 = Pin 10.

PS2X ps2x;  //create instance for ps2 controller
int max_speed = 100; //signal from joy sticks
int error = 0;  //for controler fx
byte type = 0;  //ps2 controler type
byte vibrate = 1;  //vibrate fx on/off
int RX = 0, RY = 0, js_y, js_x; //init joy stick positions
int js_x_offset = 0, js_y_offset = 0, js_x_adj = 0, js_y_adj = 0; //offsets and adj speed values after normalizing
int ds1 = 3; //delay speed for motor loops
float speed2 = 0, speed1 = 0;

void setup() {

  Serial.begin(9600);
  delay(300);
  //added delay to give wireless ps2 module some time to startup, before configuring it

  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, true, true);

  //  if (error == 0) {
  //    Serial.print("Found Controller, configured successful ");
  //    Serial.print("pressures = ");
  //    if (pressures)
  //      Serial.println("true ");
  //    else
  //      Serial.println("false");
  //    Serial.print("rumble = ");
  //    if (rumble)
  //      Serial.println("true)");
  //    else
  //      Serial.println("false");
  //    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
  //    Serial.println("holding L1 or R1 will print out the analog stick values.");
  //    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  //  }
  //  else if (error == 1)
  //    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
  //
  //  else if (error == 2)
  //    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");
  //
  //  else if (error == 3)
  //    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  //  Serial.print(ps2x.Analog(1), HEX);

  //  type = ps2x.readType();
  //  switch (type) {
  //    case 0:
  //      Serial.print("Unknown Controller type found ");
  //      break;
  //    case 1:
  //      Serial.print("DualShock Controller found ");
  //      break;
  //    case 2:
  //      Serial.print("GuitarHero Controller found ");
  //      break;
  //    case 3:
  //      Serial.print("Wireless Sony DualShock Controller found ");
  //      break;
  //  }
  ps2x.read_gamepad(); //read contoler outputs
  delay(600);
  js_y = -ps2x.Analog(PSS_LY) + 128; //joystick position converter to speed
  js_x = ps2x.Analog(PSS_LX) - 128;
  js_y_offset = js_y;//speed_y - speed_y_offset;
  js_x_offset = js_x;//speed_x - speed_x_offset;
  motor_fl.setSpeed(0);
  motor_fr.setSpeed(0);
  motor_bl.setSpeed(0);
  motor_br.setSpeed(0);
}

void loop ()
{
  ps2x.read_gamepad(); //read contoler outputs
  js_y = -ps2x.Analog(PSS_LY) + 128; //joystick position converter to speed
  js_x = ps2x.Analog(PSS_LX) - 128;
  //speed_y = map(ps2x.Analog(PSS_LY), 0 , 360, 0 , 255);
  //speed_x = map(ps2x.Analog(PSS_LX), 0 , 360, 0 , 255);
  //speed_y=ps2x.Analog(PSS_LY);
  //speed_x=ps2x.Analog(PSS_LX);

  if (ps2x.ButtonPressed(PSB_SELECT)) // press select to re-zero the joy sticks
  {
    js_y_offset = js_y;
    js_x_offset = js_x;
    Serial.print("OffSets");
    Serial.print(js_x_offset);
    Serial.print(",");
    Serial.print(js_y_offset);
    Serial.print(" ");
  }

  js_y_adj = js_y - js_y_offset;
  if (js_y_adj > 128)  {     js_y_adj = 128;  }
  else if (js_y_adj < -128)  {    js_y_adj = -128;  }

  js_x_adj = js_x - js_x_offset;
  if (js_x_adj > 128)  {    js_x_adj = 128;  }
  else if (js_x_adj < -128)  {    js_x_adj = -128;  }

  if (((ps2x.Button(PSB_PAD_DOWN) || ps2x.Button(PSB_PAD_UP)) || (ps2x.Button(PSB_PAD_LEFT) || ps2x.Button(PSB_PAD_RIGHT))) || (ps2x.Button(PSB_L2) || ps2x.Button(PSB_R2))) {
    speed1=max_speed/2;
    
    if (ps2x.Button(PSB_PAD_DOWN))
    {
      motor_fl.setSpeed(-speed1);
      motor_fr.setSpeed(-speed1);
      motor_bl.setSpeed(-speed1);
      motor_br.setSpeed(-speed1);
      delay (ds1);
    }
    if (ps2x.Button(PSB_PAD_UP))
    {
      motor_fl.setSpeed(speed1);
      motor_fr.setSpeed(speed1);
      motor_bl.setSpeed(speed1);
      motor_br.setSpeed(speed1);
      delay (ds1);
    }
    if (ps2x.Button(PSB_PAD_LEFT))
    {
      motor_fl.setSpeed(-speed1);
      motor_fr.setSpeed(speed1);
      motor_bl.setSpeed(speed1);
      motor_br.setSpeed(-speed1);
      delay (ds1);
    }
    if (ps2x.Button(PSB_PAD_RIGHT))
    {
      motor_fl.setSpeed(speed1);
      motor_fr.setSpeed(-speed1);
      motor_bl.setSpeed(-speed1);
      motor_br.setSpeed(speed1);
      delay (ds1);
    }
    if (ps2x.Button(PSB_R2))
    {
      motor_fl.setSpeed(speed1);
      motor_fr.setSpeed(-speed1);
      motor_bl.setSpeed(speed1);
      motor_br.setSpeed(-speed1);
      delay (ds1);
    }
    if (ps2x.Button(PSB_L2))
    {
      motor_fl.setSpeed(-speed1);
      motor_fr.setSpeed(speed1);
      motor_bl.setSpeed(-speed1);
      motor_br.setSpeed(speed1);
      delay (ds1);
    }
  }
  
  if (((abs(js_x_adj) >= 5 && abs(js_y_adj) >= 5)) || (abs(js_x_adj) >= 15 || abs(js_y_adj) >= 15)) {
    speed1 = max(js_x_adj, js_y_adj);
    if (speed1 > max_speed)  {      speed1 = max_speed;    }
    else if (speed1 < -max_speed)  {      speed1 = -max_speed;    }
    
    speed2 = speed1 * 2 * (((1 - abs(js_x_adj) / sqrt(sq(js_x_adj) + sq(js_y_adj))) + (abs(js_y_adj) / sqrt(sq(js_x_adj) + sq(js_y_adj)))) / 2 - .5);
    if (speed2 > max_speed)  {      speed2 = max_speed;    }
    else if (speed2 < -max_speed)  {      speed2 = -max_speed;    }

    if (js_y_adj >= 0 && js_x_adj >= 0) {
      motor_fl.setSpeed(speed1);
      motor_br.setSpeed(speed1);
      motor_fr.setSpeed(speed2);
      motor_bl.setSpeed(speed2);
      delay (ds1);
    }
    else if (js_y_adj >= 0 && js_x_adj <= 0) {
      motor_fl.setSpeed(speed2);
      motor_br.setSpeed(speed2);
      motor_fr.setSpeed(speed1);
      motor_bl.setSpeed(speed1);
      delay (ds1);
    }
    else if (js_y_adj <= 0 && js_x_adj >= 0) {
      motor_fl.setSpeed(speed2);
      motor_br.setSpeed(speed2);
      motor_fr.setSpeed(-speed1);
      motor_bl.setSpeed(-speed1);
      delay (ds1);
    }
    else if (js_y_adj <= 0 && js_x_adj <= 0) {
      motor_fl.setSpeed(-speed1);
      motor_br.setSpeed(-speed1);
      motor_fr.setSpeed(speed2);
      motor_bl.setSpeed(speed2);
      delay (ds1);
    }
  }

  Serial.print("Stick Values:");
  Serial.print(js_x_adj); //ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
  Serial.print(",");
  Serial.print(js_y_adj); //ps2x.Analog(PSS_LX), DEC);
  Serial.print(",");
  Serial.print(speed1);
  Serial.print(",");
  Serial.println(speed2);
  //delay(100);
  speed1=0; speed2=0;
  motor_fl.setSpeed(0);
  motor_fr.setSpeed(0);
  motor_bl.setSpeed(0);
  motor_br.setSpeed(0);
}
