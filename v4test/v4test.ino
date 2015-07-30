/*
Values:
P-Tape - 33
D-Tape - 60
S-Tape - 920

*/

#include <avr/EEPROM.h>
#include <phys253.h>
#include <avr/interrupt.h>
#include <LiquidCrystal.h>

//To determine where on course the robot is
volatile unsigned int NUM = 0;

/*
=================
== MENU SYSTEM ==
=================
*/
class MenuItem
{
  public:
  String    Name;
  uint16_t  Value;
  uint16_t* EEPROMAddress;
  static uint16_t MenuItemCount;
  MenuItem(String name)
  {
    MenuItemCount++;
    EEPROMAddress = (uint16_t*)(2 * MenuItemCount);
    Name      = name;
    Value         = eeprom_read_word(EEPROMAddress);
  }
  void Save()
  {
    eeprom_write_word(EEPROMAddress, Value);
  }
};

// Initializing the menu
uint16_t MenuItem::MenuItemCount = 0;
/* Add the menu items here */
MenuItem Speed            = MenuItem("Speed");
MenuItem ProportionalGainTape = MenuItem("P-gain Tape");
MenuItem DerivativeGain   = MenuItem("D-gain Tape");
MenuItem IntegralGain     = MenuItem("I-gain Tape");
MenuItem ThresholdTape    = MenuItem("Thresh Tape");
MenuItem menuItems[]      = {Speed, ProportionalGainTape, DerivativeGain, IntegralGain, ThresholdTape};

/*
===========
== SETUP ==
===========
*/

void setup()
{
  #include <phys253setup.txt>
  LCD.clear();
  LCD.home();
}

int count_setup = 0;

/*
==================
== TINAH INPUTS ==
==================
*/

//Motor
const int MOTOR_LEFT = 3; //PWM output for left motor
const int MOTOR_RIGHT = 2; //PWM output for right motor
const int MOTOR_CRANE_HEIGHT = 1; //Motor for arm height
const int MOTOR_CRANE_ANGLE = 0; //Motor for arm angle

//Analog
const int QRD_LEFT = 1; //Left QRD for tape following
const int QRD_RIGHT = 0; //Right QRD for tape following
const int QRD_PET_RIGHT = 3; //QRD for locating pets back
const int QRD_PET_LEFT = 2; //QRD for locating pets front
const int POTENTIOMETER_CRANE_HEIGHT = 5; //Rotary potentiometer for crane arm
const int POTENTIOMETER_CRANE_ANGLE = 4; //Rotary potentiometer for crane arm
const int IR = 6;

//Servo
const int SERVO_PLATE = 0; //Servo to drop pet

//Digital
const int SWITCH_PLATE = 3; //Switch to see if pet is on plate
const int ROT_LEFT = 0; //Rotary encoder for left wheel
const int ROT_RIGHT = 1; //Rotary encoder for right wheel

/*
===========================
== ARM CONTROL CONSTANTS ==
===========================
*/

// Angle -> horizontal movement of the arm
// Height -> vertical movement of the arm

// Speed
const int SPEED_HEIGHT = 190;
const int SPEED_ANGLE = 65;

// PID Constants
const int P_HEIGHT = 20;
const int P_ANGLE = 1;
const int I_HEIGHT = 24;
const int I_ANGLE = 1;
const int I_MAX_HEIGHT = 150;
const int I_MAX_ANGLE = 150;
const int D_HEIGHT = 0;
const int D_ANGLE = 4;

// Positions
const int ARM_UP = 920;
const int ARM_HOR = 830;
const int ARM_DOWN = 700;
const int ARM_PICKUP = 600;
const int ARM_LEFT = 250;
const int ARM_CENTRE = 500;
const int ARM_RIGHT = 700;
const int SHIFT = 50; // The amount the arm shifts on each attempt

// Range of where the arm will be in an "error-free" zero
const int DEADBAND_HEIGHT = 15;
const int DEADBAND_ANGLE = 25;

// Other
const int MAX_TIME_LONG = 2000; //Max time the arm can move down for pickup (low pet)
const int MAX_TIME_SHORT = 500; //Max time the arm can move down for pickup (high pet)
const int MAX_ANALOG = 1023; // for converting arduino resolution to speed
const int PET_QRD_THRESHOLD = 400; // when the arm will stop to pick up pets

//For reference
const int HEIGHT = 1;
const int ANGLE = 2;

/*
===============================
== ROTARY ENCORDER CONSTANTS ==
===============================
*/

// wheel dimensions
const float DIST_PER_TAPE = PI; //Distance wheel moves per tape hit (cm)
const float LENGTH_OF_AXLE = 23.7; //Length of axle (cm)
const int WAIT_AFTER_PID = 700;

// Variables
int TURNS_LEFT = 0;
int TURNS_RIGHT = 0;
int prev_enc_left = 0;
int prev_enc_right = 0;
int cur_enc_left = 0;
int cur_enc_right = 0;

/*
=================
== HOME SCREEN ==
=================
*/

void loop()
{
  if(count_setup == 0) {
    //Get in start position
    stopDrive();
    setServo(SERVO_PLATE, 0);
    ArmPID(HEIGHT,ARM_UP);
    ArmPID(ANGLE,ARM_CENTRE);
    ArmPID(HEIGHT,ARM_DOWN);
    motor.speed(MOTOR_CRANE_HEIGHT,0);
    count_setup++;
  }

  LCD.clear(); LCD.home();
  LCD.print("Start: Menu");
  LCD.setCursor(0, 1);
  LCD.print("Stop: PID");
  delay(100);

  // opens the menu
  if (startbutton()) {
    delay(100);
    if (startbutton()) { Menu(); }
  }

  // runs the control loop
  if (stopbutton()) {
    delay(100);
    if (stopbutton()) { mainStart(); }
  }
}

/*
==================
== CONTROL LOOP ==
==================
*/

/*
Main loop that controls the robot and the specific code required to get each pet.
NUM represents the pet/location
NUM == 0; start of course
NUM == 1; past 1st pet
NUM == 2; Past 2nd pet
NUM == 3; Past 3rd pet
NUM == 4; At 4th pet
NUM == 5; On way back from 4th pet
NUM == 6; At 3rd pet
NUM == 7; At 2nd pet
NUM == 8; At 1st pet
*/
void mainStart()
{
  LCD.clear(); LCD.home();
  LCD.print("NUM");
  while(true) {
    // at the start, sets the arm to fit in the door then runs PID code
    if (NUM == 0) {
      ArmPID(HEIGHT,ARM_HOR);
      PIDTape();
    }
    if (NUM == 4) {
      stopDrive();
      moveTo(-10, 27, false);
      moveTo(-25, 40, false);
      ArmPID(HEIGHT,ARM_UP);
      pickup(ARM_RIGHT);
      ArmPID(HEIGHT,ARM_HOR);
      moveTo(185, 30, true);
      NUM++;
      PIDTape();
    }
    if (NUM == 6) {
      stopDrive();
      moveTo(0, 10, false);
      ArmPID(HEIGHT,ARM_UP);
      pickup(ARM_LEFT);
      findTape();
      PIDTape();
    }
    if (NUM == 7 || NUM == 8) {
      stopDrive();
      moveTo(0, 30, false);
      moveTo(-20, -10, false);
      ArmPID(HEIGHT,ARM_UP);
      pickup(ARM_LEFT);
      findTape();
      if (NUM == 8){ArmPID(HEIGHT,ARM_HOR);}
      PIDTape();
    }
  }
}

/*
===================
== MOTOR CONTROL ==
===================
*/

/*
Stops the drive motors.
*/
void stopDrive() {
  motor.speed(MOTOR_LEFT, 0);
  motor.speed(MOTOR_RIGHT, 0);
}

/*
Moves left and right until it finds tape again
*/
void findTape() {
  int THRESHOLD = menuItems[4].Value;
  int compensator = 0;
  int count = 0;
  moveTo(-25,0,true);
  while(analogRead(QRD_LEFT) < THRESHOLD && count < 4) {
    moveTo(50+compensator,0,true);
    if(analogRead(QRD_LEFT) > THRESHOLD) {return;}
    moveTo(-50+compensator,0,true);
    compensator += 5;
    count ++;
  }
  stopDrive();
}

void centreIR() {
  int irVal = 0;
  int error = 0;
  int prev_error = 0;
  int THRESHOLD = MAX_ANALOG;
  while(prev_error < error) {
    prev_error = error;
    error = THRESHOLD - analogRead(IR);
    motor.speed(MOTOR_LEFT, 100);
  }
  stopDrive();
}

/*
====================
== ROTARY ENCODER ==
====================
*/

/*
Move the robot to a position based on a given angle and distance
+ve angle turns left
-ve angle turns right
*/
void moveTo(int angle, float distance, bool tape)
{
  //Vars
  int THRESHOLD = menuItems[4].Value;

  //First change angle
  int angleSpeed = 100;
  int linSpeed = 100;
  int angleTurns = ceil((abs(angle)*((LENGTH_OF_AXLE*PI)/360))/DIST_PER_TAPE);
  int linTurns = ceil(abs(distance)/DIST_PER_TAPE);
  bool leftDone = false;
  bool rightDone = false;
  TURNS_RIGHT = 0; TURNS_LEFT = 0;

  //debugging
  LCD.clear(); LCD.home();
  LCD.print(angleTurns); LCD.print("  "); LCD.print(linTurns);

  if (angle > 0) {
    motor.speed(MOTOR_LEFT, -1*angleSpeed);
    motor.speed(MOTOR_RIGHT, angleSpeed);
  } else if (angle == 0) {
    stopDrive();
  } else {
    motor.speed(MOTOR_LEFT, angleSpeed);
    motor.speed(MOTOR_RIGHT, -1*angleSpeed);
  }
  // LCD.clear(); LCD.home(); LCD.print("Turning");
  while((leftDone == false || rightDone == false) && angle != 0) {
    if(tape && analogRead(QRD_LEFT) > THRESHOLD) {return;}
    checkEnc();
    LCD.setCursor(0,1); LCD.print(TURNS_LEFT); LCD.print("  "); LCD.print(TURNS_RIGHT);
    if(TURNS_LEFT >= angleTurns) {motor.speed(MOTOR_LEFT,0); leftDone = true;}
    if(TURNS_RIGHT >= angleTurns) {motor.speed(MOTOR_RIGHT,0); rightDone = true;}
  }

  //Then change linear distance
  TURNS_LEFT = 0; TURNS_RIGHT = 0;
  leftDone = false; rightDone = false;
  if (distance > 0) {
    motor.speed(MOTOR_LEFT,linSpeed);
    motor.speed(MOTOR_RIGHT,linSpeed);
  } else if (distance == 0) {
    stopDrive();
  } else {
    motor.speed(MOTOR_LEFT,-1*linSpeed);
    motor.speed(MOTOR_RIGHT,-1*linSpeed);
  }
  // LCD.clear(); LCD.home(); LCD.print("Moving");
  while((leftDone == false || rightDone == false) && distance != 0) {
    if(tape && analogRead(QRD_LEFT) > THRESHOLD) {stopDrive(); return;}
    checkEnc();
    LCD.setCursor(0,1); LCD.print(TURNS_LEFT); LCD.print("  "); LCD.print(TURNS_RIGHT);
    if(TURNS_LEFT == linTurns) {motor.speed(MOTOR_LEFT,0); leftDone = true;}
    if(TURNS_RIGHT == linTurns) {motor.speed(MOTOR_RIGHT,0); rightDone = true;}
  }
  stopDrive();
}

/*
Check encoders and update turns
*/
void checkEnc()
{
  cur_enc_left = digitalRead(ROT_LEFT);
  cur_enc_right = digitalRead(ROT_RIGHT);

  if(prev_enc_left == HIGH && cur_enc_left == LOW) { TURNS_LEFT++; }
  if(prev_enc_right == HIGH && cur_enc_right == LOW) { TURNS_RIGHT++; }

  prev_enc_left = cur_enc_left; prev_enc_right = cur_enc_right;
}

/*
=================
== ARM CONTROL ==
=================
*/

/*
Sets the position of a servo to the specified angle
Params:
Servo - servo we want to control
Angle - sets the servo to this angle
*/
void setServo(int servo, int angle)
{
  if(servo == 0) {RCServo0.write(angle);}
  else if(servo == 1) {RCServo1.write(angle);}
  else {RCServo2.write(angle);}
}

/*
The arm moves up and to the centre of the box and then releases the pet
into the box
*/
void dropoff()
{
  ArmPID(HEIGHT, ARM_UP);
  ArmPID(ANGLE, ARM_CENTRE);
  setServo(SERVO_PLATE, 90);
  while(digitalRead(SWITCH_PLATE) == LOW) {}
  delay(500);
  setServo(SERVO_PLATE, 0);
}

/*
The arm will move to either the right or the left and then moves down.
The arm will then dropoff the pet in the box.
Params:
Side - the side of the robot we want to pick up on
*/
void pickup(int side)
{
  int angle = 0;
  int attempt = 0;
  ArmPID(HEIGHT, ARM_UP);
  while (digitalRead(SWITCH_PLATE) == HIGH && attempt < 2) {
    switch (attempt) {
      case 0:
        angle = side;
        break;
      case 1:
        if(side == ARM_RIGHT) {angle = side + SHIFT;}
        else {angle = side - SHIFT;}
        break;
      default:
        angle = side;
    }
    ArmPID(ANGLE, angle);
    ArmPID(HEIGHT, ARM_DOWN);
    ArmPID(HEIGHT, ARM_HOR);
    attempt++;
  }
  dropoff();
}

/*
========================
== TAPE FOLLOWING PID ==
========================
*/

// TODO: CLEAN UP MESSY CONTROL IF STATEMENTS

/*
PID control for tape following.
For reference:
  P - if too high can cause osscillations
  D - acts as damping
  */
void PIDTape()
{
  LCD.clear(); LCD.home();
  LCD.print("NUM");

  //Variables
  int P = menuItems[1].Value; //Proportional gain value
  int D = menuItems[2].Value; //Derivative gain value
  int S = menuItems[0].Value; //Speed
  int THRESHOLD = menuItems[4].Value; //threshold for switch from white to black
  int qrd_left = 0; //Value of left qrd
  int qrd_right = 0; //Value of right qrd
  int qrd_pet = 0;
  int error = -1; //Current error
  int last_error = 0;
  int recent_error = 0; //Recent error
  int proportional = 0; //Proportional control
  int derivative = 0; //Derivative control
  int duration_recent = 0; //Number of loops on recent error
  int duration_last = 0; //Number of loops on last error
  int compensation = 0; //Compensation

  // converting S into a value that is <255
  int spd = (int)((float)S*((float)255/(float)MAX_ANALOG));

  long start_time = 0;
  int count = 0;

  //PID loop
  while (true) {
    //Read QRD's
    qrd_left = analogRead(QRD_LEFT);
    qrd_right = analogRead(QRD_RIGHT);
    if(NUM > 5) {qrd_pet = analogRead(QRD_PET_LEFT);}
    else {qrd_pet = analogRead(QRD_PET_RIGHT);}

    // LCD.clear(); LCD.home();
    // LCD.print(qrd_pet); LCD.print("  "); LCD.print(analogRead(QRD_PET_RIGHT));

    if(count == 0) {start_time = millis(); count++;}
    if((millis() - start_time) > WAIT_AFTER_PID) {
      if(qrd_pet > PET_QRD_THRESHOLD) {
        NUM++;
        count--;
        LCD.setCursor(0,1); LCD.print(NUM);
        if(NUM == 4 || NUM == 6 || NUM == 7 || NUM == 8) {return;}
      }
    }

    /*Determine error
    * <0 its to the left
    * >0 its to the right
    * 0 its dead on
    */

    //left on white
    if(qrd_left < THRESHOLD) {
      //right on white
      if(qrd_right < THRESHOLD) {
        if(last_error < 0) {error = -5;} //LCD.setCursor(0,1);LCD.print("L2");
        else {error = 5;} //LCD.setCursor(0,1);LCD.print("R2");
      }
      //right on black
      else{error = -1;} //LCD.setCursor(0,1);LCD.print("L1");
    }
    //left on black
    else {
      //right on white
      if(qrd_right < THRESHOLD){error = 1;} //LCD.setCursor(0,1);LCD.print("R1");
      //right on black
      else{error = 0;} //LCD.setCursor(0,1);LCD.print("CE");
    }

    //determine control factors

    //Proportional control
    proportional = P*error;

    //Derivative
    if(error != last_error) {
      recent_error = last_error;
      duration_recent = duration_last;
      last_error = error;
      duration_last = 1;
    }
    else {
      duration_last++;
    }
    derivative = (int)(((float)D*(float)(error - recent_error))/((float)(duration_recent + duration_last)));

    //Compensation
    compensation = proportional + derivative;

    //Plant control (compensation +ve means move right)
    motor.speed(MOTOR_LEFT,spd + compensation);
    motor.speed(MOTOR_RIGHT,spd - compensation);
  }
}

/*
=============
== ARM PID ==
=============
*/

// TODO: TUNE THIS CODE SO IT IS MORE RELIABLE

/*
PID control that operates on the homebrew servos.
For reference:
  P - value decreases as it gets closer to the deadband
  D - acts as the damping force as we move closer to the deadband
  I - helps push us into the deadband if the location is just outside
Params:
  dim - specifies which motor to use, either the horizontal (angle) or the vertical (hieght)
  pos - the position to set the arm too
*/

void ArmPID(int dim, int pos)
{
  //Set variables
  int P_gain = 0;
  int I_gain = 0;
  int max_speed = 0;
  int maxI = 0;
  int MOTOR = 0;
  int PIN = 0;
  int D_gain = 0;
  int deadband = 0;
  bool high_pet = false;
  bool low_pet = false;
  int cur_angle = 0;

  //Height
  if(dim == HEIGHT) {
    cur_angle = analogRead(POTENTIOMETER_CRANE_ANGLE);
    P_gain = P_HEIGHT;
    D_gain = D_HEIGHT;
    I_gain = I_HEIGHT;
    max_speed = SPEED_HEIGHT;
    maxI = I_MAX_HEIGHT;
    MOTOR = MOTOR_CRANE_HEIGHT;
    PIN = POTENTIOMETER_CRANE_HEIGHT;
    deadband = DEADBAND_HEIGHT;
    if (pos == ARM_DOWN) {
      if ((ARM_RIGHT - (SHIFT + DEADBAND_ANGLE)) <= cur_angle && cur_angle <= (ARM_RIGHT + (SHIFT + DEADBAND_ANGLE))){high_pet = true;}
      else {low_pet = true;}
    }
  }
  //Angle
  else {
    P_gain = P_ANGLE;
    I_gain = I_ANGLE;
    D_gain = D_ANGLE;
    max_speed = SPEED_ANGLE;
    maxI = I_MAX_ANGLE;
    MOTOR = MOTOR_CRANE_ANGLE;
    PIN = POTENTIOMETER_CRANE_ANGLE;
    deadband = DEADBAND_ANGLE;
   }

  // Variables
  int pot = 0;
  int count = 0;
  int target = 0;

  // PID variables
  float proportional = 0;
  float integral = 0;
  float derivative = 0;
  float compensator = 0;

  // Errors
  float error = 0;
  float last_error = 0;

  // Timing
  long start_pid = millis();

  unsigned long last_integral_update_ms = 0;
  const unsigned int integral_update_delay_ms = 5;

  while(true){

    if(digitalRead(SWITCH_PLATE) == LOW && pos == ARM_DOWN) {return;}
    if (high_pet) {
      if ((millis() - start_pid) > MAX_TIME_SHORT) {return;}
    } else if (low_pet) {
      if ((millis() - start_pid) > MAX_TIME_LONG) {return;}
    }

    pot = analogRead(PIN);

    error = (pot - pos) / 10.0;

    if( pot <= ( pos + deadband ) && pot >= ( pos - deadband)) {
      error = 0;
      target++;
    }
    else { target = 0; }

    proportional = P_gain * error;
    derivative = D_gain * (error - last_error);
    integral = I_gain * (error) + integral;

    // handling integral gain
    if ( integral > maxI) { integral = maxI;}
    if ( integral < -maxI) { integral = -maxI;}
    if( error == 0) { integral = 0; }

    compensator = proportional + integral + derivative;

    // setting max speed for the small motor

    if( compensator > max_speed) compensator = max_speed;
    if( compensator < -max_speed) compensator = -max_speed;

    motor.speed(MOTOR, compensator);

    last_error = error;

    if ( target > 1000 ) {
      return;
    }

  }
}

/*
==========
== MENU ==
==========
*/

/*
Control code for the menu where we can adjust values to tune PID control
(only for tape following)
*/
void Menu()
{
  LCD.clear(); LCD.home();
  LCD.print("Entering menu");
  delay(500);

  while (true) {
    /* Show MenuItem value and knob value */
    int menuIndex = knob(6) * (MenuItem::MenuItemCount) / 1024;
    LCD.clear(); LCD.home();
    LCD.print(menuItems[menuIndex].Name); LCD.print(" "); LCD.print(menuItems[menuIndex].Value);
    LCD.setCursor(0, 1);
    LCD.print("Set to "); LCD.print(knob(7)); LCD.print("?");
    delay(100);

    /* Press start button to save the new value */
    if (startbutton())
    {
      delay(100);
      if (startbutton())
      {
        menuItems[menuIndex].Value = knob(7);
        menuItems[menuIndex].Save();
        delay(250);
      }
    }

    /* Press stop button to exit menu */
    if (stopbutton())
    {
      delay(100);
      if (stopbutton())
      {
        LCD.clear(); LCD.home();
        LCD.print("Leaving menu");
        delay(500);
        return;
      }
    }
  }
}
