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

//How many pets to get
volatile unsigned int PETS = 6;

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
==========
== MODE ==
==========
*/

const bool DEBUG = true;

/*
==========
== MISC ==
==========
*/

const int PID_TIMEOUT = 2000;

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
const int QRD_PET_RIGHT = 5; //QRD for locating pets back
const int QRD_PET_LEFT = 7; //QRD for locating pets front
const int POTENTIOMETER_CRANE_HEIGHT = 6; //Rotary potentiometer for crane arm
const int POTENTIOMETER_CRANE_ANGLE = 4; //Rotary potentiometer for crane arm
const int IR_LEFT = 2;
const int IR_RIGHT = 3;

//Servo
const int SERVO_PLATE = 0; //Servo to drop pet
const int SERVO_FRONT = 1; //Servo to pick up pet 6

//Digital
const int SWITCH_PLATE = 3; //Switch to see if pet is on plate
const int SWITCH_FRONT = 4; //Switch to bump into bucket
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
const int SPEED_HEIGHT = 85;
const int SPEED_ANGLE = 80;

// PID Constants
const int P_HEIGHT = 10;
const int I_HEIGHT = 10;
const int I_MAX_HEIGHT = 150;
const int D_HEIGHT = 25;
const int P_ANGLE = 15;
const int I_ANGLE = 0;
const int I_MAX_ANGLE = 150;
const int D_ANGLE = 5;

// Positions for main arm
// const int ARM_UP = 830;
// const int ARM_UP_2 = 840;
// const int ARM_HOR = 750;
// const int ARM_DOWN_LOW = 590;
// const int ARM_DOWN_HIGH = 690;
const int ARM_UP = 290;
const int ARM_UP_2 = 260;
const int ARM_HOR = 380;
const int ARM_DOWN_LOW = 520;
const int ARM_DOWN_HIGH = 420;
const int ARM_LEFT = 200;
const int ARM_CENTRE = 480;
const int ARM_RIGHT = 670;
const int SHIFT = 70; // The amount the arm shifts on each attempt

// Positions for front arm
const int FRONT_FORWARD_CENTRE = 143;
const int FRONT_FORWARD_LEFT = 180;
const int FRONT_FORWARD_RIGHT = 120;
const int FRONT_PET = 30;
const int FRONT_BACK = 0;

// Range of where the arm will be in an "error-free" zero
const int DEADBAND_HEIGHT = 20;
const int DEADBAND_ANGLE = 25;

// Other
const int MAX_PICKUP_TIME = 2000; //Max time the arm can move down for pickup (low pet)pet)
const int TIMEOUT = 4000;
const int MAX_ANALOG = 1023; // for converting arduino resolution to speed
const int PET_QRD_THRESHOLD = 600; // when the arm will stop to pick up pets

//For reference
const int HEIGHT = 1;
const int ANGLE = 2;

//For filtering
const int FILTER_SAMPLES = 4;
unsigned int height_pot = 0;

/*
===============================
== ROTARY ENCORDER CONSTANTS ==
===============================
*/

// wheel dimensions
const float DIST_PER_TAPE = PI; //Distance wheel moves per tape hit (cm)
const float LENGTH_OF_AXLE = 23.7; //Length of axle (cm)
const int WAIT_AFTER_PID = 700;
const int TURN_SPEED = 100;
const int LIN_SPEED = 150;
const int LEFT = -1;
const int RIGHT = 1;

// Variables
int TURNS_LEFT = 0;
int TURNS_RIGHT = 0;
int prev_enc_left = 0;
int prev_enc_right = 0;
int cur_enc_left = 0;
int cur_enc_right = 0;

/*
==================
== IR CONSTANTS ==
==================
*/

//PID
const int P_IR = 50;
const int D_IR = 60;
const int SPEED_IR = 700;

//Other
const int STOP_RE_1 = 34;
const int EXIT_RE = 1;
const int EXIT_SWI = 2;
const int EXIT_TAPE = 3;
const int IR_TIMEOUT = 4000;

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
    ArmPID(HEIGHT,ARM_UP,false);
    ArmPID(ANGLE,ARM_CENTRE,false);
    setServo(SERVO_FRONT,FRONT_BACK);
    ArmPID(HEIGHT,ARM_DOWN_LOW,true);
    count_setup++;
  }

  LCD.clear(); LCD.home();
  LCD.print("Start: Menu");
  LCD.setCursor(0, 1);
  LCD.print("Pets: "); LCD.print(PETS);
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

  //Change number of pets to get
  if (digitalRead(SWITCH_FRONT) == LOW) {
    delay(100);
    if (digitalRead(SWITCH_FRONT) == LOW) {
      switch (PETS) {
        case 6:
          PETS = 3;
          break;
        case 3:
          PETS = 6;
          break;
        default:
          PETS = 6;
      }
    }
    delay (500);
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
    // Strategy for 6 pets
    if (NUM == 0) {
      ArmPID(HEIGHT,ARM_HOR, false);
      PIDTape(PETS == 3);
    }
    if (NUM == 3) {
      moveTo(50, 0, false);
      moveTo(270, 0, true);
      setServo(SERVO_FRONT,FRONT_PET);
      NUM = 5;
      PIDTape(false);
    }
    if (NUM == 4) {
      moveTo(-20, 10, false);
      moveTo(5, 17, false);
      setServo(SERVO_FRONT,FRONT_FORWARD_CENTRE);
      delay(500);
      moveTo(-35, 0, false);
      PIDIR(EXIT_RE);
      ArmPID(HEIGHT,ARM_UP,false);
      pickup(ARM_RIGHT, true);
      ArmPID(HEIGHT,ARM_HOR,false);
      PIDIR(EXIT_SWI);
      pickupFront(3);
      moveTo(0,-50,false);
      moveTo(180,0,false);
      setServo(SERVO_FRONT,FRONT_PET);
      PIDIR(EXIT_TAPE);
      moveTo(0,10,false);
      findTape(LEFT);
      NUM++;
      PIDTape(false);
    }
    if (NUM == 6) {
      moveTo(0, 10, false);
      // ArmPID(HEIGHT,ARM_UP,false);
      pickup(ARM_LEFT, true);
      findTape(LEFT);
      PIDTape(false);
    }
    if (NUM == 7) {
      moveTo(0, 30, false);
      moveTo(-20, -17, false);
      // ArmPID(HEIGHT,ARM_UP,false);
      pickup(ARM_LEFT, true);
      findTape(RIGHT);
      PIDTape(false);
    }
    if (NUM == 8) {
      moveTo(0, 28, false);
      moveTo(-20, -13, false);
      // ArmPID(HEIGHT,ARM_UP,false);
      pickup(ARM_LEFT, false);
      setServo(SERVO_FRONT,FRONT_FORWARD_CENTRE);
      delay(500);
      moveTo(-60, 0, true);
      PIDTape(false);
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
void findTape(int side) {
  int THRESHOLD = menuItems[4].Value;
  int compensator = 5;
  int count = 0;
  moveTo(-50*side,0,true);
  while(analogRead(QRD_LEFT) < THRESHOLD && count < 4) {
    moveTo((100+compensator)*side,0,true);
    if(analogRead(QRD_LEFT) > THRESHOLD) {stopDrive(); return;}
    moveTo((-100+compensator)*side,0,true);
    compensator += 5;
    count ++;
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
if tape is true it stop when the fron QRD's hit tape
*/
void moveTo(int angle, float distance, bool tape)
{
  //Vars
  int THRESHOLD = menuItems[4].Value;

  //First change angle
  int angleTurns = ceil((abs(angle)*((LENGTH_OF_AXLE*PI)/360))/DIST_PER_TAPE);
  int linTurns = ceil(abs(distance)/DIST_PER_TAPE);
  bool leftDone = false;
  bool rightDone = false;
  TURNS_RIGHT = 0; TURNS_LEFT = 0;

  //debugging
  LCD.clear(); LCD.home();
  LCD.print(angleTurns); LCD.print("  "); LCD.print(linTurns);

  if (angle > 0) {
    motor.speed(MOTOR_LEFT, -1*TURN_SPEED);
    motor.speed(MOTOR_RIGHT, TURN_SPEED);
  } else if (angle == 0) {
    stopDrive();
  } else {
    motor.speed(MOTOR_LEFT, TURN_SPEED);
    motor.speed(MOTOR_RIGHT, -1*TURN_SPEED);
  }
  // LCD.clear(); LCD.home(); LCD.print("Turning");
  while((leftDone == false || rightDone == false) && angle != 0) {
    if(tape && analogRead(QRD_LEFT) > THRESHOLD) {stopDrive(); return;}
    checkEnc();
    if(DEBUG) {
      LCD.setCursor(0,1); LCD.print(TURNS_LEFT); LCD.print("  "); LCD.print(TURNS_RIGHT);
    }
    if(TURNS_LEFT >= angleTurns) {motor.speed(MOTOR_LEFT,0); leftDone = true;}
    if(TURNS_RIGHT >= angleTurns) {motor.speed(MOTOR_RIGHT,0); rightDone = true;}
  }

  //Then change linear distance
  TURNS_LEFT = 0; TURNS_RIGHT = 0;
  leftDone = false; rightDone = false;
  if (distance > 0) {
    motor.speed(MOTOR_LEFT,LIN_SPEED);
    motor.speed(MOTOR_RIGHT,LIN_SPEED);
  } else if (distance == 0) {
    stopDrive();
  } else {
    motor.speed(MOTOR_LEFT,-1*LIN_SPEED);
    motor.speed(MOTOR_RIGHT,-1*LIN_SPEED);
  }
  // LCD.clear(); LCD.home(); LCD.print("Moving");
  while((leftDone == false || rightDone == false) && distance != 0) {
    if(tape && analogRead(QRD_LEFT) > THRESHOLD) {stopDrive(); return;}
    checkEnc();
    if(DEBUG) {
      LCD.setCursor(0,1); LCD.print(TURNS_LEFT); LCD.print("  "); LCD.print(TURNS_RIGHT);
    }
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
Measurements with filters
*/
void filter_measure() {
  int new_value = analogRead(POTENTIOMETER_CRANE_HEIGHT);
  height_pot = (FILTER_SAMPLES-1)*height_pot + new_value;
  height_pot = height_pot / FILTER_SAMPLES;
}

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
void dropoff(bool drop)
{
  ArmPID(HEIGHT, ARM_UP, false);
  ArmPID(ANGLE, ARM_CENTRE, false);
  if(drop) {
    ArmPID(HEIGHT, ARM_UP_2, false);
    setServo(SERVO_PLATE, 90);
  }
  while(digitalRead(SWITCH_PLATE) == LOW && drop) {}
  delay(500);
  if(NUM == 4) {ArmPID(HEIGHT,ARM_HOR,true);}
  else {ArmPID(HEIGHT,ARM_UP,false);}
  setServo(SERVO_PLATE, 0);
}

/*
The arm will move to either the right or the left and then moves down.
The arm will then dropoff the pet in the box.
Params:
Side - the side of the robot we want to pick up on
*/
void pickup(int side, bool drop)
{
  int angle = 0;
  int height = ARM_DOWN_LOW;
  int height_check = ARM_HOR;
  int attempt = 0;
  // ArmPID(HEIGHT, ARM_UP, false);
  while (digitalRead(SWITCH_PLATE) == HIGH && attempt < 2) {
    switch (attempt) {
      case 0:
        angle = side;
        if(side == ARM_RIGHT) {
          height = ARM_DOWN_HIGH;
          height_check = ARM_UP;
        }
        break;
      case 1:
        if(side == ARM_RIGHT) {
          angle = side + SHIFT;
        }
        else {angle = side - SHIFT;}
        break;
      default:
        angle = side;
    }
    ArmPID(ANGLE, angle, false);
    ArmPID(HEIGHT, height, true);
    ArmPID(HEIGHT, height_check, false);
    attempt++;
  }
  dropoff(drop);
}

/*
Performs actions to pickup 6th pet
*/
void pickupFront(int times) {
  int count = 0;
  delay(500);
  for (int i = 0; i < times; i++) {
    if(count == 0) {
      setServo(SERVO_FRONT,FRONT_FORWARD_LEFT);
      delay(600);
      count ++;
    } else {
      setServo(SERVO_FRONT,FRONT_FORWARD_RIGHT);
      delay(600);
      count --;
    }
  }
  setServo(SERVO_FRONT,FRONT_FORWARD_CENTRE);
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

void PIDTape(bool timeout)
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
  int count_debug = 0;

  //PID loop
  while (true) {
    //Read QRD's
    qrd_left = analogRead(QRD_LEFT);
    qrd_right = analogRead(QRD_RIGHT);
    if(NUM > 5) {qrd_pet = analogRead(QRD_PET_LEFT);}
    else {qrd_pet = analogRead(QRD_PET_RIGHT);}

    //debugging
    if(DEBUG) {
      if (count > 500) {
        LCD.clear(); LCD.home();
        LCD.print("L: "); LCD.print(spd + compensation); LCD.print(" R: "); LCD.print(spd - compensation);
        LCD.setCursor(0,1);
        LCD.print("L: "); LCD.print(qrd_left); LCD.print(" R: "); LCD.print(qrd_right);
        count_debug = 0;
      }
      count_debug ++;
    }

    if(timeout && NUM == 3 && (millis() - start_time) > PID_TIMEOUT) {stopDrive(); return;}

    if(count == 0) {start_time = millis(); count++;}
    if((millis() - start_time) > WAIT_AFTER_PID) {
      if(qrd_pet > PET_QRD_THRESHOLD) {
        NUM++;
        count--;
        LCD.setCursor(0,1); LCD.print(NUM);
        if(NUM == 4 || NUM == 6 || NUM == 7 || NUM == 8) {stopDrive(); return;}
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
============
== IR PID ==
============
*/

/*
exit:

*/

void PIDIR(int exitVal)
{
  //Variables
  int irVal_left = 0; //Value of IR sensor
  int irVal_right = 0;
  float error = 0; //Current error
  float last_error = 0; //Previous error
  float proportional = 0; //Proportional control
  float derivative = 0; //Derivative cotrol
  float compensation = 0; //The final compensation value
  int count = 0;
  int turns = ceil(STOP_RE_1/DIST_PER_TAPE);
  TURNS_LEFT = 0;
  TURNS_RIGHT = 0;
  int spd_left = 0;
  int spd_right = 0;
  int THRESHOLD = menuItems[4].Value;

  int spd = (int)((float)SPEED_IR*((float)255/(float)1023));

  long start_pid = millis();

  //PID loop
  while (true)
  {

    // For debugging
    if (DEBUG) {
      if (count > 500){
        //For debugging
        LCD.clear(); LCD.home();
        LCD.print("l: "); LCD.print(irVal_left); LCD.print(" r: "); LCD.print(irVal_right);
        LCD.setCursor(0,1); LCD.print("error: "); LCD.print(error);
        count = 0;
      }
      count++;
    }

    //Read IR
    irVal_left = analogRead(IR_LEFT);
    irVal_right = analogRead(IR_RIGHT);

    //Stop at RE stop value
    if(exitVal == EXIT_RE) {
      checkEnc();
      if((TURNS_LEFT >= turns && TURNS_RIGHT >= turns) || (millis() - start_pid) > IR_TIMEOUT) {
        if(DEBUG) {
          LCD.clear(); LCD.home(); LCD.print("RE Thresh");
        }
        stopDrive();
        return;
      }
    }

    //Exit if front switch hits
    if(exitVal == EXIT_SWI && digitalRead(SWITCH_FRONT) == LOW) {stopDrive(); return;}

    //Exit if pet qrd's see tape
    if(exitVal == EXIT_TAPE && (analogRead(QRD_PET_LEFT) > PET_QRD_THRESHOLD || analogRead(QRD_RIGHT) > PET_QRD_THRESHOLD)) {stopDrive(); return;}

    /*Determine error
    * <0 its to the left
    * >0 its to the right
    * 0 its dead on
    */
    error = (irVal_left - irVal_right)/((irVal_right + irVal_left)/2.0);

    //Proportional control
    proportional = P_IR*error;

    //Derivative
    derivative = D_IR*(error - last_error);

    //Compensation
    compensation = proportional+derivative;

    if(spd + compensation < 0) {spd_left = 0;}
    else {spd_left = spd + compensation;}
    if(spd - compensation < 0) {spd_right = 0;}
    else {spd_right = spd - compensation;}

    //Plant control (compensation +ve means move right)
    motor.speed(MOTOR_LEFT,spd_left);
    motor.speed(MOTOR_RIGHT,spd_right);

    last_error = error;
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

void ArmPID(int dim, int pos, bool swi)
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
    P_gain = P_HEIGHT;
    D_gain = D_HEIGHT;
    I_gain = I_HEIGHT;
    max_speed = SPEED_HEIGHT;
    maxI = I_MAX_HEIGHT;
    MOTOR = MOTOR_CRANE_HEIGHT;
    PIN = POTENTIOMETER_CRANE_HEIGHT;
    deadband = DEADBAND_HEIGHT;
    if (swi) {
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

    //For debugging
    if(DEBUG) {
      if (count > 500){
        LCD.clear(); LCD.home();
        LCD.print("target: "); LCD.print(pos);
        LCD.setCursor(0,1);
        LCD.print("Cur: "); LCD.print(pot);
        count = 0;
      }
      count++;
    }

    //Check if max time is reached
    if((millis() - start_pid) > TIMEOUT) {motor.speed(MOTOR,0);return;}

    //Stop timer in case arm stuck
    if(digitalRead(SWITCH_PLATE) == LOW && swi) {motor.speed(MOTOR,0); return;}
    if (high_pet) {
      if ((millis() - start_pid) > MAX_PICKUP_TIME) {motor.speed(MOTOR,0);return;}
    } else if (low_pet) {
      if ((millis() - start_pid) > MAX_PICKUP_TIME) {motor.speed(MOTOR,0);return;}
    }

    //Read current position
    if (dim == HEIGHT) {filter_measure(); pot = height_pot;}
    pot = analogRead(PIN);

    //Determine error
    error = (pot - pos) / 10.0;

    //Check if arm is within deadband of target
    if( pot <= ( pos + deadband ) && pot >= ( pos - deadband)) {
      if (dim == HEIGHT && target == 0) {target = 900;}
      target++;
    }
    else { target = 0; }

    //Calculating compensation
    proportional = P_gain * error;
    derivative = D_gain * (error - last_error);
    integral = (I_gain * error) + integral;

    // handling integral gain
    if ( integral > maxI) { integral = maxI;}
    if ( integral < -maxI) { integral = -maxI;}
    if( error == 0) { integral = 0; }

    compensator = proportional + integral + derivative;

    if( compensator > max_speed) compensator = max_speed;
    if( compensator < -max_speed) compensator = -max_speed;

    motor.speed(MOTOR, compensator);

    last_error = error;

    //Break if arm is at target for long time
    if ( target > 1000 ) {
      motor.speed(MOTOR,0);
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
