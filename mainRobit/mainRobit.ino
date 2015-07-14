#include <avr/EEPROM.h>
#include <phys253.h>          
#include <LiquidCrystal.h>

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
		Name 		  = name;
		Value         = eeprom_read_word(EEPROMAddress);
	}
	void Save()
	{
		eeprom_write_word(EEPROMAddress, Value);
	}
};
 
uint16_t MenuItem::MenuItemCount = 0;
/* Add the menu items here */
MenuItem Speed            = MenuItem("Speed");
MenuItem ProportionalGainTape = MenuItem("P-gain Tape");
MenuItem DerivativeGain   = MenuItem("D-gain Tape");
MenuItem IntegralGain     = MenuItem("I-gain Tape");
MenuItem ThresholdTape    = MenuItem("Threshold Tape");
MenuItem ProportionalGainIR = MenuItem("P-gain IR");
MenuItem ThresholdIR      = MenuItem("Threshold IR");
MenuItem menuItems[]      = {Speed, ProportionalGainTape, DerivativeGain, IntegralGain, ThresholdTape, ProportionalGainIR, ThresholdIR};
 
void setup()
{
  #include <phys253setup.txt>
  LCD.clear();
  LCD.home();
  
  int MOTOR_LEFT = 2; //PWM output for left motor
  int MOTOR_RIGHT = 3; //PWM output for right motor
  int QRD_LEFT = 0; //Left QRD for tape following
  int QRD_RIGHT = 1; //Right QRD for tape following
  int QRD_PET = 2; //QRD for locating pets
  int SWITCH_PLATE = 1; //Switch to see if pet is on plate
  int SERVO_CRANE = 1; //Servo for rotation of crane arm
  int SERVO_FRONT = 2; //Servo for front arm
  int MOTOR_CRANE = 1; //Motor of arm movement
  int SWITCH_CRANE_1 = 2; //Switch for the crane arm in the down position
  int SWITCH_CRANE_2 = 3; //Switch for the crane arm in the up position
  
  int NUM = 0;
  int MAX_ANALOG = 1023;
}



void loop()
{ 
  motor.speed(MOTOR_LEFT,0);
  motor.speed(MOTOR_RIGHT,0);
  
  LCD.clear(); LCD.home();
  LCD.print("Start: Menu");
  LCD.setCursor(0, 1);
  LCD.print("Stop: PID");
  delay(100);
 
  if (startbutton())
  {
    delay(100);
    if (startbutton())
    {
      Menu();
    }
  }

  if (stopbutton())
  {
    delay(100);
    if (stopbutton())
    {
      mainStart();  
    } 
  }
}
 
void mainStart()
{
  if (NUM == 0) {
    setServo(SERVO_CRANE, 90);
    moveArm(0);
    PIDTape();
  }
  if (NUM == 4) {
    pickupFront();
    PIDIR();
  }
}

void PIDTape()
{  
  LCD.clear(); LCD.home();
  LCD.print("Following");
  
  //Variables
  int MAX_INTEGRAL = 50; //Maximum integral term value
  int P = menuItems[1].Value; //Proportional gain value
  int D = menuItems[2].Value; //Derivative gain value
  int I = menuItems[3].Value; //Integral gain value
  int S = menuItems[0].Value; //Speed
  int THRESHOLD = menuItems[4].Value; //threshold for switch from white to black
  int qrd_left = 0; //Value of left qrd
  int qrd_right = 0; //Value of right qrd
  int qrd_pet = 0; //Value of pet qrd
  int error = 0; //Current error
  int last_error = 0; //Previous error
  int recent_error = 0; //Recent error
  int total_error = 0; //Integral of the error
  int proportional = 0; //Proportional control
  int derivative = 0; //Derivative control
  int integral = 0; //Integral control
  int duration_recent = 0; //Number of loops on recent error
  int duration_last = 0; //Number of loops on last error
  int compensation = 0; // 
  
  int spd = (int)((float)S*((float)255/(float)MAX_ANALOG));
  
  int WAIT_IN_MILLI = 2000;
  int last_time = 0;
  
  //PID loop
  while (true)
  {
    //Read QRD's
    qrd_left = analogRead(QRD_LEFT);
    qrd_right = analogRead(QRD_RIGHT);
    qrd_pet = analogRead(QRD_PET);
    
    if ((millis()-last_time) > WAIT_IN_MILLI) {
      if(qrd_pet > THRESHOLD) {NUM++; last_time=millis();}
    }
    if(NUM == 4 || NUM == 5 || NUM == 6 || NUM == 6) {break;}
    
    /*Determine error
    * <0 its to the left
    * >0 its to the right
    * 0 its dead on
    */
    
    //left on white
    if(qrd_left < THRESHOLD){
      //right on white
      if(qrd_right < THRESHOLD){
        if(last_error < 0) {error = -5;LCD.setCursor(0,1);LCD.print("L2");}
        else {error = 5;LCD.setCursor(0,1);LCD.print("R2");}
      }
      //right on black
      else{error = -1;LCD.setCursor(0,1);LCD.print("L1");}
    }
    //left on black
    else{
      //right on white
      if(qrd_right < THRESHOLD){error = 1;LCD.setCursor(0,1);LCD.print("R1");}
      //right on black
      else{error = 0;LCD.setCursor(0,1);LCD.print("CE");}
    }
    
    //determine control factors
    
    //Proportional control
    proportional = P*error;
    
    //Integral error
    total_error += error;
    integral = I*total_error;
    if(integral > MAX_INTEGRAL) {integral = MAX_INTEGRAL;}
    if(integral < -MAX_INTEGRAL) {integral = -MAX_INTEGRAL;}
    
    //Derivative
    if(error != last_error){
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
    compensation = proportional + integral + derivative;
    
    //Plant control (compensation +ve means move right)
    motor.speed(MOTOR_LEFT,spd + compensation);
    motor.speed(MOTOR_RIGHT,spd - compensation);
  }
}

void PIDIR()
{ 
  //Variables
  int IR_PIN = 0; //Pin for IR sensor
  int P = menuItems[5].Value; //Proportional gain value
  int S = menuItems[0].Value; //Speed
  int THRESHOLD = menuItems[6].Value; //Threshold max IR value
  int irVal = 0; //Value of IR sensor
  int error = 0; //Current error
  int lastError = 0; //Previous error
  int proportional = 0; //Proportional control
  int compensation = 0; //The final compensation value
  int newDirection = 0; //-1 for left, 1 for right
  int oldDirection = 0; //-1 for left, 1 for right
  
  int spd = (int)((float)S*((float)255/(float)1023));
  
  //PID loop
  while (true)
  {
    
    //For debugging
    LCD.clear(); LCD.home();
    LCD.print(error); LCD.print("  "); LCD.print(oldDirection);
    
    //Read QRD's
    irVal = analogRead(IR_PIN);
    
    /*Determine error
    * <0 its to the left
    * >0 its to the right
    * 0 its dead on
    */
    
    error = THRESHOLD - irVal;
    
    //Was going right
    if (oldDirection > 0) {
      //Going in wrong direction
      if (lastError < error) {newDirection = -1;}
      //Going right direction
      else {newDirection = 1;}
    //Was going left
    } else {
      //Going wrong direction
      if (lastError < error) {newDirection = 1;} 
      //Going right direction
      else {newDirection = -1;}
    }
    
    //Proportional control
    proportional = P*error;
    
    //Compensation
    compensation = proportional*newDirection;
    
    //Plant control (compensation +ve means move right)
    motor.speed(MOTOR_LEFT,spd + compensation);
    motor.speed(MOTOR_RIGHT,spd - compensation);
    
    oldDirection = newDirection;
    lastError = error;
  }
}

void Menu()
{
	LCD.clear(); LCD.home();
	LCD.print("Entering menu");
	delay(500);
 
	while (true)
	{
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

void setServo(int servo, int angle) {
  if(servo == 1) {RCServo1.write(angle);}
  else if(servo == 2) {RCServo2.write(angle);}
  else {RCServo3.write(angle);}
}

void moveArm(int pos) {
  
}

void pickupFront() {
  
}
