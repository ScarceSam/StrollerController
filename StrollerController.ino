/***************************************************************************** 
 *  Stroller Controller V1.0 - MakerFaire Version
 *  Continuously calibrates and reads joystick
 *  Outputs motor speed info to motor controller 
 *  Plays sounds T00-T03 when joystick buttons are pressed
 *  
 *  Arduino PinOut
 *  A0 - Thumb Stick
 *  A1 - X Axis
 *  A2 - Y axis
 *  A3 - Max speed adjustment
 *  A4 - sound set shifting
 *  A5 - SFX_ACT
 *  D0 - Control box button
 *  D1 - Joystick Trigger button
 *  D2 - Joystick Side thumb button
 *  D3 - Joystick Pinky button
 *  D4 - Joystick Thumb button
 *  D5 - MCU
 *  D6 - Fan
 *  D7 - Soundboard Rst
 *  D8 - LED#1
 *  D9 - LED#2
 *  D10 - Soundboard UG
 *  D11 - Soundboard RX
 *  D12 - SoundBoard TX
 *  D13 - LED#3
 *****************************************************************************/

#include <Bounce2.h>
#include <SoftwareSerial.h>
#include <Adafruit_Soundboard.h>
#include <SyRenSimplified.h>

/*******************
 * set defines
 *******************/
 
//set led pins constants & array
#define LED_PIN1 8
#define LED_PIN2 9
#define LED_PIN3 13
#define NUM_LEDS 3
//LED pins into an array for easy itteration
const uint8_t LED_PINS[NUM_LEDS] = {LED_PIN1, LED_PIN2, LED_PIN3};

//set button constants & array
#define CONTROL_BUTTON 0  
#define TRIGGER 1         //plays sound T00
#define SIDE_BUTTON 2     //plays sound T01
#define PINKY_BUTTON 3    //plays sound T02
#define THUMB_BUTTON 4    //plays sound T03
#define NUM_BUTTONS 5     //Button pins into an array for easy itteration
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {CONTROL_BUTTON, TRIGGER, SIDE_BUTTON, 
                                          PINKY_BUTTON, THUMB_BUTTON};

//set joystick constants
#define X_PIN A1
#define Y_PIN A2
#define START_DELAY 1000
#define BUFFER 25

//set max speed constant
#define SPEED A3

//set fan constant
#define FAN 6

#define MCU 5

#define RAMP 20

//set SFX board constants
#define SFX_TX 12
#define SFX_RX 11
#define SFX_RST 7
#define SFX_UG 10
#define SFX_ACT 19

/*******************
 * set needed variables
 *******************/
 
//joystick variables
int xMax = 512;
int xMid = 512;
int xMin = 512;
int yMax = 512;
int yMid = 512;
int yMin = 512;
int xValue = 0;
int yValue = 0;
unsigned long bootUpTime = 0;

//variables for readButtons()
int sound = 0;
unsigned long requestTime = 0;

//needed variables for flash()
unsigned long timestamp = 0;
bool flashState = LOW;

//variable for sound set shifting
unsigned long resetTime = 0;
int soundSet = 0;

//variables for motor control
int rMotor = 0;
int lMotor = 0;
float rMax = 0.0;
float rMin = 0.0;
float lMax = 0.0;
float lMin = 0.0;

//variale to monitor loop speed
bool loopFlash = 0;

//
Bounce * buttons = new Bounce[NUM_BUTTONS];

//initiate Serial for sound board
SoftwareSerial ssSfx = SoftwareSerial(SFX_TX, SFX_RX);
Adafruit_Soundboard sfx = Adafruit_Soundboard(&ssSfx, NULL, SFX_RST);

SoftwareSerial ssSaber = SoftwareSerial(NOT_A_PIN, MCU);
SyRenSimplified SR(ssSaber);

void setup() {
  
  //set joystick mid pint for calibration.
  xMid = analogRead(X_PIN);
  yMid = analogRead(Y_PIN);

  //setup debouncing for all buttons
  for (int i = 0; i < NUM_BUTTONS; i++) {
    buttons[i].attach( BUTTON_PINS[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
    buttons[i].interval(25);              // interval in ms
  }

  //Set LEDs as outputs
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }

  //Set sound board to UART controll
  pinMode(SFX_UG, OUTPUT);
  digitalWrite(SFX_UG, LOW);

  //set input to monitor soundboard activity
  pinMode(SFX_ACT, INPUT_PULLUP);

  //start serial communication with Sound board
  ssSaber.begin(9600);

  //set motors to stop incase they powered down in motion
  SR.motor(64);
  SR.motor(-64);
  
  ssSfx.begin(9600);

  //Flash LED1 untill soundbaord is connected
  if (!sfx.reset()) {
    flash(LED_PIN1, 50);
    while (1);
  }
  digitalWrite(LED_PIN1, LOW);

  //capture bootup time to add to start delay
  bootUpTime = millis();
}

void loop() {

  //Toggle LED at the beginning of loop to monitor for delays in system
  loopFlash = !loopFlash;
  digitalWrite(LED_PIN1, loopFlash);

  //read raw joystick position
  xValue = analogRead(X_PIN);
  yValue = analogRead(Y_PIN);

  //set the joystick's usable range
  calibrateJoystick();

  //allow time for proper calibration befor going into operation
  if (millis() < START_DELAY + bootUpTime) {

    //flash LED2 while in initial calibration mode
    flash(LED_PIN2, 50);

    //if calibration has not been started extend calibration delay
    if (xMax < 712 || xMin > 312 || yMax < 712 || yMin > 312) {

      bootUpTime = millis();
    }

  }else if (millis() < START_DELAY + bootUpTime + 50) {
    
    //turn off LED2
    digitalWrite(LED_PIN2, LOW);
    
  }else{
    
    //read & translate the joysticks values to usable numbers for the Motors
    readJoystick();
    maths();
    drive();
  
    //read button inputs if a sound has not been called for and a sound isnt palying
    if(sound == 0 && millis() > requestTime + 500 && digitalRead(SFX_ACT) == 1) {
      readButtons();
    }
  
    //play sound if one is queued up
    if(sound != 0 && millis() > resetTime + 1000) {
    play();
    }
  }
}


void calibrateJoystick() {
  
  //calibrate joystick by tracking 
  //the max and min raw input readings.
  if (xValue > xMax) {
    xMax = xValue;
  }
  else if (xValue < xMin) {
    xMin = xValue;
  }
  
  if (yValue > yMax) {
    yMax = yValue;
  }
  else if (yValue < yMin) {
    yMin = yValue;
  }
}


void readJoystick() {
  /*********************
   * 
   * read joystick by reading its raw value and maping it to 
   * a range of -100 to 100,  w/mid point mapped to 0. 
   * x values: -100==FullLeft 100==FullRight
   * y values: -100==FullBack 100==FullForward
   * 
   ********************/
  if(xValue > xMid + BUFFER) {
    xValue = map(xValue, xMid + BUFFER, xMax, 0, -100);
  }
  else if(xValue < xMid - BUFFER) {
    xValue = map(xValue, xMin, xMid - BUFFER, 100, 0);
  }
  else{
    xValue = 0;
  }

  if(yValue > yMid + BUFFER) {
    yValue = map(yValue, yMid + BUFFER, yMax, 0, -100);
  }
  else if(yValue < yMid - BUFFER) {
    yValue = map(yValue, yMin, yMid - BUFFER, 100, 0);
  }
  else{
    yValue = 0;
  }
}

void maths() {
  /*********************
   * 
   * takes the x joystick value and finds 
   * the range to map the y value to to get
   * the individual motor speeds. then maps 
   * the desired motor speed percentage to 
   * the int needed by the motor controller.
   * 
   ********************/
  if (xValue == 0) {
  
    rMotor = yValue;
    lMotor = yValue;
  
  }else if (xValue > 0) {
    
    lMax = (0.00625*(xValue*xValue))+(0.875*xValue)+100;
    lMin = floatMap((float)xValue, 0, 100, -100, -50);
    lMotor = floatMap(yValue, -100, 100, lMin, lMax);

    rMax = floatMap(xValue, 0, 100, 100, 50);
    rMin = (-0.00625*(xValue*xValue))+(-0.875*xValue)-100;
    rMotor = floatMap(yValue, -100, 100, rMin, rMax);
  
  }else if (xValue < 0) {
        
    lMax = floatMap(xValue, 0, -100, 100, 50);
    lMin = (-0.00625*(xValue*xValue))+(0.875*xValue)-100;
    lMotor = floatMap(yValue, -100, 100, lMin, lMax);

    rMax = (0.00625*(xValue*xValue))+(-0.875*xValue)+100;
    rMin = floatMap(xValue, 0, -100, -100, -50);
    rMotor = floatMap(yValue, -100, 100, rMin, rMax);
  }

  //restricts the output range based on the position of the speed potentiometer
  int maxSpeed = map(analogRead(SPEED), 0, 1023, 0, 100);

  //limit the speed value then convert it to the selected max range
  lMotor = map(constrain(lMotor, -100, 100), -100, 100, -maxSpeed, maxSpeed);
  rMotor = map(constrain(rMotor, -100, 100), -100, 100, -maxSpeed, maxSpeed);
  
  int fanSpeed = map((abs(lMotor) + abs(rMotor)), 0, 200, 75, 130);
  if (fanSpeed < 80){
    fanSpeed = 0;
  }
  analogWrite(FAN, fanSpeed);
}


void drive() {

  static int targetLeft;
  static int currentLeft;
  static int targetRight;
  static int currentRight;
  static unsigned long lastChange = 0;
  static unsigned long lastChangeLeft = 0;
  static unsigned long lastChangeRight = 0;

  targetLeft = lMotor;
  targetRight = rMotor;

  if ( currentLeft == 0 )
  {
    if ( targetLeft < currentLeft )
    {
      currentLeft--;
      SR.motor(map(currentLeft, -100, 100, 0, -127));
    }
    if ( targetLeft > currentLeft ) 
    {
      currentLeft++;
      SR.motor(map(currentLeft, -100, 100, 0, -127));
    }
  }
  else if ( currentLeft > 0 )
  {
    if ( targetLeft < currentLeft )
    {
      currentLeft--;
      SR.motor(map(currentLeft, -100, 100, 0, -127));
    }
    if ( targetLeft > currentLeft && millis() > lastChangeLeft + RAMP ) 
    {
      currentLeft++;
      SR.motor(map(currentLeft, -100, 100, 0, -127));
      lastChangeLeft = millis();
    }
  }
  else if ( currentLeft < 0 )
  {
    if ( targetLeft < currentLeft && millis() > lastChangeLeft + RAMP )
    {
      currentLeft--;
      SR.motor(map(currentLeft, -100, 100, 0, -127));
      lastChangeLeft = millis();
    }
    if ( targetLeft > currentLeft ) 
    {
      currentLeft++;
      SR.motor(map(currentLeft, -100, 100, 0, -127));
    }
  }


  if ( currentRight == 0 )
  {
    if ( targetRight < currentRight )
    {
      currentRight--;
      SR.motor(map(currentRight, -100, 100, 0, 127));
    }
    if ( targetRight > currentRight ) 
    {
      currentRight++;
      SR.motor(map(currentRight, -100, 100, 0, 127));
    }
  }
  else if ( currentRight > 0 )
  {
    if ( targetRight < currentRight )
    {
      currentRight--;
      SR.motor(map(currentRight, -100, 100, 0, 127));
    }
    if ( targetRight > currentRight && millis() > lastChangeRight + RAMP ) 
    {
      currentRight++;
      SR.motor(map(currentRight, -100, 100, 0, 127));
      lastChangeRight = millis();
    }
  }
  else if ( currentRight < 0 )
  {
    if ( targetRight < currentRight && millis() > lastChangeRight + RAMP )
    {
      currentRight--;
      SR.motor(map(currentRight, -100, 100, 0, 127));
      lastChangeRight = millis();
    }
    if ( targetRight > currentRight ) 
    {
      currentRight++;
      SR.motor(map(currentRight, -100, 100, 0, 127));
    }
  }

/*
  
  if ( millis() > lastChange + RAMP) 
  {
    if ( targetLeft > currentLeft )
    {
      currentLeft++;
      SR.motor(map(currentLeft, -100, 100, 0, -127));
    }
    else if ( targetLeft < currentLeft )
    {
      currentLeft--;
      SR.motor(map(currentLeft, -100, 100, 0, -127));
    }
    
    if ( targetRight > currentRight )
    {
      currentRight++;
      SR.motor(map(currentRight, -100, 100, 0, 127));
    }
    else if ( targetRight < currentRight )
    {
      currentRight--;
      SR.motor(map(currentRight, -100, 100, 0, 127));
    }
    lastChange = millis();
  }  
*/  
  //SR.motor(map(lMotor, -100, 100, 0, 127));
  //SR.motor(map(rMotor, -100, 100, 0, -127));
}


float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void readButtons() {
  
  for (int i = 0; i < NUM_BUTTONS; i++)  {
    // Update the Bounce instance :
    buttons[i].update();
    
    // If it fell, set button variable
    if ( buttons[i].fell() ) {
      sound = i + 4 * map(analogRead(A4), 0, 1023, 2, 0);
      requestTime = millis();
      break;
    }
  }
}


void flash(int LED, int pulse) {

  //flash given LED with given pulse rate
  if ( millis() > timestamp + pulse) {
    timestamp = millis();
    flashState = !flashState;
    digitalWrite( LED, flashState); 
  }
}


void play() {
  
  //try to play sound, if failed- reset SFXboard else reset sound variable
  if(!sfx.playTrack((uint8_t)sound)) {
    digitalWrite(SFX_RST, LOW);
    pinMode(SFX_RST, OUTPUT);
    delay(10);
    pinMode(SFX_RST, INPUT);
    resetTime = millis(); 
    sound = 0;
  }else{
    sound = 0;
  }
}
