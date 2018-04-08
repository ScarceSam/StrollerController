/***************************************************************************** 
 *  Stroller Controller V0.1 
 *  Continuously calibrates and reads joystick
 *  Plays sounds T00-T03 when joystick buttons are pressed
 *  
 *  Arduino PinOut
 *  A0 - Thumb Stick
 *  A1 - X Axis
 *  A2 - Y axis
 *  A3 - Volume (not used)
 *  A4 - sound set shifting
 *  A5 - Max speed adjustment
 *  D0 - Control box button
 *  D1 - Joystick Trigger button
 *  D2 - Joystick Side thumb button
 *  D3 - Joystick Pinky button
 *  D4 - Joystick Thumb button
 *  D5 - MCU
 *  D6 - MCU
 *  D7 - Soundboard Rst (temporarily connected to D5)
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

//set SFX board constants
#define SFX_TX 12
#define SFX_RX 11
#define SFX_RST 5
#define SFX_UG 10

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

//needed variables for flash()
unsigned long timestamp = 0;
bool flashState = LOW;

//variable for sound set shifting
int soundSet = 0;

//variables for motor control
int rMotor = 0;
int lMotor = 0;
float rMax = 0.0;
float rMin = 0.0;
float lMax = 0.0;
float lMin = 0.0;

//
Bounce * buttons = new Bounce[NUM_BUTTONS];

//initiate Serial for sound board
SoftwareSerial ssSfx = SoftwareSerial(SFX_TX, SFX_RX);

//
Adafruit_Soundboard sfx = Adafruit_Soundboard(&ssSfx, NULL, SFX_RST);

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
  pinMode( SFX_UG, OUTPUT);
  digitalWrite( SFX_UG, LOW);

  //start serial communication with Sound board
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
    
    //translate the joysticks values to usable numbers for the Motors
    readJoystick();
  
    //read button inputs if a sound has not been called for
    if(sound == 0) {
      readButtons();
    }
  
    //play sound if one is queued up
    if(sound != 0) {
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
   * a range of -100-100,  w/mid point mapped to 0. 
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

void translateToMotors() {
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

  lMotor = floatMap(constrain(lMotor, -100, 100), -100, 100, -127, 127);
  rMotor = floatMap(constrain(rMotor, -100, 100), -100, 100, -127, 127);
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
      sound = i + 1;
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
    delay(10); 
  }else{
    sound = 0;
  }
}
