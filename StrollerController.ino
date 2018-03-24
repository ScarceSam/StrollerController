/* Stroller Controller V0.1 
 *  Continuously calibrates and reads joystick
 *  Plays sounds T00-T03 when joystick buttons are pressed
 *  
 *  Arduino PinOut
 *  A0 - Thumb Stick
 *  A1 - X Axis
 *  A2 - Y axis
 *  A3 - Volume (not used)
 *  A4 - 
 *  A5 - 
 *  D0 - Control box button
 *  D1 - Joystick Trigger button
 *  D2 - Joystick Side thumb button
 *  D3 - Joystick Pinky button
 *  D4 - Joystick Thumb button
 *  D5 - MCU
 *  D6 - MCU
 *  D7 - Gear Shifting Servo (not used)
 *  D8 - LED#1
 *  D9 - LED#2
 *  D10 - Soundboard UG
 *  D11 - Soundboard RX
 *  D12 - SoundBoard TX
 *  D13 - LED#3
 */
#include <Bounce2.h>
#include <SoftwareSerial.h>
#include <Adafruit_Soundboard.h>

//set led pins
#define LED_PIN1 8
#define LED_PIN2 9
#define LED_PIN3 13
#define NUM_LEDS 3
//LED pins into an array for easy itteration
const uint8_t LED_PINS[NUM_LEDS] = {LED_PIN1, LED_PIN2, LED_PIN3};

//set button pins
#define CONTROL_BUTTON 0  
#define TRIGGER 1         //plays sound T00
#define SIDE_BUTTON 2     //plays sound T01
#define PINKY_BUTTON 3    //plays sound T02
#define THUMB_BUTTON 4 //plays sound T03
#define NUM_BUTTONS 5//Button pins into an array for easy itteration
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {CONTROL_BUTTON, TRIGGER, SIDE_BUTTON, 
                                          PINKY_BUTTON, THUMB_BUTTON};

//set joystick inputs
#define X_PIN A1
#define Y_PIN A2

//set SFX board pins
#define SFX_TX 12
#define SFX_RX 11
#define SFX_RST 5
#define SFX_UG 10

//set needed variables
//joystick variables
int xMax = 512;
int xMid = 512;
int xMin = 512;
int yMax = 512;
int yMid = 512;
int yMin = 512;
int xValue = 0;
int yValue = 0;

//variables for readButtons()
int sound = 0;

//needed variables for flash()
long long timestamp = 0;
bool flashState = LOW;

//variable for sound shifting
int soundSet = 0;

Bounce * buttons = new Bounce[NUM_BUTTONS];

SoftwareSerial ssSfx = SoftwareSerial(SFX_TX, SFX_RX);
Adafruit_Soundboard sfx = Adafruit_Soundboard(&ssSfx, NULL, SFX_RST);

void setup() {
  //set joystick mid pint for future calibration.
  xMid = analogRead(X_PIN);
  yMid = analogRead(Y_PIN);

  //setup debouncing for all buttons
  for (int i = 0; i < NUM_BUTTONS; i++) {
    buttons[i].attach( BUTTON_PINS[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
    buttons[i].interval(25);              // interval in ms
  }

  //Set LEDs as inputs
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

//  Serial.begin(9600);
}

void loop() {

  xValue = analogRead(X_PIN);
  yValue = analogRead(Y_PIN);
  calibrateJoystick();
  readJoystick();
  if(sound == 0) {
    readButtons();
  }
  play();
}


void calibrateJoystick() {
  //calibrate joystick by tracking 
  //the max and mid raw input readings.
  if(xValue > xMax) {
    xMax = xValue;
  }
  if(xValue < xMin) {
    xMin = xValue;
  }
  if(yValue > yMax) {
    yMax = yValue;
  }
  if(yValue < yMin) {
    yMin = yValue;
  }
}


void readJoystick() {
  //read joystick by reading its raw value and maping it to 
  //a -100 to 100 range w/mid point mapped to 0. 
  //x values: -100==FullLeft 100==FullRight
  //y values: -100==FullBack 100==FullForward
  if(xValue >= xMid) {
    xValue = map(xValue, xMid, xMax, 0, -100);
  }
  else if(xValue < xMid) {
    xValue = map(xValue, xMin, xMid, 100, 0);
  }

  if(yValue >= yMid) {
    yValue = map(yValue, yMid, yMax, 0, -100);
  }
  else if(yValue < yMid) {
    yValue = map(yValue, yMin, yMid, 100, 0);
  }
  delay(2);
}


void readButtons() {
  
  for (int i = 0; i < NUM_BUTTONS; i++)  {
    // Update the Bounce instance :
    buttons[i].update();
    // If it fell, flag the need to toggle the LED
    if ( buttons[i].fell() ) {
      sound = i + 1;
      break;
    }
  }
}


void flash(int LED, int pause) {
  if ( millis() > timestamp + pause) {
    timestamp = millis();
    flashState = !flashState;
    digitalWrite( LED, flashState); 
  }
}


void play() {
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


/*void printDebugInfo() {
  
  Serial.print("xMid= ");
  Serial.print(xMid);
  Serial.print(", yMid= ");
  Serial.print(yMid);
  Serial.print(", xAxis= ");
  Serial.print(xValue);
  Serial.print(", yAxis= ");
  Serial.print(yValue);
  Serial.print(", sound= ");
  Serial.print(sound);  
  Serial.print("\n");
  
}
*/
