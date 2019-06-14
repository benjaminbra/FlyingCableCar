/*
* Firmware for the ”2WD Ultrasonic Motor Robot Car Kit”
*
* Stephen A. Edwards
*
* Hardware configuration :
* A pair of DC motors driven by an L298N H bridge motor driver
* An HC−SR04 ultrasonic range sensor mounted atop a small hobby servo
*/
#include <Servo.h>
#include <math.h>
#include <EnableInterrupt.h>

#define SERIAL_PORT_SPEED 57600
#define RC_NUM_CHANNELS  5

#define RC_CH1  0 // Forward Axis
#define RC_CH2  1 // Left Axis
#define RC_CH3  2 // Servo Left Axis
#define RC_CH4  3 // Hand mode
#define RC_CH5  4 // Servo Nitro 

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH3_INPUT  A2
#define RC_CH4_INPUT  A3
#define RC_CH5_INPUT  A4

uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

Servo servo;
Servo nitro;
// Ultrasonic Module pins
const int trigPin = 13; // 10 microsecond high pulse causes chirp , wait 50 us
const int echoPin = 12; // Width of high pulse indicates distance
// Servo motor that aims ultrasonic sensor .
const int servoPin = 11; // PWM output for hobby servo
// Motor control pins : L298N H bridge
const int enAPin = 6; // Left motor PWM speed control
const int in1Pin = 7; // Left motor Direction 1
const int in2Pin = 5; // Left motor Direction 2
const int in3Pin = 4; // Right motor Direction 1
const int in4Pin = 2; // Right motor Direction 2
const int enBPin = 3; // Right motor PWM speed control
const int inNitroPin = 8;

enum Motor { LEFT, RIGHT };

// Set motor speed: 255 full ahead, −255 full reverse , 0 stop
void go( enum Motor m, int speed) {
  digitalWrite (m == LEFT ? in1Pin : in3Pin , speed > 0 ? HIGH : LOW );
  digitalWrite (m == LEFT ? in2Pin : in4Pin , speed <= 0 ? HIGH : LOW );
  analogWrite(m == LEFT ? enAPin : enBPin, speed < 0 ? -speed : speed );
}

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }
void calc_ch5() { calc_input(RC_CH5, RC_CH5_INPUT); }

float * getControllerValue() {
  rc_read_values();

  static float r[5];
  // r[0] is the acceleration we want for the car (from 255 to -255)
  // r[1] is the rotation of wheels(from 255(left) to -255(rigth))
  r[0] = -((((rc_values[RC_CH1]/10.0)-110)/81)*500 - 255); // Forward Axis
  r[1] = -((((rc_values[RC_CH2]/10.0)-110)/81)*500 - 255); // Left Axis
  r[2] = 6 - ((((rc_values[RC_CH3]/10.0)-110)/81)*6); // Servo Left Axis
  r[3] = -((((rc_values[RC_CH4]/10.0)-110)/81)*500 - 255); // Hand Mode
  r[4] = -((((rc_values[RC_CH5]/10.0)-110)/81)*500 - 255); // Nitro Mode

  return r;
}

// Initial motor test :
// left motor forward then back
// right motor forward then back
void testMotors ()
{
  static int speed[8] = { 128, 255, 128, 0 , -128, -255, -128, 0};
  go(RIGHT, 0);
  for (unsigned char i = 0 ; i < 8 ; i++)
    go(LEFT, speed[i]), delay (200);
    
  for (unsigned char i = 0 ; i < 8 ; i++)
    go(RIGHT, speed[i]), delay (200);
}

// Read distance from the ultrasonic sensor , return distance in mm
//
// Speed of sound in dry air , 20C is 343 m/s
// pulseIn returns time in microseconds (10ˆ−6)
// 2d = p * 10ˆ−6 s * 343 m/s = p * 0.00343 m = p * 0.343 mm/us
unsigned int readDistance () {
  digitalWrite ( trigPin , HIGH );
  delayMicroseconds (10);
  digitalWrite ( trigPin , LOW );
  unsigned long period = pulseIn ( echoPin, HIGH );
  return period * 343 / 2000;
}

#define NUM_ANGLES 5
unsigned char sensorAngle[NUM_ANGLES] = { 40, 50, 60, 70, 80 };
unsigned int distance[NUM_ANGLES];
// Scan the area ahead by sweeping the ultrasonic sensor left and right
// and recording the distance observed. This takes a reading , then
// sends the servo to the next angle. Call repeatedly once every 50 ms or so.
void readNextDistance ()
{
  static unsigned char angleIndex = 0;
  static signed char step = 1;
  distance[angleIndex ] = readDistance();
  angleIndex += step ;
  if (angleIndex == NUM_ANGLES - 1) step = -1;
  else if (angleIndex == 0) step = 1;
  servo.write(sensorAngle[angleIndex]);
}

void autoMode(){
  readNextDistance ();
  // See if something is too close at any angle
  unsigned char tooClose = 0;
  
  for (unsigned int i = 0; i < NUM_ANGLES; i++) {
    if ( distance[i] < 300) {
      tooClose = 1; 
    }
  }
  if (tooClose) {
    // Something's nearby: back up left
    go(LEFT, -180);
    go(RIGHT, -80);
  } else {
    // Nothing in our way: go forward
    go(LEFT, 255);
    go(RIGHT, 255);
  }
}

int * controllerMoving(float forwardAxis, float leftAxis){
  float left = 0;
  float right = 0;
  
  if(forwardAxis > 70 || forwardAxis < -70 || leftAxis > 70 || leftAxis < -70){

    if(forwardAxis > 70){
      if(leftAxis > 70){
        left = 128;
        right = 255;
      } else if(leftAxis < -70){
        left = 255;
        right = 128;
      } else {
        left = 255;
        right = 255;
      }
    } else if (forwardAxis < -70){
      if(leftAxis > 70){
        left = -128;
        right = -255;
      } else if(leftAxis < -70){
        left = -255;
        right = -128;
      } else {
        left = -255;
        right = -255;
      }
    } else {
      if(leftAxis > 70){
        left = -128;
        right = 128;
      } else if(leftAxis < -70){
        left = 128;
        right = -128;
      } 
    }
    
  }
  
  static int r[2];
  r[0] = (int) left;
  r[1] = (int) right;
  
  return r;
}

void motorAngle(float leftAxis){
  int roundAxis = (int) floor(leftAxis);
  if(roundAxis >= 6){
    roundAxis = 4;
  } else if(roundAxis <= 0){
    roundAxis = 0;
  } else if(roundAxis == 5) {
    roundAxis = 3;
  } else if(roundAxis == 2 || roundAxis == 3 || roundAxis == 4){
    roundAxis = 2;
  }
  servo.write(sensorAngle[roundAxis]);
}

void nitroHandler(float power){
  int nitroSpeed = 0;
  if(power > 175){
    nitroSpeed = 0;
  } else if (power > 100) {
    nitroSpeed = 1600;
  } else if (power > 75) {
    nitroSpeed = 1700;
  } else if (power > -75) {
    nitroSpeed = 1800;
  } else {
    nitroSpeed = 1900;
  } 
  
  nitro.writeMicroseconds(nitroSpeed);
}

// Initial configuration
//
// Configure the input and output pins
// Center the servo
// Turn off the motors
// Test the motors
// Scan the surroundings once
//
void setup () {
  Serial.begin(SERIAL_PORT_SPEED);
  pinMode(trigPin , OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite ( trigPin , LOW);
  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enBPin, OUTPUT);
  pinMode(inNitroPin, OUTPUT);
  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);

  
  servo.attach(servoPin);
  nitro.attach(inNitroPin);
  
  delay(5000);
  
  servo.write(90);
  // Starting Nitro Servo by sending a small signal
  nitro.writeMicroseconds(500);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  enableInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);
  
  go(LEFT, 0);
  go(RIGHT, 0);
  testMotors();
  // Scan the surroundings before starting
  //servo.write(sensorAngle[3]);
  delay(200);

  servo.write(sensorAngle[3]);
  delay(2000);
  
}

// Main loop:
//
// Get the next sensor reading
// If anything appears to be too close , back up
// Otherwise, go forward
//
void loop() {
  
  float * commands = getControllerValue();
  if(commands[3] > 100){
    int * axis = controllerMoving(commands[0], commands[1]);
    motorAngle(commands[2]);
    
    go(LEFT, axis[0]);
    go(RIGHT, axis[1]);
  } else if(commands[3] < 100){
    autoMode();
  }
  nitroHandler(commands[4]);
  // Check the next direction in 50 ms
  delay (50);

}
