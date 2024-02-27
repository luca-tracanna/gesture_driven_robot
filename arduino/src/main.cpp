/* Example code for HC-SR04 ultrasonic distance sensor with Arduino. 
   No library required. More info: https://www.makerguides.com */
#include <Arduino.h>
#include <Servo.h>


const uint8_t SERVO_RIGHT = 5;
const uint8_t SERVO_FRONT = 4;
const uint8_t SERVO_LEFT = 10;

const uint8_t RIGHT_ECHO = 6;
const uint8_t RIGHT_TRIGGER = 7;
const uint8_t FRONT_ECHO = 2;
const uint8_t FRONT_TRIGGER = 3;
const uint8_t LEFT_ECHO = 11;
const uint8_t LEFT_TRIGGER = 12;

Servo servo_left;
Servo servo_front;
Servo servo_right;

void setup() {
  pinMode(LEFT_ECHO, INPUT);
  pinMode(LEFT_TRIGGER, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(FRONT_TRIGGER, OUTPUT);
  pinMode(RIGHT_ECHO, INPUT);
  pinMode(RIGHT_TRIGGER, OUTPUT);
  servo_left.attach(SERVO_LEFT);
  servo_right.attach(SERVO_RIGHT);
  servo_front.attach(SERVO_FRONT);
  Serial.begin(9600);
  servo_left.write(54);
  servo_front.write(54);
  servo_right.write(54);
  while (!Serial) delay(10); //attendo connessione seriale

  delay(500);
}

unsigned long readDistance(const uint8_t TRIGGER, const uint8_t ECHO){
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds( 10 );
  digitalWrite(TRIGGER, LOW);
  unsigned long distance = pulseIn(ECHO, HIGH, 10000 ) * 0.017;
  return distance == 0 ? 200 : distance;
}

int j = 1, i = 55;
void loop() {

  servo_left.write(i);
  servo_front.write(i);
  servo_right.write(i);
  
  unsigned long left_dist = readDistance(LEFT_TRIGGER, LEFT_ECHO);
  // delay(20);
  unsigned long front_dist = readDistance(FRONT_TRIGGER, FRONT_ECHO);
  // delay(20);
  unsigned long right_dist = readDistance(RIGHT_TRIGGER, RIGHT_ECHO);

  Serial.print(i);
  Serial.print(' ');
  Serial.print(left_dist);
  Serial.print(' ');
  Serial.print(front_dist);
  Serial.print(' ');
  Serial.println(right_dist);



  if(i == 55) {
    j = 1;
  } 
  

  if(i == 124)  {
    j = - 1;
    // i = 54;
    servo_left.write(i);
    servo_front.write(i);
    servo_right.write(i);
    
  }
  i += j;

  
}