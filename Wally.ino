#include <Wire.h>                                             // Calls for I2C bus library
#include <Servo.h>

#define MD25ADDRESS         0x58                              // Address of the MD25
#define SPEED1              0x00                              // Byte to send speed to both motors for forward and backwards motion if operated in MODE 2 or 3 and Motor 1 Speed if in MODE 0 or 1
#define SPEED2              0x01                              // Byte to send speed for turn speed if operated in MODE 2 or 3 and Motor 2 Speed if in MODE 0 or 1
#define ENCODERONE          0x02                              // Byte to read motor encoder 1
#define ENCODERTWO          0x06                              // Byte to read motor encoder 2
#define ACCELERATION        0xE                               // Byte to define motor acceleration
#define CMD                 0x10                              // Byte to reset encoder values
#define MODE_SELECTOR       0xF                               // Byte to change between control MODES

Servo dropperServo;
Servo armServo;
Servo clawServo;
Servo rocketServo;
Servo pushServo;

/*
 * Pin selections
 */
int echoPinFore = 2;
int trigPinFore = 3;
int startPin = 4;
int echoPinBack = 5;
int trigPinBack = 6;
int rocketPin = 7;
int clawServoPin = 8;
int armServoPin = 9;
int pushPin = 10;
int dropperServoPin = 11;

int led = 13;

/*
 * Misc Variables
 */
int DualSpeedValue = 0;                                       // Combined motor speed variable
int Mode = 2;                                                 // MODE in which the MD25 will operate selector value 
float wheelSep = 29.7;
float wheel1 = 0;                                             //wheel 1 travel dist
float wheel2 = 0;                                             //wheel 2 travel dist 
const float pi = 3.141593;
const int maxFwdSpeed = 100; 
long startTime = 0;
int pos = 0;
int motorSpeed = 255;                                         //choose value 0 to 255


void setup(){
  dropperServo.attach(dropperServoPin);            
  armServo.attach(armServoPin);
  clawServo.attach(clawServoPin);
  rocketServo.attach(rocketPin);
  rocketServo.write(155);
  dropperServo.write(120);
  armServo.write(90);
  pushServo.attach(pushPin);
  pushServo.write(10);
  rocketServo.write(30);
  pinMode(trigPinFore, OUTPUT);
  pinMode(echoPinFore, INPUT);
  pinMode(trigPinBack, OUTPUT);
  pinMode(echoPinBack, OUTPUT);
  pinMode(startPin, INPUT_PULLUP);
  pinMode(led, OUTPUT);                                       // LEDs to check sensor
  Wire.begin();                                               // Begin I2C bus
  Serial.begin(9600);                                         // Begin serial
  delay(100);                                                 // Wait for everything to power up
  Wire.beginTransmission(MD25ADDRESS);                        // Set MD25 operation MODE
  Wire.write(MODE_SELECTOR);
  Wire.write(Mode);                                           
  Wire.endTransmission();
  encodeReset();                                              // Calls a function that resets the encoder values to 0 
}


void loop(){ 
  delay(500);
  YELLOW();
  delay(999999);
}


void Go(float Dist, int Speed){
  checkTime();
  int newSPD = Speed;
  if (Dist < 0){ newSPD = 255-Speed;} 
  forward(Dist, newSPD);
  stopMotor();
  delay(200);
  encoder1();
  encoder2();
  float correcSpeed = 125;
  if (Dist < 0){ correcSpeed = 131;}
  float difference1 = abs(encoder1()) - abs(Dist);
  float difference2 = abs(encoder2()) - abs(Dist);
  Serial.print(difference1);
  encodeReset();
  correction(difference1, difference2, correcSpeed);
  stopMotor();
  encodeReset();
}


void checkForwardDistance() {
  long duration, distance;
  digitalWrite(trigPinFore, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinFore, HIGH);

  delayMicroseconds(10); // Added this line
  digitalWrite(trigPinFore, LOW);
  duration = pulseIn(echoPinFore, HIGH);
  distance = (duration/2) / 29.1;
  if (distance < 15) {                                            // This is where the LED On/Off happens
    digitalWrite(led,HIGH);                                       // When the Red condition is met, the Green LED should turn off
    stopMotor();
    delay(500);
    checkForwardDistance();
    }
  else {
    digitalWrite(led,LOW);
  }
  delay(50);
}


void checkBackwardDistance() {
  long duration, distance;
  digitalWrite(trigPinBack, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPinBack, HIGH);

  delayMicroseconds(10); // Added this line
  digitalWrite(trigPinBack, LOW);
  duration = pulseIn(echoPinBack, HIGH);
  distance = (duration/2) / 29.1;
  if (distance < 15) {  // This is where the LED On/Off happens
    digitalWrite(led,HIGH); // When the Red condition is met, the Green LED should turn off
    //stopMotor();
    delay(500);
    checkBackwardDistance();
    }
  else {
    digitalWrite(led,LOW);
  }
  delay(50);
}


void pickUp(){
  armDown();
  Go(8, 148);
  delay(10);
  clawClose();
  delay(10);
  Go(-8, 145);
  delay(10);
  armUp();
  delay(50);
  clawOpen();
  delay(10);
  push();
  delay(10);
}


void startCheck(){
  if (digitalRead(startPin) == LOW){ startCheck();}
}


void start(){
  startCheck();
  startTime = millis();
  Serial.print("BEGIN time = ");
  Serial.print(startTime);
}


void checkTime(){
  if ((millis() - startTime) >= 90000){
    stopMotor();
    rocket();
    delay(9000000);
  }
}

  
void push(){
  delay(10);
  pushServo.write(146);
  delay(1000);
  pushServo.write(10);
  delay(10);
}


void drop(){
 dropperServo.write(0);
 delay(2000);
 dropperServo.write(120);
 delay(10) ;
}


void rocket(){
  delay(100);
  rocketServo.write(30);
  delay(1500);
  rocketServo.write(160);
  delay(100);
}

void clawOpen(){
  delay(10);
  clawServo.write(90);
  delay(10);
}

void clawClose(){
  delay(10);
  clawServo.write(10);
  delay(10);
}

void clawDrop(){
  delay(100);
  clawServo.write(35);
  delay(100);
}


void armDown(){
  delay(10);
  for (pos = 89; pos >= 0; pos -= 1) {                  // goes from 180 degrees to 0 degrees
    armServo.write(pos);                                // tell servo to go to position in variable 'pos'
    delay(9);                                           // waits 15ms for the servo to reach the position
  }
  delay(10);
}


void armUp(){
  delay(10);
  for (pos = 0; pos <= 89; pos += 1) {                  // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    armServo.write(pos);                                // tell servo to go to position in variable 'pos'
    delay(9);                                           // waits 15ms for the servo to reach the position
  }
  delay(10);
}


void encodeReset(){                                             // This function resets the encoder values to 0
  Wire.beginTransmission(MD25ADDRESS);
  Wire.write(CMD);
  Wire.write(0x20);                                         
  Wire.endTransmission(); 
  delay(50);
  }
float encoder1(){                                                // Function to read and display value of encoder 1 as a long
  Wire.beginTransmission(MD25ADDRESS);                           // Send byte to get a reading from encoder 1
  Wire.write(ENCODERONE);
  Wire.endTransmission();
  Wire.requestFrom(MD25ADDRESS, 4);                               // Request 4 bytes from MD25
  while(Wire.available() < 4);                                    // Wait for 4 bytes to arrive
  long poss1 = Wire.read();                                       // First byte for encoder 1, HH.
  poss1 <<= 8;
  poss1 += Wire.read();                                           // Second byte for encoder 1, HL
  poss1 <<= 8;
  poss1 += Wire.read();                                           // Third byte for encoder 1, LH
  poss1 <<= 8;
  poss1  +=Wire.read();                                           // Fourth byte for encoder 1, LLalue
  delay(5);                                                       // Wait for everything to make sure everything is sent
  return(poss1*0.0873);                                               // Convert encoder value to cm
  }

float encoder2(){                                                     // Function to read and display velue of encoder 2 as a long
  Wire.beginTransmission(MD25ADDRESS);           
  Wire.write(ENCODERTWO);
  Wire.endTransmission();
  Wire.requestFrom(MD25ADDRESS, 4);                                   // Request 4 bytes from MD25
  while(Wire.available() < 4);                                        // Wait for 4 bytes to become available
  long poss2 = Wire.read();                                           // First byte for encoder 2, HH
  poss2 <<= 8;
  poss2 += Wire.read();                                               // Second byte for encoder 2, HL             
  poss2 <<= 8;
  poss2 += Wire.read();                                               // Third byte for encoder 2, LH             
  poss2 <<= 8;
  poss2  +=Wire.read();                                               // Fourth byte for encoder 2, LLalue
  delay(5);                                                          // Wait to make sure everything is sent
  return(poss2*0.0873);                                               // Convert encoder value to cm
}


void stopMotor(){                                                     // Function to stop motors
  Wire.beginTransmission(MD25ADDRESS);                                // Sets the acceleration to register 10 (0.65s)
  Wire.write(ACCELERATION);
  Wire.write(10);
  Wire.endTransmission();
  Wire.beginTransmission(MD25ADDRESS);                                // Stops motors motor 1 if operated in MODE 0 or 1 and Stops both motors if operated in MODE 2 or 3
  Wire.write(SPEED1);
  Wire.write(128);
  Wire.endTransmission();
  Wire.beginTransmission(MD25ADDRESS);                                // Stops motors motor 2 when operated in MODE 0 or 1 and Stops both motors while in turning sequence if operated in MODE 2 or 3
  Wire.write(SPEED2);
  Wire.write(128);
  Wire.endTransmission();
  delay(50);
  encoder1();                                                         // Calls a function that reads value of encoder 1
  encoder2();                                                         // Calls a function that reads value of encoder 2
  //Serial.print("Encoder 1 Distance CM : ");                         // Displays las recorded traveled distance
  //Serial.print(encoder1());
  //Serial.print("   ");
  //Serial.print("Encoder 2 Distance CM : ");
  //Serial.print(encoder2());
  //Serial.println(" ");
}  


void changeMode(int mode)                                               // Set MD25 operation MODE
{
  Wire.beginTransmission(MD25ADDRESS);                        
  Wire.write(MODE_SELECTOR);
  Wire.write(mode);                                           
  Wire.endTransmission();
}


void correction(float difference1, float difference2, int correctionSpeed){
  checkTime();
  encoder1();
  encoder2();
  if (abs(encoder1()) <= abs(difference1) && abs(encoder2()) <= abs(difference2)){
    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(ACCELERATION);
    Wire.write(1);
    Wire.endTransmission();
    Wire.beginTransmission(MD25ADDRESS);
    Wire.write(SPEED1);
    Wire.write(correctionSpeed);
    Wire.endTransmission();
    correction(difference1, difference2, correctionSpeed);
  }
}


void correctionTurn(float diff1, float diff2, int correcSpeed){               //enter speed of 1-128
  checkTime();
  encoder1();
  encoder2();
  if (abs(encoder1()) <= abs(diff1) && abs(encoder2()) <= abs(diff2)){
    Wire.beginTransmission(MD25ADDRESS);                                      // Sets the acceleration to register 1 (6.375s)
    Wire.write(ACCELERATION);
    Wire.write(1);
    Wire.endTransmission();
    Wire.beginTransmission(MD25ADDRESS);                                      // Sets a combined motor speed value
    Wire.write(SPEED2);
    Wire.write(correcSpeed);
    Wire.endTransmission();
    correctionTurn(diff1, diff2, correcSpeed);
  }
}


void forward(float distance, int DualSpeedValue){
  checkTime();
  encoder1();
  encoder2();
  if (distance > 0){
    if (abs(encoder1()) <= abs(distance) && abs(encoder2()) <= abs(distance)){
      checkForwardDistance();
      Wire.beginTransmission(MD25ADDRESS);
      Wire.write(ACCELERATION);
      Wire.write(1);
      Wire.endTransmission();
      Wire.beginTransmission(MD25ADDRESS);
      Wire.write(SPEED1);
      Wire.write(DualSpeedValue);
      Wire.endTransmission();
      forward(distance, DualSpeedValue);
    }
  }
  else{
    if (abs(encoder1()) <= abs(distance) && abs(encoder2()) <= abs(distance)){
      checkBackwardDistance();
      Wire.beginTransmission(MD25ADDRESS);
      Wire.write(ACCELERATION);
      Wire.write(1);
      Wire.endTransmission();
      Wire.beginTransmission(MD25ADDRESS);
      Wire.write(SPEED1);
      Wire.write(DualSpeedValue);
      Wire.endTransmission();
      forward(distance, DualSpeedValue);
    }
  }
}
int calcSpeed(double dist, double aim, int maxspeed, int limit)
{
  float vel = abs(maxspeed * ((aim - dist)/aim));
  int vel2 = vel;
  vel2 += (vel - vel2 < 0.5) ? 0:1;
  if(vel2 < limit) vel2 = limit;
  if(vel2 == 0) vel2 = 1;
  return vel2;
}


void arcTravel(int arcrad, int arcangle){                                   // Function to make the robot travel in an arc.
  encodeReset();
  int forwardSpeed = 40;                                                    // Sets ArcSpeed.
  float vel = forwardSpeed * ((arcrad - wheelSep/2.0)/(arcrad + wheelSep/2.0));           // Calculates speed for inner wheel
  int vel2 = vel;
  vel2 += (vel - vel2 < 0.5) ? 0:1;
  changeMode(0);
  double distance1 = 0;
  double traveldist = 2.0 * (arcrad + wheelSep/2.0) * (arcangle/360.0) * pi;
  while(distance1 < traveldist)
  {
    forwardSpeed = calcSpeed(distance1, traveldist, 40, 40);
    float vel = forwardSpeed * ((arcrad - wheelSep/2.0)/(arcrad + wheelSep/2.0));
    int vel2 = vel;
    vel2 += (vel - vel2 < 0.5) ? 0:1;
    Wire.beginTransmission(MD25ADDRESS);                                      // Sets the acceleration to register 10 (0.65s)
    Wire.write(ACCELERATION); 
    Wire.write(10);
    Wire.endTransmission();
    Wire.beginTransmission(MD25ADDRESS);                      // Stops motors motor 1 if operated in MODE 0 or 1 and Stops both motors if operated in MODE 2 or 3
    Wire.write(SPEED1);
    Wire.write(128+forwardSpeed);
    Wire.endTransmission();
    Wire.beginTransmission(MD25ADDRESS);                      // Stops motors motor 2 when operated in MODE 0 or 1 and Stops both motors while in turning sequence if operated in MODE 2 or 3
    Wire.write(SPEED2);
    Wire.write(128+vel2);
    Wire.endTransmission();
    distance1 = encoder1();
  }
  forwardSpeed = maxFwdSpeed;
  changeMode(2);
  stopMotor();
  delay(100);
}


void turn(float angle, int DualSpeedValue){                            // This function turns the platform right by a previously set angle
  checkTime();
  float wheel1 = (angle/360.0)*pi*wheelSep;
  float wheel2 = -(wheel1);
  //Serial.print ("the arc dist is : ");
  //Serial.print (wheel1);
  //Serial.print("   ");
  encoder1();                                                           // Calls a function that reads value of encoder 1
  encoder2();                                                           // Calls a function that reads value of encoder 2
    if (abs(encoder1()) <= abs(wheel1) && abs(encoder2()) <= abs(wheel2)){     // If statement to check the status of the traveled distance
    Wire.beginTransmission(MD25ADDRESS);                                // Sets the acceleration to register 1 (6.375s)
    Wire.write(ACCELERATION);
    Wire.write(1);
    Wire.endTransmission();
    Wire.beginTransmission(MD25ADDRESS);                                // Sets a combined motor speed value
    Wire.write(SPEED2);
    Wire.write(DualSpeedValue);
    Wire.endTransmission();
    turn(angle, DualSpeedValue);
  }
}


void rightTurn(float ang, int spd){
  checkTime();
  int propSpeed = spd;
  int speedC = 120;
  if (ang < 0 ){
    propSpeed = 255-spd;
    speedC = 255-speedC;}
  turn(ang, propSpeed);
  stopMotor();
  float wheel1 = (ang/360.0)*pi*wheelSep;
  float wheel2 = -(wheel1);
  delay(50);
  encoder1();
  encoder2();
  float diff1 = (encoder1()) - wheel1;
  float diff2 = (encoder2()) - wheel2;
  Serial.print("diff1=    ");
  Serial.print(diff1);
  encodeReset();
  correctionTurn((diff1), (diff2), speedC);
  stopMotor();
  encodeReset();
}


void BLUE(){
  delay(1000);
  start();
  delay(200);
  Go(75, 148);
  delay(500);
  rightTurn(30, 150);
  delay(500);
  Go(30, 150);
  delay(500);
  rightTurn(-120, 150);
  clawOpen();
  pickUp();
  pickUp();
  pickUp();
  pickUp();
  delay(500);
  rightTurn(-158.7, 150);
  delay(500);
  Go(30, 150);
  armDown();
  clawClose();
  armUp();
  delay(50);
  clawOpen();
  Go(11.3, 150);
  rightTurn(23.7, 150);
  Go(58.5, 150);
  armDown();
  clawClose();
  armUp();
  delay(50);
  clawOpen();
  rightTurn(70, 150);
  Go(40, 150);
  rightTurn(90, 150);
  drop();
}


void YELLOW(){
  start();
  delay(100);
  Go(75, 150);
  delay(10);
  rightTurn(-29, 150);
  delay(10);
  Go(26, 150);
  delay(10);
  rightTurn(120, 150);
  clawOpen();
  delay(10);
  pickUp();
  pickUp();
  pickUp();
  pickUp();
  delay(10);
  rightTurn(140, 150);
  armDown();
  delay(10);
  Go(7, 150);
  forward(4, 140);
  stopMotor();
  encodeReset();
  clawClose();
  armUp();
  delay(50);
  clawOpen();
  push();
  //rightTurn(-8, 150);
  armDown();
  Go(76, 170);
  forward(6, 140);
  stopMotor();
  encodeReset();
  clawClose();
  armUp();
  delay(50);
  clawOpen();
  push();
  rightTurn(-45, 150);
  Go(33, 150);
  rightTurn(90, 150);
  drop();
  Go(-12, 145);
  drop();
  Go(25, 145);
  drop();
  rightTurn(-70, 150);
  Go(-70, 150);
  drop();
  drop();
  rocket();
  delay(30000);
}

