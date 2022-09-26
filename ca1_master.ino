// assessment code final draft

#include <Servo.h>
Servo myServo;
int val;    // variable for servo position
volatile long int encoder_pos = 0; // DC motor position
int motor_speed; 
int pin1 = 6;   // motor pin1
int pin2 = 5;  // motor pin2
int encoderPinA = 2; 
int encoderPinB = 3; 
int error = 10; // permissible error

//default mode
int defaultMode = 0;
int destFloor = 1;
int currentFloor = 1;
int setPos = 0;   // Max value is 65535
int servoPos = 45;

//keyboard
char incomingByte;
char k;
int KB_DataAvailable=A5;//Data ready pin from Keyboard
int KB_A=10;
int KB_B=11;
int KB_C=12;
int KB_D=13;

//timer
int period = 10000;
long oldtime = 0;
char pressFlag = 'n';
unsigned long duration;

// led pins
int LED1 = A0;
int LED2 = A1;
int LED3 = A2;

// sensor 
int opto = A3;
int optoState = 0;

// Declare keypad layout
char keys[] = {'1','2','3','F',
               0,0,0,'E',
               0,0,0,0,
               'A',0,'B','C'};

// setup
void setup() {
  // initialise DC Encoder
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoder, RISING); // Enable the external interrupt

  //initialise Servo
  myServo.attach(7);
  myServo.write(45);
  
  //initialize pins for keypad
  pinMode(KB_DataAvailable,INPUT);
  pinMode(KB_A,INPUT);
  pinMode(KB_B,INPUT);
  pinMode(KB_C,INPUT);
  pinMode(KB_D,INPUT);
  Serial.begin(9600);          //  setup serial

  // leds setup
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  // opto 
  pinMode(opto, INPUT);

}

/////////////  Functions /////////////////

// DC encoders
void encoder(){
  if(digitalRead(encoderPinB) == HIGH){
    encoder_pos++;
  }else{
    encoder_pos--;
  }
}
void MotorClockwise(int power){
  if(power >60){
  analogWrite(pin1, power);
  digitalWrite(pin2, LOW);
  }else{
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
}
void MotorCounterClockwise(int power){
  if(power > 60){
  analogWrite(pin2, power);
  digitalWrite(pin1, LOW);
  }else{
    digitalWrite(pin2, LOW);
    digitalWrite(pin1, LOW);
  }
}

// Keyboard 
char KB_Read() {
  int ka,kb,kc,kd,i;
  char k;
  ka=digitalRead(KB_A); //read encoder output A
  kb=digitalRead(KB_B); //read encoder output B
  kc=digitalRead(KB_C); //read encoder output C
  kd=digitalRead(KB_D); //read encoder output D
  k=ka+kb*2+kc*4+kd*8; // combine the encoder outputs
  // Serial.println(keys[k]);
  if (k==7) {
    pressFlag = 'y';
  }
  return k;
}

void closeDoor(){
 optoState = digitalRead(opto);
  if (optoState==LOW){
    myServo.write(45);
    servoPos = 45;
    delay(50);
  }
  else{
    k=7;
  }
}

void openDoor() {
    myServo.write(135);
    servoPos = 135; 
    delay(50);
    if (pressFlag == 'y') {
      oldtime = 0;
      duration = 0;
      pressFlag = 'n';
    }
    if (oldtime == 0) {
      oldtime = millis();
    }
    duration =  millis();
    if ((duration - oldtime) > period) {
      Serial.println("duration");
      Serial.println(duration);
      k=3;
      duration = 0;
      oldtime = 0;
    }
}

void LEDon() {
  if (k==0) {
      digitalWrite(LED1, LOW); 
    }
  if (k==1) {
      digitalWrite(LED2, LOW); 
    }
  if (k==2) {
      digitalWrite(LED3, LOW); 
    }
}

void LEDoff() {
      digitalWrite(LED1, HIGH); 
      digitalWrite(LED2, HIGH); 
      digitalWrite(LED3, HIGH); 
}

void liftMotor() {
    if (abs(setPos - encoder_pos)> error && (setPos > encoder_pos)) {
    motor_speed = 100; // set the speed of the motor
   } 
  else if (abs(setPos - encoder_pos)> error && (setPos < encoder_pos)){
    motor_speed = -100;
   }
  else {
    motor_speed = 0;
   }
   
  if(motor_speed > 0){
    MotorClockwise(motor_speed);
   }
  else {
    MotorCounterClockwise(abs(motor_speed));
   }
}

void current_Floor() {
   if ((encoder_pos >= 0 - error) && (encoder_pos <= 0 + error)) {
      currentFloor = 1;
    }
  if ((encoder_pos >= 405 - error) && (encoder_pos <= 405 + error)) {
      currentFloor = 2;
    }

Gine Hann MAE, [3/8/2021 12:55 PM]
if ((encoder_pos >= 810 - error) && (encoder_pos <= 810 + error)) {
      currentFloor = 3;
    }
}

// destination floor
void dest_Floor() {
  if ((k==0) || (k==12)) {
    destFloor = 1;
   }
  if ((k==1) || (k==14)) {
    destFloor = 2;
   }
  if ((k==2) || (k==15)) {
    destFloor = 3;
   }
}

void goToFloor() {
  if ((k==0) || (k==12)) {
    setPos = 0;
   }
  if ((k==1) || (k==14)) {
    setPos = 405;

}
  if ((k==2) || (k==15)) {
    setPos = 810;
   }
}

/////////////// main working codes ////////////////
void loop() {
  liftMotor();
  current_Floor();
  dest_Floor();
  LEDoff();

  if(digitalRead(KB_DataAvailable)) {
    k = KB_Read(); //read the keypad
    delay(20);
   }
  
  if (k==7){
    openDoor();
  }
  
  if (k==3){ 
    closeDoor();
  }

// if default mood = 0, 1st floor, close door. 

  if (defaultMode == 0) {
    int setPos = 0;   
    int servoPos = 45;
    if ( (k==7) || (k==12) || (k==14) || (k==15) ) {
      defaultMode = 1;
    }
  } 

// else, run normally 

  else {
    //outside lift, k= a,b,c
    if ((k==12) || (k==14) || (k==15)) {

      // called
      if (destFloor != currentFloor) {
        Serial.println("out called");

        // if door is opened
        if (servoPos== 135) {
          closeDoor(); 
        }

        // if door is closed
        if (servoPos== 45) {
          goToFloor();
        }
      }

      // reached
      if (destFloor == currentFloor) {
        openDoor(); 
        Serial.println("out reached");
      }
    }

    //inside lift, k= 0,1,2
    if ((k==0) || (k==1) || (k==2)) {

      // called
      if (destFloor != currentFloor) {
        LEDon();
        Serial.println("in called");
        
        // if door is opened
        if (servoPos== 135) {
          closeDoor(); 
        }
        
        // if door is closed
        if (servoPos== 45) {
          goToFloor();
        }
      }

      // reached
      if (destFloor == currentFloor) {
        Serial.println("in reached");
        openDoor(); 
      }
    }
  }
  
//    Serial.println(k);
//    Serial.println("k:" + String(k) + " " + "setPos:" + String(setPos) + " " + "encoder_pos:" + String(encoder_pos) + " " + "destFloor:" + String(destFloor) + " " + "currentFloor:" + String(currentFloor) + " " + "servoPos:" + String(servoPos));
    delay(10);

}
