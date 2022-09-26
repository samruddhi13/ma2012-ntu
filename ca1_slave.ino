// assessment slave 

#include <SoftwareSerial.h>
#define RxD 13
#define TxD 12

SoftwareSerial slave(RxD, TxD);

// define LCD, piezobuzzer, pushbutton, 

void setup()
{
  Serial.begin(9600);                                //start serial monitor, for communicating with the PC

  pinMode(RxD, INPUT);                               //set mode of receive pin
  pinMode(TxD, OUTPUT);                              //set mode of transmit pin
  slave.begin(9600);                                 //start the serial "port"

  // initialise LCD, piezobuzzer, pushbutton, 
  //                                   
}

void loop()
{
  // to receive currentFloor from master, then display in LCD
  // to receive key '9' (emergency) from master, then sound piezo buzzer & display HELP in LCD
  // to send pushbutton input to master
}
