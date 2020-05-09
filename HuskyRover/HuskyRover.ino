#include <PID_v1.h>
#include "HUSKYLENS.h"


#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define SerialUSB SERIAL_PORT_USBVIRTUAL
#endif

HUSKYLENS huskylens;

#include <Servo.h> //servo library

Servo myservoa, myservob; // create servo objects to control servos

#define MotorPin 20
#define SteeringPin 21
#define RC1_pin 3
#define RC2_pin 4
#define RC3_pin 5
#define SwitchPin 10

// PID Details
double Setpoint, Input, Output;
double Kp=2, Ki=0, Kd=3;
PID HuskyPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


const int BAUD_RATE = 19200;
double slope;
int RC;
float x_1,x_2,y_1,y_2;
unsigned long RC1_value;
unsigned long RC2_value;
unsigned long RC3_value;
boolean RC_throttle = true;  // if you want to control the throttle manually when it's under OpenMV control

int steer, motor;

unsigned long time;
unsigned long lasttime = 0;
bool LEDState = LOW;

void setup() {
    Serial.begin(BAUD_RATE);   // USB port
    Serial2.begin(115200);
      //turn the PID on
    Setpoint = 0;
    HuskyPID.SetMode(AUTOMATIC);
    HuskyPID.SetSampleTime(20); // Do it at 50Hz 
    while (!huskylens.begin(Serial2))
    {
        SerialUSB.println(F("Begin failed!"));
        SerialUSB.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        SerialUSB.println(F("2.Please recheck the connection."));
        delay(100);
    }
    pinMode(RC1_pin, INPUT);
    pinMode(RC2_pin, INPUT);
    pinMode(RC3_pin, INPUT);
    myservoa.attach(SteeringPin);// attach servo on Output 1 to servo object
    myservob.attach(MotorPin);// attach servo on Output 2 to servo object
    pinMode(LED_BUILTIN, OUTPUT);  // enable LED 
}

void RCcontrol() {
  Serial.println(RC3_value);  // print switch state for debugging
  RC1_value = pulseIn(RC1_pin, HIGH);  // read rc inputs
  RC2_value = pulseIn(RC2_pin, HIGH);
  myservoa.write(RC1_value); // mirror values to output
  myservob.write(RC2_value);
  delay (20);
  
}

void Huskycontrol() {
      if (!huskylens.request()) SerialUSB.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
      else if(!huskylens.isLearned()) SerialUSB.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
      else if(!huskylens.available()) SerialUSB.println(F("No block or arrow appears on the screen!"));
      else {
        if (huskylens.available())
        {
            HUSKYLENSResult result = huskylens.read();
            if (result.command == COMMAND_RETURN_ARROW){
              x_1 = result.xOrigin;
              y_1 = result.yOrigin;
              x_2 = result.xTarget;
              y_2 = result.yTarget;
              slope = (y_2-y_1)/(x_2-x_1);
              Input = slope;
              HuskyPID.Compute();
              steer = Output + 1500;
//              Serial.print("Setpoint: ");
//              Serial.print(Setpoint);
              Serial.print("Input: ");
              Serial.print(Input);
//              Serial.print("Steer: ");
//              Serial.println(steer);
              
              motor=1500;
              if (RC_throttle) {
//                motor = pulseIn(RC2_pin, HIGH);
                }
              }
            else{
              SerialUSB.println("Object unknown!");
              }
            steer = constrain(steer,1300,1700);
            motor = constrain(motor,1000,2000);
            Serial.print(" Steer: ");
            Serial.println(steer);
//            Serial.print(" Motor: ");
//            Serial.println(motor);
            myservoa.write(steer); // send values to output
            myservob.write(motor);
        }
      }
}  

void loop() {
  time = millis();
  if (time > lasttime + 1000) {   // flash the LED every second to show "resting" mode
      lasttime = time;
      LEDState = !LEDState; // reverse the LED state
      digitalWrite(LED_BUILTIN, LEDState);   // turn on or off the LED
       }
//  RC3_value = pulseIn(RC3_pin, HIGH);
//  if (RC3_value > 1500) {RCcontrol();}   // Use the CH5 switch to decide whether to pass through RC commands or take OpenMV commands
//    else {Huskycontrol();}
  Huskycontrol();
}
