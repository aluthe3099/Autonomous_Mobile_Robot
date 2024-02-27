// PID motor position control.
// Thanks to Brett Beauregard for his nice PID library http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

#include <PinChangeInt.h>
#include <Wire.h>
#include <PID_v1.h>
#define encodPinA1      2                       // Quadrature encoder A pin
#define encodPinB1      8                       // Quadrature encoder B pin
#define M1              9                       // PWM outputs to L298N H-Bridge motor driver module
#define M2              10

#define address 0x80
SoftwareSerial serial(13,12);
RoboClaw rc(&serial, 10000);

double kp =1, ki =20 , kd =0;             // modify for optimal performance
double input1 = 0, input2 = 0, output1 = 0, output2 = 0, setpoint1 = 0, setpoint2 = 0;
unsigned long lastTime,now;
volatile long encoderPos1 = 0,last_pos1=0;
volatile long encoderPos2 = 0,last_pos2=0;
PID myPID1(&input1, &output2, &setpoint1, kp, ki, kd,DIRECT);  
PID myPID2(&input2, &output2, &setpoint2, kp, ki, kd,DIRECT);  
void setup() {
  rc.begin(38400);
  
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetSampleTime(1);
  myPID1.SetOutputLimits(0, 127);

}

void loop() {
  uint8_t status1, status2;
  bool valid1, valid2;

  now = millis();
  int timeChange = (now - lastTime);
  if(timeChange>=500)
  {
    encoderPos1 = roboclaw.readEncM1(address, &status1, &valid1)
    encoderPos2 = roboclaw.readEncM2(address, &status2, &valid2)
    input1 = (360.0*1000*(encoderPos1-last_pos1)) /(1856.0*(now - lastTime));
    input2 = (360.0*1000*(encoderPos2-last_pos2)) /(1856.0*(now - lastTime));

    lastTime=now;
    last_pos1=encoderPos1;
    last_pos2=encoderPos2;
  }
  
  myPID1.Compute();                                    // calculate new output
  myPID2.Compute();
  pwmOut(output1, output2);                                     // drive L298N H-Bridge module
  delay(10);
}

void pwmOut(int out1, int out2) {                                // to H-Bridge board
  rc.ForwardBackwardM1(address, out1); // forward
  rc.ForwardBackwardM2(address, out2);
}
