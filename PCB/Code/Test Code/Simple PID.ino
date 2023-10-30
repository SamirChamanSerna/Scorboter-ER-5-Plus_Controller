#include <SimpleFOC.h>
#include <PID_v1.h>


#define PWMf (9)
#define PWMb (8)

// Variables
double target = 0.0;
double Input, Output, Setpoint;
double position;



Encoder encoder = Encoder(4, 5, 1680);
// interrupt routine initialization
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, 2,0,0, DIRECT);



// Espesificar Target
void serialLoop(){
  static String received_chars;

    while (Serial.available()){
      char inChar = (char) Serial.read();
      received_chars += inChar;
      if (inChar == '\n')
      {
        target = received_chars.toDouble();
        Serial.print("Target = "); Serial.println(target);
        received_chars = "";
      }
    }

}


void setup() {
  // monitoring port
  Serial.begin(115200);

 //Encoder
  // enable/disable quadrature mode
  encoder.quadrature = Quadrature::OFF;
  // initialize encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
//Encoder

//Motor
  pinMode(PWMf, OUTPUT);
  pinMode(PWMb, OUTPUT);

  //PID
  Input = 0;
  Setpoint = 0;

  myPID.SetOutputLimits(0, 255);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

}


void loop() {
  serialLoop();

  // update the sensor values 
  encoder.update();

  position = (encoder.getAngle());

  Input = position;
  Setpoint = target;
  myPID.Compute();

  if (position < target) {
      myPID.SetControllerDirection(DIRECT);
      digitalWrite(PWMb, LOW);
      analogWrite(PWMf, Output * 255);
  }

  if (position > target) {
      myPID.SetControllerDirection(REVERSE);
      digitalWrite(PWMf, LOW);
      analogWrite(PWMb, Output * 255);
  }

  // display the angle and the angular velocity to the terminal
  Serial.print(position);
  Serial.print("\t");
  Serial.println(target);
}