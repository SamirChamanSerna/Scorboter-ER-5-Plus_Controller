
#include <PID_v1.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>


#define m1Dir (8)
#define m1PWM (9)
#define m2Dir (12)
#define m2PWM (13)

Encoder enc1(2, 3);
Encoder enc2(20, 21);

// Variables
double target = 0.0;
double target2 = -target;
double Input, Output, Setpoint;
double Input2, Output2, Setpoint2;
double position;
double position2;





//Specify the links and initial tuning parameters
PID myPID1(&Input, &Output, &Setpoint, 26,0,0, DIRECT);
//Specify the links and initial tuning parameters
PID myPID2(&Input2, &Output2, &Setpoint2, 25,0,0, DIRECT);



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


//Motor
  pinMode(m1Dir, OUTPUT);
  pinMode(m1PWM, OUTPUT);

  //PID
  Input = 0;
  Setpoint = 0;

  myPID1.SetOutputLimits(0, 255);
  myPID2.SetOutputLimits(0, 255);

  //turn the PID on
  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);

}


void loop() {
  serialLoop();
  
  //angulo
  double pe1 = (enc1.read() / 25.6);
  double pe2 = (enc2.read() / 25.6);

  position = (pe1);
  position2 = (pe2);

  Input = position;
  Setpoint = target;
  myPID1.Compute();
  Input2 = position2;
  target2 = -target;
  Setpoint2 = target2;
   myPID2.Compute();

  if (position > target) {
      myPID1.SetControllerDirection(REVERSE);
      digitalWrite(m1Dir, LOW);
      analogWrite(m1PWM, Output);
  }

  if (position < target) {
      myPID1.SetControllerDirection(DIRECT);
      digitalWrite(m1Dir, HIGH);
      analogWrite(m1PWM, Output);
  }

  if (position2 > target2) {
      myPID2.SetControllerDirection(REVERSE);
      digitalWrite(m2Dir, LOW);
      analogWrite(m2PWM, Output2);
  }

  if (position2 < target2) {
      myPID2.SetControllerDirection(DIRECT);
      digitalWrite(m2Dir, HIGH);
      analogWrite(m2PWM, Output2);
  }

  // display the angle and the angular velocity to the terminal
  Serial.print(position);
  Serial.print("\t");
  Serial.print(-position2);
  Serial.print("\t");
  Serial.println(target);
}