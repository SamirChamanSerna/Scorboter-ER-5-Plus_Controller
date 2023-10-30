#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "SimpleDCMotor.h"

// DCMotor object
DCMotor motor = DCMotor();
// DCDriver object
DCDriver2PWM driver = DCDriver2PWM(8, 9);


//Encoder
Encoder encoder = Encoder(4, 5, 1680);
// interrupt routine initialization
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}


// Commander object, used for serial control
Commander commander = Commander(Serial);
// motor control function - this is needed to link the incoming commands 
// to the motor object
void onMotor(char* cmd){ commander.motor(&motor, cmd); }


void setup() {
  // monitoring port
  Serial.begin(115200);


  while (!Serial) {};   // wait for serial connection
  // enable debug output to the serial port
  SimpleFOCDebug::enable();
  
  // enable/disable quadrature mode
  encoder.quadrature = Quadrature::OFF;

// basic driver setup - set power supply voltage
  driver.voltage_power_supply = 6.0f;
 
  driver.voltage_limit = 6.0f;
  // Optionally set the PWM frequency.
  driver.pwm_frequency = 5000;


  // init driver
  driver.init();
  // initialize encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  // link motor to driver
  motor.linkDriver(&driver);
  // link sensor to motor
  motor.linkSensor(&encoder);

  motor.sensor_direction = Direction::CCW;


  // set a voltage limit on the motor, optional. The value set here
  // has to be lower than the power supply voltage.
  motor.voltage_limit = 6.0f;
  motor.velocity_limit = 2.0f;

  // control type - for this example we use velocity mode.
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  // init motor
  motor.init();
  // set the PID parameters for velocity control.
  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 0.0f;
  motor.PID_velocity.D = 0.0f;
  // output ramp limits the rate of change of the velocity, e.g. limits the
  // accelleration.
  motor.PID_velocity.output_ramp = 200.0f;
  // low pass filter time constant. higher values smooth the velocity measured
  // by the sensor, at the cost of latency and control responsiveness.
  motor.LPF_velocity.Tf = 0.01f;
  // set the target velocity to 0, we use the commander to set it later
  motor.target = 0.0f;
  // enable motor
  motor.enable();
  // add the motor and its control function to the commander
  commander.add('M', onMotor, "dc motor");
  // enable monitoring on Serial port
  motor.useMonitoring(Serial);
  Serial.println("Initialization complete.");
}

void loop() {

  motor.move(); // target speed can be set via commander input

  // call commander.run() once per loop iteration, it will process incoming commands
  commander.run();

  // call motor.monitor() once per loop iteration, it will print the motor state
  motor.monitor();
}
