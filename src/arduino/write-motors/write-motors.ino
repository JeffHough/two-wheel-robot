#include "SerialTransfer.h"

// Serial transfer object:
SerialTransfer myTransfer;

// The maximum pwm value:
constexpr int kMaxPWM = 255;

// Set up the enable and PWM pins:
constexpr int kForwardEnableA = 2;
constexpr int kBackwardEnableA = 4;
constexpr int kPWMA = 3;

constexpr int kForwardEnableB = 7;
constexpr int kBackwardEnableB = 8;
constexpr int kPWMB = 5;

// Declare the motor struct:
struct Motor{
  // The pins:
  int forward_enable;
  int backward_enable;
  int pwm;

  // Store the current desired speed:
  double spd;
};

Motor motors[2];

void setup() {
  Motor motor_A = {kForwardEnableA, kBackwardEnableA, kPWMA, 0.0};
  Motor motor_B = {kForwardEnableB, kBackwardEnableB, kPWMB, 0.0};
  // Put the motors into the array:
  motors[0] = motor_A;
  motors[1] = motor_B;

  // put your setup code here, to run once:
  Serial.begin(115200);
  myTransfer.begin(Serial);

  for (const auto& motor : motors)
  {
    pinMode(motor.forward_enable, OUTPUT);
    pinMode(motor.backward_enable, OUTPUT);
    pinMode(motor.pwm, OUTPUT);
  }
}

// Here, we will continuously recieve JSON formatted inputs in the form of:
//{motor_A: double spd, motor_B: double spd};

void loop() {
  // First, write to each motor its current desired speed:
  for (const auto& motor : motors)
  {
    if (motor.spd >= 0.0)
    {
      digitalWrite(motor.forward_enable, HIGH);
      digitalWrite(motor.backward_enable, LOW);
    }
    else
    {
      digitalWrite(motor.forward_enable, LOW);
      digitalWrite(motor.backward_enable, HIGH);
    }
    analogWrite(motor.pwm, abs(static_cast<int>(motor.spd * kMaxPWM)));
  }

  // If serial is available, update the values:
  if(myTransfer.available())
  {
    double desired_motor_speeds[2];
    myTransfer.rxObj(desired_motor_speeds);
    motors[0].spd = desired_motor_speeds[0];
    motors[1].spd = desired_motor_speeds[1];
  
  }

}
