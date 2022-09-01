// Include the Arduino JSON library:
#include <ArduinoJson.h>

// Set up the Serial constants:
constexpr int kTimeOut = 10; // Timeout in milliseconds.

// The maximum pwm value:
constexpr int kMaxPWM = 255;

// Set up the enable and PWM pins:
constexpr int kForwardEnableA = 2;
constexpr int kBackwardEnableA = 4;
constexpr int kPWMA = 3;

constexpr int kForwardEnableB = 7;
constexpr int kBackwardEnableB = 8;
constexpr int kPWMB = 5;

// Decare the json income stream, along with its size:
StaticJsonDocument<1000> doc;

struct Motor{
  // The pins:
  int forward_enable;
  int backward_enable;
  int pwm;

  // Store the current desired speed:
  double spd;
};

Motor motors[2];
Motor motor_A;
Motor motor_B;

void setup() {
  motor_A = {kForwardEnableA, kBackwardEnableA, kPWMA, 0.0};
  motor_B = {kForwardEnableB, kBackwardEnableB, kPWMB, 0.0};
  // Put the motors into the array:
  motors[0] = motor_A;
  motors[1] = motor_B;
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(kTimeOut); // Set the timeout, in milliseconds for receiving a JSON msg:

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

  // Now, listen to the new desired speed:
  if (Serial.available())
  {
    String json = Serial.readString();  // read command from serial port

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, json.c_str());

    // Test if parsing succeeds.
    if (error) 
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    // Fetch values.
    //
    // Most of the time, you can rely on the implicit casts.
    // In other case, you can do doc["time"].as<long>();
    motors[0].spd = static_cast<double>(doc["motor_A"]);
    motors[1].spd = static_cast<double>(doc["motor_B"]);
    /*
    Serial.println("motor A speed is: ");
    Serial.println(static_cast<double>(doc["motor_A"]));
    Serial.println("motor B speed is: ");
    Serial.println(static_cast<double>(doc["motor_B"]));   
    */ 

    }
}
