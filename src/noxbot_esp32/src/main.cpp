#include <Arduino.h>

/* ================= PIN DEFINITIONS ================= */

// Front Left
#define FL_IN2 25
#define FL_IN1 26
#define FL_PWM 27

// Rear Left
#define RL_IN2 14
#define RL_IN1 13
#define RL_PWM 12

// Front Right
#define FR_IN2 33
#define FR_IN1 18
#define FR_PWM 23

// Rear Rightt
#define RR_IN2 21
#define RR_IN1 22
#define RR_PWM 19

// Standby (shared)
#define STBY 32

/* ================= PWM CONFIG ================= */

#define PWM_FREQ 20000
#define PWM_RES  8

#define FL_CH 0
#define RL_CH 1
#define FR_CH 2
#define RR_CH 3

/* ================= WATCHDOG ================= */

const unsigned long timeout_ms = 300;
unsigned long last_cmd_time = 0;

/* ================= STATE ================= */

int left_pwm  = 0;
int right_pwm = 0;

/* ================= FUNCTION DECLARATIONS ================= */

void parseCommand(const String &cmd);
void driveMotor(int in1, int in2, int channel, int pwm);
void stopMotors();

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200);
  delay(100); // allow USB CDC to come up
  Serial.println("Noxbot ESP32 ready (115200 baud)");

  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(RL_IN1, OUTPUT);
  pinMode(RL_IN2, OUTPUT);

  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  // PWM setup
  ledcSetup(FL_CH, PWM_FREQ, PWM_RES);
  ledcSetup(RL_CH, PWM_FREQ, PWM_RES);
  ledcSetup(FR_CH, PWM_FREQ, PWM_RES);
  ledcSetup(RR_CH, PWM_FREQ, PWM_RES);

  ledcAttachPin(FL_PWM, FL_CH);
  ledcAttachPin(RL_PWM, RL_CH);
  ledcAttachPin(FR_PWM, FR_CH);
  ledcAttachPin(RR_PWM, RR_CH);

  stopMotors();
}

/* ================= LOOP ================= */

void loop() 
{
  // Read serial data line by line (non-blocking)
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    
    if (line.length() > 0) {
      parseCommand(line);
      last_cmd_time = millis();
      
      // Optional: Echo for debugging (comment out for production)
      // Serial.print("RX -> ");
      // Serial.println(line);
    }
  }

  // Safety watchdog
  if (millis() - last_cmd_time > timeout_ms) {
    stopMotors();
  }
}

/* ================= PARSER ================= */

void parseCommand(const String &cmd) {
  // Remove debug echo that interferes with serial communication
  // Serial.println(cmd);  
  
  int l_idx = cmd.indexOf("L:");
  int r_idx = cmd.indexOf("R:");

  if (l_idx == -1 || r_idx == -1) return;

  left_pwm  = cmd.substring(l_idx + 2, r_idx).toInt();
  right_pwm = cmd.substring(r_idx + 2).toInt();

  // LEFT SIDE (same value for now)
  driveMotor(FL_IN1, FL_IN2, FL_CH, left_pwm);
  driveMotor(RL_IN1, RL_IN2, RL_CH, left_pwm);

  // RIGHT SIDE (same value for now)
  driveMotor(FR_IN1, FR_IN2, FR_CH, right_pwm);
  driveMotor(RR_IN1, RR_IN2, RR_CH, right_pwm);
}

/* ================= MOTOR DRIVER ================= */

void driveMotor(int in1, int in2, int channel, int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(channel, pwm);
  }
  else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(channel, -pwm);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(channel, 0);
  }
}

/* ================= EMERGENCY STOP ================= */

void stopMotors() {
  ledcWrite(FL_CH, 0);
  ledcWrite(RL_CH, 0);
  ledcWrite(FR_CH, 0);
  ledcWrite(RR_CH, 0);

  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  digitalWrite(RL_IN1, LOW);
  digitalWrite(RL_IN2, LOW);
  digitalWrite(FR_IN1, LOW);
  digitalWrite(FR_IN2, LOW);
  digitalWrite(RR_IN1, LOW);
  digitalWrite(RR_IN2, LOW);
}
