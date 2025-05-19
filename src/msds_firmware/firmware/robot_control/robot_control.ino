#include <PID_v1.h> // To convert velocity to PWM
#include <EnableInterrupt.h> // To use interrupts for encoders
 
// === Front Left Motor ===
#define FL_IN1 12
#define FL_IN2 11
#define FL_EN  13
// Encoder Interrupt Pins
#define FL_ENC_A 15
#define FL_ENC_B 14

// === Front Right Motor ===
#define FR_IN1 10
#define FR_IN2 9
#define FR_EN  8
// Encoder Interrupt Pins
#define FR_ENC_A 3
#define FR_ENC_B 2

// === Rear Left Motor ===
#define RL_IN1 5
#define RL_IN2 6
#define RL_EN  7
// Encoder Interrupt Pins
#define RL_ENC_A 20
#define RL_ENC_B 21

// === Rear Right Motor ===
#define RR_IN1 22
#define RR_IN2 23
#define RR_EN  4
// Encoder Interrupt Pins
#define RR_ENC_A 18
#define RR_ENC_B 19

// === Encoder Counters ===
unsigned int countFL = 0;
unsigned int countFR = 0;
unsigned int countRL = 0;
unsigned int countRR = 0;

String sign_FL = "p";  // 'p' = positive, 'n' = negative
String sign_FR = "p";  
String sign_RL = "p";
String sign_RR = "p";

// PID
// Setpoint - Desired
double cmd_vel_FL = 0.0;     // rad/s
double cmd_vel_FR = 0.0;     // rad/s
double cmd_vel_RL = 0.0;     // rad/s
double cmd_vel_RR = 0.0;     // rad/s

// Input - Measurement
double meas_vel_FL = 0.0;     // rad/s
double meas_vel_FR = 0.0;     // rad/s
double meas_vel_RL = 0.0;     // rad/s
double meas_vel_RR = 0.0;     // rad/s

// Output - PWM Command
double pwm_FL = 0.0;          // 0-255
double pwm_FR = 0.0;          // 0-255
double pwm_RL = 0.0;          // 0-255
double pwm_RR = 0.0;          // 0-255

// PID Tuning
double Kp_FR = 30.0;
double Ki_FR = 10.0;
double Kd_FR = 0.2;

// double Kp_FL = 34;
// double Ki_FL = 12;
// double Kd_FL = 0.2;
double Kp_FL = 30.0;
double Ki_FL = 10.0;
double Kd_FL = 0.2;

double Kp_RR = 30.0;
double Ki_RR = 10.0;
double Kd_RR = 0.2;

double Kp_RL = 30.0;
double Ki_RL = 10.0;
double Kd_RL = 0.2;

// PID Controllers which ensure actual speed reaches and stays at the desired speed
// The PID constructor takes the estimated velocity, the PWM output, the target velocity, and the PID tuning parameters
// The last parameter is the PID mode (DIRECT or REVERSE)
// In this case, we want to use DIRECT mode, as we want to increase the PWM output when the error is positive
PID pidFL(&meas_vel_FL, &pwm_FL, &cmd_vel_FL, Kp_FL, Ki_FL, Kd_FL, DIRECT);
PID pidFR(&meas_vel_FR, &pwm_FR, &cmd_vel_FR, Kp_FR, Ki_FR, Kd_FR, DIRECT);
PID pidRL(&meas_vel_RL, &pwm_RL, &cmd_vel_RL, Kp_RL, Ki_RL, Kd_RL, DIRECT);
PID pidRR(&meas_vel_RR, &pwm_RR, &cmd_vel_RR, Kp_RR, Ki_RR, Kd_RR, DIRECT);

// === Timing ===
unsigned long last_update = 0; 
const unsigned long interval = 100; // 100 millisecond

void setup() {
  // Set motor pins as outputs
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_EN, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_EN, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT); pinMode(RL_EN, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT); pinMode(RR_EN, OUTPUT);

  // Set Motor Rotation Direction
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW);
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW);

  // Start PIDs
  pidFL.SetMode(AUTOMATIC);
  pidFR.SetMode(AUTOMATIC);
  pidRL.SetMode(AUTOMATIC);
  pidRR.SetMode(AUTOMATIC);

  // Begin Serial Communication
  Serial.begin(500000); 
  Serial2.begin(500000); 

  // Set encoder pins as inputs
  pinMode(FL_ENC_B, INPUT);
  pinMode(FR_ENC_B, INPUT);
  pinMode(RL_ENC_B, INPUT);
  pinMode(RR_ENC_B, INPUT);

  // Set up interrupts and callbacks for encoders
  enableInterrupt(FL_ENC_A, frontLeftEncoderCallback, RISING);
  enableInterrupt(FR_ENC_A, frontRightEncoderCallback, RISING);
  enableInterrupt(RL_ENC_A, rearLeftEncoderCallback, RISING);
  enableInterrupt(RR_ENC_A, rearRightEncoderCallback, RISING);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial2.available())
  {
    String cmd = Serial2.readStringUntil('\n'); // Format: "flp0.23,frn0.21,rrp0.25,rln0.22,"
    cmd.trim();

    // Serial.println("CMD RECEIVED: " + cmd);
    parseWheelCommand(cmd, "fl", &cmd_vel_FR, FR_IN1, FR_IN2);
    parseWheelCommand(cmd, "fr", &cmd_vel_FL, FL_IN1, FL_IN2);
    parseWheelCommand(cmd, "rr", &cmd_vel_RR, RR_IN1, RR_IN2);
    parseWheelCommand(cmd, "rl", &cmd_vel_RL, RL_IN1, RL_IN2);
  }

  // Encoder
  unsigned long now = millis();
  if (now - last_update >= interval) // if 100ms has passed
  {
    // Convert encoder count to velocity
    double conversion = (10 * (60.0 / 616.0)) * 0.10472; // 616 counts per revolution, 0.10472 rad/s
    // The conversion factor is based on the wheel's characteristics and the encoder's resolution
    // The formula is: (counts / counts_per_revolution) * (2 * pi / time_interval)
    // The 10 is because we are measuring in 100ms intervals, and the 60 is to convert to minutes
    // The 0.10472 is the conversion from rpm to rad/s. 1 rpm = 0.10472 rad/s
    // The 385.0 is the number of counts per revolution for the encoder
    meas_vel_FL = (countFL * conversion); 
    meas_vel_FR = (countFR * conversion); 
    meas_vel_RL = (countRL * conversion); 
    meas_vel_RR = (countRR * conversion);
    
    // Compute PID for each wheel
    // This will calculate the PWM value based on the error between the desired and measured velocity
    // Then update the PWM output
    pidFL.Compute();
    pidFR.Compute();
    pidRL.Compute();
    pidRR.Compute();

    // Clamp PWM to safe bounds
    // pwm_FL = constrain(pwm_FL, 0, 255);
    // pwm_FR = constrain(pwm_FR, 0, 255);
    // pwm_RL = constrain(pwm_RL, 0, 255);
    // pwm_RR = constrain(pwm_RR, 0, 255);

    // Ignore commands smaller than inertia to avoid jitter
    if (cmd_vel_FL == 0.0) pwm_FL = 0.0;
    if (cmd_vel_FR == 0.0) pwm_FR = 0.0;
    if (cmd_vel_RL == 0.0) pwm_RL = 0.0;
    if (cmd_vel_RR == 0.0) pwm_RR = 0.0;

    // Send encoder data to serial
    String encoder_read = "fl" + sign_FL + String(meas_vel_FL) + "," +
                          "fr" + sign_FR + String(meas_vel_FR) + "," +
                          "rr" + sign_RR + String(meas_vel_RR) + "," +
                          "rl" + sign_RL + String(meas_vel_RL);
      
    Serial2.println(encoder_read);
    // Serial.println(encoder_read);
    // Serial.print(cmd_vel_RL);   // Target
    // Serial.print(",");
    // Serial.println(meas_vel_RL); // Actual


    last_update = now; // Update the last update time

    // Reset counter
    countFL = 0;  
    countFR = 0;
    countRL = 0;
    countRR = 0;

    // Set the motor speed
    analogWrite(FL_EN, pwm_FL);
    analogWrite(FR_EN, pwm_FR);
    analogWrite(RL_EN, pwm_RL);
    analogWrite(RR_EN, pwm_RR);
  }
}

void parseWheelCommand(String cmd, String label, double* cmd_vel, int in1, int in2) {
  // Format of cmd - the full string message: "flp0.23,frn0.21,rrp0.25,rln0.22,"
  // label is the wheel identifier, e.g., "fr" or "rl"
  // cmd_vel is the pointer to the double variable that stores the velocity (e.g., &cmd_vel_FR)
  // in1 / in2 are the motor direction control pins (used to set forward/reverse)

  int start = cmd.indexOf(label); // finds the index where the substring label (e.g., rl) appears in the cmd string.
  if (start == -1) return; // If it doesn’t exist, skip processing for this wheel.

  char direction = cmd.charAt(start + 2); // finds the 3rd character after the label e.g 'p' or 'n'
  bool forward = (direction == 'p');

  int value_start = start + 3; // velocity value starts from the 4th character
  int value_end = cmd.indexOf(',', value_start); // where the comma ends this velocity.
  if (value_end == -1) value_end = cmd.length(); // just in case

  // Get the numeric string and convert
  String val_str = cmd.substring(value_start, value_end);
  *cmd_vel = val_str.toFloat();

  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);

  // Serial.print(label);
  // Serial.print(": ");
  // Serial.print(forward ? "+" : "-");
  // Serial.println(*cmd_vel);
}

// New pulse from Front Left Wheel Encoder
void frontLeftEncoderCallback()
{
  if(digitalRead(FL_ENC_B) == HIGH)
  {
    sign_FL = "n";
  }
  else
  {
    sign_FL = "p";
  }
  countFL++;
}
// New pulse from Front Right Wheel Encoder
void frontRightEncoderCallback()
{
  if(digitalRead(FR_ENC_B) == HIGH)
  {
    sign_FR = "p";
  }
  else
  {
    sign_FR = "n";
  }
  countFR++;
}
// // New pulse from Rear Left Wheel Encoder
void rearLeftEncoderCallback()
{
  if(digitalRead(RL_ENC_B) == HIGH)
  {
    sign_RL = "n";
  }
  else
  {
    sign_RL = "p";
  }
  countRL++;
}
// // New pulse from Rear Right Wheel Encoder
void rearRightEncoderCallback()
{
  if(digitalRead(RR_ENC_B) == HIGH)
  {
    sign_RR = "p";
  }
  else
  {
    sign_RR = "n";
  }
  countRR++;
}
