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

// Interpret Serial Messages
bool is_front_left_wheel_cmd = false; // true if the command is for the front left wheel
bool is_front_right_wheel_cmd = false;
bool is_rear_left_wheel_cmd = false;
bool is_rear_right_wheel_cmd = false;

bool is_front_left_wheel_forward = true; // true if the front left wheel is moving forward
bool is_front_right_wheel_forward = true;
bool is_rear_left_wheel_forward = true;
bool is_rear_right_wheel_forward = true;

char value[] = "00.00"; // Buffer for command value - velocity to be sent to the motors
uint8_t value_idx = 0; // Index for command value
bool is_cmd_complete = false; // true if the command is complete

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
double pwm_FL = 0.0;             // 0-255
double pwm_FR = 0.0;             // 0-255
double pwm_RL = 0.0;             // 0-255
double pwm_RR = 0.0;             // 0-255

// PID Tuning
double Kp_FL = 11.5;
double Ki_FL = 7.5;
double Kd_FL = 0.1;

double Kp_FR = 12.8;
double Ki_FR = 8.3;
double Kd_FR = 0.1;

double Kp_RL = 11.5;
double Ki_RL = 7.5;
double Kd_RL = 0.1;

double Kp_RR = 12.8;
double Ki_RR = 8.3;
double Kd_RR = 0.1;

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
  Serial.begin(115200); 

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

  // Alternatively, you can use attachInterrupt
  // attachInterrupt(digitalPinToInterrupt(FL_ENC_A), frontLeftEncoderCallback, RISING);
  // attachInterrupt(digitalPinToInterrupt(FR_ENC_A), frontRightEncoderCallback, RISING);
  // attachInterrupt(digitalPinToInterrupt(RL_ENC_A), rearLeftEncoderCallback, RISING);
  // attachInterrupt(digitalPinToInterrupt(RR_ENC_A), rearRightEncoderCallback, RISING);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial.available())
  {
    // Read message character by character
    char chr = Serial.read(); // Format:  "frp0.23,fln0.21,rrp0.25,rln0.22,"

    // Front Left and Right Wheel
    if (chr == 'f' || chr == 'r') {
      char next = Serial.read();
      if (next == 'l') {
        is_front_left_wheel_cmd = (chr == 'f');
        is_front_right_wheel_cmd = false;
        is_rear_left_wheel_cmd = (chr == 'r');
        is_rear_right_wheel_cmd = false;
      } else if (next == 'r') {
        is_front_left_wheel_cmd = false;
        is_front_right_wheel_cmd = (chr == 'f');
        is_rear_left_wheel_cmd = false;
        is_rear_right_wheel_cmd = (chr == 'r');
        is_cmd_complete = false; // Reset command completion status
      }
      value_idx = 0; // As we are reading a new command, reset the index
    } else if (chr == 'p' || chr == 'n') {
      bool forward = (chr == 'p');

      if (is_front_left_wheel_cmd) {
        is_front_left_wheel_forward = forward;
        // change the direction of the rotation
        digitalWrite(FL_IN1, forward ? HIGH : LOW);
        digitalWrite(FL_IN2, forward ? LOW : HIGH);
        sign_FL = forward ? "p" : "n";
      } else if (is_front_right_wheel_cmd) {
        is_front_right_wheel_forward = forward;
        // change the direction of the rotation
        digitalWrite(FR_IN1, forward ? HIGH : LOW);
        digitalWrite(FR_IN2, forward ? LOW : HIGH);
        sign_FR = forward ? "p" : "n";
      } else if (is_rear_left_wheel_cmd) {
        is_rear_left_wheel_forward = forward;
        // change the direction of the rotation
        digitalWrite(RL_IN1, forward ? HIGH : LOW);
        digitalWrite(RL_IN2, forward ? LOW : HIGH);
        sign_RL = forward ? "p" : "n";
      } else if (is_rear_right_wheel_cmd) {
        // change the direction of the rotation
        is_rear_right_wheel_forward = forward;
        digitalWrite(RR_IN1, forward ? HIGH : LOW);
        digitalWrite(RR_IN2, forward ? LOW : HIGH);
        sign_RR = forward ? "p" : "n";
      }
    } else if (chr == ',') { // Separation character which indicates end of a velocity command
      double cmd_val = atof(value); // Convert string to double

      if (is_front_left_wheel_cmd) cmd_vel_FL = cmd_val;
      else if (is_front_right_wheel_cmd) cmd_vel_FR = cmd_val;
      else if (is_rear_right_wheel_cmd) cmd_vel_RR = cmd_val;
      else if (is_rear_left_wheel_cmd) {
        cmd_vel_RL = cmd_val;
        is_cmd_complete = true;
      }
      // Reset for next velocity command
      value_idx = 0;
      memset(value, 0, sizeof(value)); 
      // value[0] = '0';
      // value[1] = '0';
      // value[2] = '.';
      // value[3] = '0';
      // value[4] = '0';
      // value[5] = '\0'; // Closing character of the array
    } else { // Set the velocity comand value
      if (value_idx < 5) { // Ensure we don't overflow the buffer
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Encoder
  unsigned long now = millis();
  if (now - last_update >= interval) // if 100ms has passed
  {
    // Convert encoder count to velocity
    double conversion = (10 * (60.0 / 385.0)) * 0.10472; // 10ms, 385 counts per revolution, 0.10472 rad/s
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
    String encoder_read = "fr" + sign_FR + String(meas_vel_FR) + "," +
                              "fl" + sign_FL + String(meas_vel_FL) + "," +
                              "rr" + sign_RR + String(meas_vel_RR) + "," +
                              "rl" + sign_RL + String(meas_vel_RL) + ",";
    Serial.println(encoder_read);

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
// New pulse from Rear Left Wheel Encoder
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
// New pulse from Rear Right Wheel Encoder
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