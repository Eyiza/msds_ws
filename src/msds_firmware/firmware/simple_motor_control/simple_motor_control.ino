// === Front Left Motor ===
#define FL_IN1 12
#define FL_IN2 11
#define FL_EN  13

// === Front Right Motor ===
#define FR_IN1 10
#define FR_IN2 9
#define FR_EN  8

// === Rear Left Motor ===
#define RL_IN1 5
#define RL_IN2 6
#define RL_EN  7

// === Rear Right Motor ===
#define RR_IN1 22
#define RR_IN2 23
#define RR_EN  4

// === Motor speed ===
int motorSpeed = 150; // value between 0-255

void setup() {
  // Set all motor pins as outputs
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_EN, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_EN, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT); pinMode(RL_EN, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT); pinMode(RR_EN, OUTPUT);

  // === Move Forward ===
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);

  analogWrite(FL_EN, motorSpeed);
  analogWrite(FR_EN, motorSpeed);
  analogWrite(RL_EN, motorSpeed);
  analogWrite(RR_EN, motorSpeed);
}

void loop() {
  // Robot keeps moving forward
}
