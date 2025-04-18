#include <EnableInterrupt.h>

// Encoder tick counters
volatile long encoderTicks_FL = 0;
volatile long encoderTicks_FR = 0;
volatile long encoderTicks_RL = 0;
volatile long encoderTicks_RR = 0;

// === Encoder Pin Assignments ===
// Front Left
#define FL_ENC_A 15
#define FL_ENC_B 14

// Front Right
#define FR_ENC_A 3
#define FR_ENC_B 2

// Rear Left
#define RL_ENC_A 20
#define RL_ENC_B 21

// Rear Right
#define RR_ENC_A 18
#define RR_ENC_B 19

void setup() {
  Serial.begin(9600);

  // Set all encoder pins as inputs
  pinMode(FL_ENC_A, INPUT); pinMode(FL_ENC_B, INPUT);
  pinMode(FR_ENC_A, INPUT); pinMode(FR_ENC_B, INPUT);
  pinMode(RL_ENC_A, INPUT); pinMode(RL_ENC_B, INPUT);
  pinMode(RR_ENC_A, INPUT); pinMode(RR_ENC_B, INPUT);

  // Attach pin change interrupts using EnableInterrupt
  enableInterrupt(FL_ENC_A, updateFL, CHANGE);
  enableInterrupt(FR_ENC_A, updateFR, CHANGE);
  enableInterrupt(RL_ENC_A, updateRL, CHANGE);
  enableInterrupt(RR_ENC_A, updateRR, CHANGE);
}

void loop() {
  Serial.print("FL: "); Serial.print(encoderTicks_FL);
  Serial.print(" | FR: "); Serial.print(encoderTicks_FR);
  Serial.print(" | RL: "); Serial.print(encoderTicks_RL);
  Serial.print(" | RR: "); Serial.println(encoderTicks_RR);
  delay(2000);
}

// === ISR functions using EnableInterrupt ===
void updateFL() {
  bool A = digitalRead(FL_ENC_A);
  bool B = digitalRead(FL_ENC_B);
  encoderTicks_FL += (A == B) ? 1 : -1;
}

void updateFR() {
  bool A = digitalRead(FR_ENC_A);
  bool B = digitalRead(FR_ENC_B);
  encoderTicks_FR += (A == B) ? 1 : -1;
}

void updateRL() {
  bool A = digitalRead(RL_ENC_A);
  bool B = digitalRead(RL_ENC_B);
  encoderTicks_RL += (A == B) ? 1 : -1;
}

void updateRR() {
  bool A = digitalRead(RR_ENC_A);
  bool B = digitalRead(RR_ENC_B);
  encoderTicks_RR += (A == B) ? 1 : -1;
}
