// #include <EnableInterrupt.h>

// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_in2 13  // Dir Motor A
#define L298N_in1 12  // Dir Motor A

#define right_encoder_phaseA 18  // Interrupt 
#define right_encoder_phaseB 19  

unsigned int right_encoder_counter = 0;
String right_encoder_sign = "p"; // Direction. p means positive/clockwise direction
double right_wheel_meas_vel = 0.0;    // Velocity vad/s

void setup() {
  // Set pin modes
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  
  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  Serial.begin(115200);

  pinMode(right_encoder_phaseB, INPUT);

  // Set up interrupt for right encoder
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  // enableInterrupt(right_encoder_phaseA, rightEncoderCallback, RISING);
}

void loop() {
  right_wheel_meas_vel = (10 * right_encoder_counter * (60.0/385.0)) * 0.10472;

  // Message to be sent to the serial port
  String encoder_read = "r" + right_encoder_sign + String(right_wheel_meas_vel);
  Serial.println(encoder_read);

  // Reset the counter
  right_encoder_counter = 0;

  // Set the motor speed
  analogWrite(L298N_enA, 100);

  // Delay for 100ms
  delay(100);
}

void rightEncoderCallback()
{
  if(digitalRead(right_encoder_phaseB) == HIGH)
  {
    right_encoder_sign = "p"; // Positive/clockwise direction
  }
  else
  {
    right_encoder_sign = "n"; // Negative/anti-clockwise direction
  }
  right_encoder_counter++;
}
