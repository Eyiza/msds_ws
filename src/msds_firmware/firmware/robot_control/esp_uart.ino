
#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(500000);
  Serial2.begin(500000, SERIAL_8N1, RXD2, TXD2);
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\r');
    // cmd.trim(); // Clean up \r or \n
    Serial2.println(cmd);
  }
  if (Serial2.available()) {
    String encoder_read = Serial2.readStringUntil('\n');
    Serial.println(encoder_read);
  }
}