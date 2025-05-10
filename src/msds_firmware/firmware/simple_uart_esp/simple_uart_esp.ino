#define RXD2 16  // ESP32 GPIO
#define TXD2 17

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

void loop() {
  if (Serial2.available()) {
    String msg = Serial2.readStringUntil('\n');
    Serial.println("Received from Mega: " + msg);
    Serial2.println("Thank you");
  }
}
