void setup() {
    Serial.begin(9600);      // For monitor
    Serial2.begin(9600);     // TX2 = 16, RX2 = 17
  }
  
  void loop() {
    Serial2.println("Hello from Mega Serial2");
    delay(1000);
    String msg = Serial2.readStringUntil('\n');
    Serial.println("Received from ESP: " + msg);
  }
  