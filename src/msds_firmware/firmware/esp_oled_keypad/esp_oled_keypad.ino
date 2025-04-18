#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ezButton.h>

#define SCREEN_WIDTH 128 // OLED width
#define SCREEN_HEIGHT 64 // OLED height
#define OLED_RESET     -1 
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const char* ssid = "Michael's Home";       // Replace with your Wi-Fi SSID
const char* password = "Michaels"; // Replace with your Wi-Fi password

#define KEY_NUM 4  // Number of keys
#define PIN_KEY_1 27  
#define PIN_KEY_2 14  
#define PIN_KEY_3 25  
#define PIN_KEY_4 26  

ezButton keypad_1x4[] = {
  ezButton(PIN_KEY_1),
  ezButton(PIN_KEY_2),
  ezButton(PIN_KEY_3),
  ezButton(PIN_KEY_4)
};

void setup() {
  Serial.begin(115200);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  
  // Connect to Wi-Fi
  Serial.print("Connecting to Wi-Fi");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);
  display.println("Connecting to Wi-Fi...");
  display.display();
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWi-Fi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  display.clearDisplay();
  display.setCursor(0, 10);
  display.println("Wi-Fi Connected!");
  display.setCursor(0, 25);
  display.print("IP: ");
  display.println(WiFi.localIP());
  display.display();

  // Initialize keypad
  for (byte i = 0; i < KEY_NUM; i++) {
    keypad_1x4[i].setDebounceTime(100);
  }
}

void loop() {
  int key = getKeyPressed();
  if (key) {
    Serial.print("Key ");
    Serial.print(key);
    Serial.println(" pressed!");

    display.clearDisplay();
    display.setCursor(0, 10);
    display.println("Key Pressed:");
    display.setCursor(40, 30);
    display.setTextSize(2);
    display.print(key);
    display.display();
  }
}

int getKeyPressed() {
  for (byte i = 0; i < KEY_NUM; i++) {
    keypad_1x4[i].loop();
  }

  for (byte i = 0; i < KEY_NUM; i++) {
    if (keypad_1x4[i].isPressed()) {
      return (i + 1);
    }
  }
  return 0;
}
