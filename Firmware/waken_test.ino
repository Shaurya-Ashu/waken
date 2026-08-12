#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SDA_PIN 9
#define SCL_PIN 10
#define INT_IMU_PIN 35
#define SW1_PIN 37
#define SW3_PIN 36
#define EC_A_PIN 18
#define EC_B_PIN 15
#define EC_C_PIN 16
#define EC_S_PIN 17
#define BUZ_PIN 41
#define PGND_PIN 38

#define OLED_ADDR 0x3C
#define MPU_ADDR 0x68
#define MAX30101_ADDR 0x57
#define MCP9808_ADDR 0x18

Adafruit_SSD1306 display(128, 64, &Wire, -1);

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

uint8_t readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

void i2cScan() {
  Serial.println("I2C SCAN:");
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (i2cPresent(addr)) Serial.printf("Found 0x%02X\n", addr);
  }
}

void testOLED() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED: FAIL");
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("WAKER TEST OK");
  display.display();
  Serial.println("OLED: PASS");
}

void testIMU() {
  if (!i2cPresent(MPU_ADDR)) { Serial.println("IMU: NOT FOUND"); return; }
  uint8_t who = readReg(MPU_ADDR, 0x75);
  Serial.printf("IMU WHOAMI=0x%02X %s\n", who, who == 0x70 ? "PASS" : "FAIL");
  Serial.printf("INT_IMU pin state: %d\n", digitalRead(INT_IMU_PIN));
}

void testHR() {
  if (!i2cPresent(MAX30101_ADDR)) { Serial.println("HR: NOT FOUND"); return; }
  uint8_t id = readReg(MAX30101_ADDR, 0xFF);
  Serial.printf("HR PARTID=0x%02X %s\n", id, id == 0x15 ? "PASS" : "FAIL");
}

void testTemp() {
  if (!i2cPresent(MCP9808_ADDR)) { Serial.println("TEMP: NOT FOUND"); return; }
  Wire.beginTransmission(MCP9808_ADDR);
  Wire.write(0x05);
  Wire.endTransmission(false);
  Wire.requestFrom(MCP9808_ADDR, (uint8_t)2);
  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    msb &= 0x1F;
    float temp;
    if (msb & 0x10) {
      msb &= 0x0F;
      temp = 256 - (msb * 16.0 + lsb / 16.0);
    } else {
      temp = msb * 16.0 + lsb / 16.0;
    }
    Serial.printf("TEMP=%.2fC PASS\n", temp);
  } else {
    Serial.println("TEMP: READ FAIL");
  }
}

void testBuzzer() {
  tone(BUZ_PIN, 2000, 200);
  delay(300);
  Serial.println("BUZZER: PULSED");
}

void testButtons() {
  Serial.printf("SW1:%d SW3:%d EC_A:%d EC_B:%d EC_C:%d EC_S:%d\n",
    !digitalRead(SW1_PIN), !digitalRead(SW3_PIN),
    !digitalRead(EC_A_PIN), !digitalRead(EC_B_PIN),
    !digitalRead(EC_C_PIN), !digitalRead(EC_S_PIN));
}

void testFlyback() {
  ledcAttach(PGND_PIN, 20000, 8);
  ledcWrite(PGND_PIN, 20);
  delay(50);
  ledcWrite(PGND_PIN, 0);
  Serial.println("FLYBACK: GATE PULSED (verify with scope, no load connected)");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(INT_IMU_PIN, INPUT);
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW3_PIN, INPUT_PULLUP);
  pinMode(EC_A_PIN, INPUT_PULLUP);
  pinMode(EC_B_PIN, INPUT_PULLUP);
  pinMode(EC_C_PIN, INPUT_PULLUP);
  pinMode(EC_S_PIN, INPUT_PULLUP);
  pinMode(BUZ_PIN, OUTPUT);

  i2cScan();
  testOLED();
  testIMU();
  testHR();
  testTemp();
  testBuzzer();
  testFlyback();
}

void loop() {
  testButtons();
  delay(500);
}
