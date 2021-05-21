#include <Arduino.h>
#include <vector>
#include <WiFi.h>
#include <Wire.h>
#include "SPI.h"
#include "Adafruit_GFX.h"  //for matrix led
#include "Adafruit_LEDBackpack.h"

#include "KB_initBoard.h"
#include "KB_music.h"
#include "KB_LDR.h"
#include "KB_LM73.h"
#include "KB_ht16k33.h"
#include "MCP7941x.h"

#include "TFT_eSPI.h"  //for matrix led

TFT_eSPI tft = TFT_eSPI();
MCP7941x rtc = MCP7941x();

KB_board board = KB_board();
KB_music music = KB_music();
KB_LDR ldr = KB_LDR();
KB_LM73 lm73 = KB_LM73();
KB_8x16Matrix matrix = KB_8x16Matrix();

typedef int Number;
typedef int Boolean;
using namespace std;

uint16_t tmp = 0;
uint16_t tmp1 = 0;
uint16_t tmp2 = 0;
uint16_t tmp3 = lm73.readTemp();

void setup() {
  board.begin();
  music.begin();
  lm73.begin();
  matrix.displayBegin();
  ldr.begin();
  Serial.begin(115200);

  Wire.beginTransmission(0x70);
  delayMicroseconds(1);
  Wire.write(0x01);
  Wire.write((0x00));
  Wire.endTransmission();
  delay(100);

  tft.init();
  tft.setRotation(1);
  tft.invertDisplay(1);
  tft.fillScreen(TFT_BLACK);
  delay(100);

  Wire.beginTransmission(0x70);
  delayMicroseconds(1);
  Wire.write(0xEF);
  Wire.endTransmission();
  delay(30);
  matrix.scrollText(String(String("KB32FT")));

  Wire.beginTransmission(0x70);  // transmit to device #4
  delayMicroseconds(1);
  Wire.write(0x04);            // sends one byte
  Wire.write(TFT_WHITE >> 8);  // sends one byte
  Wire.write(TFT_WHITE);       // sends one byte
  Wire.endTransmission();      // stop transmitting
  matrix.scrollText(String(String("WHITE...")));
  delay(100);

  Wire.beginTransmission(0x70);
  delayMicroseconds(1);
  Wire.write(0x01);
  Wire.write((0x01));
  Wire.endTransmission();
  delay(100);

  tft.begin();
  tft.setRotation(1);
  tft.invertDisplay(1);
  tft.fillScreen(TFT_BLACK);
  delay(100);

  Wire.beginTransmission(0x70);
  delayMicroseconds(1);
  Wire.write(0xEF);
  Wire.endTransmission();
  delay(30);
  tft.fillScreen(0x0);

  tft.setUTF8Font(CF_KN_R_09_EN, CF_KN_R_09_TH, NULL);
  tft.setTextColor(0xf800, 0x0);
  tft.drawUTF8String(String("ทดสอบ"), 50, 0, GFXFF);

  tft.setUTF8Font(CF_KN_R_09_EN, CF_KN_R_09_TH, NULL);
  tft.setTextColor(0xfb20, 0x0);
  tft.drawUTF8String(String("แสง"), 10, 20, GFXFF);

  tft.setUTF8Font(CF_KN_R_09_EN, CF_KN_R_09_TH, NULL);
  tft.setTextColor(0xffe0, 0x0);
  tft.drawUTF8String(String("อุณหภูมิ"), 10, 40, GFXFF);

  tft.setUTF8Font(CF_KN_R_09_EN, CF_KN_R_09_TH, NULL);
  tft.setTextColor(0xcb39, 0x0);
  tft.drawUTF8String(String("S1 ="), 10, 60, GFXFF);

  tft.setUTF8Font(CF_KN_R_09_EN, CF_KN_R_09_TH, NULL);
  tft.setTextColor(0xcb39, 0x0);
  tft.drawUTF8String(String("S2 ="), 80, 60, GFXFF);
  delay(30);

  delay(100);
  Wire.beginTransmission(0x70);
  delayMicroseconds(1);
  Wire.write(0x02);
  Wire.write((0x00));
  Wire.endTransmission();
}

void loop() {
  if (((int)digitalRead(KB_BUTTON1)) == 0 &&
      ((int)digitalRead(KB_BUTTON2)) == 0) {
    delay(1);
    Wire.requestFrom(0x70, 6);  // transmit to device
    tmp = Wire.read();
    tmp = (tmp << 8) | Wire.read();
    tmp1 = Wire.read();
    tmp1 = (tmp1 << 8) | Wire.read();
    tmp2 = Wire.read();
    tmp2 = (tmp2 << 8) | Wire.read();
    Wire.endTransmission();  // stop transmitting
    delay(1);

    uint16_t tmpp = tmp1 - tmp3;
    Wire.beginTransmission(0x70);  // transmit to device #4
    delayMicroseconds(1);
    Wire.write(0x01);        // sends one byte
    Wire.write(tmpp >> 8);   // sends one byte
    Wire.write(tmpp);        // sends one byte
    Wire.endTransmission();  // stop transmitting
    Serial.println("setnewTemp:OK");

    delay(100);
    Wire.beginTransmission(0x70);
    delayMicroseconds(1);
    Wire.write(0x02);
    Wire.write((0x01));
    Wire.endTransmission();

    delay(1000);
  } else {
    tft.setUTF8Font(CF_KN_R_09_EN, CF_KN_R_09_TH, NULL);
    tft.setTextColor(0x0, 0x0);
    tft.drawUTF8String(String("          "), 60, 20, GFXFF);
    tft.setTextColor(0xfb20, 0x0);
    tft.drawUTF8String(String(String(ldr.mapLDRinvert())), 60, 20, GFXFF);
    tft.setTextColor(0x0, 0x0);
    tft.drawUTF8String(String("          "), 80, 40, GFXFF);
    tft.setTextColor(0xffe0, 0x0);
    tft.drawUTF8String(String(String(lm73.readTemp())), 80, 40, GFXFF);
    tft.setTextColor(0x0, 0x0);
    tft.drawUTF8String(String("  "), 45, 60, GFXFF);
    tft.setTextColor(0xcb39, 0x0);
    tft.drawUTF8String(String(String(digitalRead(KB_BUTTON1))), 45, 60, GFXFF);
    tft.setTextColor(0x0, 0x0);
    tft.drawUTF8String(String("  "), 120, 60, GFXFF);
    tft.setTextColor(0xcb39, 0x0);
    tft.drawUTF8String(String(String(digitalRead(KB_BUTTON2))), 120, 60, GFXFF);

    delay(1);
    Wire.requestFrom(0x70, 6);  // transmit to device
    tmp = Wire.read();
    tmp = (tmp << 8) | Wire.read();
    tmp1 = Wire.read();
    tmp1 = (tmp1 << 8) | Wire.read();
    tmp2 = Wire.read();
    tmp2 = (tmp2 << 8) | Wire.read();
    Wire.endTransmission();  // stop transmitting
    delay(1);
    tmp3 = lm73.readTemp() * 10;
    Serial.print(tmp);
    Serial.print(",");
    Serial.print(tmp1);
    Serial.print(",");
    Serial.print(tmp2);
    Serial.print(",");
    Serial.println(tmp3);
    digitalWrite(KB_USB, 1);
  }
  music.tone(2093, 100);
  digitalWrite(KB_USB, 0);
  digitalWrite(2, 0);
  digitalWrite(12, 0);
  delay(100);
  digitalWrite(2, 1);
  digitalWrite(12, 1);
}