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
#include "Kalman.h"
#include "TFT_eSPI.h"  //for matrix led

TFT_eSPI tft = TFT_eSPI();
MCP7941x rtc = MCP7941x();
Kalman mpu;
KB_board board = KB_board();
KB_music music = KB_music();
KB_LDR ldr = KB_LDR();
KB_LM73 lm73 = KB_LM73();
KB_8x16Matrix matrix = KB_8x16Matrix();

typedef int Number;
typedef int Boolean;
using namespace std;

int i;

void setup() {
  board.begin();
  music.begin();
  lm73.begin();
  matrix.displayBegin();
  ldr.begin();
  mpu.begin();

  tft.setmode(1);
}
void loop() {
  tft.fillScreen(0xffff);
  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(5, 5);
  tft.setTextColor(0xf800);
  tft.println(String("Test Display"));
  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(40, 40);
  tft.setTextColor(0x0);
  tft.println(String("KB32-FT"));
  delay(2000);
  tft.fillScreen(0xffff);
  for (i = 0; i <= 80; i += 5) {
    tft.drawLine(0, 0, 180, i, 0xf800);
    delay(50);
  }
  for (i = 0; i <= 80; i += 5) {
    tft.drawLine(180, 0, 0, i, 0x13);
    delay(50);
  }
  for (i = 80; i >= 0; i -= 5) {
    tft.drawLine(0, 80, 180, i, 0x4c0);
    delay(50);
  }
  for (i = 80; i >= 0; i -= 5) {
    tft.drawLine(180, 80, 0, i, 0xffe6);
    delay(50);
  }
  tft.fillScreen(0xffff);
  tft.drawRect(70, 30, 50, 30, 0x7e0);
  delay(1000);
  tft.fillRect(70, 30, 50, 30, 0x7e0);
  delay(1000);
  tft.fillScreen(0xffff);
  tft.drawCircle(64, 32, 20, 0x1f);
  delay(1000);
  tft.fillCircle(64, 32, 20, 0x1f);
  delay(1000);
}