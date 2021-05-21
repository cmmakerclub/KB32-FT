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

int16_t accX;
int16_t accY;
int16_t accZ;

void setup() {
  board.begin();
  music.begin();
  lm73.begin();
  matrix.displayBegin();
  ldr.begin();

  tft.setmode(1);
  tft.fillScreen(0x0);

  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.setTextColor(0xffff);
  tft.println(String("Accelerometer"));

  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(0, 20);
  tft.setTextColor(0xf800);
  tft.println(String("X="));

  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(0, 40);
  tft.setTextColor(0xffe0);
  tft.println(String("Y="));

  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(0, 60);
  tft.setTextColor(0xfcdf);
  tft.println(String("Z="));

  mpu.begin();
}
void loop() {
  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(45, 20);
  tft.setTextColor(0x0);
  tft.println(accX);
  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(45, 40);
  tft.setTextColor(0x0);
  tft.println(accY);
  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(45, 60);
  tft.setTextColor(0x0);
  tft.println(accZ);
  accX = mpu.Read(1, 0, 0);
  delay(1);
  accY = mpu.Read(2, 0, 0);
  delay(1);
  accZ = mpu.Read(3, 0, 0);
  delay(1);
  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(45, 20);
  tft.setTextColor(0xf800);
  tft.println(accX);
  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(45, 40);
  tft.setTextColor(0xffe0);
  tft.println(accY);
  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(45, 60);
  tft.setTextColor(0xfcdf);
  tft.println(accZ);
  delay(20);
}