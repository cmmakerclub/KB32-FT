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

Number lux;

void setup() {
  board.begin();
  music.begin();
  lm73.begin();
  matrix.displayBegin();
  ldr.begin();
  mpu.begin();

  tft.setmode(1);
  tft.fillScreen(0x0);

  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(35, 0);
  tft.setTextColor(0xffff);
  tft.println(String("Luxmeter"));

  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(36, 20);
  tft.setTextColor(0xffec);
  tft.println(String("AutoGain"));

  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(115, 60);
  tft.setTextColor(0xf800);
  tft.println(String("Lux"));
}
void loop() {
  // tft.setTextFont(GLCD);
  tft.setTextSize(4);
  tft.setCursor(10, 50);
  tft.setTextColor(0x0);
  tft.println(lux);
  lux = ldr.mapLDRlux();
  // tft.setTextFont(GLCD);
  tft.setTextSize(4);
  tft.setCursor(10, 50);
  tft.setTextColor(0xf800);
  tft.println(lux);
  delay(50);
}