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

void setup() {
  board.begin();
  music.begin();
  lm73.begin();
  matrix.displayBegin();
  ldr.begin();
  mpu.begin();

  tft.setmode(0);
}
void loop() {
  matrix.drawBitmap(
      0, 0, (uint8_t *)"\x0\x0\x0\x0\x0\x0\x0\x4\x16\x35\x55\x95\xfd\x5\x6\x4");
  delay(500);
  matrix.drawBitmap(
      0, 0, (uint8_t *)"\x0\x0\x0\x4\x16\x35\x55\x95\xfd\x5\x6\x4\x0\x0\x0\x0");
  delay(500);
  matrix.drawBitmap(
      0, 0, (uint8_t *)"\x4\x16\x35\x55\x95\xfd\x5\x6\x4\x0\x0\x0\x0\x0\x0\x0");
  delay(500);
}