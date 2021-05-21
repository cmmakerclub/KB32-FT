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
int j;

void setup() {
  board.begin();
  music.begin();
  lm73.begin();
  matrix.displayBegin();
  ldr.begin();
  mpu.begin();

  tft.setmode(0);
  for (i = 7; i >= 0; i--) {
    for (j = 0; j <= 15; j++) {
      matrix.drawPixel(j, i, 1);
      delay(10);
    }
  }
  matrix.printText(0, 0, " ");
  delay(1000);
  matrix.drawLine(0, 0, 15, 7, 1);
  delay(1000);
  matrix.drawLine(15, 0, 0, 7, 1);
  delay(1000);
}
void loop() {
  matrix.printText(0, 0, " ");
  matrix.drawRect(5, 1, 6, 6, 1);
  delay(1000);
  matrix.printText(0, 0, " ");
  matrix.drawCircle(7, 3, 2, 1);
  delay(1000);
  matrix.fillCircle(7, 3, 2, 1);
  delay(1000);
  matrix.printText(0, 0, " ");
  matrix.drawTriangle(3, 6, 13, 6, 8, 1, 1);
  delay(1000);
  matrix.fillTriangle(3, 6, 13, 6, 8, 1, 1);
  delay(1000);
}