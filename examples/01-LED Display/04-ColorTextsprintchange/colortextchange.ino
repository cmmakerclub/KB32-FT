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
  tft.setcolorBg(0x0);
  matrix.printText(0, 0, String(String("  ")));
  delay(20);
}
void loop() {
  tft.setcolorText(0xffff);
  matrix.printText(0, 0, String(String("  ")));
  delay(20);
  matrix.printText(0, 0, String(String("WH")));
  delay(1000);
  tft.setcolorText(0xf800);
  matrix.printText(0, 0, String(String("  ")));
  delay(20);
  matrix.printText(0, 0, String(String("RE")));
  delay(1000);
  tft.setcolorText(0x37e6);
  matrix.printText(0, 0, String(String("  ")));
  delay(20);
  matrix.printText(0, 0, String(String("GR")));
  delay(1000);
  tft.setcolorText(0x319f);
  matrix.printText(0, 0, String(String("  ")));
  delay(20);
  matrix.printText(0, 0, String(String("BU")));
  delay(1000);
  tft.setcolorText(0xffe0);
  matrix.printText(0, 0, String(String("  ")));
  delay(20);
  matrix.printText(0, 0, String(String("YL")));
  delay(1000);
  tft.setcolorText(0xfcc0);
  matrix.printText(0, 0, String(String("  ")));
  delay(20);
  matrix.printText(0, 0, String(String("OR")));
  delay(1000);
  tft.setcolorText(0xfb2c);
  matrix.printText(0, 0, String(String("  ")));
  delay(20);
  matrix.printText(0, 0, String(String("PI")));
  delay(1000);
  tft.setcolorText(0xc999);
  matrix.printText(0, 0, String(String("  ")));
  delay(20);
  matrix.printText(0, 0, String(String("PU")));
  delay(1000);
}