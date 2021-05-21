#include "KB32FT.h"
#include <Wire.h>

void KB_32FT::begin(void) {
    Wire.begin();
    Wire1.begin(4, 5);
}

uint16_t KB_32FT::readTempSTM1() {
    Wire.requestFrom(0x70, 6);  // transmit to device
    tmp = Wire.read();
    tmp = (tmp << 8) | Wire.read();
    tmp1 = Wire.read();
    tmp1 = (tmp1 << 8) | Wire.read();
    tmp2 = Wire.read();
    tmp2 = (tmp2 << 8) | Wire.read();
    Wire.endTransmission();  // stop transmitting
    delay(1);

    return tmp;
}

uint16_t KB_32FT::readTempSTM2() {
    Wire.requestFrom(0x70, 6);  // transmit to device
    tmp = Wire.read();
    tmp = (tmp << 8) | Wire.read();
    tmp1 = Wire.read();
    tmp1 = (tmp1 << 8) | Wire.read();
    tmp2 = Wire.read();
    tmp2 = (tmp2 << 8) | Wire.read();
    Wire.endTransmission();  // stop transmitting
    delay(1);

    return tmp1;
}

uint16_t KB_32FT::readTempSTM3() {
    Wire.requestFrom(0x70, 6);  // transmit to device
    tmp = Wire.read();
    tmp = (tmp << 8) | Wire.read();
    tmp1 = Wire.read();
    tmp1 = (tmp1 << 8) | Wire.read();
    tmp2 = Wire.read();
    tmp2 = (tmp2 << 8) | Wire.read();
    Wire.endTransmission();  // stop transmitting
    delay(1);

    return tmp2;
}