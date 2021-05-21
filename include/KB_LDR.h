#ifndef KB_LDR_h
#define KB_LDR_h

#include <Arduino.h>
#include <Wire.h>

#define LDR_PIN 36
//#define high_light 0
//#define low_light 1

class KB_LDR
{
public:
    void begin(void);
    uint16_t mapLDR();
    uint16_t mapLDRinvert();
    uint16_t mapLDRlux();
    float getLDR();
    float adc_read();
    void LuxSetGain(uint8_t G);
    uint16_t LuxLowGain();
    uint16_t LuxHighGain();
    uint16_t LuxRead();

protected:
    float ldr;

private:
    //float adc_offset;
    //uint8_t ldr_gain_new = low_light, ldr_gain_prev = low_light;
    float swldr = 600;
    uint8_t currentGain;
    //uint8_t G =1; //start high gain
};

#endif /* KB_LDR_h */
