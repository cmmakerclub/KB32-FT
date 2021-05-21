#include "KB_LDR.h"
#include "esp_system.h"
#include "driver/adc.h"
#include <Wire.h>
#define MAX_LDR_VALUE 3400 // calibated value
#define AVG_SAMPLE 20      // 100 sample
#define SMOOTH_DELAY 1     //  ms delay
void KB_LDR::begin(void)
{
    analogSetPinAttenuation(KB_LDR_PIN, ADC_0db);
    Wire.beginTransmission(0x70);
    delayMicroseconds(1);
    Wire.write(0x02);
    Wire.write((0x00));
    Wire.endTransmission();
}

float KB_LDR::getLDR()
{
    return analogRead(KB_LDR_PIN);
}

uint16_t KB_LDR::mapLDRlux()
{
    static uint8_t ldrmode;
    static uint32_t value;

    if (ldrmode == 0)
    {
        value = 0;
        for (int i = 0; i < AVG_SAMPLE; i++)
        {
            value += analogRead(KB_LDR_PIN);
            delay(SMOOTH_DELAY);
        }
        ldr = value / AVG_SAMPLE;

        ldr = (((float)ldr * 3.3f) / 4095.0f) * 1000;
        ldr = map(ldr, 0, 3300, swldr, 9660);
    }

    if (ldrmode == 1)
    {
        value = 0;
        for (int i = 0; i < AVG_SAMPLE; i++)
        {
            value += analogRead(KB_LDR_PIN);
            delay(SMOOTH_DELAY);
        }
        ldr = value / AVG_SAMPLE;

        ldr = (((float)ldr * 3.3f) / 4095.0f) * 1000;
        ldr = map(ldr, 0, 3300, 0, 966);
    }

    if (ldrmode == 0 && ldr <= swldr)
    {
        Wire.beginTransmission(0x70);
        delayMicroseconds(1);
        Wire.write(0x02);
        Wire.write(0x01);
        Wire.endTransmission();
        ldrmode = 1;
        delay(250);
    }

    if (ldrmode == 1 && ldr > swldr)
    {
        Wire.beginTransmission(0x70);
        delayMicroseconds(1);
        Wire.write(0x02);
        Wire.write(0x00);
        Wire.endTransmission();
        ldrmode = 0;
        delay(250);
    }

    if (ldr > 740 && ldr < 780)
    {
        Wire.beginTransmission(0x70);
        delayMicroseconds(1);
        Wire.write(0x02);
        Wire.write(0x01);
        Wire.endTransmission();
        delay(250);
        float value1 = analogRead(KB_LDR_PIN);
        Wire.beginTransmission(0x70);
        delayMicroseconds(1);
        Wire.write(0x02);
        Wire.write(0x00);
        Wire.endTransmission();
        delay(250);
        float value2 = analogRead(KB_LDR_PIN);
        if (abs(value1 - value2) < 50)
        {
            swldr = (value1 + value2) / 2;
            //Serial.print("sw ");
            //Serial.println(swldr);
        }
    }

    return ldr;
}

uint16_t KB_LDR::mapLDR()
{
    LuxSetGain(1);//start high gain
    //uint16_t readLDR;
    uint32_t value = 0;
    for (int i = 0; i < AVG_SAMPLE; i++)
    {
        value += analogRead(KB_LDR_PIN);
        delay(SMOOTH_DELAY);
    }
    value = value / AVG_SAMPLE;
    //Serial.println(value);
    if (value > MAX_LDR_VALUE)
    {
        value = MAX_LDR_VALUE;
    }
    ldr = ((MAX_LDR_VALUE - value) * 100) / MAX_LDR_VALUE;
    return ldr;
}

uint16_t KB_LDR::mapLDRinvert()
{
    LuxSetGain(1);//start high gain
    //uint16_t readLDR;
    uint32_t value = 0;
    for (int i = 0; i < AVG_SAMPLE; i++)
    {
        value += analogRead(KB_LDR_PIN);
        delay(SMOOTH_DELAY);
    }
    value = value / AVG_SAMPLE;
    //Serial.println(value);
    if (value > 4095)
    {
        value = 4095;
    }
    ldr = ((4095 - value) * 100) / 4095;
    ldr = map(ldr, 0, 100, 100, 0);
    return ldr;
}

void KB_LDR::LuxSetGain(uint8_t G)
{
    if (G == 0 && currentGain != 0)
    {
        Wire.beginTransmission(0x70);
        delayMicroseconds(1);
        Wire.write(0x02);
        Wire.write(0x00);
        Wire.endTransmission();
        delay(250);

        currentGain = 0;
    }
    if (G == 1 && currentGain != 1)
    {
        Wire.beginTransmission(0x70);
        delayMicroseconds(1);
        Wire.write(0x02);
        Wire.write(0x01);
        Wire.endTransmission();
        delay(250);

        currentGain = 1;
    }
}

// uint16_t KB_LDR::LuxRead()
// {
//     if (currentGain == 0)
//     {
//         return LuxLowGain();
//     }
//     if (currentGain == 1)
//     {
//         return LuxHighGain();
//     }
// }

uint16_t KB_LDR::LuxLowGain()
{
    LuxSetGain(0);
    uint32_t value = 0;
    for (int i = 0; i < AVG_SAMPLE; i++)
    {
        value += analogRead(KB_LDR_PIN);
        delay(SMOOTH_DELAY);
    }
    ldr = value / AVG_SAMPLE;

    ldr = (((float)ldr * 3.3f) / 4095.0f) * 1000;
    ldr = map(ldr, 0, 3300, 0, 9660);

    return ldr;
}

uint16_t KB_LDR::LuxHighGain()
{
    LuxSetGain(1);
    uint32_t value = 0;
    for (int i = 0; i < AVG_SAMPLE; i++)
    {
        value += analogRead(KB_LDR_PIN);
        delay(SMOOTH_DELAY);
    }
    ldr = value / AVG_SAMPLE;
    ldr = (((float)ldr * 3.3f) / 4095.0f) * 1000;
    ldr = map(ldr, 0, 3300, 0, 966);

    return ldr;
}

// float KB_LDR::adc_read()
// {
//     float tmp = 0;
//     for (int i = 0; i < 20; i++)
//         tmp += (analogRead(LDR_PIN)), delay(2);
//     if (ldr_gain_prev == high_light)
//     {
//         ldr = (tmp + adc_offset);
//         ldr = (((float)ldr * 3.3f) / 4095.0f) * 1000;
//         ldr = map(ldr, 0, 3300, 0, 9660);
//     }

//     if (ldr_gain_prev == low_light)
//     {
//         ldr = (tmp);
//         ldr = (((float)ldr * 3.3f) / 4095.0f) * 1000;
//         ldr = map(ldr, 0, 3300, 0, 966);
//     }
//     return ldr / 20.0f;
// }

// void set_ldr_gain(uint8_t gain)
// {
//     Wire.beginTransmission(0x70);
//     Wire.write(0x02);
//     Wire.write(gain);
//     Wire.endTransmission();
//     delay(200);
// }

// uint16_t KB_LDR::mapLDRlux()
// {
//     //  if (adc_read() + adc_offset > (3000 + adc_offset) && ldr_gain_prev == low_light)
//     //    ldr_gain_new = high_light;
//     //
//     //  if (adc_read() + adc_offset < (900 + adc_offset) && ldr_gain_prev == high_light)
//     //    ldr_gain_new = low_light;

//     ldr_gain_new = 1 - ldr_gain_new;

//     if (ldr_gain_prev == high_light && ldr_gain_new == low_light)
//     {
//         ldr_gain_prev = ldr_gain_new;
//         // int16_t adc = adc_read();
//         set_ldr_gain(ldr_gain_new);
//         adc_offset = 0;
//     }

//     if (ldr_gain_prev == low_light && ldr_gain_new == high_light)
//     {
//         int16_t adc = adc_read();
//         ldr_gain_prev = ldr_gain_new;
//         set_ldr_gain(ldr_gain_new);
//         //    adc_offset = 0;
//         adc_offset = adc - adc_read() + adc_offset;
//     }

//     // if (ldr_gain_prev == high_light)
//     // {
//     //     ldr = (adc_read() + adc_offset);
//     //     ldr = (((float)ldr * 3.3f) / 4095.0f) * 1000;
//     //     ldr = map(ldr, 0, 3300, 0, 9660);
//     // }

//     // if (ldr_gain_prev == low_light)
//     // {
//     //     ldr = (adc_read() + adc_offset);
//     //     ldr = (((float)ldr * 3.3f) / 4095.0f) * 1000;
//     //     ldr = map(ldr, 0, 3300, 0, 966);
//     // }
//     return (adc_read() + adc_offset);
// }
