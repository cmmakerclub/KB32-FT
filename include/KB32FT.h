#ifndef KB_32FT_h
#define KB_32FT_h

#include <Arduino.h>
#include <Wire.h>

class KB_32FT
{
public:
    void begin(void);
    uint16_t readTempSTM1();
    uint16_t readTempSTM2();
    uint16_t readTempSTM3();

protected:

private:
    uint16_t tempSTM;
    uint16_t tmp = 0;
    uint16_t tmp1 = 0;
    uint16_t tmp2 = 0;
};

#endif /* KB_LM73_h */