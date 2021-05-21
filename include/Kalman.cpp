/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */
#include <Wire.h>
#include "Kalman.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
Kalman kalmanX;        // Create the Kalman instances
Kalman kalmanY;

void Kalman::begin(void)
{
    Wire1.begin(4, 5, 400000);

    i2cData[0] = 9;    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g

    while (i2cWrite(0x19, i2cData, 4, false))
        ; // Write to all four registers at once

    while (i2cRead(0x1D, i2cData, 1))
        ;
    i2cData[0] = i2cData[0] | 0x04;

    while (i2cWrite(0x1D, i2cData, 1, false))
        ; // Write to all four registers at once

    while (i2cWrite(0x6B, 0x01, true))
        ; // PLL with X axis gyroscope reference and disable sleep mode

    while (i2cRead(0x75, i2cData, 1))
        ;
    if (i2cData[0] != 0x68)
    { // Read "WHO_AM_I" register
        Serial.print(F("Error reading sensor"));
        while (1)
            ;
    }

    delay(100); // Wait for sensor to stabilize

    /* Set kalman and gyro starting angle */
    while (i2cRead(0x3B, i2cData, 6))
        ;
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

    // Serial.print(accX);
    // Serial.print(",");
    // Serial.print(accY);
    // Serial.print(",");
    // Serial.print(accZ);
    // Serial.print(",");
    // Serial.println();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;

    timer = micros();
}

Kalman::Kalman()
{
    /* We will set the variables like so, these can also be tuned by the user */
    Q_angle = 0.001f;
    Q_bias = 0.003f;
    R_measure = 0.03f;

    angle = 0.0f; // Reset the angle
    bias = 0.0f;  // Reset bias

    P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman::getAngle(float newAngle, float newRate, float dt)
{
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    rate = newRate - bias;
    angle += dt * rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
};

void Kalman::setAngle(float angle) { this->angle = angle; }; // Used to set angle, this should be set as the starting angle
float Kalman::getRate() { return this->rate; };              // Return the unbiased rate

/* These are used to tune the Kalman filter */
void Kalman::setQangle(float Q_angle) { this->Q_angle = Q_angle; };
void Kalman::setQbias(float Q_bias) { this->Q_bias = Q_bias; };
void Kalman::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman::getQangle() { return this->Q_angle; };
float Kalman::getQbias() { return this->Q_bias; };
float Kalman::getRmeasure() { return this->R_measure; };

uint8_t Kalman::i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop)
{
    return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t Kalman::i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop)
{
    Wire1.beginTransmission(IMUAddress);
    Wire1.write(registerAddress);
    Wire1.write(data, length);
    uint8_t rcode = Wire1.endTransmission(sendStop); // Returns 0 on success
    if (rcode)
    {
        Serial.print(F("i2cWrite failed: "));
        Serial.println(rcode);
    }
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t Kalman::i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes)
{
    uint32_t timeOutTimer;
    Wire1.beginTransmission(IMUAddress);
    Wire1.write(registerAddress);
    uint8_t rcode = Wire1.endTransmission(false); // Don't release the bus
    if (rcode)
    {
        Serial.print(F("i2cRead failed: "));
        Serial.println(rcode);
        return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
    }
    Wire1.requestFrom(IMUAddress, nbytes, (uint8_t) true); // Send a repeated start and then release the bus after reading
    for (uint8_t i = 0; i < nbytes; i++)
    {
        if (Wire1.available())
            data[i] = Wire1.read();
        else
        {
            timeOutTimer = micros();
            while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire1.available())
                ;
            if (Wire1.available())
                data[i] = Wire1.read();
            else
            {
                Serial.println(F("i2cRead timeout"));
                return 5; // This error value is not already taken by endTransmission
            }
        }
    }
    return 0; // Success
}

uint16_t Kalman::Read(uint8_t acc, uint8_t gyo, uint8_t tmp)
{
    // i2cRead(0x3B, i2cData, 14);
    // accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    // accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    // accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    // tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    // gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    // gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    // gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);

    if (acc == 1 && gyo == 0 && tmp == 0) //ACCEL X Readout
    {
        i2cRead(0x3B, i2cData, 2);
        return (int16_t)((i2cData[0] << 8) | i2cData[1]);
        acc = 0;
    }
    if (acc == 2 && gyo == 0 && tmp == 0) //ACCEL Y Readout
    {
        i2cRead(0x3D, i2cData, 2);
        return (int16_t)((i2cData[0] << 8) | i2cData[1]);
        acc = 0;
    }
    if (acc == 3 && gyo == 0 && tmp == 0) //ACCEL Z Readout
    {
        i2cRead(0x3F, i2cData, 2);
        return (int16_t)((i2cData[0] << 8) | i2cData[1]);
        acc = 0;
    }
    if (acc == 0 && gyo == 1 && tmp == 0) //Gyro X Readout
    {
        i2cRead(0x43, i2cData, 2);
        return (int16_t)((i2cData[0] << 8) | i2cData[1]);
        gyo = 0;
    }
    if (acc == 0 && gyo == 2 && tmp == 0) //Gyro Y Readout
    {
        i2cRead(0x45, i2cData, 2);
        return (int16_t)((i2cData[0] << 8) | i2cData[1]);
        gyo = 0;
    }
    if (acc == 0 && gyo == 3 && tmp == 0) //Gyro Z Readout
    {
        i2cRead(0x47, i2cData, 2);
        return (int16_t)((i2cData[0] << 8) | i2cData[1]);
        gyo = 0;
    }
    if (acc == 0 && gyo == 0 && tmp == 1)
    {
        i2cRead(0x41, i2cData, 2);
        return (int16_t)((i2cData[0] << 8) | i2cData[1]);
        tmp = 0;
    }
    else
    {
        return 0;
    }
}

float Kalman::readAngle(uint8_t ag)
{
    i2cRead(0x3B, i2cData, 14);
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);
    ;

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
    double roll = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
    {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    }
    else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt) + 0.7f; // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt) + 3.5f;
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90))
    {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
    }
    else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate;                        // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
        gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;

        /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

    //  Serial.print(roll); Serial.print("\t");
    //  Serial.print(gyroXangle); Serial.print("\t");
    //  Serial.print(compAngleX); Serial.print("\t");
    //Serial.print(kalAngleX);
    //Serial.print(",");

    //  Serial.print("\t");

    //  Serial.print(pitch); Serial.print("\t");
    //  Serial.print(gyroYangle); Serial.print("\t");
    //  Serial.print(compAngleY); Serial.print("\t");
    //Serial.print(kalAngleY);
    //Serial.print(",");

    //Serial.print("\r\n");

    //static float x, y, cx, cy;
    if (ag == 0)
    {
        return kalAngleX;
    }
    if (ag == 1)
    {
        return kalAngleY;
    }
    else
    {
        return 0;
    }
}
