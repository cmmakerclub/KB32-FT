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
#include <Arduino.h>
#ifndef _Kalman_h_
#define _Kalman_h_

class Kalman
{
public:
    void begin(void);
    Kalman();

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float getAngle(float newAngle, float newRate, float dt);

    void setAngle(float angle); // Used to set angle, this should be set as the starting angle
    float getRate();            // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQangle(float Q_angle);
    /**
     * setQbias(float Q_bias)
     * Default value (0.003f) is in Kalman.cpp. 
     * Raise this to follow input more closely,
     * lower this to smooth result of kalman filter.
     */
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);

    float getQangle();
    float getQbias();
    float getRmeasure();

    uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
    uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
    uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);

    uint16_t Read(uint8_t acc, uint8_t gyo, uint8_t tmp);
    float readAngle(uint8_t ag);

protected:
private:
    /* IMU Data */
    double accX, accY, accZ;
    double gyroX, gyroY, gyroZ;
    int16_t tempRaw;
    double gyroXangle, gyroYangle; // Angle calculate using the gyro only
    double compAngleX, compAngleY; // Calculated angle using a complementary filter
    double kalAngleX, kalAngleY;   // Calculated angle using a Kalman filter
    uint32_t timer;
    uint8_t i2cData[14]; // Buffer for I2C data

    const uint8_t IMUAddress = 0x68;   // AD0 is logic low on the PCB
    const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

    /* Kalman filter variables */
    float Q_angle;   // Process noise variance for the accelerometer
    float Q_bias;    // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle;   // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias;    // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate;    // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
};

#endif
