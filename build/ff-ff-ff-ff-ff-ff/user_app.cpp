#include <Arduino.h>
#include <vector>
#include <WiFi.h>
#include <Wire.h>
#include "SPI.h"
#include "Adafruit_GFX.h" //for matrix led
#include "Adafruit_LEDBackpack.h"

#define REMOTEXY_MODE__ESP32CORE_WIFI_POINT
#include <RemoteXY.h>
#define REMOTEXY_WIFI_SSID "Drone"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377
#define ARDUINO_RUNNING_CORE 1
#include <VL53L0X.h>
#include "MPU6050.h"
#include "FreeRTOS.h"
#include <MadgwickAHRS.h>
#include <Battery.h>

#include "KB_initBoard.h"
#include "KB_music.h"
#include "KB_LDR.h"
#include "KB_LM73.h"
#include "KB_ht16k33.h"
#include "MCP7941x.h"
#include "Kalman.h"
#include "TFT_eSPI.h" //for matrix led

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

char print_buf[128];

// RemoteXY configurate
      #pragma pack(push, 1)
      uint8_t RemoteXY_CONF[] = {255, 5,   0,  1,  0,   70,  0,   11,  13,  0,   5,
                                 32,  0,   8,  30, 30,  1,   26,  31,  5,   32,  70,
                                 8,   30,  30, 1,  26,  31,  2,   0,   39,  2,   22,
                                 11,  36,  26, 31, 31,  79,  78,  0,   79,  70,  70,
                                 0,   66,  0,  46, 30,  9,   29,  37,  26,  129, 0,
                                 39,  14,  22, 3,  24,  65,  108, 116, 105, 116, 117,
                                 100, 101, 32, 67, 111, 110, 116, 114, 111, 108, 0};

  // this structure defines all the variables and events of your control interface
  struct {

    // input variables
    int8_t joystick_1_x; // =-100..100 x-coordinate joystick position
    int8_t joystick_1_y; // =-100..100 y-coordinate joystick position
    int8_t joystick_2_x; // =-100..100 x-coordinate joystick position
    int8_t joystick_2_y; // =-100..100 y-coordinate joystick position
    uint8_t switch_1; // =1 if switch ON and =0 if OFF

    // output variables
    int8_t level_1; // =0..100 level position

      // other variable
    uint8_t connect_flag;  // =1 if wire connected, else =0

  } RemoteXY;
  #pragma pack(pop)

  /////////////////////////////////////////////
  //           END RemoteXY include          //
  /////////////////////////////////////////////
#define M1     4
  #define M2     5
  #define M3     6
  #define M4     7

  #define LEDC_TIMER_11_BIT  11
  #define LEDC_BASE_FREQ     16000
  #define sampleFreq         200.0f // 200 hz sample rate!
  #define GYROSCOPE_SENSITIVITY       16.4f


  const int S1_pin = 16;
  const int S2_pin = 14;

  const int G_led = 12;
  const int R_led = 2;

  const int M1_pin = 15;
  const int M2_pin = 17;
  const int M3_pin = 25;
  const int M4_pin = 26;

  const int scl_pin = 5;
  const int sda_pin = 4;

  const int vbatt_pin = 39;

  VL53L0X sensor;
  float high;
  uint16_t battery_level;

  Madgwick filter;

  MPU6050 accelgyro;
  //MPU6050 accelgyro(0x69); // <-- use for AD0 high

  int16_t ax, ay, az;
  int16_t gx, gy, gz;


  volatile float Error_yaw = 0, Errer_pitch = 0, Error_roll = 0;                   //States Error
  volatile float Sum_Error_yaw = 0, Sum_Error_pitch = 0, Sum_Error_roll = 0;      // Sum of error
  volatile float D_Error_yaw = 0, D_Error_pitch = 0, D_Error_roll = 0;            // error dot
  volatile float Del_yaw = 0, Del_pitch = 0, Del_roll = 0;                        // Delta states value for rotate axis
  volatile float t_compensate = 0;
  volatile float T_center_minus = 0;
  volatile float y_roll = 0, y_pitch = 0, y0_roll = 0, y0_pitch = 0;
  volatile float rMat[3][3] = { 0 };

  volatile float Kp_roll = 8.75;
  volatile float Ki_roll = 2;
  volatile float Kd_roll = 4;

  volatile float Kp_pitch = 8.75;
  volatile float Ki_pitch = 2;
  volatile float Kd_pitch = 4;

  volatile float Kp_yaw = 6;
  volatile float Ki_yaw = 1;
  volatile float Kd_yaw = 0;

  volatile float Kp_T = 3;
  volatile float Ki_T = 0.3;
  volatile float Kd_T = 1.0;

  volatile float Ref_altitude, Error_T, Sum_Error_T, D_Error_T, Buf_D_Error_T;

  volatile float  motor_A = 0, motor_B = 0, motor_C = 0, motor_D = 0, T_center = 0;// Motors output value

  SemaphoreHandle_t Mutex_i2c;

  void TaskBlink(void* pvParameters);
  void TaskAnalogReadA3(void* pvParameters);

  void motor_drive(uint8_t channel, int32_t value, int32_t valueMax = 2048) {
    if (value < 0) value = 0;
    uint32_t duty = min(value, valueMax);
    ledcWrite(channel, duty);
  }

  float lpf(float alfa, float new_data, float prev_data)
  {
    float output = prev_data + (alfa * (new_data - prev_data));
    return output;
  }

  Battery batt = Battery(3300, 4200, vbatt_pin);
  float vv_batt;
  float lpf(float x, float y) {
    return y + 0.03f * (x - y);
  }

  float Ref_altitude_S;

  /*--------------------------------------------------*/
  /*---------------------- Tasks ---------------------*/
  /*--------------------------------------------------*/

  void angle_controller(void* pvParameters)  // This is a task.
  {
    (void)pvParameters;

    float axf, ayf, azf, gxf, gyf, gzf;

    xSemaphoreTake(Mutex_i2c, portMAX_DELAY);
    Serial.println("Initializing I2C devices...");

    accelgyro.reset();
    vTaskDelay(150);

    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);

    accelgyro.initialize();

    accelgyro.setClockSource(3);

    accelgyro.setDLPFMode(2);

    accelgyro.setRate(4); // 1000 hz /(1+4) = 200 hz

    vTaskDelay(100);

    // use the code below to change accel/gyro offset values
    Serial.println("Updating internal sensor offsets...");

    accelgyro.CalibrateAccel(20);
    accelgyro.CalibrateGyro(20);
    accelgyro.PrintActiveOffsets();

    // initialize digital LED_BUILTIN on pin 13 as an output.

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    xSemaphoreGive(Mutex_i2c);

    float cal_pitch;
    float cal_roll;
    float Buf_D_Error_yaw;
    float Buf_D_Errer_pitch;
    float Buf_D_Error_roll;
    float roll;
    float pitch;
    float heading_speed;
    float _roll;
    float _pitch;
    float _heading;
    float _heading_speed;
    float x1;
    float x2;
    float x3;
    float x4;

    float D_Ref_pitch = 0;
    float D_Ref_roll = 0;

    uint32_t start_time = xTaskGetTickCount();

    for (;;) // A Task shall never return or exit.
    {
      digitalWrite(G_led, HIGH);
      xSemaphoreTake(Mutex_i2c, portMAX_DELAY);
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      xSemaphoreGive(Mutex_i2c);

      axf = (float)ax * (2.0 / 32768.0f);
      ayf = (float)ay * (2.0 / 32768.0f);
      azf = (float)az * (2.0 / 32768.0f);
      gxf = (float)gx * (250.0 / 32768.0f);
      gyf = (float)gy * (250.0 / 32768.0f);
      gzf = (float)gz * (250.0 / 32768.0f);

      filter.updateIMU(gxf, gyf, gzf, axf, ayf, azf);

      _roll = filter.getRoll();
      _pitch = filter.getPitch();
      _heading = filter.getYaw();
      _heading_speed = gzf;

      x1 = (float)RemoteXY.joystick_1_x / 100.0f;
      // x2 = (float)RemoteXY.joystick_1_y / 100.0f;
      x3 = (float)RemoteXY.joystick_2_x / 100.0f;
      x4 = (float)RemoteXY.joystick_2_y / 100.0f;

      float roll_f = x3 * 10;
      float pitch_f = x4 * 10;
      // T_center = x2 * 2000;
      heading_speed = -x1 * 135;


      pitch = lpf(0.04f, pitch_f, pitch);
      roll = lpf(0.04f, roll_f, roll);

      Error_roll = (float)roll - ((float)_roll);
      Errer_pitch = (float)pitch - ((float)_pitch);
      Error_yaw = ((float)heading_speed - ((float)_heading_speed));

      if (RemoteXY.switch_1 == 0)
        T_center = (float)RemoteXY.joystick_1_y * 20.0f;


      if (T_center > 500)
      {

        Sum_Error_yaw = constrain((Sum_Error_yaw + (Error_yaw / sampleFreq)), -100.0f, 100.0f);
        Sum_Error_pitch = constrain((Sum_Error_pitch + (Errer_pitch / sampleFreq)), -100.0f, 100.0f);
        Sum_Error_roll = constrain((Sum_Error_roll + (Error_roll / sampleFreq)), -100.0f, 100.0f);

      }

      D_Error_yaw = 0;
      D_Error_pitch = (pitch - Buf_D_Errer_pitch) * sampleFreq - gyf;
      D_Error_roll = (roll - Buf_D_Error_roll) * sampleFreq - gxf;

      Buf_D_Error_roll = roll;
      Buf_D_Errer_pitch = pitch;

      Del_yaw = (Kp_yaw * Error_yaw) + (Ki_yaw * Sum_Error_yaw) + constrain((Kd_yaw * D_Error_yaw), -1500, 1500);
      Del_pitch = (Kp_pitch * Errer_pitch) + (Ki_pitch * Sum_Error_pitch) + constrain((Kd_pitch * D_Error_pitch), -1500, 1500);
      Del_roll = (Kp_roll * Error_roll) + (Ki_roll * Sum_Error_roll) + constrain((Kd_roll * D_Error_roll), -1500, 1500);

      if (T_center > 100) {

        motor_A = T_center + Del_pitch + Del_roll + Del_yaw;
        motor_B = T_center + Del_pitch - Del_roll - Del_yaw;
        motor_C = T_center - Del_pitch - Del_roll + Del_yaw;
        motor_D = T_center - Del_pitch + Del_roll - Del_yaw;

        motor_drive(M1, motor_A);
        motor_drive(M2, motor_B);
        motor_drive(M3, motor_C);
        motor_drive(M4, motor_D);

      }
      else {

        motor_drive(M1, 0);
        motor_drive(M2, 0);
        motor_drive(M3, 0);
        motor_drive(M4, 0);

      }

      digitalWrite(G_led, LOW);

      vTaskDelayUntil(&start_time, 5);
    }
  }

  void attitude_controller(void* pvParameters)  // This is a task.
  {
    (void)pvParameters;
    //float high;

    xSemaphoreTake(Mutex_i2c, portMAX_DELAY);
    sensor.init();
    sensor.setTimeout(20000); // 20000 us , 50 hz
    sensor.startContinuous(20);
    xSemaphoreGive(Mutex_i2c);




    uint32_t start_time = xTaskGetTickCount();
    for (;;)
    {
      digitalWrite(R_led, HIGH);
      xSemaphoreTake(Mutex_i2c, portMAX_DELAY);
      high = constrain(sensor.readRangeContinuousMillimeters(), 0, 1200); //
      xSemaphoreGive(Mutex_i2c);

      float x2 = (float)RemoteXY.joystick_1_y / 100.0f;

      if (RemoteXY.switch_1 == 0) {
        Ref_altitude = 0;
      }
          else {

        Ref_altitude = constrain(Ref_altitude + x2 * 5, 0, 1000);
        if (RemoteXY.joystick_1_y <= -100) Ref_altitude = 0;
        if (RemoteXY.connect_flag == 0) Ref_altitude = 0;
        if (battery_level <= 3300) Ref_altitude = 0;


        high = constrain(high * cosf(filter.getRollRadians()) * cosf(filter.getPitchRadians()), 0, 1500);

        Buf_D_Error_T = Error_T;
        Error_T = (float)Ref_altitude - ((float)high);

        if (Ref_altitude > 150)
        {
          Sum_Error_T = constrain((Sum_Error_T + (Error_T / 50)), -3000.0f, 3000.0f);
        }

        D_Error_T = (Error_T - Buf_D_Error_T) * 50;


        if (Ref_altitude > 150)
        {
          T_center = (Kp_T * Error_T) + (Ki_T * Sum_Error_T) + constrain((Kd_T * D_Error_T), 0, 1500);
        }
        else {
          T_center = 0;
        }
      }
      // Serial.println(Ref_altitude);

      battery_level = lpf(analogRead(vbatt_pin) * 2 * 3600 / 4095,battery_level);
      vv_batt = constrain(batt.level(battery_level), 0, 100);
      RemoteXY.level_1 = vv_batt;
      digitalWrite(R_led, LOW);
      vTaskDelayUntil(&start_time, 20);
    }
  }


void setup()
{
  board.begin();
  music.begin();
  lm73.begin();
  matrix.displayBegin();
  ldr.begin();
  
  tft.setmode(1);
  tft.fillScreen(0x0);

    //tft.setTextFont(GLCD);
    tft.setTextSize(3);
    tft.setCursor(30, 0);
    tft.setTextColor(0xf800);
    tft.println(String("Drone"));

    //tft.setTextFont(GLCD);
    tft.setTextSize(2);
    tft.setCursor(0, 40);
    tft.setTextColor(0xffff);
    tft.println(String("Carlibration."));

      // RemoteXY connection settings
      
      
      
      
      

      

      RemoteXY_Init();
      delay(100);



        
        
        
        
        
        

  

  


  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);

  pinMode(G_led, OUTPUT);
  pinMode(R_led, OUTPUT);
  digitalWrite(G_led, HIGH);
  digitalWrite(R_led, HIGH);

  pinMode(S1_pin, INPUT_PULLUP);
  pinMode(S2_pin, INPUT_PULLUP);

  pinMode(M1_pin, OUTPUT);
  pinMode(M2_pin, OUTPUT);
  pinMode(M3_pin, OUTPUT);
  pinMode(M4_pin, OUTPUT);

  pinMode(M1_pin, OUTPUT);

  batt.begin(3300, 2.0, &sigmoidal);

  ledcSetup(M1, LEDC_BASE_FREQ, LEDC_TIMER_11_BIT);
  ledcSetup(M2, LEDC_BASE_FREQ, LEDC_TIMER_11_BIT);
  ledcSetup(M3, LEDC_BASE_FREQ, LEDC_TIMER_11_BIT);
  ledcSetup(M4, LEDC_BASE_FREQ, LEDC_TIMER_11_BIT);

  ledcAttachPin(M1_pin, M1);
  ledcAttachPin(M2_pin, M2);
  ledcAttachPin(M3_pin, M3);
  ledcAttachPin(M4_pin, M4);

  Mutex_i2c = xSemaphoreCreateMutex();

  Wire.begin(4, 5);
  Wire.setClock(400000);

  xTaskCreatePinnedToCore(
    angle_controller
    , "angle_controller"
    , 1024 * 2
    , NULL
    , 2
    , NULL
    , ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    attitude_controller
    , "attitude_controller"
    , 1024 * 2
    , NULL
    , 1
    , NULL
    , ARDUINO_RUNNING_CORE);

  //RemoteXY_Init();
  delay(100);
}
void loop()
{
          RemoteXY_Handler();
        delay(30);
        RemoteXY_Handler();
        delay(30);
        RemoteXY_Handler();
        delay(30);
        RemoteXY_Handler();
        delay(30);

  
}