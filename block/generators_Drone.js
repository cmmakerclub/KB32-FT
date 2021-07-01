module.exports = function (Blockly) {
  "use strict";

  let remotexy_username = "";
  let remotexy_password = "";

  Blockly.JavaScript["remotexybegin"] = function (block) {
    var text_remotexy_username = block.getFieldValue("USERNAME");
    var text_remotexy_password = block.getFieldValue("PASSWORD");
    var text_remotexy_port = block.getFieldValue("PORT");
    //var text_mqtt_client_id = block.getFieldValue("MQTT_CLIENT_ID");

    remotexy_username = text_remotexy_username;
    remotexy_password = text_remotexy_password;

    // TODO: Assemble JavaScript into code variable.
    var code = `
    #EXTINC #define REMOTEXY_MODE__ESP32CORE_WIFI_POINT #END
    #EXTINC #include <RemoteXY.h> #END
    #EXTINC #define REMOTEXY_WIFI_SSID "${text_remotexy_username}" #END
    #EXTINC #define REMOTEXY_WIFI_PASSWORD "${text_remotexy_password}" #END
    #EXTINC #define REMOTEXY_SERVER_PORT ${text_remotexy_port} #END

    #FUNCTION
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

    #END

    #SETUP
    // RemoteXY connection settings 
    RemoteXY_Init();
    delay(100);

    #END
    \n
    `;
    return code;
  };

  Blockly.JavaScript['remotexyrun'] = function (block) {
    // TODO: Assemble JavaScript into code variable.
    var code =
      `
      RemoteXY_Handler();
      delay(30);
      RemoteXY_Handler();
      delay(30);
      RemoteXY_Handler();
      delay(30);
\n
`;
    return code;
  };

  Blockly.JavaScript['remotemarun'] = function (block) {
    // TODO: Assemble JavaScript into code variable.
    var code =
      `
      remoteReceiveData();
\n
`;
    return code;
  };

  Blockly.JavaScript["remotemabegin"] = function (block) {
    // TODO: Assemble JavaScript into code variable.
    var code =
      `
      #EXTINC #include "VL53L0X.h" #END
      #EXTINC #include "MPU6050.h" #END
      #EXTINC #include "FreeRTOS.h" #END
      #EXTINC #include "MadgwickAHRS.h" #END
      #EXTINC #include "Battery.h" #END
      #EXTINC #include <WiFiUDP.h> #END
      #EXTINC #include <EEPROM.h> #END

      #EXTINC #define ARDUINO_RUNNING_CORE 1 #END
      #EXTINC #define DEFAULT_SSID_LENGTH 16 #END

#VARIABLE

#END

#FUNCTION

struct st_t {
  long lo;
  byte by;
  double db;
};

typedef struct {
  int8_t startByte;
  char ssid[DEFAULT_SSID_LENGTH];
} Alive;

typedef struct {
  int8_t startByte;
  float height;
} ToFHeight;

typedef struct {
  int8_t startByte;
  float roll_kp;
  float roll_ki;
  float roll_kd;
  float pitch_kp;
  float pitch_ki;
  float pitch_kd;
  float yaw_kp;
  float yaw_ki;
  float yaw_kd;
  float height_kp;
  float height_ki;
  float height_kd;
  char ssid[DEFAULT_SSID_LENGTH];
} TuningData;

typedef struct {
  int8_t startByte;
  int16_t roll_kp;
  int16_t roll_ki;
  int16_t roll_kd;
  int16_t pitch_kp;
  int16_t pitch_ki;
  int16_t pitch_kd;
  int16_t yaw_kp;
  int16_t yaw_ki;
  int16_t yaw_kd;
  int16_t height_kp;
  int16_t height_ki;
  int16_t height_kd;
  char ssid[DEFAULT_SSID_LENGTH];
} SaveCalibration;

typedef struct {
  int8_t startByte;
  int16_t joyStickLeftX;
  int16_t joyStickLeftY;
  int16_t joyStickRightX;
  int16_t joyStickRightY;
  int8_t controlMode;
  char ssid[DEFAULT_SSID_LENGTH];
} JoyStickControl;

IPAddress remoteIP;
unsigned int remotePort;
WiFiUDP udp;
// WiFiUDP udpSend;
byte data[512] = {0};
byte dataRollPitchYaw[512] = {0};
byte bdata[4] = {0};
byte bdataHeight[64] = {0};
byte bdataCalibration[512] = {0};
byte bdataAlive[64] = {0};

uint32_t chipId = 0;
unsigned int localPort = 12345;
unsigned int localPortSend = 12346;
char accessPointName[DEFAULT_SSID_LENGTH] = {'\\0'};
char defaultESPWiFiName[DEFAULT_SSID_LENGTH] = {'\\0'};
char accessPointPassword[DEFAULT_SSID_LENGTH] = {'\\0'};

bool isSSID(char* ssid);
void loadTuningData();
String readEEPROM(int index, int length);
void saveTuningData(int i);
int writeEEPROM(int index, String text);
String intToString(int value, int length);
String floatToString(float value, int length, int decimalPalces);
String hexToString(byte value);

// global
int8_t c_joystick_1_x = 0;  // =-100..100 x-coordinate joystick position
int8_t c_joystick_1_y = 0;  // =-100..100 y-coordinate joystick position
int8_t c_joystick_2_x = 0;  // =-100..100 x-coordinate joystick position
int8_t c_joystick_2_y = 0;  // =-100..100 y-coordinate joystick position
int8_t c_control_mode = 0;
// end custom

#define M1 0
#define M2 1
#define M3 2
#define M4 3

#define LEDC_TIMER_11_BIT 11
#define LEDC_BASE_FREQ 16000
#define sampleFreq 200.0f  // 200 hz sample rate!
#define GYROSCOPE_SENSITIVITY 16.4f

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
uint8_t hh;
uint16_t bb;

Madgwick filter;

MPU6050 accelgyro;
// MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

volatile float Error_yaw = 0, Errer_pitch = 0, Error_roll = 0;  // States Error
volatile float Sum_Error_yaw = 0, Sum_Error_pitch = 0,
               Sum_Error_roll = 0;  // Sum of error
volatile float D_Error_yaw = 0, D_Error_pitch = 0,
               D_Error_roll = 0;  // error dot
volatile float Del_yaw = 0, Del_pitch = 0,
               Del_roll = 0;  // Delta states value for rotate axis
volatile float t_compensate = 0;
volatile float T_center_minus = 0;
volatile float y_roll = 0, y_pitch = 0, y0_roll = 0, y0_pitch = 0;
volatile float rMat[3][3] = {0};

float Kp_roll = 8.75;
float Ki_roll = 2;
float Kd_roll = 4;

float Kp_pitch = 8.75;
float Ki_pitch = 2;
float Kd_pitch = 4;

float Kp_yaw = 6;
float Ki_yaw = 1;
float Kd_yaw = 0;

float Kp_T = 2.5;
float Ki_T = 0.1;
float Kd_T = 1.0;

float calaccx = 0;
float calaccy = 0;
float calaccz = 0;
float calgyx = 0;
float calgyy = 0;
float calgyz = 0;

float accx = 0;
float accy = 0;
float accz = 0;
float gyx = 0;
float gyy = 0;
float gyz = 0;

volatile float Ref_altitude = 1, Error_T, Sum_Error_T, D_Error_T, Buf_D_Error_T;

volatile float motor_A = 0, motor_B = 0, motor_C = 0, motor_D = 0,
               T_center = 0;  // Motors output value

SemaphoreHandle_t Mutex_i2c;

void TaskBlink(void* pvParameters);
void TaskAnalogReadA3(void* pvParameters);

void motor_drive(uint8_t channel, int32_t value, int32_t valueMax = 2048) {
  if (value < 0) value = 0;
  uint32_t duty = min(value, valueMax);
  ledcWrite(channel, duty);
}

float lpf(float alfa, float new_data, float prev_data) {
  float output = prev_data + (alfa * (new_data - prev_data));
  return output;
}

Battery batt = Battery(2800, 4200, vbatt_pin);
float vv_batt;
float lpf(float x, float y) {
  return y + 0.25f * (x - y);
}

unsigned long battery_time1 = 0;
unsigned long battery_time2 = 0;
unsigned long height_time1 = 0;
unsigned long height_time2 = 0;
unsigned long alive_time1 = 0;
unsigned long alive_time2 = 0;
unsigned long check_alive_time1 = 0;
unsigned long check_alive_time2 = 0;

float batteryLevelBuffer1 = 0;
float batteryLevelBuffer2 = 0;
float batteryLevelBuffer3 = 0;
uint8_t batteryLevelCount1 = 0;
uint8_t batteryLevelCount2 = 0;
uint8_t batteryLevelCount3 = 0;

bool isAlive = false;

void alive() {
  Alive _alive = {0};
  _alive.startByte = 9;
  sprintf(accessPointName, "KB32FT-%lu", chipId);
  String myssid = String("KB32FT-") + String(chipId);
  myssid.toCharArray(_alive.ssid, DEFAULT_SSID_LENGTH);
  accessPointName[DEFAULT_SSID_LENGTH - 1] = '\\0';
  memcpy(bdataAlive, &_alive, sizeof(Alive));

  udp.beginPacket(udp.remoteIP(), localPortSend);
  udp.write(bdataAlive, sizeof(Alive));
  udp.endPacket();
}

bool signalConnect() {
  check_alive_time2 = millis();
  alive_time2 = millis();

  if (alive_time2 - alive_time1 > 1000) {
    // response to remote app
    alive();
    alive_time1 = millis();
  }

  if (check_alive_time2 - check_alive_time1 > 3000) {
    // if lost signal more than 3 second, return disconnect.
    isAlive = false;
  }

  return isAlive;
}

// the setup function runs once when you press reset or power the board

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

  /** | ACCELEROMETER | GYROSCOPE
    DLPF_CFG | Bandwidth | Delay | Bandwidth | Delay | Sample Rate
    -------- - +---------- - +-------- + ---------- - +-------- + ------------ -
    0 | 260Hz | 0ms | 256Hz | 0.98ms | 8kHz
    1 | 184Hz | 2.0ms | 188Hz | 1.9ms | 1kHz
    2 | 94Hz | 3.0ms | 98Hz | 2.8ms | 1kHz
    3 | 44Hz | 4.9ms | 42Hz | 4.8ms | 1kHz
    4 | 21Hz | 8.5ms | 20Hz | 8.3ms | 1kHz
    5 | 10Hz | 13.8ms | 10Hz | 13.4ms | 1kHz
    6 | 5Hz | 19.0ms | 5Hz | 18.6ms | 1kHz
  */
  accelgyro.setDLPFMode(2);

  accelgyro.setRate(4);  // 1000 hz /(1+4) = 200 hz

  vTaskDelay(100);

  // use the code below to change accel/gyro offset values

  accelgyro.setXAccelOffset(accx);
  accelgyro.setYAccelOffset(accy);
  accelgyro.setZAccelOffset(accz);

  accelgyro.setXGyroOffset(gyx);
  accelgyro.setYGyroOffset(gyy);
  accelgyro.setZGyroOffset(gyz);

  // Serial.println("Updating internal sensor offsets...");
  //      accelgyro.CalibrateAccel(20);
  //      accelgyro.CalibrateGyro(20);
  //      accelgyro.PrintActiveOffsets();
  //      int16_t calmpudata[6];
  //      accelgyro.PrintActiveOffsetsMA(calmpudata);
  //
  //      Serial.println(calmpudata[0]);
  //      Serial.println(calmpudata[1]);
  //      Serial.println(calmpudata[2]);
  //      Serial.println(calmpudata[3]);
  //      Serial.println(calmpudata[4]);
  //      Serial.println(calmpudata[5]);

  // initialize digital LED_BUILTIN on pin 13 as an output.

  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful"
                                            : "MPU6050 connection failed");
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

  for (;;)  // A Task shall never return or exit.
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

    x1 = (float)c_joystick_1_x / 100.0f;
    x3 = (float)c_joystick_2_x / 100.0f;
    x4 = (float)c_joystick_2_y / 100.0f;

    float roll_f = x3 * 9;
    float pitch_f = x4 * 9;
    // T_center = x2 * 2000;
    heading_speed = -x1 * 135;

    pitch = lpf(0.04f, pitch_f, pitch);
    roll = lpf(0.04f, roll_f, roll);

    Error_roll = (float)roll - ((float)_roll);
    Errer_pitch = (float)pitch - ((float)_pitch);
    Error_yaw = ((float)heading_speed - ((float)_heading_speed));

    if (c_control_mode == 0) T_center = (float)c_joystick_1_y * 20.0f;

    //    Serial.println(T_center);

    if (T_center > 500) {
      Sum_Error_yaw = constrain((Sum_Error_yaw + (Error_yaw / sampleFreq)),
                                -100.0f, 100.0f);
      Sum_Error_pitch = constrain(
          (Sum_Error_pitch + (Errer_pitch / sampleFreq)), -100.0f, 100.0f);
      Sum_Error_roll = constrain((Sum_Error_roll + (Error_roll / sampleFreq)),
                                 -100.0f, 100.0f);
    }

    // D_Error_yaw = (Error_yaw - Buf_D_Error_yaw) * sampleFreq;
    // D_Error_pitch = (Errer_pitch - Buf_D_Errer_pitch) * sampleFreq;
    // D_Error_roll = (Error_roll - Buf_D_Error_roll) * sampleFreq;

    D_Error_yaw = 0;
    D_Error_pitch = (pitch - Buf_D_Errer_pitch) * sampleFreq - gyf;
    D_Error_roll = (roll - Buf_D_Error_roll) * sampleFreq - gxf;

    Buf_D_Error_roll = roll;
    Buf_D_Errer_pitch = pitch;

    Del_yaw = (Kp_yaw * Error_yaw) + (Ki_yaw * Sum_Error_yaw) +
              constrain((Kd_yaw * D_Error_yaw), -1500, 1500);
    Del_pitch = (Kp_pitch * Errer_pitch) + (Ki_pitch * Sum_Error_pitch) +
                constrain((Kd_pitch * D_Error_pitch), -1500, 1500);
    Del_roll = (Kp_roll * Error_roll) + (Ki_roll * Sum_Error_roll) +
               constrain((Kd_roll * D_Error_roll), -1500, 1500);

    if (T_center > 100) {
      motor_A = T_center + Del_pitch + Del_roll + Del_yaw;
      motor_B = T_center + Del_pitch - Del_roll - Del_yaw;
      motor_C = T_center - Del_pitch - Del_roll + Del_yaw;
      motor_D = T_center - Del_pitch + Del_roll - Del_yaw;

      motor_drive(M1, motor_A);
      motor_drive(M2, motor_B);
      motor_drive(M3, motor_C);
      motor_drive(M4, motor_D);

    } else {
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
  // float high;

  xSemaphoreTake(Mutex_i2c, portMAX_DELAY);
  sensor.init();
  sensor.setTimeout(20000);  // 20000 us , 50 hz
  sensor.startContinuous(20);
  xSemaphoreGive(Mutex_i2c);

  uint32_t start_time = xTaskGetTickCount();
  for (;;) {
    digitalWrite(R_led, HIGH);
    xSemaphoreTake(Mutex_i2c, portMAX_DELAY);
    high = constrain(sensor.readRangeContinuousMillimeters(), 0, 1500);  //

    height_time2 = millis();

    if (height_time2 - height_time1 > 500) {
      ToFHeight tofHeight;
      tofHeight.startByte = 3;
      tofHeight.height = high;
      memcpy(bdataHeight, &tofHeight, sizeof(ToFHeight));

      udp.beginPacket(udp.remoteIP(), localPortSend);
      udp.write(bdataHeight, sizeof(ToFHeight));
      udp.endPacket();

      height_time1 = millis();
    }

    xSemaphoreGive(Mutex_i2c);

    float x2 = (float)c_joystick_1_y / 100.0f;

    if (c_control_mode == 0) {
      Ref_altitude = 0;
    } else {
      Ref_altitude = constrain(Ref_altitude + x2 * 5, 0, 1000);
      if (c_joystick_1_y <= -100) Ref_altitude = 0;
      if (isAlive == 0) Ref_altitude = 0;
      //if (battery_level <= 3300) Ref_altitude = 0;

      high = constrain(
          high * cosf(filter.getRollRadians()) * cosf(filter.getPitchRadians()),
          0, 1500);

      Buf_D_Error_T = Error_T;
      Error_T = (float)Ref_altitude - ((float)high);

      if (Ref_altitude > 150) {
        Sum_Error_T =
            constrain((Sum_Error_T + (Error_T / 50)), -3000.0f, 3000.0f);
      }

      D_Error_T = (Error_T - Buf_D_Error_T) * 50;

      if (Ref_altitude > 150) {
        T_center = (Kp_T * Error_T) + (Ki_T * Sum_Error_T) +
                   constrain((Kd_T * D_Error_T), 0, 1500);
      } else {
        T_center = 0;
      }
    }

    battery_time2 = millis();

    //    uint16_t battery_level = analogRead(vbatt_pin) * 2 * 3600 / 4095;
    //    vv_batt = lpf(constrain(batt.level(battery_level), 0, 100), vv_batt);
    battery_level = lpf(analogRead(vbatt_pin) * 2 * 3600 / 4095, battery_level);
    vv_batt = constrain(batt.level(battery_level), 0, 100);

    if (battery_time2 - battery_time1 > 2000) {
      bdata[0] = 1;
      bdata[1] = vv_batt;

      udp.beginPacket(udp.remoteIP(), localPortSend);
      udp.write(bdata, 2);
      udp.endPacket();
      battery_time1 = millis();
    }

    digitalWrite(R_led, LOW);
    vTaskDelayUntil(&start_time, 20);
  }
}

bool isSSID(char* ssid) {
  return (strcmp(ssid, accessPointName) == 0 ||
          strcmp(ssid, defaultESPWiFiName) == 0 || strlen(ssid) == 0);
}

String readEEPROM(int index, int length) {
  String text = "";
  char ch = 1;

  for (int i = index; (i < (index + length)) && ch; ++i) {
    if (ch = EEPROM.read(i)) {
      text.concat(ch);
    }
  }

  return text;
}

void EEPROM_Write(float* num, int MemPos) {
  byte ByteArray[4];
  memcpy(ByteArray, num, 4);
  for (int x = 0; x < 4; x++) {
    EEPROM.write((MemPos * 4) + x, ByteArray[x]);
  }
}

void EEPROM_Read(float* num, int MemPos) {
  byte ByteArray[4];
  for (int x = 0; x < 4; x++) {
    ByteArray[x] = EEPROM.read((MemPos * 4) + x);
  }
  memcpy(num, ByteArray, 4);
}

void remoteReceiveData() {
  signalConnect();
  int numberOfBytes = udp.parsePacket();

  if (numberOfBytes > 0) {
    udp.read(data, numberOfBytes);
    remoteIP = udp.remoteIP();
    remotePort = udp.remotePort();

    if (data[0] == 0x00) {
      //      Serial.println(">>> pad controller");
      JoyStickControl joyStickData = {0};
      memcpy(&joyStickData, data, sizeof(JoyStickControl));
      joyStickData.ssid[DEFAULT_SSID_LENGTH - 1] = '\\0';

      if (isSSID(joyStickData.ssid)) {
        //        Serial.println("!!!! checksum match !!!!");
        c_joystick_1_x = joyStickData.joyStickLeftX;
        c_joystick_1_y = joyStickData.joyStickLeftY;
        c_joystick_2_x = joyStickData.joyStickRightX;
        c_joystick_2_y = joyStickData.joyStickRightY;
        c_control_mode = joyStickData.controlMode;
      }
    } else if (data[0] == 0x01) {
      Serial.println(">>> read EEPROM");
      float resultf = 0;

      TuningData tuningData;
      tuningData.startByte = 2;
      // roll
      EEPROM_Read(&resultf, 1);
      tuningData.roll_kp = resultf;
      EEPROM_Read(&resultf, 2);
      tuningData.roll_ki = resultf;
      EEPROM_Read(&resultf, 3);
      tuningData.roll_kd = resultf;
      // pitch
      EEPROM_Read(&resultf, 4);
      tuningData.pitch_kp = resultf;
      EEPROM_Read(&resultf, 5);
      tuningData.pitch_ki = resultf;
      EEPROM_Read(&resultf, 6);
      tuningData.pitch_kd = resultf;
      // yaw
      EEPROM_Read(&resultf, 7);
      tuningData.yaw_kp = resultf;
      EEPROM_Read(&resultf, 8);
      tuningData.yaw_ki = resultf;
      EEPROM_Read(&resultf, 9);
      tuningData.yaw_kd = resultf;
      // height
      EEPROM_Read(&resultf, 10);
      tuningData.height_kp = resultf;
      EEPROM_Read(&resultf, 11);
      tuningData.height_ki = resultf;
      EEPROM_Read(&resultf, 12);
      tuningData.height_kd = resultf;

      sprintf(accessPointName, "KB32FT-%lu", chipId);
      String myssid = String("KB32FT-") + String(chipId);
      myssid.toCharArray(tuningData.ssid, DEFAULT_SSID_LENGTH);
      accessPointName[DEFAULT_SSID_LENGTH - 1] = '\\0';

      Serial.println(">>> load calibration");
      memcpy(dataRollPitchYaw, &tuningData, sizeof(TuningData));
      udp.beginPacket(udp.remoteIP(), localPortSend);
      udp.write(dataRollPitchYaw, sizeof(TuningData));
      udp.endPacket();

    } else if (data[0] == 0x02) {
      SaveCalibration saveCalibration = {0};
      memcpy(&saveCalibration, data, sizeof(SaveCalibration));
      saveCalibration.ssid[DEFAULT_SSID_LENGTH - 1] = '\\0';

      if (isSSID(saveCalibration.ssid)) {
        Serial.println(">>> save calibration");
        Kp_roll = saveCalibration.roll_kp / 100.00f;
        Ki_roll = saveCalibration.roll_ki / 100.00f;
        Kd_roll = saveCalibration.roll_kd / 100.00f;

        Kp_pitch = saveCalibration.pitch_kp / 100.00f;
        Ki_pitch = saveCalibration.pitch_ki / 100.00f;
        Kd_pitch = saveCalibration.pitch_kd / 100.00f;

        Kp_yaw = saveCalibration.yaw_kp / 100.00f;
        Ki_yaw = saveCalibration.yaw_ki / 100.00f;
        Kd_yaw = saveCalibration.yaw_kd / 100.00f;

        Kp_T = saveCalibration.height_kp / 100.00f;
        Ki_T = saveCalibration.height_ki / 100.00f;
        Kd_T = saveCalibration.height_kd / 100.00f;

        Serial.println(">>> begin write eeprom");
        EEPROM_Write(&Kp_roll, 1);
        EEPROM_Write(&Ki_roll, 2);
        EEPROM_Write(&Kd_roll, 3);

        EEPROM_Write(&Kp_pitch, 4);
        EEPROM_Write(&Ki_pitch, 5);
        EEPROM_Write(&Kd_pitch, 6);

        EEPROM_Write(&Kp_yaw, 7);
        EEPROM_Write(&Ki_yaw, 8);
        EEPROM_Write(&Kd_yaw, 9);

        EEPROM_Write(&Kp_T, 10);
        EEPROM_Write(&Ki_T, 11);
        EEPROM_Write(&Kd_T, 12);
        EEPROM.commit();
        Serial.println(">>> write eeprom successfully.");
        byte res[1];
        res[0] = 4;
        udp.beginPacket(udp.remoteIP(), localPortSend);
        udp.write(res, sizeof(1));
        udp.endPacket();
      }
    } else if (data[0] == 0x09) {
      isAlive = true;
      alive_time1 = millis();
      check_alive_time1 = millis();
    }
  }
}
#END

#SETUP

delay(100);
tft.setmode(1);
tft.fillScreen(0x0);

// tft.setTextFont(GLCD);
tft.setTextSize(2);
tft.setCursor(25, 0);
tft.setTextColor(0xf800);
tft.println(String("Calibrate"));

// tft.setTextFont(GLCD);
tft.setTextSize(2);
tft.setCursor(0, 30);
tft.setTextColor(0xffff);
tft.print(String("Press S1"));
delay(1000);
tft.setTextSize(2);
tft.setCursor(0, 30);
tft.setTextColor(0xffff);
tft.print(String("Press S1."));
delay(1000);
tft.setTextSize(2);
tft.setCursor(0, 30);
tft.setTextColor(0xffff);
tft.print(String("Press S1.."));
delay(1000);
tft.setTextSize(2);
tft.setCursor(0, 30);
tft.setTextColor(0xffff);
tft.print(String("Press S1..."));
delay(1000);

// initialize serial communication at 115200 bits per second:
Serial.begin(115200);
// init eeprom
EEPROM.begin(512);

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

if (digitalRead(S1_pin) == 0) {
  tft.setTextSize(2);
  tft.setCursor(0, 30);
  tft.setTextColor(0xfb20);
  tft.print(String("Press S1...OK"));
  // Serial.println("Updating internal sensor Cal...");
  accelgyro.CalibrateAccel(20);
  accelgyro.CalibrateGyro(20);
  int16_t calmpudata[6];
  accelgyro.PrintActiveOffsetsMA(calmpudata);
  calaccx = calmpudata[0];
  calaccy = calmpudata[1];
  calaccz = calmpudata[2];
  calgyx = calmpudata[3];
  calgyy = calmpudata[4];
  calgyz = calmpudata[5];
  Serial.println("calibrateGet......");
  Serial.println(calaccx);
  Serial.println(calaccy);
  Serial.println(calaccz);
  Serial.println(calgyx);
  Serial.println(calgyy);
  Serial.println(calgyz);

  // test write EEPROM
  Serial.println(">>> write EEPROM");
  EEPROM_Write(&calaccx, 13);
  EEPROM_Write(&calaccy, 14);
  EEPROM_Write(&calaccz, 15);
  EEPROM_Write(&calgyx, 16);
  EEPROM_Write(&calgyy, 17);
  EEPROM_Write(&calgyz, 18);
  EEPROM.commit();
  Serial.println(">>> write eeprom successfully.");
  // tft.setTextFont(GLCD);
  tft.setTextSize(2);
  tft.setCursor(0, 60);
  tft.setTextColor(0xffff);
  tft.println(String("Calibrate OK!"));
  delay(2000);
} else {
  Serial.println("No Updating internal sensor cal...");
  tft.setTextSize(2);
  tft.setCursor(0, 60);
  tft.setTextColor(0xffff);
  tft.println(String("No! Calibrate"));
  delay(2000);
}
float calibrate = 0;
Serial.println("calibrateGet......OK");
// ACC
EEPROM_Read(&calibrate, 13);
accx = calibrate;
EEPROM_Read(&calibrate, 14);
accy = calibrate;
EEPROM_Read(&calibrate, 15);
accz = calibrate;
// Gyro
EEPROM_Read(&calibrate, 16);
gyx = calibrate;
EEPROM_Read(&calibrate, 17);
gyy = calibrate;
EEPROM_Read(&calibrate, 18);
gyz = calibrate;

Serial.println(accx);
Serial.println(accy);
Serial.println(accz);
Serial.println(gyx);
Serial.println(gyy);
Serial.println(gyz);
Serial.println(">>> read eeprom successfully.");

tft.fillScreen(0x0);
// tft.setTextFont(GLCD);
tft.setTextSize(3);
tft.setCursor(30, 0);
tft.setTextColor(0xf800);
tft.println(String("Drone"));

// tft.setTextFont(GLCD);
tft.setTextSize(2);
tft.setCursor(0, 30);
tft.setTextColor(0xffff);
tft.println(String("Remote MA OK!"));

battery_time1 = millis();
height_time1 = millis();
alive_time1 = millis();
check_alive_time1 = millis();

xTaskCreatePinnedToCore(angle_controller, "angle_controller", 1024 * 2, NULL,
                        2, NULL, ARDUINO_RUNNING_CORE);

xTaskCreatePinnedToCore(attitude_controller, "attitude_controller", 1024 * 2,
                        NULL, 1, NULL, ARDUINO_RUNNING_CORE);

// start custom
for (int i = 0; i < 17; i = i + 8) {
  chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
}
WiFi.mode(WIFI_STA);
WiFi.disconnect();
delay(100);
byte mac[6] = {0};
WiFi.macAddress(mac);
sprintf(defaultESPWiFiName, "ESP_%#02X%#02X%#02X", mac[3], mac[4], mac[5]);
defaultESPWiFiName[DEFAULT_SSID_LENGTH - 1] = '\\0';
sprintf(accessPointName, "KB32FT-%lu", chipId);
accessPointName[DEFAULT_SSID_LENGTH - 1] = '\\0';
sprintf(accessPointPassword, "KB32FT-%lu", chipId);
accessPointPassword[DEFAULT_SSID_LENGTH - 1] = '\\0';
Serial.println();
Serial.println(String("SSID : ") + accessPointName);
Serial.println(String("PASS : ") + accessPointPassword);
Serial.println(String("PORT : ") + localPort);
Serial.println(String("DEFAULT WIFI : ") + defaultESPWiFiName);
Serial.println();
WiFi.softAP(accessPointName, accessPointPassword);
WiFi.mode(WIFI_AP_STA);
udp.begin(localPort);
// end custom
delay(100);

#END

//remoteReceiveData();

\n
`;
    return code;
  };


  Blockly.JavaScript['dronecal'] = function (block) {
    // TODO: Assemble JavaScript into code variable.
    var code =
      `
      #EXTINC #define ARDUINO_RUNNING_CORE 1 #END
      #EXTINC #include <VL53L0X.h> #END
      #EXTINC #include "MPU6050.h" #END
      #EXTINC #include "FreeRTOS.h" #END
      #EXTINC #include <MadgwickAHRS.h> #END
      #EXTINC #include <Battery.h> #END

#VARIABLE
char print_buf[128];
#END

#FUNCTION

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

volatile float Kp_T = 2.5;
volatile float Ki_T = 0.1;
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
#END


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
\n
`;
    return code;
  };

  Blockly.JavaScript['dronebegin'] = function (block) {
    var setXAccelOffset = block.getFieldValue("setXAccelOffset");
    var setYAccelOffset = block.getFieldValue("setYAccelOffset");
    var setZAccelOffset = block.getFieldValue("setZAccelOffset");
    var setXGyroOffset = block.getFieldValue("setXGyroOffset");
    var setYGyroOffset = block.getFieldValue("setYGyroOffset");
    var setZGyroOffset = block.getFieldValue("setZGyroOffset");

    // TODO: Assemble JavaScript into code variable.
    var code =
      `
      #EXTINC #define ARDUINO_RUNNING_CORE 1 #END
      #EXTINC #include <VL53L0X.h> #END
      #EXTINC #include "MPU6050.h" #END
      #EXTINC #include "FreeRTOS.h" #END
      #EXTINC #include <MadgwickAHRS.h> #END
      #EXTINC #include <Battery.h> #END

#VARIABLE
char print_buf[128];
#END

#FUNCTION

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

volatile float Kp_T = 2.5;
volatile float Ki_T = 0.1;
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

Battery batt = Battery(2800, 4200, vbatt_pin);
float vv_batt;
float lpf(float x, float y) {
  return y + 0.025f * (x - y);
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

  accelgyro.setXAccelOffset(${setXAccelOffset});
  accelgyro.setYAccelOffset(${setYAccelOffset});
  accelgyro.setZAccelOffset(${setZAccelOffset});
  accelgyro.setXGyroOffset(${setXGyroOffset});
  accelgyro.setYGyroOffset(${setYGyroOffset});
  accelgyro.setZGyroOffset(${setZGyroOffset});

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

    float roll_f = x3 * 9;
    float pitch_f = x4 * 9;
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
    high = constrain(sensor.readRangeContinuousMillimeters(), 0, 1500); // 
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
#END


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
\n
`;
    return code;
  };


  Blockly.JavaScript['dronerun'] = function (block) {
    // TODO: Assemble JavaScript into code variable.
    var code =
      `
      delay(30);
      RemoteXY_Handler();
      delay(30);
      RemoteXY_Handler();
      delay(30);
      RemoteXY_Handler();
      delay(30);
\n
`;
    return code;
  };

  Blockly.JavaScript['drone_readbat'] = function (block) {
    return [
      'battery_level',
      Blockly.JavaScript.ORDER_ATOMIC
    ];
  };

  Blockly.JavaScript['sensor_tof'] = function (block) {
    return [
      'high/10',
      Blockly.JavaScript.ORDER_ATOMIC
    ];
  };

};
