
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "REG.h"
#include "wit_c_sdk.h"

/*
Test on ESP32 with JY901S sensor and servo gimbal

 JY901S           ESP32
  VCC      <--->   5V/3.3V
  SCL      <--->   SCL(22)
  SDA      <--->   SDA(21)
  GND      <--->    GND

 Servo            ESP32
  Signal   <--->   GPIO18
  VCC      <--->   5V
  GND      <--->   GND
*/

// Servo configuration
Servo gimbalServo;
//const int SERVO_PIN = 12;          // GPIO pin for servo control
//const int SERVO_MIN_ANGLE = 30;    // Minimum servo angle (degrees)
//const int SERVO_MAX_ANGLE = 180;   // Maximum servo angle (degrees)
//const int LEVEL_ANGLE = 90;        // Servo angle when camera is level

// IMU data update flags
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static char s_cDataUpdate = 0, s_cCmd = 0xff;

// PID Controller parameters
float kp = 2.2;    // Proportional gain
float ki = 0.105;    // Integral gain  
float kd = 0.04;   // Derivative gain

// PID variables
float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;

// Auto-tuning variables
bool autoTuningEnabled = false;
float autoTuneAmplitude = 5.0;
unsigned long autoTuneStartTime = 0;
float maxError = 0;
float oscillationPeriod = 0;
int oscillationCount = 0;
unsigned long lastPeakTime = 0;
bool tuningComplete = true;

// Smoothing filter
const int FILTER_SIZE = 5;
float angleBuffer[FILTER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

// Global angle variables
float fAcc[3], fGyro[3], fAngle[3];

// // Function prototypes
// static void CmdProcess(void);
// static void AutoScanSensor(void);
// static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
// static int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
// static int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);
// static void Delayms(uint16_t ucMs);
// static void ShowHelp(void);
// void CopeCmdData(unsigned char ucData);

// // Gimbal control functions
// void setupServo();
// float applyMovingAverage(float newAngle);
// float pidControl(float targetAngle, float currentAngle);
// int constrainServoAngle(float angle);
// void updateGimbal(float tiltAngle);
// void handleGimbalCommands();
// void performAutoTuning(float error);

void setup() {
  // Initialize I2C and Serial
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  delay(1000);
  
  // Initialize servo
  setupServo();
  
  // Initialize WIT motion sensor
  WitInit(WIT_PROTOCOL_I2C, 0x34);  // Try 0x50 first, common for JY901S
  WitI2cFuncRegister(IICwriteBytes, IICreadBytes);
  WitRegisterCallBack(CopeSensorData);
  WitDelayMsRegister(Delayms);
  
  // Initialize angle buffer
  for(int i = 0; i < FILTER_SIZE; i++) {
    angleBuffer[i] = 0;
  }
  
  Serial.print("\r\n********************** Single Axis Gimbal Controller **********************\r\n");
  Serial.println("Gimbal Commands:");
  Serial.println("  P<value> - Set Kp (e.g., P2.5)");
  Serial.println("  I<value> - Set Ki (e.g., I0.1)");
  Serial.println("  D<value> - Set Kd (e.g., D0.05)");
  Serial.println("  AUTO - Start auto-tuning");
  Serial.println("  STATUS - Show current PID values");
  Serial.println("  RESET - Reset PID controller");
  Serial.println("IMU Commands:");
  ShowHelp();
  
  // Try to find sensor if not responding
  delay(1000);
  WitReadReg(AX, 12);
  delay(100);
  if(s_cDataUpdate == 0) {
    Serial.println("Sensor not found at 0x34, trying auto-scan...");
    AutoScanSensor();
  }
}

void loop() {
  // Read IMU data
  WitReadReg(AX, 12);
  delay(50);  // Reduced delay for better gimbal response
  
  // Handle serial commands
  while (Serial.available()) {
    CopeCmdData(Serial.read());
  }
  
  // Process IMU commands
  CmdProcess();
  
  // Process gimbal commands
  handleGimbalCommands();
  
  // Update gimbal based on IMU data
  if(s_cDataUpdate) {
    // Convert raw data to angles
    for(int i = 0; i < 3; i++) {
      fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
      fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
      fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
    }
    
    // Use Roll angle for single-axis gimbal control
    // Roll (fAngle[0]) represents rotation around X-axis
    updateGimbal(fAngle[0]);

    // Optional: Print sensor data (comment out for cleaner gimbal output)
    /*
    if(s_cDataUpdate & ANGLE_UPDATE) {
      Serial.print("Roll:");
      Serial.print(fAngle[0], 3);
      Serial.print(" Pitch:");
      Serial.print(fAngle[1], 3);
      Serial.print(" Yaw:");
      Serial.println(fAngle[2], 3);
      s_cDataUpdate &= ~ANGLE_UPDATE;
    }
    */

    s_cDataUpdate = 0;
  }
}

void setupServo() {
  gimbalServo.attach(12, 500, 2400);
  gimbalServo.write(90);
  delay(1000);
  Serial.println("Servo initialized at level position (90°)");
}

float applyMovingAverage(float newAngle) {
  angleBuffer[bufferIndex] = newAngle;
  bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
  
  if (!bufferFilled && bufferIndex == 0) {
    bufferFilled = true;
  }
  
  float sum = 0;
  int count = bufferFilled ? FILTER_SIZE : bufferIndex;
  
  for (int i = 0; i < count; i++) {
    sum += angleBuffer[i];
  }
  
  return sum / count;
}

float pidControl(float targetAngle, float currentAngle) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  
  if (deltaTime <= 0) deltaTime = 0.01;
  
  float error = targetAngle - currentAngle;
  
  // Proportional term
  float proportional = kp * error;
  
  // Integral term
  integral += error * deltaTime;
  float integralTerm = ki * integral;
  
  // Derivative term
  float derivative = (error - previousError) / deltaTime;
  float derivativeTerm = kd * derivative;
  
  float output = proportional + integralTerm + derivativeTerm;
  
  previousError = error;
  lastTime = currentTime;
  
  return output;
}

int constrainServoAngle(float angle) {
  if (angle < 45) return 45;
  if (angle > 180) return 180;
  return (int)angle;
}

void updateGimbal(float tiltAngle) {
  float smoothedTilt = applyMovingAverage(tiltAngle);
  float targetAngle = 0;  // Target is level
  float error = targetAngle - smoothedTilt;
  
  // Perform auto-tuning if enabled
  if (autoTuningEnabled && !tuningComplete) {
    performAutoTuning(error);
  }

  
  // Calculate correction using PID controller
  float correction = pidControl(targetAngle, smoothedTilt);
  
  // Calculate required servo angle
  float servoAngle = 90 - correction;
  
  // Constrain to servo limits
  int finalAngle = constrainServoAngle(servoAngle);
  
  // Move servo
  gimbalServo.write(finalAngle);
  
  // Debug output
  Serial.print("Tilt: ");
  Serial.print(smoothedTilt, 2);
  Serial.print("° | Error: ");
  Serial.print(error, 2);
  Serial.print("° | Servo: ");
  Serial.print(finalAngle);
  Serial.print("° | PID: P=");
  Serial.print(kp, 2);
  Serial.print(" I=");
  Serial.print(ki, 3);
  Serial.print(" D=");
  Serial.println(kd, 3);
}

void handleGimbalCommands() {
  static String inputBuffer = "";
  
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        String input = inputBuffer;
        input.trim();
        inputBuffer = "";
        
        // Check for gimbal commands
        if (input.charAt(0) == 'P' && input.length() > 1) {
          float newKp = input.substring(1).toFloat();
          if (newKp >= 0) {
            kp = newKp;
            Serial.print("Kp set to: ");
            Serial.println(kp, 3);
          }
        }
        else if (input.charAt(0) == 'I' && input.length() > 1) {
          float newKi = input.substring(1).toFloat();
          if (newKi >= 0) {
            ki = newKi;
            Serial.print("Ki set to: ");
            Serial.println(ki, 3);
          }
        }
        else if (input.charAt(0) == 'D' && input.length() > 1) {
          float newKd = input.substring(1).toFloat();
          if (newKd >= 0) {
            kd = newKd;
            Serial.print("Kd set to: ");
            Serial.println(kd, 3);
          }
        }
        else if (input.equalsIgnoreCase("STATUS")) {
          Serial.print("Current PID values - P: ");
          Serial.print(kp, 3);
          Serial.print(", I: ");
          Serial.print(ki, 3);
          Serial.print(", D: ");
          Serial.print(kd, 3);
          Serial.print(", Auto-tuning: ");
          Serial.println(autoTuningEnabled ? "ON" : "OFF");
        }
      }
    } else {
      inputBuffer += c;
    }
  }
}

void performAutoTuning(float error) {
  unsigned long currentTime = millis();
  float absError = abs(error);
  
  if (absError > maxError) {
    maxError = absError;
  }
  
  static float lastError = 0;
  static bool peakDetected = false;
  
  if ((lastError < 0 && error > 0) || (lastError > 0 && error < 0)) {
    if (!peakDetected) {
      oscillationCount++;
      
      if (oscillationCount > 1) {
        oscillationPeriod = (currentTime - lastPeakTime) / 1000.0;
      }
      lastPeakTime = currentTime;
      peakDetected = true;
    }
  } else {
    peakDetected = false;
  }
  
  lastError = error;
  
  if (oscillationCount >= 6 && oscillationPeriod > 0) {
    float Ku = kp;
    float Tu = oscillationPeriod * 2;
    
    kp = 0.6 * Ku;
    ki = (2.0 * kp) / Tu;
    kd = (kp * Tu) / 8.0;
    
    autoTuningEnabled = false;
    tuningComplete = true;
    
    Serial.println("Auto-tuning complete!");
    Serial.print("New PID values - P: ");
    Serial.print(kp, 3);
    Serial.print(", I: ");
    Serial.print(ki, 3);
    Serial.print(", D: ");
    Serial.println(kd, 3);
  }
  
  if (currentTime - autoTuneStartTime > 30000) {
    autoTuningEnabled = false;
    Serial.println("Auto-tuning timed out.");
  }
}

// Original IMU functions
void CopeCmdData(unsigned char ucData) {
  static unsigned char s_ucData[50], s_ucRxCnt = 0;
  
  s_ucData[s_ucRxCnt++] = ucData;
  if(s_ucRxCnt<3)return;
  if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
  if(s_ucRxCnt >= 3) {
    if((s_ucData[1] == '\r') && (s_ucData[2] == '\n')) {
      s_cCmd = s_ucData[0];
      memset(s_ucData,0,50);
      s_ucRxCnt = 0;
    }
    else {
      s_ucData[0] = s_ucData[1];
      s_ucData[1] = s_ucData[2];
      s_ucRxCnt = 2;
    }
  }
}

static int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0)
    return 0;

  Wire.requestFrom(dev, length);
  
  unsigned long startTime = millis();
  while (Wire.available() < length) {
    if (millis() - startTime > 100) {
      return 0;
    }
  }

  for (int x = 0; x < length; x++) {
    data[x] = Wire.read();
  }

  return 1;
}

static int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(data, length);
  if (Wire.endTransmission() != 0) {
    return 0;
  }
  return 1;
}

static void ShowHelp(void) {
  Serial.print("IMU Commands:\r\n");
  Serial.print("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
  Serial.print("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
  Serial.print("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
  Serial.print("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
  Serial.print("UART SEND:h\\r\\n   help.\r\n");
}

static void CmdProcess(void) {
  switch(s_cCmd) {
    case 'a': if(WitStartAccCali() != WIT_HAL_OK) Serial.print("\r\nSet AccCali Error\r\n");
      break;
    case 'm': if(WitStartMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
      break;
    case 'e': if(WitStopMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
      break;
    case 'u': if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
      break;
    case 'U': if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
      break;
    case 'h': ShowHelp();
      break;
    default: return;
  }
  s_cCmd = 0xff;
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum) {
  int i;
  for(i = 0; i < uiRegNum; i++) {
    switch(uiReg) {
      case AZ:
        s_cDataUpdate |= ACC_UPDATE;
        break;
      case GZ:
        s_cDataUpdate |= GYRO_UPDATE;
        break;
      case HZ:
        s_cDataUpdate |= MAG_UPDATE;
        break;
      case Yaw:
        s_cDataUpdate |= ANGLE_UPDATE;
        break;
      default:
        s_cDataUpdate |= READ_UPDATE;
        break;
    }
    uiReg++;
  }
}

static void Delayms(uint16_t ucMs) {
  delay(ucMs);
}

static void AutoScanSensor(void) {
  int i, iRetry;
  
  for(i = 0; i < 0x7F; i++) {
    WitInit(WIT_PROTOCOL_I2C, i);
    iRetry = 2;
    do {
      s_cDataUpdate = 0;
      WitReadReg(AX, 3);
      delay(5);
      if(s_cDataUpdate != 0) {
        Serial.print("find 0x");
        Serial.print(i, HEX);
        Serial.print(" addr sensor\r\n");
        return;
      }
      iRetry--;
    }while(iRetry);		
  }
  Serial.print("can not find sensor\r\n");
  Serial.print("please check your connection\r\n");
}
