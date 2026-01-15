/*
Remote Pressure Monitoring System - Jan 2025
Production DAQ with Dual PID + Custom Filters
*/

#include <Wire.h>
#include <PID_v1.h>
#include <HX711.h>
#include <Adafruit_BMP085.h>
#include <EEPROM.h>

#define PRESS_ADDR  0x77
#define STRAIN_DT   3
#define STRAIN_SCK  2
#define PUMP_PIN    9
#define CAL_BUTTON  4
#define LED_PIN     13

HX711 strain;
Adafruit_BMP085 bmp;
// TOP OF FILE - after #includes
double pressInput = 0.0;
double pressOutput = 0.0;
double pressSetpoint = 1013.25;
double strainInput = 0.0;
double strainOutput = 0.0;
double strainSetpoint = 0.0;

// THEN PID lines (correct):
PID pressPID(&pressInput, &pressOutput, &pressSetpoint, 2.1, 4.9, 1.2, DIRECT);
PID strainPID(&strainInput, &strainOutput, &strainSetpoint, 3.8, 8.2, 1.7, DIRECT);


// Custom filter state
float pressEMA = 1013.25, strainEMA = 0.0;
float pressBuffer[15], strainBuffer[15];
int bufIdx = 0;
float kalmanP[2][2] = {{0.001, 0}, {0, 0.003}};
float kalmanStrain = 0, kalmanBias = 0;

// System state
unsigned long lastUpdate = 0, sampleCount = 0;
bool calibrating = false;
float baselinePress = 1013.25, baselineStrain = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(CAL_BUTTON, INPUT_PULLUP);
  
  if (!bmp.begin()) {
    Serial.println("BMP085 ERROR");
    while (1) digitalWrite(LED_PIN, HIGH);
  }
  
  strain.begin(STRAIN_DT, STRAIN_SCK);
  strain.set_scale(-12850.6);
  strain.tare(20);
  
  pressPID.SetMode(AUTOMATIC);
  pressPID.SetOutputLimits(0, 255);
  strainPID.SetMode(AUTOMATIC);
  strainPID.SetOutputLimits(-2.5, 2.5);
  
  Serial.println("Remote Monitor v1.0 READY");
  digitalWrite(LED_PIN, HIGH);
}

float emaFilter(float input, float &state, float alpha) {
  state = alpha * input + (1 - alpha) * state;
  return state;
}

float firFilter(float* buffer, int idx) {
  float sum = 0;
  for (int i = 0; i < 15; i++) {
    sum += buffer[(idx + i) % 15];
  }
  return sum / 15.0;
}

void kalmanUpdate(float meas, float &angle, float &bias, float (&P)[2][2]) {
  float y = meas - angle;
  float S = 0.001 + 0.03;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;
  
  angle += K0 * y;
  bias += K1 * y;
  
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];
  P[0][0] = P00_temp - K0 * P00_temp;
  P[1][0] = P[1][0] - K1 * P00_temp;
  P[0][1] = P01_temp - K0 * P[0][1];
  P[1][1] += 0.003;
}

void readSensors() {
  // Pressure
  float rawPress = bmp.readPressure() / 100.0;
  float rawTemp = bmp.readTemperature();
  
  pressEMA = emaFilter(rawPress, pressEMA, 0.08);
  pressBuffer[bufIdx] = pressEMA;
  pressInput = firFilter(pressBuffer, bufIdx);
  
  // Strain
  strain.power_up();
  float rawStrain = strain.get_units(8);
  strain.power_down();
  
  kalmanUpdate(rawStrain, kalmanStrain, kalmanBias, kalmanP);
  strainEMA = emaFilter(kalmanStrain, strainEMA, 0.12);
  strainBuffer[bufIdx] = strainEMA;
  strainInput = firFilter(strainBuffer, bufIdx);
  
  bufIdx = (bufIdx + 1) % 15;
}

void handleCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("SET_PRESS:")) {
      pressSetpoint = cmd.substring(9).toFloat();
      Serial.print("OK_PRESS:"); Serial.println(pressSetpoint, 2);
    }
    if (cmd.startsWith("SET_STRAIN:")) {
      strainSetpoint = cmd.substring(11).toFloat() / 1e6;
      Serial.print("OK_STRAIN:"); Serial.println(strainSetpoint * 1e6, 1);
    }
    if (cmd == "CALIB") {
      calibrating = true;
      baselinePress = pressInput;
      baselineStrain = strainInput;
      Serial.println("CALIB_OK");
    }
    if (cmd == "ZERO") {
      strain.tare(25);
      Serial.println("ZERO_OK");
    }
  }
  
  if (digitalRead(CAL_BUTTON) == LOW) {
    calibrating = true;
    delay(200);
  }
}

void loop() {
  readSensors();
  pressPID.Compute();
  strainPID.Compute();
  
  analogWrite(PUMP_PIN, (int)pressOutput);
  
  handleCommands();
  
  if (calibrating && sampleCount % 20 == 0) {
    calibrating = false;
    Serial.print("BASELINE:P"); Serial.print(baselinePress, 2);
    Serial.print(" S"); Serial.println(baselineStrain * 1e6, 1);
  }
  
  if (millis() - lastUpdate > 300) {
    Serial.print("DATA:");
    Serial.print(pressInput, 2); Serial.print(",");
    Serial.print(strainInput * 1e6, 1); Serial.print(",");
    Serial.print(bmp.readTemperature(), 1); Serial.print(",");
    Serial.print(pressSetpoint, 2); Serial.print(",");
    Serial.print((int)pressOutput);
    Serial.println();
    
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    lastUpdate = millis();
    sampleCount++;
  }
}
