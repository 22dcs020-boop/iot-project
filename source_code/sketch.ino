#include <WiFi.h>
#include <HTTPClient.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"

// ─────────────────── CLOUD SETTINGS ─────────────────────────
const char* WIFI_SSID  = "Wokwi-GUEST";
const char* WIFI_PASS  = "";
const char* TS_API_KEY = "CW9CQBP1F1BW8C7P"; 
const char* TS_SERVER  = "http://api.thingspeak.com/update";

// I2C pins for the display
#define I2C_SDA 9
#define I2C_SCL 8

// System pins (Using safe ADC pins 4 & 5 to prevent Serial Monitor crashing)
#define PIN_POT_TEMP   4   // P1: Temperature setpoint
#define PIN_POT_FLOW   5   // P2: Flow setpoint
#define PIN_SERVO_TEMP 20  // S1: 3-way valve (temperature)
#define PIN_SERVO_FLOW 26  // S2: 2-way valve (flow)
#define PIN_DHT        21  // T1: Mixed water temperature sensor
#define DHTTYPE DHT22

DHT dht(PIN_DHT, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD 16x2 initialization

Servo servoTemp;
Servo servoFlow;

// --- System physical constants ---
const float T_COLD = 15.0;     // Input cold water temperature
const float T_HOT = 65.0;      // Input hot water temperature
const float T_CRITICAL = 50.0; // Scald protection threshold (burn limit)

// --- PI controller parameters ---
float Kp = 4.0; // Proportional coefficient
float Ki = 0.5; // Integral coefficient
float integral_sum = 0; // Accumulated error sum

unsigned long last_time = 0;
const unsigned long DT = 2000; // Sampling time (2 seconds)

unsigned long cloudTimer = -15000; // Force immediate cloud upload on boot

// Global State Variables for ThingSpeak
float T1_actual = 0;
float P1_temp = 0;
int setpoint_Flow = 0;
int servo1_angle = 0;
int currentState = -1; // 0 - Normal, 1 - Flow closed, 2 - Emergency

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.print(F("\n[WIFI] Connecting to Wokwi-GUEST... Please wait 10-15s!\n"));
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("\n[WIFI] Connected!"));
  } else {
    Serial.println(F("\n[WIFI] Failed to connect. Wokwi servers might be busy."));
  }
}

void uploadToThingSpeak() {
  if (millis() - cloudTimer < 15000) return; // ThingSpeak 15s API limit
  cloudTimer = millis();

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) return; // Skip if still no Wi-Fi
  }

  // Build the HTTP request with our 5 tracked variables
  String url = String(TS_SERVER) + "?api_key=" + TS_API_KEY +
               "&field1=" + String(T1_actual, 1) +
               "&field2=" + String(P1_temp, 1) +
               "&field3=" + String(setpoint_Flow) +
               "&field4=" + String(servo1_angle) +
               "&field5=" + String(currentState);

  HTTPClient http;
  http.begin(url);
  int httpCode = http.GET();

  if (httpCode > 0) {
    Serial.print("[THINGSPEAK] Upload Success! HTTP: ");
    Serial.println(httpCode);
  } else {
    Serial.print("[THINGSPEAK] Upload Failed: ");
    Serial.println(http.errorToString(httpCode).c_str());
  }
  http.end();
}

void setup() {
  Serial.begin(115200);
  delay(100);
  
  // I2C initialization for screen on pins 9 and 8
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();
  
  // Set ESP32 ADC to 12-bit (0-4095)
  analogReadResolution(12); 
  
  // SAFE Servo initialization (Prevents Wokwi freezing)
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoTemp.setPeriodHertz(50);
  servoFlow.setPeriodHertz(50);
  
  servoTemp.attach(PIN_SERVO_TEMP);
  servoFlow.attach(PIN_SERVO_FLOW);
  
  dht.begin();
  
  // Starting valve positions (closed / cold water)
  servoTemp.write(0);
  servoFlow.write(0);
  
  // Integrator initialization
  integral_sum = 90.0 / Ki; 
  
  Serial.println("System 'Smart Faucet' started.");
  
  // Initial Wi-Fi connection
  connectWiFi();
}

void loop() {
  unsigned long current_time = millis();
  
  // Execute controller logic with DT step (2 seconds)
  if (current_time - last_time >= DT) {
    last_time = current_time;
    
    // 1. Read flow setpoint (P2)
    int val_P2 = analogRead(PIN_POT_FLOW);
    setpoint_Flow = map(val_P2, 0, 4095, 0, 180); 
    
    // 2. Read temperature setpoint (P1)
    // Setpoint range from 20 to 45 degrees (safe zone)
    int val_P1 = analogRead(PIN_POT_TEMP);
    P1_temp = map(val_P1, 0, 4095, 20, 45); 
    
    // 3. Read actual temperature (T1)
    T1_actual = dht.readTemperature();
    
    if (isnan(T1_actual)) {
      Serial.println("Error reading DHT22 sensor!");
      return; // Skip this control loop step
    }

    int nextState = 0;

    // --- SCALD PROTECTION (Emergency State) ---
    if (T1_actual >= T_CRITICAL) {
      nextState = 2; // Emergency
      servoFlow.write(0);  // Instantly close the water
      servoTemp.write(0);  // Set mixer to extreme cold position
      integral_sum = 0;    // Reset accumulated error
      
      Serial.println("!!! EMERGENCY: SCALD RISK !!! Flow closed.");
    } 
    else {
      // --- PI CONTROL ---
      float e = P1_temp - T1_actual; 
      float U_pre = (Kp * e) + (Ki * (integral_sum + e * (DT / 1000.0)));
      
      // Anti-windup
      if ((U_pre >= 0) && (U_pre <= 180)) {
        integral_sum += e * (DT / 1000.0);
      }
      
      float U = (Kp * e) + (Ki * integral_sum);
      servo1_angle = (int)U;
      
      // Hard constraint for the [0; 180] range
      servo1_angle = constrain(servo1_angle, 0, 180);
      
      // Apply angles to servos
      servoTemp.write(servo1_angle);
      servoFlow.write(setpoint_Flow);

      // Check for closed flow
      if (setpoint_Flow == 0) {
        nextState = 1; // Flow closed
      } else {
        nextState = 0; // Normal
      }
    }

    // --- OUTPUT TO LCD SCREEN ---
    // Clear screen only when state changes
    if (nextState != currentState) {
      lcd.clear();
      currentState = nextState;
    }

    switch (currentState) {
      case 2: // EMERGENCY
        lcd.setCursor(0, 0); lcd.print("! EMERGENCY !T>50");
        lcd.setCursor(0, 1); lcd.print("Flow closed     ");
        break;

      case 1: // FLOW CLOSED
        lcd.setCursor(0, 0); lcd.print("Flow closed     ");
        lcd.setCursor(0, 1); lcd.print("S1: 0deg S2: 0deg"); 
        break;

      case 0: // NORMAL OPERATION
        // Row 1: Temperature (Actual and Setpoint)
        lcd.setCursor(0, 0);
        lcd.print("T:"); lcd.print((int)T1_actual); lcd.print("C ");
        lcd.print("Set:"); lcd.print((int)P1_temp); lcd.print("C  ");
        
        // Row 2: Servo angles
        lcd.setCursor(0, 1);
        lcd.print("S1:"); lcd.print(servo1_angle); lcd.print((char)223); // Degree symbol
        lcd.print(" S2:"); lcd.print(setpoint_Flow); lcd.print((char)223); lcd.print("  ");
        break;
    }
  }

  // Upload to ThingSpeak every 15 seconds independently of the 2-second screen refresh loop
  uploadToThingSpeak();
}
