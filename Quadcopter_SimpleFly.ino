// *************************************************************  Library  ************************************************************* //

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>
#include <MadgwickAHRS.h>
#include <Adafruit_BMP280.h>
#include <MPU6500_WE.h>
// *************************************************************  Parameters & Variables  ************************************************************* //
// Sensors
#define MPU6500_ADDR 0x68

// Motors 
#define M1 1 
#define M2 7
#define M3 5
#define M4 3

// Objects 
MPU6500_WE imu = MPU6500_WE(MPU6500_ADDR);
Adafruit_BMP280 bmp;
Preferences preferences;
Madgwick MadgwickFilter;

const int PWM_FREQ = 400; 
const int PWM_RES  = 14;      
const int MIN_DUTY = 6554; 
const int MAX_DUTY = 13107;

// Timer 
unsigned long lastInnerTime = 0;
unsigned long lastOuterTime = 0;
unsigned long lastBaroTime  = 0;
unsigned long lastDNSTime = 0;

const float innerHz = 400.0;    // Inner loop rate (Madgwick + inner PID)
const float outerHz = 100.0;    // Outer loop rate (angle PID / altitude)
const float baroHz  = 50.0;     // Barometer update rate

const float innerDt = 1.0 / innerHz;
const float outerDt = 1.0 / outerHz;
const float baroDt  = 1.0 / baroHz;

// Initials
float targetRoll = 0 ,targetPitch = 0 ,targetYaw = 0;
float currentRoll = 0 ,currentPitch = 0 ,currentYaw = 0;
float yaw_setpoint = 0.0; // deg
float yaw_rate_target = 0;
float yaw_ref = 0.0f;
float baroAltitude = 0;
float currentAltitude = 0;
float altitude_setpoint = 0.0; // meter
float altitude_rate_target = 0.0;
float altitude_baseline = 0.0;
float max_altitude = 0.0;
float min_altitude = 0.0;
float g = 9.80665; // m/s^2


int baseSpeed = 1300;
int idleSpeed = 1150; 
int integralLimit = 50;
float altitudeIntegralLimit = 50;
float range_altitude = 15;
float maxRateChange = 2; // m/s
float TrimStep = 0.1;
float kff_roll = 0.6; // FeedForward Roll
float kff_pitch = 0.6; // FeedForward Pitch
float kff_yaw = 0.6; // FeedForward Yaw
float kff_altitude = 0.8; // FeedForward Altitude 

float altitude = 0;
float velocityZ = 0;
float previousBaro = 0;

float ax_bias = 0, ay_bias = 0, az_bias = 0;
float gx_bias = 0, gy_bias = 0, gz_bias = 0;

float estAltitude = 0.0f; 
float estVelocityZ = 0.0f; 

// ค่า Gain ในการจูน (ต้องจูนหน้างาน)
// ยิ่งมาก = เชื่อ Baro มาก (แก้ Drift เร็ว แต่ Noise เยอะ)
// ยิ่งน้อย = เชื่อ Accel มาก (นิ่ง สมูท แต่ Drift ง่าย)
float kP_pos = 1.0f;  // แก้ตำแหน่ง (Position correction)
float kP_vel = 0.5f;  // แก้ความเร็ว (Velocity correction)

// PID Control
struct PID_t {
  float P;
  float I;
  float D;
};

PID_t pidRoll_rate, pidPitch_rate, pidYaw_rate, pidAltitude_rate, pidRoll_angle, pidPitch_angle, pidYaw_angle, pidAltitude_m, pidAltitude_height;
float trimRoll, trimPitch, trimYaw, trimAltitude;

// Outer loop PID (deg and m control)
float rollKp_angle = 7.0 ,rollKi_angle = 0.0 ,rollKd_angle = 0.01;  
float rollError_angle, rollPrevError_angle = 0, rollIntegral_angle = 0;
float rollOutput_angle; 

float pitchKp_angle = 7.0 ,pitchKi_angle = 0.0 ,pitchKd_angle = 0.01; 
float pitchError_angle, pitchPrevError_angle = 0, pitchIntegral_angle = 0;
float pitchOutput_angle;

float yawKp_angle = 4.0 ,yawKi_angle = 0.0 ,yawKd_angle = 0.0; 
float yawError_angle, yawPrevError_angle = 0, yawIntegral_angle = 0;
float yawOutput_angle;

float altitudeKp_m = 1.0 ,altitudeKi_m = 0.0 ,altitudeKd_m = 0.0;
float altitudeError_m, altitudePrevError_m = 0, altitudeIntegral_m = 0;
float altitudeOutput_m;

// Inner loop PID (rate control ==> deg/s and m/s)
float rollKp_rate = 1.0 ,rollKi_rate = 0.0 ,rollKd_rate = 0.05; 
float rollError_rate, rollPrevError_rate = 0, rollIntegral_rate = 0;
float rollOutput_rate; 

float pitchKp_rate = 1.0 ,pitchKi_rate = 0.0 ,pitchKd_rate = 0.05; 
float pitchError_rate, pitchPrevError_rate = 0, pitchIntegral_rate = 0;
float pitchOutput_rate;

float yawKp_rate = 3.0 ,yawKi_rate = 0.0 ,yawKd_rate = 0.03; 
float yawError_rate, yawPrevError_rate = 0, yawIntegral_rate = 0;
float yawOutput_rate;

float altitudeKp_rate = 70.0 ,altitudeKi_rate = 0.00 ,altitudeKd_rate = 3.5;
float altitudeError_rate, altitudePrevError_rate = 0, altitudeIntegral_rate = 0;
float altitudeOutput_rate;

// Filters
float gyroX_filtered = 0; 
float gyroY_filtered = 0;
float gyroZ_filtered = 0;
float accX_filtered  = 0;
float accY_filtered  = 0;
float accZ_filtered  = 0;
float alt_filtered  = 0;

// EMA filter
float emaGyroX = 0;
float emaGyroY = 0;
float emaGyroZ = 0;
float emaAccX = 0;
float emaAccY = 0;
float emaAccZ = 0;
float emaAlt = 0;
float emaVZ = 0;

float alphaGyro = 0.8f; 
float alphaAcc  = 0.7f;  
float alphaAlt  = 0.5f;

// Median filter 
float gyroX_buf[9] = {0}, gyroY_buf[9] = {0}, gyroZ_buf[9] = {0};
float accX_buf[9]  = {0}, accY_buf[9]  = {0}, accZ_buf[9]  = {0};
float alt_buf[7]  = {0};
size_t gyroX_idx = 0, gyroY_idx = 0, gyroZ_idx = 0;
size_t accX_idx  = 0, accY_idx  = 0, accZ_idx  = 0;
size_t alt_idx  = 0;

// State
bool swlPressed = false;
bool swrPressed = false;
bool lastArmButton = false;
bool Armed = false;
bool headlessMode = false; 
bool OnFlying = false;
bool initial_altitude = false;
bool initial_yaw = false;
volatile bool calibrateAccelGyroRequested = false;
bool AccelGyroisCalibrating = false;
bool AccelGyrocalibrationDone = false;
bool isBaroUpdated = false;

// ==== Joystick Data ====
int XL, YL, XR, YR;
bool SWL, SWR, ButtonL, ButtonR, webCommand;
// ==== Double Click Detect ====
const unsigned long clickInterval = 300;

// WiFi AP config
String ssid;
const char* password = "12345678";
IPAddress apIP(192,168,10,1);
const byte DNS_PORT = 53;

AsyncWebServer server(80);
DNSServer dnsServer;
bool webServerRunning = false;

// ESP Now 
String getMacAddress() {
  return WiFi.macAddress();
}

typedef struct JoyData {
  int XL, YL;
  bool SWL, ButtonL;
  int XR, YR;
  bool SWR, ButtonR;
  bool webCommand;
} JoyData;

JoyData incomingJoystickData;
uint8_t lastWebCommand = 255;

// *************************************************************  HTML  ************************************************************* //

String htmlForm(const char* message = "") {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html lang="en">
  <head>
    <meta charset="UTF-8">
    <title>SimpleFly Tuner</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <link href="https://fonts.googleapis.com/css2?family=Orbitron:wght@600&display=swap" rel="stylesheet">
    <style>
      * { box-sizing: border-box; }
      body { 
        margin: 0; 
        font-family: 'Orbitron', monospace; 
        background: #0b0f19; 
        color: #e0e0ff; 
        display: flex; 
        justify-content: center; 
        align-items: center; 
        min-height: 100vh; 
        padding: 10px 0; 
        overflow-x: hidden; 
        position: relative; 
      }
      .meteor-container {
        position: fixed;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
        z-index: 0; 
        pointer-events: none; 
      }
      .meteor {
        position: absolute;
        top: -50px; 
        width: 2px;
        height: 2px;
        background: #fff;
        border-radius: 50%;
        box-shadow: 0 0 5px #ffaa00, 0 0 10px #00ccff, 0 0 20px #ffaa00;
        animation: meteor-fall linear infinite;
        opacity: 0;
      }
      .meteor::before {
        content: '';
        position: absolute;
        top: 50%;
        transform: translateY(-50%);
        width: 50px; 
        height: 1px;
        background: linear-gradient(90deg, #ffaa00, transparent);
        right: 2px;
      }
      @keyframes meteor-fall {
        0% { transform: rotate(-35deg) translateX(0); opacity: 1; }
        70% { opacity: 1; }
        100% { transform: rotate(-35deg) translateX(-100vh); opacity: 0; }
      }

      /* --- 3. SPACE DECORATIONS  --- */
      .space-scene {
        position: fixed;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
        z-index: 1; 
        pointer-events: none; 
        overflow: hidden;
      }

      .moon-surface {
        position: absolute;
        bottom: -40px;
        left: -10%;
        width: 120%;
        height: 120px;
        background: #ffaa00; 
        border-radius: 50% 50% 0 0; 
        box-shadow: 0 0 50px rgba(255, 170, 0, 0.3);
      }

      .crater {
        position: absolute;
        background: #cc8800; 
        border-radius: 50%;
        box-shadow: inset 2px 2px 5px rgba(0,0,0,0.3);
      }
      .c1 { width: 60px; height: 20px; bottom: 40px; left: 20%; }
      .c2 { width: 40px; height: 15px; bottom: 20px; right: 25%; }
      .c3 { width: 30px; height: 10px; bottom: 60px; right: 10%; }
      .c4 { width: 80px; height: 25px; bottom: 10px; left: 50%; transform: translateX(-50%); }

      .ufo {
        position: absolute;
        top: 50%;
        left: 10%;
        width: 60px;
        height: 20px;
        background: #00ccff;
        border-radius: 50%;
        box-shadow: 0 0 15px #00ccff;
        animation: float-ufo 4s ease-in-out infinite;
      }
      .ufo::before { 
        content: '';
        position: absolute;
        top: -12px;
        left: 15px;
        width: 30px;
        height: 20px;
        background: rgba(255, 170, 0, 0.9); 
        border-radius: 50% 50% 0 0;
        z-index: -1;
      }
      .ufo::after { 
        content: '';
        position: absolute;
        bottom: -5px;
        left: 10px;
        width: 5px;
        height: 5px;
        border-radius: 50%;
        background: #ffaa00;
        box-shadow: 15px 0 0 #ffaa00, 30px 0 0 #ffaa00;
      }
      @keyframes float-ufo {
        0%, 100% { transform: translateY(0) rotate(-5deg); }
        50% { transform: translateY(-20px) rotate(5deg); }
      }

      .planet {
        position: absolute;
        top: 10%;
        right: 5%;
        width: 60px;
        height: 60px;
        background: linear-gradient(135deg, #ffaa00, #cc8800);
        border-radius: 50%;
        box-shadow: -5px -5px 10px rgba(0,0,0,0.5) inset, 0 0 20px rgba(255,170,0,0.4);
      }
      .ring {
        position: absolute;
        top: 50%;
        left: 50%;
        width: 100px;
        height: 20px;
        border: 4px solid #00ccff; 
        border-radius: 50%;
        transform: translate(-50%, -50%) rotate(-20deg);
        box-shadow: 0 0 10px #00ccff;
      }

      .rocket {
        position: absolute;
        bottom: 30%;
        right: 10%;
        width: 15px;
        height: 40px;
        background: #e0e0ff;
        border-radius: 50% 50% 5px 5px;
        transform: rotate(30deg);
        animation: float-rocket 6s ease-in-out infinite;
      }
      .rocket::before { 
        content: '';
        position: absolute;
        top: 0; left: 0; width: 100%; height: 15px;
        background: #ffaa00;
        border-radius: 50% 50% 0 0;
      }
      .rocket::after { 
        content: '';
        position: absolute;
        bottom: -15px; left: 2px;
        width: 10px; height: 20px;
        background: linear-gradient(#00ccff, transparent);
        border-radius: 50%;
        filter: blur(2px);
      }
      .fin {
        position: absolute;
        bottom: 0;
        width: 8px; height: 15px;
        background: #00ccff;
      }
      .fin.l { left: -8px; border-radius: 10px 0 0 0; }
      .fin.r { right: -8px; border-radius: 0 10px 0 0; }
      @keyframes float-rocket {
        0%, 100% { transform: translateY(0) rotate(30deg); }
        50% { transform: translateY(-30px) rotate(35deg); }
      }

      /* --- 4. PANEL SETUP --- */
      .panel { 
        position: relative;
        z-index: 10; 
        background: rgba(11, 15, 25, 0.95); 
        border: 2px solid #ffaa00; 
        box-shadow: 0 0 20px rgba(255, 170, 0, 0.4), 0 0 40px rgba(0, 204, 255, 0.2); 
        padding: 25px 30px; 
        border-radius: 16px; 
        max-width: 480px; 
        width: 95%; 
        backdrop-filter: blur(4px);
      }
      
      /* --- FONTS & INPUTS --- */
      h2 { font-size: 26px; color: #ffaa00; text-align: center; margin-bottom: 24px; text-shadow: 0 0 10px #ffaa00; }
      h3 { text-align: center; color: #00ccff; margin-top: 24px; border-bottom: 1px solid #ffaa00; padding-bottom: 6px; font-size: 18px; }
      label { display: block; margin-top: 12px; font-size: 14px; color: #aaaaaa; }
      
      input[type="number"], input[type="text"] { 
        width: 100%; padding: 10px; font-size: 15px; margin-top: 4px; border-radius: 8px; border: 1px solid #333; background: #151925; color: #ffaa00; font-family: 'Orbitron', monospace; transition: 0.3s ease; 
      }
      input[type="number"]:focus, input[type="text"]:focus { outline: none; border-color: #00ccff; box-shadow: 0 0 10px #ffaa00; }

      input[type="submit"] { 
        margin-top: 28px; width: 100%; padding: 12px; 
        background: linear-gradient(90deg, #ffaa00, #00ccff, #ffaa00); background-size: 200% auto;
        color: #0b0f19; font-size: 16px; font-weight: bold; border: none; border-radius: 10px; 
        box-shadow: 0 0 15px #ffaa00; cursor: pointer; transition: 0.4s ease; font-family: 'Orbitron', monospace; text-transform: uppercase; animation: glow-move 4s linear infinite;
      }
      @keyframes glow-move { 0% { background-position: 0% center; } 100% { background-position: 200% center; } }
      input[type="submit"]:hover { background-position: right center; transform: scale(1.02); box-shadow: 0 0 20px #00ccff; }

      .msg { text-align: center; font-size: 14px; color: #00ff00; margin-bottom: 12px; text-shadow: 0 0 8px #0f0; }

      .telemetry-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; background: #151925; padding: 14px; border-radius: 8px; margin-top: 8px; border: 1px solid #333; }
      .telemetry-grid div { color: #cccccc; font-size: 12px; }
      .telemetry-grid span { color: #00ccff; font-weight: bold; float: right; font-size: 12px; text-shadow: 0 0 5px #00ccff; }
    </style>
  </head>
  <body>
    <div class="meteor-container"></div>

    <div class="space-scene">
      <div class="planet">
        <div class="ring"></div>
      </div>
      
      <div class="ufo"></div>
      
      <div class="rocket">
        <div class="fin l"></div>
        <div class="fin r"></div>
      </div>

      <div class="moon-surface">
        <div class="crater c1"></div>
        <div class="crater c2"></div>
        <div class="crater c3"></div>
        <div class="crater c4"></div>
      </div>
    </div>

    <div class="panel">
      <h2>SimpleFly Tuner</h2>
)rawliteral";

  if (strlen(message) > 0) {
    html += "<div class='msg'>" + String(message) + "</div>";
  }

  String mac = getMacAddress();
  mac.toUpperCase();

  html += "<div style='text-align:center; margin-bottom:12px; color:#ffaa00; text-shadow: 0 0 8px #ffaa00;'>"
          "MAC Address: <strong>" + mac + "</strong></div>";

  html += R"rawliteral(
      <h3>Telemetry</h3>
      <div class="telemetry-grid">
        <div>Target Roll: <span id="targetRoll">0.00</span></div>
        <div>Current Roll: <span id="currentRoll">0.00</span></div>
        <div>Target Pitch: <span id="targetPitch">0.00</span></div>
        <div>Current Pitch: <span id="currentPitch">0.00</span></div>
        <div>Target Yaw Rate: <span id="targetYaw">0.00</span></div>
        <div>Current Yaw: <span id="currentYaw">0.00</span></div>
        <div>Target Alt Rate: <span id="altitude_rate_target">0.00</span></div>
        <div>Current Altitude: <span id="currentAltitude">0.00</span></div>
      </div>
  )rawliteral";

  html += R"rawliteral(
      <form method="POST" action="/save">
        <h3>PID Roll Rate</h3>
        <label for="pRoll_rate">Kp</label> <input name="pRoll_rate" type="number" step="0.01" value=")rawliteral" + String(pidRoll_rate.P, 2) + R"rawliteral(">
        <label for="iRoll_rate">Ki</label> <input name="iRoll_rate" type="number" step="0.01" value=")rawliteral" + String(pidRoll_rate.I, 2) + R"rawliteral(">
        <label for="dRoll_rate">Kd</label> <input name="dRoll_rate" type="number" step="0.01" value=")rawliteral" + String(pidRoll_rate.D, 2) + R"rawliteral(">
        
        <h3>PID Pitch Rate</h3>
        <label for="pPitch_rate">Kp</label> <input name="pPitch_rate" type="number" step="0.01" value=")rawliteral" + String(pidPitch_rate.P, 2) + R"rawliteral(">
        <label for="iPitch_rate">Ki</label> <input name="iPitch_rate" type="number" step="0.01" value=")rawliteral" + String(pidPitch_rate.I, 2) + R"rawliteral(">
        <label for="dPitch_rate">Kd</label> <input name="dPitch_rate" type="number" step="0.01" value=")rawliteral" + String(pidPitch_rate.D, 2) + R"rawliteral(">
        
        <h3>PID Yaw Rate</h3>
        <label for="pYaw_rate">Kp</label> <input name="pYaw_rate" type="number" step="0.01" value=")rawliteral" + String(pidYaw_rate.P, 2) + R"rawliteral(">
        <label for="iYaw_rate">Ki</label> <input name="iYaw_rate" type="number" step="0.01" value=")rawliteral" + String(pidYaw_rate.I, 2) + R"rawliteral(">
        <label for="dYaw_rate">Kd</label> <input name="dYaw_rate" type="number" step="0.01" value=")rawliteral" + String(pidYaw_rate.D, 2) + R"rawliteral(">

        <h3>PID Altitude Rate (m/s)</h3>
        <label for="pAltitude_rate">Kp</label> <input name="pAltitude_rate" type="number" step="0.01" value=")rawliteral" + String(pidAltitude_rate.P, 2) + R"rawliteral(">
        <label for="iAltitude_rate">Ki</label> <input name="iAltitude_rate" type="number" step="0.01" value=")rawliteral" + String(pidAltitude_rate.I, 2) + R"rawliteral(">
        <label for="dAltitude_rate">Kd</label> <input name="dAltitude_rate" type="number" step="0.01" value=")rawliteral" + String(pidAltitude_rate.D, 2) + R"rawliteral(">
        
        <h3>PID Roll Angle</h3>
        <label for="pRoll_angle">Kp</label> <input name="pRoll_angle" type="number" step="0.01" value=")rawliteral" + String(pidRoll_angle.P, 2) + R"rawliteral(">
        <label for="iRoll_angle">Ki</label> <input name="iRoll_angle" type="number" step="0.01" value=")rawliteral" + String(pidRoll_angle.I, 2) + R"rawliteral(">
        <label for="dRoll_angle">Kd</label> <input name="dRoll_angle" type="number" step="0.01" value=")rawliteral" + String(pidRoll_angle.D, 2) + R"rawliteral(">
        
        <h3>PID Pitch Angle</h3>
        <label for="pPitch_angle">Kp</label> <input name="pPitch_angle" type="number" step="0.01" value=")rawliteral" + String(pidPitch_angle.P, 2) + R"rawliteral(">
        <label for="iPitch_angle">Ki</label> <input name="iPitch_angle" type="number" step="0.01" value=")rawliteral" + String(pidPitch_angle.I, 2) + R"rawliteral(">
        <label for="dPitch_angle">Kd</label> <input name="dPitch_angle" type="number" step="0.01" value=")rawliteral" + String(pidPitch_angle.D, 2) + R"rawliteral(">
        
        <h3>PID Yaw Angle</h3>
        <label for="pYaw_angle">Kp</label> <input name="pYaw_angle" type="number" step="0.01" value=")rawliteral" + String(pidYaw_angle.P, 2) + R"rawliteral(">
        <label for="iYaw_angle">Ki</label> <input name="iYaw_angle" type="number" step="0.01" value=")rawliteral" + String(pidYaw_angle.I, 2) + R"rawliteral(">
        <label for="dYaw_angle">Kd</label> <input name="dYaw_angle" type="number" step="0.01" value=")rawliteral" + String(pidYaw_angle.D, 2) + R"rawliteral(">
        
        <h3>PID Altitude</h3>
        <label for="pAltitude_m">Kp</label> <input name="pAltitude_m" type="number" step="0.01" value=")rawliteral" + String(pidAltitude_m.P, 2) + R"rawliteral(">
        <label for="iAltitude_m">Ki</label> <input name="iAltitude_m" type="number" step="0.01" value=")rawliteral" + String(pidAltitude_m.I, 2) + R"rawliteral(">
        <label for="dAltitude_m">Kd</label> <input name="dAltitude_m" type="number" step="0.01" value=")rawliteral" + String(pidAltitude_m.D, 2) + R"rawliteral(">
        
        <h3>Trim</h3>
        <label for="trimRoll">Trim Roll</label> <input name="trimRoll" type="number" step="0.01" value=")rawliteral" + String(trimRoll, 2) + R"rawliteral(">
        <label for="trimPitch">Trim Pitch</label> <input name="trimPitch" type="number" step="0.01" value=")rawliteral" + String(trimPitch, 2) + R"rawliteral(">
        <label for="trimYaw">Trim Yaw</label> <input name="trimYaw" type="number" step="0.01" value=")rawliteral" + String(trimYaw, 2) + R"rawliteral(">
        <label for="trimAltitude">Trim Altitude</label> <input name="trimAltitude" type="number" step="0.01" value=")rawliteral" + String(trimAltitude, 2) + R"rawliteral(">
        
        <h3>Base Speed</h3>
        <label for="baseSpeed">Base Speed</label> <input name="baseSpeed" type="number" value=")rawliteral" + String(baseSpeed) + R"rawliteral(">
        <input type="submit" value="Update Settings">
      </form>
      
      <h3>Calibration</h3>
      <form method='POST' action='/calibrateAccelGyro'><input type='submit' value='Calibrate Accel & Gyro'></form>
      <form method='POST' action='/resetCalibration'><input type='submit' value='Reset Calibration'></form>
    </div>

    <script>
      const meteorContainer = document.querySelector('.meteor-container');
      const meteorCount = 20; 

      for (let i = 0; i < meteorCount; i++) {
        const meteor = document.createElement('div');
        meteor.className = 'meteor';
        meteor.style.left = `${Math.random() * 120}%`; 
        meteor.style.animationDelay = `${Math.random() * 5}s`;
        meteor.style.animationDuration = `${2 + Math.random() * 3}s`; 
        meteorContainer.appendChild(meteor);
      }
    </script>

    <script>
      setInterval(function() {
        fetch('/telemetry')
          .then(response => response.json())
          .then(data => {
            document.getElementById('targetRoll').innerText = data.targetRoll.toFixed(2);
            document.getElementById('currentRoll').innerText = data.currentRoll.toFixed(2);
            document.getElementById('targetPitch').innerText = data.targetPitch.toFixed(2);
            document.getElementById('currentPitch').innerText = data.currentPitch.toFixed(2);
            document.getElementById('targetYaw').innerText = data.targetYaw.toFixed(2);
            document.getElementById('currentYaw').innerText = data.currentYaw.toFixed(2);
            document.getElementById('altitude_rate_target').innerText = data.altitude_rate_target.toFixed(2);
            document.getElementById('currentAltitude').innerText = data.currentAltitude.toFixed(2);
          })
          .catch(error => console.error('Error fetching telemetry:', error));
      }, 50);

      setInterval(function() {
        fetch('/calibrationStatus')
          .then(response => response.text())
          .then(status => {
            const msgBox = document.querySelector('.msg');
            if (msgBox && status.trim() !== "") {
              msgBox.innerText = status;
            }
          })
          .catch(error => console.error('Error fetching calibration status:', error));
      }, 1000);
    </script>
  </body>
  </html>
)rawliteral";

  return html;
}

// *************************************************************  Functions  ************************************************************* //

// ================= Serial Console Command =================
String serialBuf = "";

void processSerialCommand(String cmd) {
  cmd.toLowerCase();

  if (cmd == "calibrate sensor") {
    if (Armed) {                                 
      Serial.println("ERROR: Please disarm first.");
      return;
    }
    Serial.println("Command received: calibrate sensor");
    calibrateAccelGyroRequested = true;             
  }
  else {
    Serial.print("Unknown command: ");
    Serial.println(cmd);
  }
}

void handleSerialConsole() {
  static unsigned long lastCharTime = 0;
  while (Serial.available()) {
    char c = Serial.read();
    lastCharTime = millis();
    if (c == '\n' || c == '\r') {                  
      serialBuf.trim();
      if (serialBuf.length()) processSerialCommand(serialBuf);
      serialBuf = "";
    } else if (serialBuf.length() < 64) {
      serialBuf += c;
    }
  }
  if (serialBuf.length() && millis() - lastCharTime > 100) {
    serialBuf.trim();
    if (serialBuf.length()) processSerialCommand(serialBuf);
    serialBuf = "";
  }
}

void printConfigToSerial() {
  Serial.println("=== Current Config ===");
  Serial.printf("PID Roll_rate: P=%.3f, I=%.3f, D=%.3f\n", pidRoll_rate.P, pidRoll_rate.I, pidRoll_rate.D);
  Serial.printf("PID Pitch_rate: P=%.3f, I=%.3f, D=%.3f\n", pidPitch_rate.P, pidPitch_rate.I, pidPitch_rate.D);
  Serial.printf("PID Yaw_rate: P=%.3f, I=%.3f, D=%.3f\n", pidYaw_rate.P, pidYaw_rate.I, pidYaw_rate.D);
  Serial.printf("PID Altitude_rate  : P=%.3f, I=%.3f, D=%.3f\n", pidAltitude_rate.P, pidAltitude_rate.I, pidAltitude_rate.D);

  Serial.printf("PID Roll_angle: P=%.3f, I=%.3f, D=%.3f\n", pidRoll_angle.P, pidRoll_angle.I, pidRoll_angle.D);
  Serial.printf("PID Pitch_angle: P=%.3f, I=%.3f, D=%.3f\n", pidPitch_angle.P, pidPitch_angle.I, pidPitch_angle.D);
  Serial.printf("PID Yaw_angle: P=%.3f, I=%.3f, D=%.3f\n", pidYaw_angle.P, pidYaw_angle.I, pidYaw_angle.D);
  Serial.printf("PID Altitude_m  : P=%.3f, I=%.3f, D=%.3f\n", pidAltitude_m.P, pidAltitude_m.I, pidAltitude_m.D);

  Serial.printf("Trim: Roll=%.3f, Pitch=%.3f, Yaw=%.3f, Altitude=%.3f\n", trimRoll, trimPitch, trimYaw, trimAltitude);
  Serial.printf("Base Speed: %d\n", baseSpeed);
  Serial.println("======================");
}

void loadConfig() {
  preferences.begin("drone", true);

  pidRoll_rate.P = preferences.getFloat("pRoll_rate", rollKp_rate);
  pidRoll_rate.I = preferences.getFloat("iRoll_rate", rollKi_rate);
  pidRoll_rate.D = preferences.getFloat("dRoll_rate", rollKd_rate);
  pidPitch_rate.P = preferences.getFloat("pPitch_rate", pitchKp_rate);
  pidPitch_rate.I = preferences.getFloat("iPitch_rate", pitchKi_rate);
  pidPitch_rate.D = preferences.getFloat("dPitch_rate", pitchKd_rate);
  pidYaw_rate.P = preferences.getFloat("pYaw_rate", yawKp_rate);
  pidYaw_rate.I = preferences.getFloat("iYaw_rate", yawKi_rate);
  pidYaw_rate.D = preferences.getFloat("dYaw_rate", yawKd_rate);
  pidAltitude_rate.P = preferences.getFloat("pAltitude_rate", altitudeKp_rate);
  pidAltitude_rate.I = preferences.getFloat("iAltitude_rate", altitudeKi_rate);
  pidAltitude_rate.D = preferences.getFloat("dAltitude_rate", altitudeKd_rate);

  pidRoll_angle.P = preferences.getFloat("pRoll_angle", rollKp_angle);
  pidRoll_angle.I = preferences.getFloat("iRoll_angle", rollKi_angle);
  pidRoll_angle.D = preferences.getFloat("dRoll_angle", rollKd_angle);
  pidPitch_angle.P = preferences.getFloat("pPitch_angle", pitchKp_angle);
  pidPitch_angle.I = preferences.getFloat("iPitch_angle", pitchKi_angle);
  pidPitch_angle.D = preferences.getFloat("dPitch_angle", pitchKd_angle);
  pidYaw_angle.P = preferences.getFloat("pYaw_angle", yawKp_angle);
  pidYaw_angle.I = preferences.getFloat("iYaw_angle", yawKi_angle);
  pidYaw_angle.D = preferences.getFloat("dYaw_angle", yawKd_angle);
  pidAltitude_m.P = preferences.getFloat("pAltitude_m", altitudeKp_m);
  pidAltitude_m.I = preferences.getFloat("iAltitude_m", altitudeKi_m);
  pidAltitude_m.D = preferences.getFloat("dAltitude_m", altitudeKd_m);

  trimRoll = preferences.getFloat("trimRoll", 0.0);
  trimPitch = preferences.getFloat("trimPitch", 0.0);
  trimYaw = preferences.getFloat("trimYaw", 0.0);
  trimAltitude = preferences.getFloat("trimAltitude", 0.0);
  baseSpeed = preferences.getInt("baseSpeed", baseSpeed);
  preferences.end();
}

void saveConfig() {
  preferences.begin("drone", false);

  preferences.putFloat("pRoll_rate", pidRoll_rate.P);
  preferences.putFloat("iRoll_rate", pidRoll_rate.I);
  preferences.putFloat("dRoll_rate", pidRoll_rate.D);
  preferences.putFloat("pPitch_rate", pidPitch_rate.P);
  preferences.putFloat("iPitch_rate", pidPitch_rate.I);
  preferences.putFloat("dPitch_rate", pidPitch_rate.D);
  preferences.putFloat("pYaw_rate", pidYaw_rate.P);
  preferences.putFloat("iYaw_rate", pidYaw_rate.I);
  preferences.putFloat("dYaw_rate", pidYaw_rate.D);
  preferences.putFloat("pAltitude_rate", pidAltitude_rate.P);
  preferences.putFloat("iAltitude_rate", pidAltitude_rate.I);
  preferences.putFloat("dAltitude_rate", pidAltitude_rate.D);

  preferences.putFloat("pRoll_angle", pidRoll_angle.P);
  preferences.putFloat("iRoll_angle", pidRoll_angle.I);
  preferences.putFloat("dRoll_angle", pidRoll_angle.D);
  preferences.putFloat("pPitch_angle", pidPitch_angle.P);
  preferences.putFloat("iPitch_angle", pidPitch_angle.I);
  preferences.putFloat("dPitch_angle", pidPitch_angle.D);
  preferences.putFloat("pYaw_angle", pidYaw_angle.P);
  preferences.putFloat("iYaw_angle", pidYaw_angle.I);
  preferences.putFloat("dYaw_angle", pidYaw_angle.D);
  preferences.putFloat("pAltitude_m", pidAltitude_m.P);
  preferences.putFloat("iAltitude_m", pidAltitude_m.I);
  preferences.putFloat("dAltitude_m", pidAltitude_m.D);

  preferences.putFloat("trimRoll", trimRoll);
  preferences.putFloat("trimPitch", trimPitch);
  preferences.putFloat("trimYaw", trimYaw);
  preferences.putFloat("trimAltitude", trimAltitude);
  preferences.putInt("baseSpeed", baseSpeed);
  preferences.end();
}

void calibrateAccelGyro() {

    imu.autoOffsets();

    xyzFloat aOffs = imu.getAccOffsets();  // get acceleration offsets
    xyzFloat gOffs = imu.getGyrOffsets();  // get gyroscope offsets 

    ax_bias = aOffs.x;
    ay_bias = aOffs.y;
    az_bias = aOffs.z;
    gx_bias = gOffs.x;
    gy_bias = gOffs.y;
    gz_bias = gOffs.z;
}

void saveCalibration() {
    preferences.begin("mpu-calib", false);
    preferences.putFloat("accX", ax_bias);
    preferences.putFloat("accY", ay_bias);
    preferences.putFloat("accZ", az_bias);

    preferences.putFloat("gyroX", gx_bias);
    preferences.putFloat("gyroY", gy_bias);
    preferences.putFloat("gyroZ", gz_bias);

    preferences.putBool("calibrated", true);
    preferences.end();

    Serial.println("Calibration saved.");
}

void loadCalibration() {
    preferences.begin("mpu-calib", true);
    bool calibrated = preferences.getBool("calibrated", false);
    if(calibrated){

        xyzFloat accOffset, gyrOffset;

        accOffset.x = preferences.getFloat("accX", 0.0f);
        accOffset.y = preferences.getFloat("accY", 0.0f);
        accOffset.z = preferences.getFloat("accZ", 0.0f);

        gyrOffset.x = preferences.getFloat("gyroX", 0.0f);
        gyrOffset.y = preferences.getFloat("gyroY", 0.0f);
        gyrOffset.z = preferences.getFloat("gyroZ", 0.0f);

        imu.setAccOffsets(accOffset);
        imu.setGyrOffsets(gyrOffset);

        Serial.println("Calibration loaded.");
    } else {
        Serial.println("No calibration found.");
    }
    preferences.end();
}

void resetCalibration() {
    ax_bias = ay_bias = az_bias = 0;
    gx_bias = gy_bias = gz_bias = 0;

    preferences.begin("mpu-calib", false);
    preferences.clear();
    preferences.end();

    Serial.println("Calibration reset.");
}

void resetPID() {
  // ===== Inner PID (rate) =====
  rollIntegral_rate = 0.0;
  rollPrevError_rate = 0.0;
  rollOutput_rate = 0.0;

  pitchIntegral_rate = 0.0;
  pitchPrevError_rate = 0.0;
  pitchOutput_rate = 0.0;

  yawIntegral_rate = 0.0;
  yawPrevError_rate = 0.0;
  yawOutput_rate = 0.0;

  altitudeIntegral_rate = 0.0;
  altitudePrevError_rate = 0.0;
  altitudeOutput_rate = 0.0;

  // ===== Outer PID (angle / altitude) =====
  rollIntegral_angle = 0.0;
  rollPrevError_angle = 0.0;
  rollOutput_angle = 0.0;

  pitchIntegral_angle = 0.0;
  pitchPrevError_angle = 0.0;
  pitchOutput_angle = 0.0;

  yawIntegral_angle = 0.0;
  yawPrevError_angle = 0.0;
  yawOutput_angle = 0.0;

  altitudeIntegral_m = 0.0;
  altitudePrevError_m = 0.0;
  altitudeOutput_m = 0.0;
}

void startWebServer() {
  Serial.println("Starting Async Web Server...");
  if (!webServerRunning) {
    WiFi.mode(WIFI_AP_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);

    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(ssid.c_str(), password, 1, 0, 4);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    dnsServer.start(DNS_PORT, "*", apIP);

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/html", htmlForm());
    });

    server.on("/telemetry", HTTP_GET, [](AsyncWebServerRequest *request){
      String json = "{";
      json += "\"targetRoll\":" + String(targetRoll) + ",";
      json += "\"targetPitch\":" + String(targetPitch) + ",";
      json += "\"targetYaw\":" + String(targetYaw) + ",";
      json += "\"altitude_rate_target\":" + String(altitude_rate_target) + ",";
      json += "\"currentRoll\":" + String(currentRoll) + ",";
      json += "\"currentPitch\":" + String(currentPitch) + ",";
      json += "\"currentYaw\":" + String(currentYaw) + ",";
      json += "\"currentAltitude\":" + String(currentAltitude);
      json += "}";
      request->send(200, "application/json", json);
    });

    server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
      if (request->hasParam("pRoll_rate", true)) pidRoll_rate.P = request->getParam("pRoll_rate", true)->value().toFloat();
      if (request->hasParam("iRoll_rate", true)) pidRoll_rate.I = request->getParam("iRoll_rate", true)->value().toFloat();
      if (request->hasParam("dRoll_rate", true)) pidRoll_rate.D = request->getParam("dRoll_rate", true)->value().toFloat();
      if (request->hasParam("pPitch_rate", true)) pidPitch_rate.P = request->getParam("pPitch_rate", true)->value().toFloat();
      if (request->hasParam("iPitch_rate", true)) pidPitch_rate.I = request->getParam("iPitch_rate", true)->value().toFloat();
      if (request->hasParam("dPitch_rate", true)) pidPitch_rate.D = request->getParam("dPitch_rate", true)->value().toFloat();
      if (request->hasParam("pYaw_rate", true)) pidYaw_rate.P = request->getParam("pYaw_rate", true)->value().toFloat();
      if (request->hasParam("iYaw_rate", true)) pidYaw_rate.I = request->getParam("iYaw_rate", true)->value().toFloat();
      if (request->hasParam("dYaw_rate", true)) pidYaw_rate.D = request->getParam("dYaw_rate", true)->value().toFloat();
      if (request->hasParam("pAltitude_rate", true)) pidAltitude_rate.P = request->getParam("pAltitude_rate", true)->value().toFloat();
      if (request->hasParam("iAltitude_rate", true)) pidAltitude_rate.I = request->getParam("iAltitude_rate", true)->value().toFloat();
      if (request->hasParam("dAltitude_rate", true)) pidAltitude_rate.D = request->getParam("dAltitude_rate", true)->value().toFloat();
      if (request->hasParam("pRoll_angle", true)) pidRoll_angle.P = request->getParam("pRoll_angle", true)->value().toFloat();
      if (request->hasParam("iRoll_angle", true)) pidRoll_angle.I = request->getParam("iRoll_angle", true)->value().toFloat();
      if (request->hasParam("dRoll_angle", true)) pidRoll_angle.D = request->getParam("dRoll_angle", true)->value().toFloat();
      if (request->hasParam("pPitch_angle", true)) pidPitch_angle.P = request->getParam("pPitch_angle", true)->value().toFloat();
      if (request->hasParam("iPitch_angle", true)) pidPitch_angle.I = request->getParam("iPitch_angle", true)->value().toFloat();
      if (request->hasParam("dPitch_angle", true)) pidPitch_angle.D = request->getParam("dPitch_angle", true)->value().toFloat();
      if (request->hasParam("pYaw_angle", true)) pidYaw_angle.P = request->getParam("pYaw_angle", true)->value().toFloat();
      if (request->hasParam("iYaw_angle", true)) pidYaw_angle.I = request->getParam("iYaw_angle", true)->value().toFloat();
      if (request->hasParam("dYaw_angle", true)) pidYaw_angle.D = request->getParam("dYaw_angle", true)->value().toFloat();
      if (request->hasParam("pAltitude_m", true)) pidAltitude_m.P = request->getParam("pAltitude_m", true)->value().toFloat();
      if (request->hasParam("iAltitude_m", true)) pidAltitude_m.I = request->getParam("iAltitude_m", true)->value().toFloat();
      if (request->hasParam("dAltitude_m", true)) pidAltitude_m.D = request->getParam("dAltitude_m", true)->value().toFloat();
      if (request->hasParam("trimRoll", true)) trimRoll = request->getParam("trimRoll", true)->value().toFloat();
      if (request->hasParam("trimPitch", true)) trimPitch = request->getParam("trimPitch", true)->value().toFloat();
      if (request->hasParam("trimYaw", true)) trimYaw = request->getParam("trimYaw", true)->value().toFloat();
      if (request->hasParam("trimAltitude", true)) trimAltitude = request->getParam("trimAltitude", true)->value().toFloat();
      if (request->hasParam("baseSpeed", true)) baseSpeed = request->getParam("baseSpeed", true)->value().toInt();
      saveConfig();
      printConfigToSerial();
      request->send(200, "text/html", htmlForm("Settings updated successfully!"));
    });

    server.on("/calibrateAccelGyro", HTTP_POST, [](AsyncWebServerRequest *request){
      calibrateAccelGyroRequested = true;
      request->send(200, "text/html", htmlForm("Accel & Gyro Calibration started. Please keep the drone still"));
    });

    server.on("/calibrationStatus", HTTP_GET, [](AsyncWebServerRequest *request){
      String status = "";
      if (AccelGyroisCalibrating) {
        status = "⏳ Calibrating Accel & Gyro...";
      } 
      else if (AccelGyrocalibrationDone) {
        status = "✅ Accel & Gyro Calibration complete!";
        AccelGyrocalibrationDone = false;
      } 
      request->send(200, "text/plain", status);
    });

    server.on("/resetCalibration", HTTP_POST, [](AsyncWebServerRequest *request){
      resetCalibration();
      request->send(200, "text/html", htmlForm("Calibration reset to default."));
    });

    server.onNotFound([](AsyncWebServerRequest *request) {
        request->send(200, "text/html", htmlForm());
    });

    server.begin();
    webServerRunning = true;
    Serial.println("Async Web Server Started");
  }
}

void stopWebServer() {
  Serial.println("Stopping Async Web Server...");
  if (webServerRunning) {
    dnsServer.stop();
    server.end();
    WiFi.softAPdisconnect(true);
    webServerRunning = false;
    Serial.println("Async Web Server Stopped");
  }
}

void readJoystickData(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len == sizeof(incomingJoystickData)) {
    memcpy(&incomingJoystickData, data, sizeof(incomingJoystickData));
  } else {
    Serial.printf("Received invalid data size: %d bytes\n", len);
  }
}

void setupESPNow() {
  WiFi.mode(WIFI_AP_STA);

  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_err_t result = esp_now_register_recv_cb(readJoystickData);
  if (result == ESP_OK) {
    Serial.println("Receive callback registered successfully");
  } else {
    Serial.printf("Failed to register receive callback: %d\n", result);
  }
}

float angleError(float target, float current) {
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

// ---------------- Median Filter ----------------
template <size_t N>
float medianFilter(float input, float (&buffer)[N], size_t &index) {
  buffer[index] = input;
  index = (index + 1) % N;

  float temp[N];
  memcpy(temp, buffer, sizeof(temp));

  // sort ค่า
  for (size_t i = 0; i < N - 1; i++) {
    for (size_t j = i + 1; j < N; j++) {
      if (temp[j] < temp[i]) {
        float t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }
  return temp[N / 2];
}

// ---------------- EMA Filter ----------------
float emaFilter(float input, float &emaPrev, float alpha) {
  emaPrev = alpha * input + (1.0f - alpha) * emaPrev;
  return emaPrev;
}

// ====== Complementary filter =====
void Complementary_Alt_Vz(float accZ_ms2, float altitudeBaro, float dt) {
    estAltitude  += estVelocityZ * dt;
    estVelocityZ += accZ_ms2 * dt;

    if (isBaroUpdated) {
        float posError = altitudeBaro - estAltitude;

        estAltitude  += posError * kP_pos * baroDt;    
        estVelocityZ += posError * kP_vel * baroDt;

        isBaroUpdated = false; 
    }
    currentAltitude = estAltitude;
    velocityZ = estVelocityZ;
}

void updateSensorsAndMadgwick(float dt) {
  xyzFloat gValue = imu.getGValues();
  xyzFloat gyr    = imu.getGyrValues();

  // Read sensor
  float accX = gValue.x;
  float accY = gValue.y;
  float accZ = gValue.z;
  float gyroX = gyr.x;
  float gyroY = gyr.y;
  float gyroZ = gyr.z;

  // ----------- Median filter -------------
  float gyroX_med = medianFilter(gyroX, gyroX_buf, gyroX_idx);
  float gyroY_med = medianFilter(gyroY, gyroY_buf, gyroY_idx);
  float gyroZ_med = medianFilter(gyroZ, gyroZ_buf, gyroZ_idx);
  float accX_med  = medianFilter(accX,  accX_buf,  accX_idx);
  float accY_med  = medianFilter(accY,  accY_buf,  accY_idx);
  float accZ_med  = medianFilter(accZ,  accZ_buf,  accZ_idx);
  float alt_med  = medianFilter(baroAltitude,  alt_buf,  alt_idx);

  // ----------- EMA filter ------------------
  gyroX_filtered = emaFilter(gyroX_med, emaGyroX, alphaGyro); 
  gyroY_filtered = emaFilter(gyroY_med, emaGyroY, alphaGyro);
  gyroZ_filtered = emaFilter(gyroZ_med, emaGyroZ, alphaGyro);
  accX_filtered  = emaFilter(accX_med,  emaAccX,  alphaAcc);
  accY_filtered  = emaFilter(accY_med,  emaAccY,  alphaAcc);
  accZ_filtered  = emaFilter(accZ_med,  emaAccZ,  alphaAcc);
  alt_filtered  = emaFilter(alt_med,  emaAlt,  alphaAlt);

  MadgwickFilter.updateIMU(gyroX_filtered, gyroY_filtered, gyroZ_filtered,
                             accX_filtered, accY_filtered, accZ_filtered);

  currentRoll  = -MadgwickFilter.getRoll();
  currentPitch =  MadgwickFilter.getPitch();
  currentYaw   = 360 - MadgwickFilter.getYaw();

  // g -> m/s^2
  float accX_ms2 = accX_filtered * g; 
  float accY_ms2 = accY_filtered * g; 
  float accZ_ms2 = accZ_filtered * g; 

  // deg -> rad
  float roll  = MadgwickFilter.getRoll()  * DEG_TO_RAD;
  float pitch = MadgwickFilter.getPitch() * DEG_TO_RAD;
  float yaw   = MadgwickFilter.getYaw()   * DEG_TO_RAD;

  // transform body -> world
  float accZ_world = (-accX_ms2 * sin(pitch))
                      + (accY_ms2 * sin(roll) * cos(pitch))
                      + (accZ_ms2 * cos(roll) * cos(pitch));

  // vertical acceleration
  float accZ_true = accZ_world - g;
  Complementary_Alt_Vz(accZ_true, alt_filtered, innerDt);
}

void readJoystick() {
  XL = incomingJoystickData.XL;
  YL = incomingJoystickData.YL;
  XR = incomingJoystickData.XR;
  YR = incomingJoystickData.YR;
  SWL = incomingJoystickData.SWL;
  SWR = incomingJoystickData.SWR;
  ButtonL = incomingJoystickData.ButtonL;
  ButtonR = incomingJoystickData.ButtonR;
}

void HandleRollTrim() {
  static bool lastRollUp = false;
  static bool lastRollDown = false;
  
  bool currRollUp   = ButtonR; 
  bool currRollDown = ButtonL;

  // Roll UP
  if (currRollUp && !lastRollUp) {
    trimRoll = trimRoll + TrimStep;
    preferences.begin("drone", false);
    preferences.putFloat("trimRoll", trimRoll);
    preferences.end();
  }
  lastRollUp = currRollUp;

  // Roll DOWN
  if (currRollDown && !lastRollDown) {
    trimRoll = trimRoll - TrimStep;
    preferences.begin("drone", false);
    preferences.putFloat("trimRoll", trimRoll);
    preferences.end();
  }
  lastRollDown = currRollDown;
}

void HandlePitchTrimAndWeb() {
  static bool lastSWL = false, lastSWR = false;
  
  static int clickCountSWL = 0;
  static int clickCountSWR = 0;
  static unsigned long lastClickTimeSWL = 0;
  static unsigned long lastClickTimeSWR = 0;

  unsigned long now = millis();
  bool currSWL = SWL;
  bool currSWR = SWR;

  if (currSWL && currSWR) {
    clickCountSWL = 0;
    clickCountSWR = 0;
    lastSWL = currSWL;
    lastSWR = currSWR;
    return; 
  }

  if (!lastSWL && currSWL) { // Rising Edge
    if (now - lastClickTimeSWL < clickInterval) {
      clickCountSWL++; 
    } else {
      clickCountSWL = 1; 
    }
    lastClickTimeSWL = now;
  }

  if (!lastSWR && currSWR) { // Rising Edge
    if (now - lastClickTimeSWR < clickInterval) {
      clickCountSWR++;
    } else {
      clickCountSWR = 1;
    }
    lastClickTimeSWR = now;
  }

  lastSWL = currSWL;
  lastSWR = currSWR;

  if (clickCountSWL == 2) {
    clickCountSWL = 0; 
  } 
  else if (clickCountSWL == 1 && (now - lastClickTimeSWL > clickInterval)) {
    if (currSWL == false) { 
      trimPitch = trimPitch + TrimStep;
      preferences.begin("drone", false);
      preferences.putFloat("trimPitch", trimPitch);
      preferences.end();
      Serial.println("Action: Pitch Trim +");
    } else {
      Serial.println("Ignored: Long Press detected");
    }
    clickCountSWL = 0; 
  }

  if (clickCountSWR == 2) {
    clickCountSWR = 0;
  }
  else if (clickCountSWR == 1 && (now - lastClickTimeSWR > clickInterval)) {
    if (currSWR == false) {
      trimPitch = trimPitch - TrimStep;
      preferences.begin("drone", false);
      preferences.putFloat("trimPitch", trimPitch);
      preferences.end();
      Serial.println("Action: Pitch Trim -");
    } else {
      Serial.println("Ignored: Long Press detected");
    }
    
    clickCountSWR = 0; 
  }
}

void updateParameters(float dt) {
  targetRoll  = constrain(incomingJoystickData.XR + trimRoll, -20, 20); // deg
  targetPitch = constrain(incomingJoystickData.YR + trimPitch, -20, 20); // deg
  targetYaw   = constrain(incomingJoystickData.XL + trimYaw, -90, 90); // deg/s

  // --- Headless Mode transform ---
  if (headlessMode == true) {
    // convention: pitch < 0 = forward 
    float dx = targetRoll;          // +right
    float dy = -targetPitch;        // +forward = away from pilot

    // yaw_error = currentYaw - yaw_ref
    float yaw_error = currentYaw - yaw_ref;
    while (yaw_error > 180.0f) yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;

    float yaw_rad = yaw_error * DEG_TO_RAD;
    float cosYaw = cosf(yaw_rad);
    float sinYaw = sinf(yaw_rad);

    float dx_rot = cosYaw * dx - sinYaw * dy;
    float dy_rot = sinYaw * dx + cosYaw * dy;

    targetRoll  = dx_rot;
    targetPitch = -dy_rot;
  }

  // --- Yaw control ---
  yaw_rate_target = targetYaw;
  yaw_setpoint += yaw_rate_target * dt;

  float rawAltitudeRate = constrain(incomingJoystickData.YL * 0.01, -2, 2); // m/s
  altitude_rate_target = rawAltitudeRate;
  // Slew-rate limit: 
  static float limitedAltitudeRate = 0;  
  float delta = rawAltitudeRate - limitedAltitudeRate;
  if (delta > maxRateChange) {
    delta = maxRateChange;
  }
  else if (delta < -maxRateChange) {
    delta = -maxRateChange;
  }
  limitedAltitudeRate += delta;

  // Update altitude setpoint 
  altitude_setpoint += limitedAltitudeRate * dt;

  // Clamp setpoint
  altitude_setpoint = constrain(altitude_setpoint, min_altitude, max_altitude);
}

// =================== Inner PID (rate) ===================
void innerPID(float dt) {
  float gyroRoll  = -gyroX_filtered;
  float gyroPitch =  gyroY_filtered;
  float gyroYaw   = -gyroZ_filtered;

  // Roll rate
  float rollError = rollOutput_angle - gyroRoll;
  rollIntegral_rate += rollError * dt;
  rollIntegral_rate = constrain(rollIntegral_rate, -integralLimit, integralLimit);
  rollOutput_rate = pidRoll_rate.P * rollError +
                    pidRoll_rate.I * rollIntegral_rate +
                    pidRoll_rate.D * ((rollError - rollPrevError_rate)/dt);
  rollOutput_rate = rollOutput_rate + (kff_roll * rollOutput_angle);  
  rollPrevError_rate = rollError;

  // Pitch rate
  float pitchError = pitchOutput_angle - gyroPitch;
  pitchIntegral_rate += pitchError * dt;
  pitchIntegral_rate = constrain(pitchIntegral_rate, -integralLimit, integralLimit);
  pitchOutput_rate = pidPitch_rate.P * pitchError +
                     pidPitch_rate.I * pitchIntegral_rate +
                     pidPitch_rate.D * ((pitchError - pitchPrevError_rate)/dt);
  pitchOutput_rate = pitchOutput_rate + (kff_pitch * pitchOutput_angle);
  pitchPrevError_rate = pitchError;

  // Yaw rate
  float yawError = yawOutput_angle - gyroYaw;
  yawIntegral_rate += yawError * dt;
  yawIntegral_rate = constrain(yawIntegral_rate, -integralLimit, integralLimit);
  yawOutput_rate = pidYaw_rate.P * yawError +
                   pidYaw_rate.I * yawIntegral_rate +
                   pidYaw_rate.D * ((yawError - yawPrevError_rate)/dt);
  yawOutput_rate = yawOutput_rate + (kff_yaw * yawOutput_angle);
  yawPrevError_rate = yawError;

  // Altitude rate
  float altitudeError = altitudeOutput_m - velocityZ;
  altitudeIntegral_rate += altitudeError * dt;
  altitudeIntegral_rate = constrain(altitudeIntegral_rate, -altitudeIntegralLimit, altitudeIntegralLimit);
  altitudeOutput_rate = pidAltitude_rate.P * altitudeError +
                        pidAltitude_rate.I * altitudeIntegral_rate +
                        pidAltitude_rate.D * ((altitudeError - altitudePrevError_rate)/dt);      
  altitudeOutput_rate = altitudeOutput_rate + (kff_altitude * altitudeOutput_m);                                  
  altitudePrevError_rate = altitudeError;
}

// =================== Outer PID (angle / altitude) ===================
void outerPID(float dt) {
  // Roll angle
  float rollError_angle = targetRoll - currentRoll;
  rollIntegral_angle += rollError_angle * dt;
  rollIntegral_angle = constrain(rollIntegral_angle, -integralLimit, integralLimit);
  rollOutput_angle = pidRoll_angle.P * rollError_angle +
                     pidRoll_angle.I * rollIntegral_angle +
                     pidRoll_angle.D * ((rollError_angle - rollPrevError_angle)/dt);
  rollPrevError_angle = rollError_angle;

  // Pitch angle
  float pitchError_angle = targetPitch - currentPitch;
  pitchIntegral_angle += pitchError_angle * dt;
  pitchIntegral_angle = constrain(pitchIntegral_angle, -integralLimit, integralLimit);
  pitchOutput_angle = pidPitch_angle.P * pitchError_angle +
                      pidPitch_angle.I * pitchIntegral_angle +
                      pidPitch_angle.D * ((pitchError_angle - pitchPrevError_angle)/dt);
  pitchPrevError_angle = pitchError_angle;

  // Yaw angle
  float yawError_angle = angleError(yaw_setpoint, currentYaw);
  yawIntegral_angle += yawError_angle * dt;
  yawIntegral_angle = constrain(yawIntegral_angle, -integralLimit, integralLimit);
  yawOutput_angle = pidYaw_angle.P * yawError_angle +
                    pidYaw_angle.I * yawIntegral_angle +
                    pidYaw_angle.D * ((yawError_angle - yawPrevError_angle)/dt);
  yawPrevError_angle = yawError_angle;

  // Altitude
  float altitudeError_m = altitude_setpoint - currentAltitude;
  float altDeadband = 0.05;
  if (abs(altitudeError_m) < altDeadband) {
      altitudeError_m = 0;
  }
  altitudeIntegral_m += altitudeError_m * dt;
  altitudeIntegral_m = constrain(altitudeIntegral_m, -altitudeIntegralLimit, altitudeIntegralLimit);
  altitudeOutput_m = pidAltitude_m.P * altitudeError_m +
                          pidAltitude_m.I * altitudeIntegral_m +
                          pidAltitude_m.D * ((altitudeError_m - altitudePrevError_m)/dt);                                           
  altitudePrevError_m = altitudeError_m;
}

void writeMotor(int pin, int us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    
    long period_us = 1000000 / PWM_FREQ; 
    uint32_t duty = (us * (1 << PWM_RES)) / period_us;
    ledcWrite(pin, duty);
}

void Arm() {               
  writeMotor(M1, idleSpeed);
  writeMotor(M2, idleSpeed);
  writeMotor(M3, idleSpeed);
  writeMotor(M4, idleSpeed);
}

void Disarm() {
  writeMotor(M1, 1000);
  writeMotor(M2, 1000);
  writeMotor(M3, 1000);
  writeMotor(M4, 1000);
}

// =================== Motor output ===================
void driveMotors() {
  float m1 = baseSpeed  + pitchOutput_rate + rollOutput_rate - yawOutput_rate + altitudeOutput_rate;
  float m2 = baseSpeed  + pitchOutput_rate - rollOutput_rate + yawOutput_rate + altitudeOutput_rate;
  float m3 = baseSpeed  - pitchOutput_rate - rollOutput_rate - yawOutput_rate + altitudeOutput_rate;
  float m4 = baseSpeed  - pitchOutput_rate + rollOutput_rate + yawOutput_rate + altitudeOutput_rate;

  m1 = constrain(m1, 1000, 2000);
  m2 = constrain(m2, 1000, 2000);
  m3 = constrain(m3, 1000, 2000);
  m4 = constrain(m4, 1000, 2000);

  writeMotor(M1, m1);
  writeMotor(M2, m2);
  writeMotor(M3, m3);
  writeMotor(M4, m4);
}

void FlightController() {
  unsigned long now = micros();

  // -------- Inner loop: 400 Hz --------
  if (now - lastInnerTime >= 1000000UL / innerHz) {
    lastInnerTime += 1000000UL / innerHz; 
    updateSensorsAndMadgwick(innerDt);     
    innerPID(innerDt);
  }

  // -------- Outer loop: 100 Hz --------
  if (now - lastOuterTime >= 1000000UL / outerHz) {
    lastOuterTime += 1000000UL / outerHz;
    outerPID(outerDt);
  }

  // -------- Barometer: 20 Hz --------
  if (now - lastBaroTime >= 1000000UL / baroHz) {
    lastBaroTime += 1000000UL / baroHz;
    baroAltitude = bmp.readAltitude(1013.25) - altitude_baseline;
    isBaroUpdated = true;
  }
}

void setupSensors() {
  imu.enableGyrDLPF();
  imu.setGyrDLPF(MPU6500_DLPF_2);
  imu.setSampleRateDivider(0);
  imu.setGyrRange(MPU6500_GYRO_RANGE_500);
  imu.setAccRange(MPU6500_ACC_RANGE_4G);
  imu.enableAccDLPF(true);
  imu.setAccDLPF(MPU6500_DLPF_2);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, 
                Adafruit_BMP280::SAMPLING_X4,   // temp oversampling
                Adafruit_BMP280::SAMPLING_X8,  // pressure oversampling
                Adafruit_BMP280::FILTER_X8,    // IIR filter
                Adafruit_BMP280::STANDBY_MS_1); // standby (ignore ใน FORCED)

  MadgwickFilter.begin(innerHz);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);
  Wire.setClock(400000); // 400 kHz Fast Mode

  ledcAttach(M1, PWM_FREQ, PWM_RES);
  ledcAttach(M2, PWM_FREQ, PWM_RES);
  ledcAttach(M3, PWM_FREQ, PWM_RES);
  ledcAttach(M4, PWM_FREQ, PWM_RES);
  writeMotor(M1, 1000);
  writeMotor(M2, 1000);
  writeMotor(M3, 1000);
  writeMotor(M4, 1000);
  delay(2000);

  // MPU6500 init
  if(!imu.init()){
    Serial.println("MPU6500 failed to init");
  }
  else{
    Serial.println("MPU6500 is connected");
  }

  // BMP280 init
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 failed to init");
  } else {
    Serial.println("BMP280 is connected");
  }
  
  setupSensors();
  setupESPNow();
  loadConfig();
  loadCalibration();

  ssid = "SimpleFly_" + WiFi.macAddress();
  ssid.replace(":", "");

  incomingJoystickData.webCommand = 1;
  webServerRunning = false;
  Serial.println("Setup done, waiting for joystick data...");
  Serial.println("SimpleFly");
  Serial.println("Control Style : Angle Mode & Headless Mode");
  Serial.println(WiFi.macAddress());
}

void loop() {
  handleSerialConsole();  
  readJoystick();
  HandleRollTrim();
  HandlePitchTrimAndWeb();

  // ---------------------- WEB SERVER ----------------------
  if (incomingJoystickData.webCommand != lastWebCommand) {
    if (incomingJoystickData.webCommand && !webServerRunning) {
      startWebServer();
    } 
    else if (!incomingJoystickData.webCommand && webServerRunning) {
      stopWebServer();
    }
    lastWebCommand = incomingJoystickData.webCommand;
  }

  if (webServerRunning) {
    dnsServer.processNextRequest();
  }

  // ---------------------- SENSOR CALIBRATION ----------------------
  if (calibrateAccelGyroRequested) { 
    AccelGyroisCalibrating = true;
    AccelGyrocalibrationDone = false;
    Serial.println("[Calibrate] Starting Accel & Gyro...");
    calibrateAccelGyro();
    saveCalibration();
    setupSensors();
    loadCalibration();
    resetPID();
    AccelGyroisCalibrating = false;
    AccelGyrocalibrationDone = true;
    calibrateAccelGyroRequested = false;
    Serial.println("[Calibrate] Accel & Gyro complete.");
  }

  // ---------------------- HEADLESS MODE (กดค้าง 3 วิ) ----------------------
  static unsigned long swlPressStart = 0;
  static unsigned long swrPressStart = 0;
  static bool swlWasPressed = false;
  static bool swrWasPressed = false;
  static bool swlActionDone = false;
  static bool swrActionDone = false;

  // ปุ่มซ้ายค้าง => เปิด headlessMode
  if (SWL && !swlWasPressed) {
    swlPressStart = millis();
    swlActionDone = false;
  }
  if (SWL && !swlActionDone && millis() - swlPressStart >= 3000) {
    headlessMode = true;
    swlActionDone = true;
  }
  if (!SWL) {
    swlPressStart = 0;
    swlActionDone = false;
  }

  // ปุ่มขวาค้าง => ปิด headlessMode
  if (SWR && !swrWasPressed) {
    swrPressStart = millis();
    swrActionDone = false;
  }
  if (SWR && !swrActionDone && millis() - swrPressStart >= 3000) {
    headlessMode = false;
    swrActionDone = true;
  }
  if (!SWR) {
    swrPressStart = 0;
    swrActionDone = false;
  }

  swlWasPressed = SWL;
  swrWasPressed = SWR;

  // ---------------------- ARM / DISARM ----------------------
  bool armButton = (SWL && SWR); 
  if (armButton && !lastArmButton) {
    Armed = !Armed;
    OnFlying = false; 
    initial_altitude = false;
    initial_yaw = false;
  }
  lastArmButton = armButton;

  if (Armed && YL != 0 && !OnFlying) {
    OnFlying = true;
  }

  // ---------------------- FLIGHT CONTROL ----------------------
  updateParameters(innerDt);
  FlightController();
  if (Armed && !OnFlying) {
    if (!initial_altitude && !initial_yaw) {

      altitude_baseline = bmp.readAltitude(1013.25);
      estAltitude = 0;   
      estVelocityZ = 0;
      altitude = 0;         
      currentAltitude = 0;   
      velocityZ = 0;          
      previousBaro = 0;      
      altitude_setpoint = 0; 
      max_altitude = range_altitude;    
      min_altitude = -range_altitude; 
      initial_altitude = true;

      yaw_ref = currentYaw;
      yaw_setpoint = currentYaw;
      initial_yaw = true;
    }
    Arm(); 
  }
  else if (Armed && OnFlying) {
    driveMotors();
  }
  else {
    Armed = false;
    OnFlying = false;
    initial_altitude = false;
    initial_yaw = false;
    Disarm();
  }
}

