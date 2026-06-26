#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Arduino.h>

// ==== PIN Definitions ====
#define XL_PIN 3 
#define YL_PIN 4  
#define SWL_PIN 6 
#define XR_PIN 1
#define YR_PIN 2 
#define SWR_PIN 5 
#define ButtonL_PIN 0
#define ButtonR_PIN 10
#define Buzzer_PIN 8 

// ==== WiFi Config ====
const char* ssid = "Controller Calibration";
const char* password = "12345678";
IPAddress apIP(192, 168, 4, 1);
DNSServer dnsServer;
WebServer server(80);
bool wifiActive = false;
bool inCalibrationMode = false;  

Preferences prefs;

// ==== Joystick Data ====
int XL, YL, XR, YR;
bool SWL, SWR, ButtonL, ButtonR, webCommand;

int XLMinMap, XLMaxMap, YLMinMap, YLMaxMap;
int XRMinMap, XRMaxMap, YRMinMap, YRMaxMap;

int XL_min = 4095, YL_min = 4095, XR_min = 4095, YR_min = 4095;
int XL_max = 0,    YL_max = 0,    XR_max = 0,    YR_max = 0;
int XL_center = 0, YL_center = 0, XR_center = 0, YR_center = 0;
int deadzone = 60;

unsigned long lastReadMicro = 0;
const unsigned long intervalMicro = 50000; // 50ms

unsigned long TrimBuzzerStart = 0; 
bool isTrimBuzzing = false;         
const long BUZ_TIME = 100; 

// ==== Double Click Detect ====
const unsigned long clickInterval = 300;

// ==== ESP-NOW ====
uint8_t ReceiverMAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
struct JoyData {
  int XL, YL;
  bool SWL, ButtonL;
  int XR, YR;
  bool SWR, ButtonR;
  bool webCommand;
};

// ==== Sci-Fi Style HTML Page (Orange & Cyan Theme) ====
const char htmlPage[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><title>Controller Calibration</title>
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<style>
  @import url('https://fonts.googleapis.com/css2?family=Orbitron:wght@700&display=swap');
  
  /* --- 1. SETUP --- */
  * { box-sizing: border-box; } 
  
  body {
    margin: 0;
    background: #0b0f19;
    font-family: 'Orbitron', monospace;
    color: #00ccff;
    user-select: none;
    min-height: 100vh;
    display: flex;
    flex-direction: column;
    align-items: center;
    padding: 2rem 1rem 4rem 1rem;
    overflow-x: hidden;
    position: relative;
  }

  /* --- 2. BACKGROUND ANIMATIONS --- */
  .meteor-container { position: fixed; top: 0; left: 0; width: 100%; height: 100%; z-index: 0; pointer-events: none; }
  .meteor {
    position: absolute; top: -50px; width: 2px; height: 2px; background: #fff; border-radius: 50%;
    box-shadow: 0 0 5px #ffaa00, 0 0 10px #00ccff, 0 0 20px #ffaa00;
    animation: meteor-fall linear infinite; opacity: 0;
  }
  .meteor::before {
    content: ''; position: absolute; top: 50%; transform: translateY(-50%); width: 50px; height: 1px;
    background: linear-gradient(90deg, #ffaa00, transparent); right: 2px;
  }
  @keyframes meteor-fall {
    0% { transform: rotate(-35deg) translateX(0); opacity: 1; }
    70% { opacity: 1; }
    100% { transform: rotate(-35deg) translateX(-100vh); opacity: 0; }
  }

  .space-scene { position: absolute; top: 0; left: 0; width: 100%; height: 100%; z-index: 1; pointer-events: none; overflow: hidden; }
  .planet {
    position: absolute; bottom: 15%; right: 5%; width: 50px; height: 50px;
    background: linear-gradient(135deg, #ffaa00, #cc8800); border-radius: 50%;
    box-shadow: -5px -5px 10px rgba(0,0,0,0.5) inset, 0 0 20px rgba(255,170,0,0.4);
  }
  .planet .ring {
    position: absolute; top: 50%; left: 50%; width: 80px; height: 15px;
    border: 3px solid #00ccff; border-radius: 50%;
    transform: translate(-50%, -50%) rotate(-20deg); box-shadow: 0 0 8px #00ccff;
  }
  .ufo {
    position: absolute; bottom: 15%; left: 5%; width: 50px; height: 18px;
    background: #00ccff; border-radius: 50%; box-shadow: 0 0 15px #00ccff;
    animation: float-ufo 5s ease-in-out infinite; opacity: 0.8;
  }
  .ufo::before {
    content: ''; position: absolute; top: -10px; left: 12px; width: 26px; height: 18px;
    background: rgba(255, 170, 0, 0.9); border-radius: 50% 50% 0 0; z-index: -1;
  }
  @keyframes float-ufo { 0%, 100% { transform: translateY(0) rotate(-5deg); } 50% { transform: translateY(-15px) rotate(5deg); } }

  /* --- 3. UI ELEMENTS --- */
  .title-container { display: flex; align-items: center; gap: 15px; margin-bottom: 2rem; position: relative; z-index: 10; }
  .lightning-icon {
    width: 48px; height: 48px; fill: #ffaa00;
    filter: drop-shadow(0 0 6px #ffaa00) drop-shadow(0 0 12px #ff8800);
    animation: flicker 2s infinite alternate;
  }
  @keyframes flicker { 0% { filter: drop-shadow(0 0 5px #ffaa00) opacity(0.8); } 100% { filter: drop-shadow(0 0 15px #ffaa00) opacity(1); } }
  
  h2 { font-size: clamp(1.8rem, 4vw, 3rem); color: #ffaa00; text-shadow: 0 0 5px #ffaa00, 0 0 15px #ffaa00; letter-spacing: 0.1em; text-align: center; }

  .container { display: flex; justify-content: center; gap: 40px; flex-wrap: wrap; position: relative; z-index: 10; }

  .joystick-container {
    background: rgba(11, 15, 25, 0.9); border-radius: 15px; padding: 20px;
    box-shadow: 0 0 15px rgba(255, 170, 0, 0.3); width: 320px; border: 2px solid #ffaa00;
    transition: box-shadow 0.3s ease; backdrop-filter: blur(4px);
  }
  .joystick-container:hover { box-shadow: 0 0 30px #ffaa00, inset 0 0 12px rgba(255, 170, 0, 0.5); }
  .joystick-title { text-transform: uppercase; font-weight: 700; color: #ffaa00; margin-bottom: 15px; font-size: 1.6em; text-shadow: 0 0 8px #ffaa00; text-align: center; }

  .values { display: flex; justify-content: space-between; margin-bottom: 15px; }
  .values .main { flex: 1; font-size: 1.2em; line-height: 1.6em; padding-left: 20px; padding-right: 10px; border-right: 1px solid #ffaa00; color: #e0e0ff; }
  .values .details { flex: 1; font-size: 1.1em; text-align: left; padding-left: 20px; color: #00ccff; }
  .values span { color: #ffaa00; font-weight: bold; }
  .details p, .values p { margin: 6px 0; font-family: monospace; }

  .joystick-graphic {
    position: relative; width: 160px; height: 160px; margin: 0 auto;
    background: radial-gradient(circle, #151925 60%, transparent 95%);
    border-radius: 50%; box-shadow: inset 0 0 12px #ffaa00, 0 0 15px rgba(255, 170, 0, 0.5);
    border: 1px solid #ffaa00;
  }
  .joystick-center { position: absolute; top: 50%; left: 50%; width: 10px; height: 10px; margin-left: -5px; margin-top: -5px; background: #00ccff; border-radius: 50%; box-shadow: 0 0 8px #00ccff; }
  .joystick-handle {
    position: absolute; width: 40px; height: 40px; background: #ffaa00; border-radius: 50%;
    box-shadow: 0 0 15px #ffaa00, inset 0 0 5px #fff;
    top: 50%; left: 50%; margin-left: -20px; margin-top: -20px;
    transition: top 0.1s ease, left 0.1s ease;
  }

  /* --- BUTTONS & INPUTS --- */
  .buttons { 
    margin-top: 30px; 
    position: relative; 
    z-index: 10; 
    width: 100%; 
    max-width: 800px; 
    text-align: center;
  }
  
  .button-row {
    display: flex;
    justify-content: center;
    gap: 15px; 
    flex-wrap: wrap; 
    margin-bottom: 20px;
  }

  .map-input {
    padding: 12px; font-size: 1em; text-align: center; border-radius: 10px;
    border: 1px solid #ffaa00; background: #151925; color: #ffaa00;
    box-shadow: inset 0 0 6px rgba(255, 170, 0, 0.2);
    font-family: 'Orbitron', monospace;
    width: 100%; /* ให้เต็มช่อง Grid */
  }
  .map-input:focus { outline: none; box-shadow: 0 0 12px #ffaa00; border-color: #00ccff; }

  button {
    padding: 14px 20px; 
    font-size: 1.1em; font-weight: 600; letter-spacing: 0.05em;
    background: linear-gradient(90deg, #ffaa00, #00ccff, #ffaa00); background-size: 200% auto;
    color: #0b0f19; border: none; border-radius: 12px; cursor: pointer;
    box-shadow: 0 0 10px #ffaa00; transition: 0.4s ease;
    animation: glow-move 4s linear infinite; font-family: 'Orbitron', monospace;
    white-space: nowrap; 
  }
  @keyframes glow-move { 0% { background-position: 0% center; } 100% { background-position: 200% center; } }
  button:hover { background-position: right center; box-shadow: 0 0 20px #00ccff; transform: scale(1.02); }

  #status { text-align: center; margin-top: 10px; font-size: 1.2em; color: #00ff00; text-shadow: 0 0 8px #0f0; min-height: 1.5em; }
  h3 { color: #00ccff !important; text-shadow: 0 0 8px #00ccff !important; border-bottom: 1px solid #ffaa00; display: inline-block; padding-bottom: 5px;}
</style>
</head><body>

<div class="meteor-container"></div>
<script>
  const meteorContainer = document.querySelector('.meteor-container');
  for (let i = 0; i < 20; i++) {
    const meteor = document.createElement('div');
    meteor.className = 'meteor';
    meteor.style.left = `${Math.random() * 120}%`; 
    meteor.style.animationDelay = `${Math.random() * 5}s`;
    meteor.style.animationDuration = `${2 + Math.random() * 3}s`; 
    meteorContainer.appendChild(meteor);
  }
</script>
<div class="space-scene">
  <div class="planet"><div class="ring"></div></div>
  <div class="ufo"></div>
</div>

<div class="title-container">
  <svg class="lightning-icon" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg"><path d="M7 2v10h3v10l7-12h-4l4-8z"/></svg>
  <h2>Controller Calibration</h2>
  <svg class="lightning-icon" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg"><path d="M7 2v10h3v10l7-12h-4l4-8z"/></svg>
</div>

<div class="container">
  <div class="joystick-container">
    <div class="joystick-title">Left Joystick</div>
    <div class="values">
      <div class="main">
        <p>X: <span id="xl">0</span></p>
        <p>Y: <span id="yl">0</span></p>
        <p>SWL: <span id="swl">0</span></p>
      </div>
      <div class="details">
        <p>Min: <br>X: <span id="xl_min">0</span><br>Y: <span id="yl_min">0</span></p>
        <p>Max: <br>X: <span id="xl_max">0</span><br>Y: <span id="yl_max">0</span></p>
        <p>Center: <br>X: <span id="xl_center">0</span><br>Y: <span id="yl_center">0</span></p>
      </div>
    </div>
    <div class="joystick-graphic"><div class="joystick-center"></div><div class="joystick-handle" id="leftHandle"></div></div>
  </div>

  <div class="joystick-container">
    <div class="joystick-title">Right Joystick</div>
    <div class="values">
      <div class="main">
        <p>X: <span id="xr">0</span></p>
        <p>Y: <span id="yr">0</span></p>
        <p>SWR: <span id="swr">0</span></p>
      </div>
      <div class="details">
        <p>Min: <br>X: <span id="xr_min">0</span><br>Y: <span id="yr_min">0</span></p>
        <p>Max: <br>X: <span id="xr_max">0</span><br>Y: <span id="yr_max">0</span></p>
        <p>Center: <br>X: <span id="xr_center">0</span><br>Y: <span id="yr_center">0</span></p>
      </div>
    </div>
    <div class="joystick-graphic"><div class="joystick-center"></div><div class="joystick-handle" id="rightHandle"></div></div>
  </div>
</div>

<div class="buttons">
  
  <div class="button-row">
    <button onclick="saveMinMax()">Save Min/Max</button>
    <button onclick="saveCenter()">Calibrate Center</button>
    <button onclick="resetValues()">Reset All</button>
  </div>
  
  <div id="status"></div>
  
  <form onsubmit="submitMapForm(); return false;" style="margin-top:30px; text-align:center;">
    <h3 style="margin-bottom: 20px;">Mapping Parameters</h3>
    <div style="display:grid; grid-template-columns:repeat(2, 1fr); gap:12px; width: 100%; max-width: 500px; margin: 0 auto;">
      <input type="number" id="XLMinMap" placeholder="XL Min Map" class="map-input">
      <input type="number" id="XLMaxMap" placeholder="XL Max Map" class="map-input">
      <input type="number" id="YLMinMap" placeholder="YL Min Map" class="map-input">
      <input type="number" id="YLMaxMap" placeholder="YL Max Map" class="map-input">
      <input type="number" id="XRMinMap" placeholder="XR Min Map" class="map-input">
      <input type="number" id="XRMaxMap" placeholder="XR Max Map" class="map-input">
      <input type="number" id="YRMinMap" placeholder="YR Min Map" class="map-input">
      <input type="number" id="YRMaxMap" placeholder="YR Max Map" class="map-input">
    </div>
    <button type="submit" style="margin-top: 25px;">💾 Save Mapping 💾</button>
  </form>

  <h3 style="margin-top: 40px; text-align: center;">Set Receiver MAC Address</h3>
  <div style="display:flex; gap: 10px; justify-content: center; flex-wrap: wrap; margin-top:10px;">
    <input type="text" id="macAddress" placeholder="08:A6:F7:21:BA:6C" class="map-input" style="min-width: 280px; width: auto;">
    <button type="button" onclick="saveMacAddress()">💾 Save MAC 💾</button>
  </div>
</div>

<script>
function saveMinMax() { fetch('/save_minmax').then(() => { document.getElementById('status').textContent = '✅ Min/Max saved ✅'; }); }
function saveCenter() { fetch('/save_center').then(() => { document.getElementById('status').textContent = '✅ Center calibrated ✅'; }); }
function resetValues() { fetch('/reset_values').then(() => { document.getElementById('status').textContent = '🗑️ All values reset 🗑️'; }); }
function updateJoystickHandle(id, x, y, centerX, centerY, minX, maxX, minY, maxY) {
  let rangeX = maxX - minX; let rangeY = maxY - minY;
  if(rangeX <= 0) rangeX = 1; if(rangeY <= 0) rangeY = 1;
  let normX = -1 * (x - centerX) / (rangeX / 2); let normY = -1 * (y - centerY) / (rangeY / 2);
  normX = Math.min(Math.max(normX, -1), 1); normY = Math.min(Math.max(normY, -1), 1);
  let radius = 60; let posX = 80 + normX * radius; let posY = 80 + normY * radius;
  let handle = document.getElementById(id); handle.style.left = posX + 'px'; handle.style.top = posY + 'px';
}
function submitMapForm() {
  const params = new URLSearchParams();
  params.append("XLMinMap", document.getElementById("XLMinMap").value);
  params.append("XLMaxMap", document.getElementById("XLMaxMap").value);
  params.append("YLMinMap", document.getElementById("YLMinMap").value);
  params.append("YLMaxMap", document.getElementById("YLMaxMap").value);
  params.append("XRMinMap", document.getElementById("XRMinMap").value);
  params.append("XRMaxMap", document.getElementById("XRMaxMap").value);
  params.append("YRMinMap", document.getElementById("YRMinMap").value);
  params.append("YRMaxMap", document.getElementById("YRMaxMap").value);
  fetch("/save_map?" + params.toString()).then(r => r.text()).then(t => document.getElementById("status").textContent = t);
}
function saveMacAddress() {
  const mac = document.getElementById("macAddress").value;
  fetch("/save_mac?mac=" + mac).then(r => r.text()).then(t => document.getElementById("status").textContent = t);
}
window.addEventListener("load", () => {
  fetch("/get_map").then(r => r.json()).then(d => {
    document.getElementById("XLMinMap").value = d.XLMinMap; document.getElementById("XLMaxMap").value = d.XLMaxMap;
    document.getElementById("YLMinMap").value = d.YLMinMap; document.getElementById("YLMaxMap").value = d.YLMaxMap;
    document.getElementById("XRMinMap").value = d.XRMinMap; document.getElementById("XRMaxMap").value = d.XRMaxMap;
    document.getElementById("YRMinMap").value = d.YRMinMap; document.getElementById("YRMaxMap").value = d.YRMaxMap;
  });
  fetch("/get_mac").then(r => r.text()).then(mac => { document.getElementById("macAddress").value = mac; });
});
setInterval(() => {
  fetch("/data").then(res => res.json()).then(d => {
    document.getElementById("xl").textContent = d.XL; document.getElementById("yl").textContent = d.YL; document.getElementById("swl").textContent = d.SWL;
    document.getElementById("xr").textContent = d.XR; document.getElementById("yr").textContent = d.YR; document.getElementById("swr").textContent = d.SWR;
    document.getElementById("xl_min").textContent = d.XL_min; document.getElementById("xl_max").textContent = d.XL_max;
    document.getElementById("yl_min").textContent = d.YL_min; document.getElementById("yl_max").textContent = d.YL_max;
    document.getElementById("xr_min").textContent = d.XR_min; document.getElementById("xr_max").textContent = d.XR_max;
    document.getElementById("yr_min").textContent = d.YR_min; document.getElementById("yr_max").textContent = d.YR_max;
    document.getElementById("xl_center").textContent = d.XL_center; document.getElementById("yl_center").textContent = d.YL_center;
    document.getElementById("xr_center").textContent = d.XR_center; document.getElementById("yr_center").textContent = d.YR_center;
    updateJoystickHandle('leftHandle', d.XL, d.YL, d.XL_center, d.YL_center, d.XL_min, d.XL_max, d.YL_min, d.YL_max);
    updateJoystickHandle('rightHandle', d.XR, d.YR, d.XR_center, d.YR_center, d.XR_min, d.XR_max, d.YR_min, d.YR_max);
  });
}, 100);
</script>
</body></html>
)rawliteral";

int readAverage(int pin, int samples = 5) {
  int sum = 0;
  for (int i = 0; i < samples; i++) sum += analogRead(pin);
  return sum / samples;
}

void readJoystick() {
  XL = readAverage(XL_PIN);
  YL = readAverage(YL_PIN);
  XR = readAverage(XR_PIN);
  YR = readAverage(YR_PIN);
  SWL = !digitalRead(SWL_PIN);
  SWR = !digitalRead(SWR_PIN);
  ButtonL = !digitalRead(ButtonL_PIN);
  ButtonR = !digitalRead(ButtonR_PIN);

  // Update min/max 
  if (XL < XL_min) XL_min = XL;
  if (XL > XL_max) XL_max = XL;

  if (YL < YL_min) YL_min = YL;
  if (YL > YL_max) YL_max = YL;

  if (XR < XR_min) XR_min = XR;
  if (XR > XR_max) XR_max = XR;

  if (YR < YR_min) YR_min = YR;
  if (YR > YR_max) YR_max = YR;
}

void loadReceiverMAC() {
  prefs.begin("mac", true);
  String macStr = prefs.getString("mac", "00:00:00:00:00:00");
  prefs.end();

  int values[6];
  if (sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x",
             &values[0], &values[1], &values[2],
             &values[3], &values[4], &values[5]) == 6) {
    for (int i = 0; i < 6; i++) {
      ReceiverMAC[i] = (uint8_t)values[i];
    }
  }
}

void sendESPNow() {
  JoyData data = {XL, YL, SWL, ButtonL, XR, YR, SWR, ButtonR, webCommand};
  esp_now_send(ReceiverMAC, (uint8_t*)&data, sizeof(data));

  if (webCommand == 1) {
    Serial.print("  Sending -> ");
    Serial.print("XL: "); Serial.print(data.XL); Serial.print("  ");
    Serial.print("YL: "); Serial.print(data.YL); Serial.print("  ");
    Serial.print("SWL: "); Serial.print(data.SWL); Serial.print("  ");
    Serial.print("ButtonL: "); Serial.print(data.ButtonL); Serial.print("  ");
    Serial.print("XR: "); Serial.print(data.XR); Serial.print("  ");
    Serial.print("YR: "); Serial.print(data.YR); Serial.print("  ");
    Serial.print("SWR: "); Serial.print(data.SWR); Serial.print("  ");
    Serial.print("ButtonR: "); Serial.print(data.ButtonR); Serial.print("  ");
    Serial.println("   📶 WebServer On");
  }
}

void handleWebData() {
  readJoystick();
  String json = "{";
  json += "\"XL\":" + String(XL) + ",";
  json += "\"YL\":" + String(YL) + ",";
  json += "\"SWL\":" + String(SWL) + ",";
  json += "\"XR\":" + String(XR) + ",";
  json += "\"YR\":" + String(YR) + ",";
  json += "\"SWR\":" + String(SWR) + ",";
  json += "\"XL_min\":" + String(XL_min) + ",\"XL_max\":" + String(XL_max) + ",";
  json += "\"YL_min\":" + String(YL_min) + ",\"YL_max\":" + String(YL_max) + ",";
  json += "\"XR_min\":" + String(XR_min) + ",\"XR_max\":" + String(XR_max) + ",";
  json += "\"YR_min\":" + String(YR_min) + ",\"YR_max\":" + String(YR_max) + ",";
  json += "\"XL_center\":" + String(XL_center) + ",";
  json += "\"YL_center\":" + String(YL_center) + ",";
  json += "\"XR_center\":" + String(XR_center) + ",";
  json += "\"YR_center\":" + String(YR_center);
  json += "}";
  server.send(200, "application/json", json);
}

void saveMinMax() {
  prefs.begin("joy", false);
  prefs.putInt("XL_min", XL_min); prefs.putInt("XL_max", XL_max);
  prefs.putInt("YL_min", YL_min); prefs.putInt("YL_max", YL_max);
  prefs.putInt("XR_min", XR_min); prefs.putInt("XR_max", XR_max);
  prefs.putInt("YR_min", YR_min); prefs.putInt("YR_max", YR_max);
  prefs.end();
  server.send(200, "text/plain", "Min/Max saved");
}

void saveCenter() {
  readJoystick();  
  prefs.begin("joy", false);
  prefs.putInt("XL_center", XL);
  prefs.putInt("YL_center", YL);
  prefs.putInt("XR_center", XR);
  prefs.putInt("YR_center", YR);
  prefs.end();

  XL_center = XL;
  YL_center = YL;
  XR_center = XR;
  YR_center = YR;

  server.send(200, "text/plain", "Center saved");
}

void loadStoredValues() {
  prefs.begin("joy", true);
  XL_min = prefs.getInt("XL_min", 4095); XL_max = prefs.getInt("XL_max", 0);
  YL_min = prefs.getInt("YL_min", 4095); YL_max = prefs.getInt("YL_max", 0);
  XR_min = prefs.getInt("XR_min", 4095); XR_max = prefs.getInt("XR_max", 0);
  YR_min = prefs.getInt("YR_min", 4095); YR_max = prefs.getInt("YR_max", 0);
  XL_center = prefs.getInt("XL_center", 0);
  YL_center = prefs.getInt("YL_center", 0);
  XR_center = prefs.getInt("XR_center", 0);
  YR_center = prefs.getInt("YR_center", 0);
  prefs.end();
}

void loadMapValues() {
  prefs.begin("map", true);
  XLMinMap = prefs.getInt("XLMinMap", -20);
  XLMaxMap = prefs.getInt("XLMaxMap", 20);
  YLMinMap = prefs.getInt("YLMinMap", -100);
  YLMaxMap = prefs.getInt("YLMaxMap", 100);
  XRMinMap = prefs.getInt("XRMinMap", -10);
  XRMaxMap = prefs.getInt("XRMaxMap", 10);
  YRMinMap = prefs.getInt("YRMinMap", 10);
  YRMaxMap = prefs.getInt("YRMaxMap", -1);
  prefs.end();
}

void resetStoredValues() {
  prefs.begin("joy", false);
  prefs.clear();  
  prefs.end();

  // ตั้งค่าดีฟอลต์กลับใหม่
  XL_min = YL_min = XR_min = YR_min = 4095;
  XL_max = YL_max = XR_max = YR_max = 0;
  XL_center = YL_center = XR_center = YR_center = 0;

  server.send(200, "text/plain", "Values reset");
}

void mapJoystick() {
  
  if (XL > XL_center + deadzone) {
    XL = map(XL, XL_center + deadzone, XL_max, 0, -XLMaxMap);
  }
  else if (XL < XL_center - deadzone) {
    XL = map(XL, XL_center - deadzone, XL_min, 0, -XLMinMap);
  }
  else {
    XL = 0 ;
  }
  if (XR > XR_center + deadzone) {
    XR = map(XR, XR_center + deadzone, XR_max, 0, -XRMaxMap);
  }
  else if (XR < XR_center - deadzone) {
    XR = map(XR, XR_center - deadzone, XR_min, 0, -XRMinMap);
  }
  else {
    XR = 0 ;
  }
  if (YL > YL_center + deadzone) {
    YL = map(YL, YL_center + deadzone, YL_max, 0, YLMaxMap);
  }
  else if (YL < YL_center - deadzone) {
    YL = map(YL, YL_center - deadzone, YL_min, 0, YLMinMap);
  }
  else {
    YL = 0 ;
  }
  if (YR > YR_center + deadzone) {
    YR = map(YR, YR_center + deadzone, YR_max, 0, YRMaxMap);
  }
  else if (YR < YR_center - deadzone) {
    YR = map(YR, YR_center - deadzone, YR_min, 0, YRMinMap);
  }
  else {
    YR = 0 ;
  }
}

void startWeb() {
  if (wifiActive) return;
  loadStoredValues();  
  loadMapValues();
  
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(ssid, password, 1, 0, 4);
  dnsServer.start(53, "*", apIP);

  server.on("/", []() { server.send_P(200, "text/html", htmlPage); });
  server.on("/data", handleWebData);
  server.on("/save_minmax", saveMinMax);
  server.on("/save_center", saveCenter);
  server.on("/reset_values", resetStoredValues);  

  // สำหรับ redirect บนอุปกรณ์
  server.on("/generate_204", []() { server.send(204, "text/plain", ""); });
  server.on("/hotspot-detect.html", []() { server.send(200, "text/html", htmlPage); });
  server.on("/gen_204", []() { server.sendHeader("Location", "http://192.168.4.1", true); server.send(302, ""); });
  server.on("/ncsi.txt", []() { server.sendHeader("Location", "http://192.168.4.1", true); server.send(302, ""); });
  server.on("/connecttest.txt", []() { server.sendHeader("Location", "http://192.168.4.1", true); server.send(302, ""); });
  server.onNotFound([]() { server.sendHeader("Location", "http://192.168.4.1", true); server.send(302, ""); });

  server.on("/save_map", []() {
    prefs.begin("map", false);
    XLMinMap = server.arg("XLMinMap").toInt(); prefs.putInt("XLMinMap", XLMinMap);
    XLMaxMap = server.arg("XLMaxMap").toInt(); prefs.putInt("XLMaxMap", XLMaxMap);
    YLMinMap = server.arg("YLMinMap").toInt(); prefs.putInt("YLMinMap", YLMinMap);
    YLMaxMap = server.arg("YLMaxMap").toInt(); prefs.putInt("YLMaxMap", YLMaxMap);
    XRMinMap = server.arg("XRMinMap").toInt(); prefs.putInt("XRMinMap", XRMinMap);
    XRMaxMap = server.arg("XRMaxMap").toInt(); prefs.putInt("XRMaxMap", XRMaxMap);
    YRMinMap = server.arg("YRMinMap").toInt(); prefs.putInt("YRMinMap", YRMinMap);
    YRMaxMap = server.arg("YRMaxMap").toInt(); prefs.putInt("YRMaxMap", YRMaxMap);
    prefs.end();
    server.send(200, "text/plain", "✅ Saved Mapping ✅");
  });

  server.on("/get_map", []() {
    String json = "{";
    json += "\"XLMinMap\":" + String(XLMinMap) + ",";
    json += "\"XLMaxMap\":" + String(XLMaxMap) + ",";
    json += "\"YLMinMap\":" + String(YLMinMap) + ",";
    json += "\"YLMaxMap\":" + String(YLMaxMap) + ",";
    json += "\"XRMinMap\":" + String(XRMinMap) + ",";
    json += "\"XRMaxMap\":" + String(XRMaxMap) + ",";
    json += "\"YRMinMap\":" + String(YRMinMap) + ",";
    json += "\"YRMaxMap\":" + String(YRMaxMap);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/save_mac", []() {
    String mac = server.arg("mac");
    prefs.begin("config", false);
    prefs.putString("ReceiverMAC", mac);
    prefs.end();
    server.send(200, "text/plain", "✅ MAC saved: " + mac);
  });

  server.on("/get_mac", []() {
    prefs.begin("config", true);
    String mac = prefs.getString("ReceiverMAC", "00:00:00:00:00:00");
    prefs.end();
    server.send(200, "text/plain", mac);
  });

  server.begin();
  wifiActive = true;
  inCalibrationMode = true;
  Serial.println("   📶 WebServer Started");
}


void stopWeb() {
  if (!wifiActive) return;
  server.stop();
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
  wifiActive = false;
  inCalibrationMode = false;
  Serial.println("   🛑 WebServer Stopped");
}

void HandleBuzzer() {
  if (isTrimBuzzing) {
    if (millis() - TrimBuzzerStart >= BUZ_TIME) {
      ledcWriteTone(Buzzer_PIN, 0); 
      isTrimBuzzing = false;             
    }
  }
}

void TrimBuzzer() {
  ledcWriteNote(Buzzer_PIN, NOTE_B, 5); 
  TrimBuzzerStart = millis();     
  isTrimBuzzing = true;               
}

void HandleRollTrim() {
  static bool lastRollUp = false;
  static bool lastRollDown = false;
  
  bool currRollUp   = ButtonR; 
  bool currRollDown = ButtonL;

  // Roll UP
  if (currRollUp && !lastRollUp && currRollDown == false) {
    TrimBuzzer();
  }
  lastRollUp = currRollUp;

  // Roll DOWN
  if (currRollDown && !lastRollDown && currRollUp == false) {
    TrimBuzzer();
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
    startWeb();
    webCommand = 1;
    clickCountSWL = 0; 
  } 
  else if (clickCountSWL == 1 && (now - lastClickTimeSWL > clickInterval)) {
    if (currSWL == false) { 
      TrimBuzzer();
      Serial.println("Action: Pitch Trim +");
    } else {
      Serial.println("Ignored: Long Press detected");
    }
    
    clickCountSWL = 0;
  }

  if (clickCountSWR == 2) {
    stopWeb();
    webCommand = 0;
    clickCountSWR = 0;
  }
  else if (clickCountSWR == 1 && (now - lastClickTimeSWR > clickInterval)) {
    if (currSWR == false) {
      TrimBuzzer();
      Serial.println("Action: Pitch Trim -");
    } else {
      Serial.println("Ignored: Long Press detected");
    }
    
    clickCountSWR = 0;
  }
}

void OnSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
}

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_max_tx_power(72);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  prefs.begin("config", true);
  String macStr = prefs.getString("ReceiverMAC", "00:00:00:00:00:00");
  prefs.end();

  sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
         &ReceiverMAC[0], &ReceiverMAC[1], &ReceiverMAC[2],
         &ReceiverMAC[3], &ReceiverMAC[4], &ReceiverMAC[5]);

  esp_now_register_send_cb(OnSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, ReceiverMAC, 6);
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.print("📡 Receiver MAC Set: ");
  Serial.println(macStr);
}

// ================= Serial Console (Remote) =================
String rcBuf = "";
enum CalMode { CAL_NONE, CAL_MINMAX, CAL_CENTER };
CalMode calMode = CAL_NONE;
unsigned long lastCalPrint = 0;
long cXLsum = 0, cYLsum = 0, cXRsum = 0, cYRsum = 0, cCount = 0;  

void handleCalibrationMode() {
  if (calMode == CAL_NONE) return;

  int xl = readAverage(XL_PIN), yl = readAverage(YL_PIN);
  int xr = readAverage(XR_PIN), yr = readAverage(YR_PIN);

  if (calMode == CAL_MINMAX) {
    if (xl < XL_min) XL_min = xl;  if (xl > XL_max) XL_max = xl;
    if (yl < YL_min) YL_min = yl;  if (yl > YL_max) YL_max = yl;
    if (xr < XR_min) XR_min = xr;  if (xr > XR_max) XR_max = xr;
    if (yr < YR_min) YR_min = yr;  if (yr > YR_max) YR_max = yr;
    if (millis() - lastCalPrint > 400) {
      lastCalPrint = millis();
      Serial.printf("calibrating... XL[%d-%d] YL[%d-%d] XR[%d-%d] YR[%d-%d]\n",
                    XL_min, XL_max, YL_min, YL_max, XR_min, XR_max, YR_min, YR_max);
    }
  }
  else if (calMode == CAL_CENTER) {
    cXLsum += xl; cYLsum += yl; cXRsum += xr; cYRsum += yr; cCount++;
    if (millis() - lastCalPrint > 400) {
      lastCalPrint = millis();
      Serial.printf("calibrating... center XL:%ld YL:%ld XR:%ld YR:%ld\n",
                    cXLsum/cCount, cYLsum/cCount, cXRsum/cCount, cYRsum/cCount);
    }
  }
}

void handleSetMac(String line) {
  int q1 = line.indexOf('"');
  int q2 = line.lastIndexOf('"');
  if (q1 < 0 || q2 <= q1) { Serial.println("ERROR: use  set mac = \"AA:BB:CC:DD:EE:FF\""); return; }
  String mac = line.substring(q1 + 1, q2); mac.trim();

  int v[6];
  if (sscanf(mac.c_str(), "%x:%x:%x:%x:%x:%x", &v[0],&v[1],&v[2],&v[3],&v[4],&v[5]) != 6) {
    Serial.println("ERROR: invalid MAC format"); return;
  }
  prefs.begin("config", false);
  prefs.putString("ReceiverMAC", mac);
  prefs.end();
  esp_now_del_peer(ReceiverMAC);
  for (int i = 0; i < 6; i++) ReceiverMAC[i] = (uint8_t)v[i];
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, ReceiverMAC, 6);
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  Serial.println("MAC saved: " + mac);
}

void processRemoteCommand(String line) {
  line.trim();
  String low = line; low.toLowerCase();

  if (low == "calibrate minmax") {
    XL_min = YL_min = XR_min = YR_min = 4095;   
    XL_max = YL_max = XR_max = YR_max = 0;
    calMode = CAL_MINMAX;
    Serial.println("=== MinMax calibration started. โยกสติ๊กทั้งสองสุดทุกทิศ แล้วพิมพ์ 'save minmax' ===");
  }
  else if (low == "save minmax") {
    calMode = CAL_NONE;
    prefs.begin("joy", false);
    prefs.putInt("XL_min", XL_min); prefs.putInt("XL_max", XL_max);
    prefs.putInt("YL_min", YL_min); prefs.putInt("YL_max", YL_max);
    prefs.putInt("XR_min", XR_min); prefs.putInt("XR_max", XR_max);
    prefs.putInt("YR_min", YR_min); prefs.putInt("YR_max", YR_max);
    prefs.end();
    Serial.println("Min/Max saved");
  }
  else if (low == "calibrate center") {
    cXLsum = cYLsum = cXRsum = cYRsum = cCount = 0;
    calMode = CAL_CENTER;
    Serial.println("=== Center calibration started. ปล่อยสติ๊กอยู่กลาง แล้วพิมพ์ 'save center' ===");
  }
  else if (low == "save center") {
    if (cCount > 0) {
      XL_center = cXLsum / cCount; YL_center = cYLsum / cCount;
      XR_center = cXRsum / cCount; YR_center = cYRsum / cCount;
    } else { 
      XL_center = readAverage(XL_PIN); YL_center = readAverage(YL_PIN);
      XR_center = readAverage(XR_PIN); YR_center = readAverage(YR_PIN);
    }
    calMode = CAL_NONE;
    prefs.begin("joy", false);
    prefs.putInt("XL_center", XL_center); prefs.putInt("YL_center", YL_center);
    prefs.putInt("XR_center", XR_center); prefs.putInt("YR_center", YR_center);
    prefs.end();
    Serial.printf("Center saved: XL:%d YL:%d XR:%d YR:%d\n", XL_center, YL_center, XR_center, YR_center);
  }
  else if (low.startsWith("set mac")) {
    handleSetMac(line);  
  }
  else {
    Serial.print("Unknown command: "); Serial.println(line);
  }
}

void handleSerialConsole() {
  static unsigned long lastCharTime = 0;
  while (Serial.available()) {
    char c = Serial.read();
    lastCharTime = millis();
    if (c == '\n' || c == '\r') {
      if (rcBuf.length()) { processRemoteCommand(rcBuf); rcBuf = ""; }
    } else if (rcBuf.length() < 80) {
      rcBuf += c;
    }
  }
  if (rcBuf.length() && millis() - lastCharTime > 100) { 
    processRemoteCommand(rcBuf); rcBuf = "";
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(SWL_PIN, INPUT_PULLUP);
  pinMode(SWR_PIN, INPUT_PULLUP);
  pinMode(ButtonL_PIN, INPUT_PULLUP);
  pinMode(ButtonR_PIN, INPUT_PULLUP);
  pinMode(Buzzer_PIN, OUTPUT);
  ledcAttach(Buzzer_PIN, 2700, 10);
  loadReceiverMAC();
  loadStoredValues();
  loadMapValues();
  setupESPNow();
}

void loop() {
  HandleBuzzer();
  handleSerialConsole();
  handleCalibrationMode();
  if (wifiActive) {
    dnsServer.processNextRequest();
    server.handleClient();
  }
  unsigned long now = micros();
  if (now - lastReadMicro >= intervalMicro) {
    lastReadMicro = now;
    XL = readAverage(XL_PIN);
    YL = readAverage(YL_PIN);
    XR = readAverage(XR_PIN);
    YR = readAverage(YR_PIN);
    SWL = !digitalRead(SWL_PIN);
    SWR = !digitalRead(SWR_PIN);
    ButtonL = !digitalRead(ButtonL_PIN);
    ButtonR = !digitalRead(ButtonR_PIN);
    HandleRollTrim();    
    HandlePitchTrimAndWeb();
    mapJoystick();
    sendESPNow();
  }
} 