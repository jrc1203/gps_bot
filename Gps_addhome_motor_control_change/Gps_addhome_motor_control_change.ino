// =======================================================================
//   AUTONOMOUS GPS BACKTRACKING BOT - ESP32 FIRMWARE (SINGLE FILE)
//   Modified with an improved Motor Control System and Mobile-Friendly UI with Live Compass
// =======================================================================

// --- Core Libraries ---
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <ArduinoJson.h>
#include <vector>

// --- Sensor Libraries ---
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

// --- WiFi Credentials ---
const char* ssid = "MySpyCar";
const char* password = "123456789";

// --- GPS Configuration ---
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial ss(2);

// --- Motor Driver (L298N) Pin & Logic Configuration ---
struct MOTOR_PINS {
  int pinEn;
  int pinIN1;
  int pinIN2;
  int pwmChannel;
};

std::vector<MOTOR_PINS> motorPins = {
  {33, 14, 12, 0}, // RIGHT_MOTOR Pins (En, IN1, IN2, PWM Channel)
  {25, 26, 27, 1}  // LEFT_MOTOR  Pins (En, IN1, IN2, PWM Channel)
};

#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define STOP 0

#define RIGHT_MOTOR 0
#define LEFT_MOTOR  1

#define FORWARD   1
#define BACKWARD -1

const int PWMFreq = 1000;
const int PWMResolution = 8;
int motorSpeed = 150;

// --- Compass Sensor (QMC5883L) ---
QMC5883LCompass compass;
float currentHeading = 0.0; // Global variable to store compass heading

#define CALIBRATION_X_OFFSET -17.00
#define CALIBRATION_Y_OFFSET -654.00
#define CALIBRATION_Z_OFFSET -308.00
#define CALIBRATION_X_SCALE  0.76
#define CALIBRATION_Y_SCALE  0.78
#define CALIBRATION_Z_SCALE  2.46

// --- Navigation & State Management ---
enum BotState { IDLE, EXPLORING, BACKTRACKING, EMERGENCY_STOP, MANUAL_CONTROL };
BotState currentState = IDLE;

struct GPSCoordinate {
  double lat;
  double lon;
};

std::vector<GPSCoordinate> loggedPath;
int backtrackingWaypointIndex = -1;
const double WAYPOINT_RADIUS_METERS = 2.0;
const double LOG_DISTANCE_METERS = 3.0;

// --- Web Server & WebSocket ---
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// =======================================================================
// EMBEDDED HTML PAGE WITH LIVE COMPASS (Updated)
// =======================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Bot Command & Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no" />
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
  <style>
    body, html { margin: 0; padding: 0; height: 100vh; width: 100vw; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; overflow: hidden; }
    .grid-container { display: grid; grid-template-columns: 320px 1fr; grid-template-rows: 100vh; height: 100%; width: 100%; }
    #map { height: 100%; width: 100%; background-color: #333; }
    .panel { background-color: #f0f2f5; padding: 15px; display: flex; flex-direction: column; overflow-y: auto; box-sizing: border-box; }
    .panel h2 { margin: 0 0 15px 0; text-align: center; }
    
    .controls button, .home-controls button { 
      display: block; width: 100%; padding: 15px; margin-bottom: 10px; font-size: 16px; 
      border: none; border-radius: 5px; color: white; cursor: pointer; transition: transform 0.1s ease;
    }
    .controls button:active, .home-controls button:active { transform: scale(0.97); }

    #btn-explore { background-color: #28a745; }
    #btn-return { background-color: #007bff; }
    #btn-stop { background-color: #dc3545; }
    #btn-add-home { background-color: #ffc107; color: #212529; }
    #btn-reset-home { background-color: #6c757d; }

    .status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 20px; }
    .info-box { background: #fff; padding: 10px; border-radius: 5px; box-shadow: 0 1px 3px rgba(0,0,0,0.1); text-align: center;}
    .info-box .label { font-size: 12px; color: #6c757d; }
    .info-box .value { font-size: 18px; font-weight: bold; }
    #state { color: #fd7e14; }

    .section-divider { margin-top: 15px; padding-top: 15px; border-top: 2px solid #dee2e6; }
    .section-divider h3 { margin: 0 0 10px 0; font-size: 14px; color: #495057; text-align: center; }
    
    /* Compass Styles */
    .compass-container { text-align: center; }
    .compass { position: relative; width: 120px; height: 120px; margin: 0 auto; background-color: #e9ecef; border-radius: 50%; border: 2px solid #ccc; }
    .compass-arrow { width: 0; height: 0; border-left: 12px solid transparent; border-right: 12px solid transparent; border-bottom: 50px solid #dc3545; position: absolute; left: 50%; top: 50%; transform-origin: center calc(100% - 12px); transition: transform 0.2s linear; }
    .compass-center-dot { width: 10px; height: 10px; background: #333; border-radius: 50%; position: absolute; top: 55px; left: 55px; z-index: 10;}
    .compass-heading { font-size: 18px; font-weight: bold; color: #007bff; margin-top: 5px; }
    .cardinal-point { position: absolute; width: 100%; text-align: center; font-weight: bold; color: #495057; }
    .north { top: -20px; }
    .south { bottom: -20px; }
    .east { right: -20px; top: 50%; transform: translateY(-50%); }
    .west { left: -20px; top: 50%; transform: translateY(-50%); }


    /* Manual Controls */
    .direction-pad { display: grid; grid-template-columns: 1fr 1fr 1fr; grid-template-rows: 1fr 1fr; gap: 5px; height: 90px; }
    .direction-btn { border: none; border-radius: 5px; background-color: #17a2b8; color: white; cursor: pointer; font-size: 14px; font-weight: bold; }
    .direction-btn:active { transform: scale(0.95); }
    #btn-manual-stop { background-color: #dc3545; grid-column: 2; }
    #btn-forward { grid-column: 2; grid-row: 1; }
    #btn-left { grid-column: 1; grid-row: 2; }
    #btn-right { grid-column: 3; grid-row: 2; }
    #btn-backward { grid-column: 2; grid-row: 2; }
    
    @media screen and (max-width: 768px) {
        body, html { overflow: visible; }
        .grid-container { grid-template-columns: 100%; grid-template-rows: auto 1fr; height: 100vh; }
        .panel { grid-row: 1; max-height: 55vh; overflow-y: auto; }
        #map { grid-row: 2; height: 100%; }
    }
  </style>
</head>
<body>
  <div class="grid-container">
    <div class="panel">
      <h2>Bot C&C</h2>
      <div class="controls">
        <button id="btn-explore" onclick="sendCommand('explore')">Start Exploring</button>
        <button id="btn-return" onclick="sendCommand('return')">Return to Home</button>
        <button id="btn-stop" onclick="sendCommand('stop')">EMERGENCY STOP</button>
      </div>
      
      <div class="home-controls section-divider">
        <h3>HOME MANAGEMENT</h3>
        <button id="btn-add-home" onclick="sendCommand('add_home')">Set Current as Home</button>
        <button id="btn-reset-home" onclick="sendCommand('reset_home')">Reset Home & Path</button>
      </div>
      
      <div class="status-grid">
        <div class="info-box"><div class="label">STATE</div><div class="value" id="state">CONNECTING...</div></div>
        <div class="info-box"><div class="label">GPS</div><div class="value" id="gps-status">NO FIX</div></div>
        <div class="info-box"><div class="label">SATELLITES</div><div class="value" id="sats">--</div></div>
        <div class="info-box"><div class="label">SPEED (KM/H)</div><div class="value" id="speed">--</div></div>
        <div class="info-box"><div class="label">PATH</div><div class="value" id="points">--</div></div>
        <div class="info-box"><div class="label">HOME</div><div class="value" id="home-coords">N/A</div></div>
      </div>
      <div class="info-box" style="margin-top:10px;"><div class="label">CURRENT LOCATION</div><div class="value" id="current-coords">N/A</div></div>
      
      <div class="compass-container section-divider">
          <h3>HEADING</h3>
          <div class="compass">
              <div class="cardinal-point north">N</div>
              <div class="cardinal-point east">E</div>
              <div class="cardinal-point south">S</div>
              <div class="cardinal-point west">W</div>
              <div class="compass-arrow" id="compass-arrow" style="transform: translate(-50%, -50%) rotate(0deg);"></div>
              <div class="compass-center-dot"></div>
          </div>
          <div class="compass-heading" id="compass-heading-value">--°</div>
      </div>
      
      <div class="car-controls section-divider">
        <h3>MANUAL CONTROLS</h3>
        <div class="speed-control"><label for="speed-slider">Motor Speed</label><input type="range" id="speed-slider" class="speed-slider" min="50" max="255" value="150" onchange="updateSpeed(this.value)"><div class="speed-value" id="speed-display">150</div></div>
        <div class="direction-pad">
          <button id="btn-forward" class="direction-btn" onmousedown="sendManualCommand('manual_forward')" onmouseup="sendManualCommand('manual_stop')" ontouchstart="sendManualCommand('manual_forward')" ontouchend="sendManualCommand('manual_stop')">Forward</button>
          <button id="btn-manual-stop" class="direction-btn" onclick="sendManualCommand('manual_stop')">STOP</button>
          <button id="btn-left" class="direction-btn" onmousedown="sendManualCommand('manual_left')" onmouseup="sendManualCommand('manual_stop')" ontouchstart="sendManualCommand('manual_left')" ontouchend="sendManualCommand('manual_stop')">Left</button>
          <button id="btn-right" class="direction-btn" onmousedown="sendManualCommand('manual_right')" onmouseup="sendManualCommand('manual_stop')" ontouchstart="sendManualCommand('manual_right')" ontouchend="sendManualCommand('manual_stop')">Right</button>
          <button id="btn-backward" class="direction-btn" onmousedown="sendManualCommand('manual_backward')" onmouseup="sendManualCommand('manual_stop')" ontouchstart="sendManualCommand('manual_backward')" ontouchend="sendManualCommand('manual_stop')">Backward</button>
        </div>
      </div>
    </div>
    <div id="map"></div>
  </div>
  
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <script>
    const map = L.map('map').setView([20, 0], 2);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { attribution: '© OpenStreetMap' }).addTo(map);

    let botMarker = null, homeMarker = null;
    let pathPolyline = L.polyline([], {color: '#007bff', weight: 5}).addTo(map);
    var ws;

    function sendCommand(cmd) { if (ws && ws.readyState === WebSocket.OPEN) { ws.send(JSON.stringify({ "command": cmd })); } }
    function sendManualCommand(cmd) { if (ws && ws.readyState === WebSocket.OPEN) { ws.send(JSON.stringify({ "command": cmd })); } }
    function updateSpeed(value) {
      document.getElementById('speed-display').textContent = value;
      if (ws && ws.readyState === WebSocket.OPEN) { ws.send(JSON.stringify({ "command": "set_speed", "speed": parseInt(value) })); }
    }

    function connectWebSocket() {
      ws = new WebSocket(`ws://${location.host}/ws`);
      
      ws.onopen = function() { console.log("WebSocket connected!"); document.getElementById('state').textContent = 'CONNECTED'; };

      ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        
        document.getElementById('state').textContent = data.state || 'N/A';
        document.getElementById('gps-status').textContent = data.valid ? 'VALID' : 'NO FIX';
        document.getElementById('sats').textContent = data.sats || '--';
        document.getElementById('speed').textContent = data.speed || '--';
        document.getElementById('points').textContent = data.points || '0';
        
        if (data.valid && data.lat != 0) {
          const latLng = [data.lat, data.lon];
          document.getElementById('current-coords').textContent = `${data.lat.toFixed(5)}, ${data.lon.toFixed(5)}`;
          if (!botMarker) { botMarker = L.marker(latLng).addTo(map); map.setView(latLng, 18); } 
          else { botMarker.setLatLng(latLng); map.panTo(latLng); }
        }
        
        if (data.path && data.path.length > 0) {
          const pathCoords = data.path.map(p => [p.lat, p.lon]);
          pathPolyline.setLatLngs(pathCoords);
          const homeCoords = data.path[0];
          document.getElementById('home-coords').textContent = `${homeCoords.lat.toFixed(5)}, ${homeCoords.lon.toFixed(5)}`;
          if(!homeMarker) {
            homeMarker = L.marker([homeCoords.lat, homeCoords.lon], { icon: L.icon({ iconUrl: 'https://cdn.rawgit.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png', shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png', iconSize: [25, 41], iconAnchor: [12, 41] }) }).addTo(map).bindPopup("Home").openPopup();
          } else { homeMarker.setLatLng([homeCoords.lat, homeCoords.lon]); }
        } else {
          pathPolyline.setLatLngs([]);
          document.getElementById('home-coords').textContent = 'N/A';
          if (homeMarker) { map.removeLayer(homeMarker); homeMarker = null; }
        }

        // Handle Compass Update
        if (data.heading !== undefined) {
          const arrow = document.getElementById('compass-arrow');
          const headingValue = document.getElementById('compass-heading-value');
          arrow.style.transform = `translate(-50%, -50%) rotate(${data.heading}deg)`;
          headingValue.textContent = `${Math.round(data.heading)}°`;
        }
      };

      ws.onclose = function() {
        console.log("WebSocket disconnected. Reconnecting in 2s...");
        document.getElementById('state').textContent = 'DISCONNECTED';
        setTimeout(connectWebSocket, 2000);
      };
      ws.onerror = function(err) { console.error('WebSocket error:', err); ws.close(); };
    }
    window.onload = connectWebSocket;
  </script>
</body>
</html>
)rawliteral";

// =======================================================================
// MOTOR CONTROL FUNCTIONS 
// =======================================================================

void rotateMotor(int motorNumber, int motorDirection) {
  if (motorDirection == FORWARD) {
    digitalWrite(motorPins[motorNumber].pinIN1, HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  } else if (motorDirection == BACKWARD) {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, HIGH);
  } else {
    digitalWrite(motorPins[motorNumber].pinIN1, LOW);
    digitalWrite(motorPins[motorNumber].pinIN2, LOW);
  }
}

void moveCar(int command) {
  switch(command) {
    case UP:      rotateMotor(RIGHT_MOTOR, FORWARD); rotateMotor(LEFT_MOTOR, FORWARD); break;
    case DOWN:    rotateMotor(RIGHT_MOTOR, BACKWARD); rotateMotor(LEFT_MOTOR, BACKWARD); break;
    case LEFT:    rotateMotor(RIGHT_MOTOR, FORWARD); rotateMotor(LEFT_MOTOR, BACKWARD); break;
    case RIGHT:   rotateMotor(RIGHT_MOTOR, BACKWARD); rotateMotor(LEFT_MOTOR, FORWARD); break;
    case STOP:
    default:      rotateMotor(RIGHT_MOTOR, STOP); rotateMotor(LEFT_MOTOR, STOP); break;
  }
}

void setMotorSpeed() {
  ledcWrite(motorPins[RIGHT_MOTOR].pwmChannel, motorSpeed);
  ledcWrite(motorPins[LEFT_MOTOR].pwmChannel, motorSpeed);
}

void stopMotors() {
  moveCar(STOP);
  ledcWrite(motorPins[RIGHT_MOTOR].pwmChannel, 0);
  ledcWrite(motorPins[LEFT_MOTOR].pwmChannel, 0);
}

void moveForward() { setMotorSpeed(); moveCar(UP); }
void moveBackward() { setMotorSpeed(); moveCar(DOWN); }
void turnRight() { setMotorSpeed(); moveCar(RIGHT); }
void turnLeft() { setMotorSpeed(); moveCar(LEFT); }


// =======================================================================
// HOME MANAGEMENT FUNCTIONS (Unchanged)
// =======================================================================
void addCurrentLocationAsHome() {
  if (gps.location.isValid()) {
    stopMotors();
    currentState = IDLE;
    loggedPath.clear();
    backtrackingWaypointIndex = -1;
    loggedPath.push_back({gps.location.lat(), gps.location.lng()});
    Serial.println("New home location set.");
  } else {
    Serial.println("Cannot set home: GPS not valid");
  }
}

void resetHomeAndPath() {
  stopMotors();
  currentState = IDLE;
  loggedPath.clear();
  backtrackingWaypointIndex = -1;
  Serial.println("Home and path data reset");
}

// =======================================================================
// NAVIGATION LOGIC (Updated to use global heading)
// =======================================================================
void navigateToWaypoint() {
    if (backtrackingWaypointIndex < 0 || !gps.location.isValid()) {
        stopMotors();
        currentState = IDLE;
        return;
    }

    GPSCoordinate target = loggedPath[backtrackingWaypointIndex];
    double distanceToTarget = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), target.lat, target.lon);

    if (distanceToTarget < WAYPOINT_RADIUS_METERS) {
        backtrackingWaypointIndex--;
        if (backtrackingWaypointIndex < 0) {
            currentState = IDLE;
            stopMotors();
            return;
        }
    }

    double targetBearing = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), target.lat, target.lon);
    
    // NOTE: We now use the global 'currentHeading' variable, which is updated continuously in the main loop()
    double headingError = targetBearing - currentHeading; 
    
    if (headingError > 180) headingError -= 360;
    if (headingError < -180) headingError += 360;

    if (abs(headingError) < 15) { moveForward(); } 
    else if (headingError > 0) { turnRight(); } 
    else { turnLeft(); }
}

// =======================================================================
// WEBSOCKET AND DATA HANDLING (Updated to send heading)
// =======================================================================
void broadcastData() {
    String jsonString;
    StaticJsonDocument<1024> doc; 

    doc["lat"] = gps.location.lat();
    doc["lon"] = gps.location.lng();
    doc["sats"] = gps.satellites.value();
    doc["speed"] = gps.speed.kmph();
    doc["valid"] = gps.location.isValid();
    doc["heading"] = currentHeading; // <-- ADDED: Send current compass heading

    switch(currentState) {
        case IDLE: doc["state"] = "IDLE"; break;
        case EXPLORING: doc["state"] = "EXPLORING"; break;
        case BACKTRACKING: doc["state"] = "BACKTRACKING"; break;
        case EMERGENCY_STOP: doc["state"] = "STOPPED"; break;
        case MANUAL_CONTROL: doc["state"] = "MANUAL"; break;
    }

    doc["points"] = loggedPath.size();

    if (loggedPath.size() > 0) {
        JsonArray path = doc.createNestedArray("path");
        for(const auto& p : loggedPath) {
            JsonObject point = path.createNestedObject();
            point["lat"] = p.lat;
            point["lon"] = p.lon;
        }
    }

    serializeJson(doc, jsonString);
    ws.textAll(jsonString);
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.printf("Client #%u connected\n", client->id());
        broadcastData(); 
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("Client #%u disconnected\n", client->id());
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg; 
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            StaticJsonDocument<128> doc;
            if (deserializeJson(doc, (char*)data) == DeserializationError::Ok) {
                const char* command = doc["command"];
                
                if (strcmp(command, "explore") == 0) {
                    if (!loggedPath.empty()) { currentState = EXPLORING; } 
                    else { Serial.println("Cannot explore: No home set"); }
                } else if (strcmp(command, "return") == 0) {
                    if (loggedPath.size() > 1) { backtrackingWaypointIndex = loggedPath.size() - 2; currentState = BACKTRACKING; } 
                    else { Serial.println("Cannot return: Not enough path points"); }
                } else if (strcmp(command, "stop") == 0) {
                    currentState = EMERGENCY_STOP;
                } else if (strcmp(command, "add_home") == 0) {
                    addCurrentLocationAsHome();
                } else if (strcmp(command, "reset_home") == 0) {
                    resetHomeAndPath();
                } else if (strcmp(command, "manual_forward") == 0) {
                    currentState = MANUAL_CONTROL; moveForward();
                } else if (strcmp(command, "manual_backward") == 0) {
                    currentState = MANUAL_CONTROL; moveBackward();
                } else if (strcmp(command, "manual_left") == 0) {
                    currentState = MANUAL_CONTROL; turnLeft();
                } else if (strcmp(command, "manual_right") == 0) {
                    currentState = MANUAL_CONTROL; turnRight();
                } else if (strcmp(command, "manual_stop") == 0) {
                    stopMotors();
                    if (currentState == MANUAL_CONTROL) { currentState = IDLE; }
                } else if (strcmp(command, "set_speed") == 0) {
                    int newSpeed = doc["speed"];
                    if (newSpeed >= 50 && newSpeed <= 255) {
                        motorSpeed = newSpeed;
                        if(currentState != IDLE && currentState != EMERGENCY_STOP) { setMotorSpeed(); }
                    }
                }
            }
        }
    }
}

// =======================================================================
// SETUP AND LOOP (Updated for continuous compass read)
// =======================================================================
void setup() {
    Serial.begin(115200);
    ss.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    Wire.begin();

    for (int i = 0; i < motorPins.size(); i++) {
        ledcSetup(motorPins[i].pwmChannel, PWMFreq, PWMResolution);
        ledcAttachPin(motorPins[i].pinEn, motorPins[i].pwmChannel);
        pinMode(motorPins[i].pinIN1, OUTPUT);
        pinMode(motorPins[i].pinIN2, OUTPUT);
    }
    stopMotors(); 
    Serial.println("Motor control system initialized.");
    
    compass.init();
    compass.setCalibrationOffsets(CALIBRATION_X_OFFSET, CALIBRATION_Y_OFFSET, CALIBRATION_Z_OFFSET);
    compass.setCalibrationScales(CALIBRATION_X_SCALE, CALIBRATION_Y_SCALE, CALIBRATION_Z_SCALE);
    Serial.println("QMC5883L compass initialized.");

    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.println("\nWiFi connected! IP: " + WiFi.localIP().toString());
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send_P(200, "text/html", index_html); });
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    ElegantOTA.begin(&server);
    server.begin();
}

unsigned long lastBroadcastTime = 0;
unsigned long lastCompassReadTime = 0;

void loop() {
    ws.cleanupClients();
    while (ss.available() > 0) { gps.encode(ss.read()); }

    // Read the compass periodically (e.g., 10 times per second) to update the global variable
    if(millis() - lastCompassReadTime > 100) {
      lastCompassReadTime = millis();
      compass.read();
      currentHeading = compass.getAzimuth();
    }
    
    switch(currentState) {
        case EXPLORING:
            moveForward();
            if (gps.location.isValid()) {
                if (loggedPath.empty()) { loggedPath.push_back({gps.location.lat(), gps.location.lng()}); } 
                else {
                    if (TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), loggedPath.back().lat, loggedPath.back().lon) > LOG_DISTANCE_METERS) {
                        loggedPath.push_back({gps.location.lat(), gps.location.lng()});
                    }
                }
            }
            break;
        case BACKTRACKING:    navigateToWaypoint(); break;
        case MANUAL_CONTROL:  break;
        case IDLE:
        case EMERGENCY_STOP:  stopMotors(); break;
    }

    if (millis() - lastBroadcastTime > 1000) {
        lastBroadcastTime = millis();
        broadcastData();
    }
    
    ElegantOTA.loop();
}