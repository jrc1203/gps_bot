# 🚗 Autonomous GPS Backtracking Bot Guide 🌏

Welcome to the **Gps_addhome_motor_control_change** project for ESP32! This bot uses GPS and compass sensors to autonomously explore, backtrack to a home location, and provides a mobile-friendly web interface for control. This guide is designed for students and beginners. Let's make robotics fun! 😃

---

## 📦 **Project Overview**

- **Hardware:** ESP32, GPS Module, QMC5883L Compass, L298N Motor Driver
- **Features:**
  - Autonomous exploration and path logging
  - Backtracking to home using GPS and live compass
  - Manual control via web interface
  - Real-time map and live heading visualization
  - Fully wireless control (WiFi)

---

## 🛠️ **Setup Instructions**

### 1. **Hardware Connections**

- **Motors:** Connect two DC motors via L298N (see pin definitions below).
- **GPS Module:** Use RX (GPIO 16) and TX (GPIO 17).
- **Compass:** Connect QMC5883L via I2C (Wire library).
- **ESP32:** All logic and networking runs here.

### 2. **Pin Configuration**

| Motor   | Enable (En) | IN1 | IN2 | PWM Channel |
|---------|-------------|-----|-----|-------------|
| Right   | 33          | 14  | 12  | 0           |
| Left    | 25          | 26  | 27  | 1           |

---

## 📡 **WiFi Setup**

- **SSID:** `MySpyCar`
- **Password:** `123456789`

You can change these in the code at the top:
```cpp
const char* ssid = "MySpyCar";
const char* password = "123456789";
```
---

## 🧭 **Sensors**

- **GPS:** TinyGPSPlus library for location & path
- **Compass:** QMC5883L for heading/azimuth

Calibration constants are set in the code for accuracy.

---

## 🌐 **Web Interface Features**

Open your browser and connect to the bot's IP (shown in the Serial Monitor after boot).

### ✨ **What's in the UI?**
- **Live map** using Leaflet.js
- **Compass** with real-time heading
- **Status boxes:** satellites, speed, state, home, path points
- **Manual control buttons** (forward, back, left, right, stop)
- **Speed slider**
- **Set/Reset home location**
- **Emergency stop**

---

## 🤖 **How the Bot Works**

### 1. **Exploring Mode** 🏞️
- Click **Start Exploring**.
- The bot moves forward, logging GPS waypoints every 3 meters.

### 2. **Set Home** 🏠
- Use **Set Current as Home** to save the current location as "home."
- This clears previous paths and starts fresh.

### 3. **Backtracking Mode** 🧭
- Click **Return to Home**.
- The bot calculates bearing to the next waypoint and adjusts heading using the compass.
- Turns left or right to minimize heading error.
- Stops once it reaches "home."

### 4. **Manual Mode** 🎮
- Use the direction pad for manual driving.
- Adjust speed with the slider.

### 5. **Emergency Stop** 🛑
- Stops motors immediately for safety.

---

## ⚙️ **Motor Control Logic**

- **Forward:** Both motors forward
- **Backward:** Both motors backward
- **Left:** Right motor forward, left backward (pivot)
- **Right:** Right motor backward, left forward (pivot)
- **Stop:** Both motors stopped

Speed is set using PWM (default 150; range 50-255).

---

## 📝 **Code Highlights**

- **Navigation Logic:** Uses GPS for location and compass for heading correction.
- **WebSocket:** Real-time updates between ESP32 and web UI.
- **ElegantOTA:** OTA firmware updates from browser.

---

## 🚦 **States**

| State            | Description                       |
|------------------|-----------------------------------|
| IDLE             | Waiting for command               |
| EXPLORING        | Logging path                      |
| BACKTRACKING     | Returning to home                 |
| MANUAL_CONTROL   | User controlling directly         |
| EMERGENCY_STOP   | All motors stopped                |

---

## 🧑‍💻 **How to Upload & Run**

1. **Install libraries:**
   - TinyGPSPlus
   - ESPAsyncWebServer
   - ElegantOTA
   - ArduinoJson
   - QMC5883LCompass

2. **Flash the code to your ESP32.**

3. **Open Serial Monitor:** Check IP address.

4. **Connect to WiFi:** Use the given SSID and password.

5. **Open browser:** Go to the bot's IP for the control panel.

---

## 🌟 **Tips for Students**

- Try changing the speed and see how it affects navigation! ⚡
- Experiment with different home locations. 🏠
- Watch the compass heading update live as you move the bot! 🧭
- Use manual control for tricky spots. 🎮
- Reset home and try a new path anytime. 🔄

---

## ❓ **Troubleshooting**

- **No GPS fix?** Make sure the module is outdoors and connected properly.
- **Motors not moving?** Double-check wiring and pin numbers.
- **Web UI not loading?** Ensure you're connected to the bot's WiFi network.

---

## 🚀 **Ready to Explore!**

Have fun building and learning with your autonomous GPS bot!  
Feel free to tweak the code and add new features.  
Robotics is all about experimenting! 🤩

---
**Made for students & makers by [jrc1203](https://github.com/jrc1203)!**
