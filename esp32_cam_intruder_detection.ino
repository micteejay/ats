/*
 * ESP32-CAM Hybrid Intruder Detection System with PIR Motion Detection
 *
 * Features:
 * - HTTP Server Mode: Direct communication with Flutter app
 * - Client Mode: Uploads images and reports to your domain
 * - Camera Integration: Captures and uploads photos ONLY when motion detected
 * - PIR Motion Detection: Intelligent image capture based on movement
 * - Alarm System: Buzzer control with configurable duration
 * - Dual Communication: Local network + Internet connectivity
 */

#include <WiFi.h>
#include "esp_camera.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>

// ==== CAMERA MODEL SELECT (AI THINKER) ====
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ==== WiFi Credentials ====
const char *ssid = "Project";
const char *password = "12345678";

// ==== API Endpoints (Your Domain) ====
const char* galleryURL   = "https://intruder.micteejay.com.ng/gallerysend.php";
const char* statusURL    = "https://intruder.micteejay.com.ng/status.php";
const char* configURL    = "https://intruder.micteejay.com.ng/config.php";
const char* intruderURL  = "https://intruder.micteejay.com.ng/intruder_alert.php";
const char* falseURL     = "https://intruder.micteejay.com.ng/false_alarm.php";

// ==== Hardware Pins ====
#define BUZZER_PIN 12     // Buzzer pin
#define LED_PIN 4         // Built-in LED
#define RELAY_PIN 13      // External alarm relay
#define STATUS_LED 2      // Status indicator LED
#define PIR_PIN 14        // PIR motion sensor pin (GPIO14)

// ==== System State ====
bool alarmActive = false;
bool motionDetected = false;
bool lastMotionState = false;
int alarmDuration = 30; // default 30 seconds
unsigned long alarmStartTime = 0;
unsigned long lastImageUpload = 0;
unsigned long lastStatusReport = 0;
unsigned long lastMotionTime = 0;
unsigned long motionCooldown = 5000; // 5 seconds cooldown between captures
unsigned long lastPeriodicCheck = 0; // For non-blocking loop

// ==== PIR Configuration ====
const unsigned long PIR_WARMUP_TIME = 10000; // 10 seconds PIR warmup
const unsigned long MOTION_COOLDOWN = 5000;  // 5 seconds between motion events
const unsigned long IMAGE_UPLOAD_INTERVAL = 30000;  // 30 seconds
const unsigned long STATUS_REPORT_INTERVAL = 10000;  // 10 seconds
const unsigned long PERIODIC_CHECK_INTERVAL = 2000; // 2 seconds for periodic checks

// ==== HTTP Server ====
WebServer server(80);

// ==== JSON Document for parsing ====
StaticJsonDocument<1024> doc;

// ==== Forward declarations ====
void handleIntruderAlert();
void handleFalseAlarm();
void handleConfig();
void handleStatus();
void handleNotFound();
void triggerAlarm(double confidence);
void turnOffAlarm();
void connectWiFi();
void initCamera();
void initPIR();
void checkMotion();
void captureAndUploadImage();
void sendStatus();
void getConfig();
void checkAlarm();

// ==== WiFi Connect ====
void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// ==== Camera Init ====
void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_CIF;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x", err);
    ESP.restart();
  }
  Serial.println("Camera initialized successfully");
}

// ==== PIR Motion Sensor Init ====
void initPIR() {
  pinMode(PIR_PIN, INPUT);
  Serial.println("PIR motion sensor initialized");
  Serial.println("Waiting for PIR sensor to warm up...");

  // PIR sensors need warmup time
  delay(PIR_WARMUP_TIME);
  Serial.println("PIR sensor ready!");
}

// ==== Motion Detection ====
void checkMotion() {
  bool currentMotion = digitalRead(PIR_PIN) == HIGH;

  // Detect motion state change
  if (currentMotion != lastMotionState) {
    if (currentMotion) {
      // Motion detected
      motionDetected = true;
      lastMotionTime = millis();
      Serial.println("🚨 MOTION DETECTED! Preparing to capture image...");

      // Visual feedback
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);

      // Capture and upload image immediately
      captureAndUploadImage();

    } else {
      // Motion stopped
      motionDetected = false;
      Serial.println("Motion stopped");
    }
    lastMotionState = currentMotion;
  }

  // Check if it's time to capture another image (motion cooldown)
  if (motionDetected && (millis() - lastMotionTime > motionCooldown)) {
    if (millis() - lastImageUpload > IMAGE_UPLOAD_INTERVAL) {
      Serial.println("Motion cooldown expired, capturing follow-up image...");
      captureAndUploadImage();
    }
  }
}

// ==== Capture and Upload Image ====
void captureAndUploadImage() {
  if (millis() - lastImageUpload < IMAGE_UPLOAD_INTERVAL) {
    Serial.println("Image upload interval not reached, skipping...");
    return;
  }

  Serial.println("📸 Capturing image...");

  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  Serial.printf("Image captured: %dx%d, %d bytes\n",
                fb->width, fb->height, fb->len);

  WiFiClientSecure client;
  client.setInsecure(); // skip SSL cert

  HTTPClient http;
  http.begin(client, galleryURL);
  http.addHeader("Content-Type", "image/jpeg");

  int httpResponseCode = http.POST(fb->buf, fb->len);
  if (httpResponseCode > 0) {
    Serial.printf("✅ Image upload success, code: %d\n", httpResponseCode);
    lastImageUpload = millis();

    // Success feedback
    for (int i = 0; i < 3; i++) {
      digitalWrite(STATUS_LED, HIGH);
      delay(50);
      digitalWrite(STATUS_LED, LOW);
      delay(50);
    }
  } else {
    Serial.printf("❌ Upload failed, code: %d\n", httpResponseCode);

    // Error feedback
    for (int i = 0; i < 5; i++) {
      digitalWrite(STATUS_LED, HIGH);
      delay(200);
      digitalWrite(STATUS_LED, LOW);
      delay(200);
    }
  }

  http.end();
  esp_camera_fb_return(fb);
}

// ==== Send Status to Domain ====
void sendStatus() {
  if (millis() - lastStatusReport < STATUS_REPORT_INTERVAL) {
    return; // Not time to report yet
  }

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, statusURL);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<300> doc;
  doc["alarm_active"] = alarmActive;
  doc["motion_detected"] = motionDetected;
  doc["uptime_ms"] = millis();
  doc["ip"] = WiFi.localIP().toString();
  doc["last_motion_time"] = lastMotionTime;
  doc["last_image_upload"] = lastImageUpload;

  String payload;
  serializeJson(doc, payload);

  int code = http.POST(payload);
  Serial.printf("Status sent to domain: %d\n", code);
  http.end();

  lastStatusReport = millis();
}

// ==== Get Config from Domain ====
void getConfig() {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, configURL);

  int code = http.GET();
  if (code == 200) {
    String res = http.getString();
    Serial.println("Config from domain: " + res);

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, res);
    if (!error) {
      if (doc.containsKey("alarm_duration")) {
        alarmDuration = doc["alarm_duration"];
        Serial.printf("Updated alarm duration: %d sec\n", alarmDuration);
      }
      if (doc.containsKey("motion_cooldown")) {
        motionCooldown = doc["motion_cooldown"] * 1000; // Convert to milliseconds
        Serial.printf("Updated motion cooldown: %d ms\n", motionCooldown);
      }
    }
  }
  http.end();
}

// ==== Check Alarm from Domain ====
void checkAlarm() {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  // Check intruder alert
  http.begin(client, intruderURL);
  int code = http.GET();
  if (code == 200) {
    String res = http.getString();
    if (res.indexOf("trigger") >= 0) {
      Serial.println("🚨 Intruder Alert from domain! Ringing buzzer...");
      triggerAlarm(1.0); // High confidence from domain trigger
    }
  }
  http.end();

  // Check false alarm
  http.begin(client, falseURL);
  code = http.GET();
  if (code == 200) {
    String res = http.getString();
    if (res.indexOf("false") >= 0) {
      Serial.println("❌ False alarm reported from domain, stopping buzzer.");
      turnOffAlarm();
    }
  }
  http.end();
}

// ==== HTTP Server Handlers ====

void handleStatus() {
  StaticJsonDocument<400> status;
  status["ip"] = WiFi.localIP().toString();
  status["alarm_active"] = alarmActive;
  status["motion_detected"] = motionDetected;
  status["uptime_ms"] = millis();
  status["last_image_upload"] = lastImageUpload;
  status["last_status_report"] = lastStatusReport;
  status["last_motion_time"] = lastMotionTime;
  status["motion_cooldown"] = motionCooldown;

  String body;
  serializeJson(status, body);
  server.send(200, "application/json", body);
}

void handleIntruderAlert() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"Missing body\"}");
    return;
  }

  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  double confidence = doc["confidence"] | 0.8;
  int durationSec = doc["alarm_config"]["duration"] | (alarmDuration / 1000);
  if (durationSec > 0 && durationSec <= 300) {
    alarmDuration = durationSec * 1000;
  }

  Serial.println("/intruder_alert received from Flutter app");
  Serial.print("Confidence: ");
  Serial.println(confidence, 2);
  Serial.print("Duration (s): ");
  Serial.println(durationSec);

  triggerAlarm(confidence);

  StaticJsonDocument<128> res;
  res["status"] = "ok";
  res["alarm_active"] = alarmActive;
  String body;
  serializeJson(res, body);
  server.send(200, "application/json", body);
}

void handleFalseAlarm() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"Missing body\"}");
    return;
  }

  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  Serial.println("/false_alarm received from Flutter app");
  turnOffAlarm();
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleConfig() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"Missing body\"}");
    return;
  }

  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  if (doc.containsKey("alarm_duration")) {
    int sec = doc["alarm_duration"];
    if (sec > 0 && sec <= 300) {
      alarmDuration = sec * 1000;
      Serial.printf("Alarm duration updated to %d seconds\n", sec);
    }
  }

  if (doc.containsKey("motion_cooldown")) {
    int cooldown = doc["motion_cooldown"];
    if (cooldown > 0 && cooldown <= 60) {
      motionCooldown = cooldown * 1000; // Convert to milliseconds
      Serial.printf("Motion cooldown updated to %d seconds\n", cooldown);
    }
  }

  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleNotFound() {
  server.send(404, "application/json", "{\"error\":\"Not Found\"}");
}

// ==== Alarm Control Functions ====

void triggerAlarm(double confidence) {
  if (alarmActive) {
    Serial.println("Alarm already active; ignoring new trigger");
    return;
  }

  Serial.print("TRIGGERING ALARM - confidence: ");
  Serial.println(confidence, 2);

  alarmActive = true;
  alarmStartTime = millis();

  // Turn on alarm outputs
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(RELAY_PIN, HIGH);

  // Start buzzer with varying frequency based on confidence
  if (confidence > 0.8) {
    tone(BUZZER_PIN, 2000); // 2kHz
  } else if (confidence > 0.5) {
    tone(BUZZER_PIN, 1500); // 1.5kHz
  } else {
    tone(BUZZER_PIN, 1000); // 1kHz
  }

  // Visual feedback
  for (int i = 0; i < 6; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
  }
  digitalWrite(STATUS_LED, HIGH);
}

void turnOffAlarm() {
  alarmActive = false;
  digitalWrite(BUZZER_PIN, LOW);
  noTone(BUZZER_PIN);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(STATUS_LED, HIGH);
  Serial.println("Alarm turned off");
}

// ==== Setup and Loop ====

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32-CAM Hybrid Intruder Detection System with PIR Motion Detection");

  // Initialize pins
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(PIR_PIN, INPUT);

  // Turn off all outputs initially
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  digitalWrite(STATUS_LED, LOW);

  // Connect to WiFi
  connectWiFi();

  // Initialize camera
  initCamera();

  // Initialize PIR sensor
  initPIR();

  // Configure HTTP server endpoints
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/intruder_alert", HTTP_POST, handleIntruderAlert);
  server.on("/false_alarm", HTTP_POST, handleFalseAlarm);
  server.on("/config", HTTP_POST, handleConfig);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");

  // Turn on status LED when ready
  digitalWrite(STATUS_LED, HIGH);

  Serial.println("System ready! Operating in hybrid mode with PIR motion detection:");
  Serial.println("- HTTP Server: Direct Flutter app communication");
  Serial.println("- Client Mode: Domain communication & intelligent image uploads");
  Serial.println("- PIR Motion Detection: Images captured only when motion detected");
}

void loop() {
  // Handle HTTP server requests (Flutter app communication)
  server.handleClient();

  // Auto stop alarm after duration
  if (alarmActive && (millis() - alarmStartTime > (unsigned long)alarmDuration)) {
    turnOffAlarm();
  }

  // Check for motion as often as possible for responsiveness
  checkMotion();

  // Perform periodic tasks like domain communication without blocking
  if (millis() - lastPeriodicCheck > PERIODIC_CHECK_INTERVAL) {
    lastPeriodicCheck = millis();

    sendStatus();     // Report status to your domain (has its own interval)
    getConfig();      // Fetch latest config from your domain
    checkAlarm();     // Check for alarm triggers from your domain
  }
}
