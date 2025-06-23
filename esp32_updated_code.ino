#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

// WiFi credentials - CHANGE THESE!
const char* ssid = "your_wifi_name";        // ← Put your WiFi name here
const char* password = "your_wifi_password"; // ← Put your WiFi password here

#define SERVO_X_PIN 12
#define SERVO_Y_PIN 13

Servo servoX;
Servo servoY;
WebServer server(80);

// Camera configuration for ESP32-CAM
#define PWDN_GPIO_NUM     32
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

void setup() {
  Serial.begin(115200);
  
  // Initialize servos
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  servoX.write(90);
  servoY.write(90);
  
  // Initialize camera
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
  config.frame_size = FRAMESIZE_SVGA;  // Better resolution
  config.jpeg_quality = 12;
  config.fb_count = 2;  // Use 2 frame buffers

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/stream", handleStream);
  server.on("/capture", handleCapture);  // NEW: Single frame capture
  server.on("/control", HTTP_POST, handleControl);
  
  server.begin();
  Serial.println("Server started");
  Serial.println("Stream URL: http://" + WiFi.localIP().toString() + "/stream");
  Serial.println("Capture URL: http://" + WiFi.localIP().toString() + "/capture");
}

void loop() {
  server.handleClient();
  delay(1); // Small delay to prevent watchdog issues
}

void handleRoot() {
  String html = "<html><body>";
  html += "<h1>ESP32-CAM Live Stream</h1>";
  html += "<p>Server is running!</p>";
  html += "<p><a href='/stream'>View Stream</a></p>";
  html += "<p><a href='/capture'>Capture Single Frame</a></p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// NEW: Single frame capture function
void handleCapture() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "-1");
  
  server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleStream() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  client.print(response);
  
  while (client.connected()) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      break;
    }
    
    if (fb->len > 0) {
      client.print("--frame\r\n");
      client.print("Content-Type: image/jpeg\r\n");
      client.print("Content-Length: ");
      client.print(fb->len);
      client.print("\r\n\r\n");
      client.write(fb->buf, fb->len);
      client.print("\r\n");
    }
    
    esp_camera_fb_return(fb);
    
    if (!client.connected()) {
      break;
    }
    
    delay(50); // Control frame rate
  }
  
  client.stop();
  Serial.println("Client disconnected");
}

void handleControl() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, server.arg("plain"));
    
    int targetX = doc["servoX"];
    int targetY = doc["servoY"];
    
    // Constrain values for MG966R servos (0-180 degrees)
    targetX = constrain(targetX, 0, 180);
    targetY = constrain(targetY, 0, 180);
    
    // Smooth servo movement for better tracking
    moveServosSmooth(targetX, targetY);
    
    server.send(200, "text/plain", "OK");
    Serial.println("Target: X=" + String(targetX) + "°, Y=" + String(targetY) + "°");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void moveServosSmooth(int targetX, int targetY) {
  // Get current positions
  int currentX = servoX.read();
  int currentY = servoY.read();
  
  // Calculate steps needed
  int stepsX = abs(targetX - currentX);
  int stepsY = abs(targetY - currentY);
  int maxSteps = max(stepsX, stepsY);
  
  // If small movement, move directly
  if (maxSteps <= 5) {
    servoX.write(targetX);
    servoY.write(targetY);
    return;
  }
  
  // Smooth movement for larger distances
  for (int step = 1; step <= maxSteps; step += 2) {  // Move in steps of 2
    int newX = currentX + (targetX - currentX) * step / maxSteps;
    int newY = currentY + (targetY - currentY) * step / maxSteps;
    
    servoX.write(newX);
    servoY.write(newY);
    delay(20);  // Small delay for smooth movement
  }
  
  // Ensure final position
  servoX.write(targetX);
  servoY.write(targetY);
}
