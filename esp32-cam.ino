#include <WiFi.h>
#include <WebSocketsClient.h>
#include "esp_camera.h"

// ===== WS PATH cố định, host/port sẽ nhận từ ESP32-S3 =====
static const char* WS_PATH = "/cam";
WebSocketsClient ws;

// ===== AI THINKER PINOUT =====
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

// ~25 fps tùy mạng
static uint32_t lastFrameMs = 0;
static const uint32_t FRAME_INTERVAL_MS = 35;

// ===== UART0 pins (ESP32-CAM): RX=GPIO3, TX=GPIO1 =====
static const int CAM_UART_RX = 3;   // U0R
static const int CAM_UART_TX = 1;   // U0T
static const uint32_t CAM_BAUD = 115200;

// ================== CONFIG ==================
struct CamConfig {
  String ssid;
  String pass;
  String host;
  uint16_t port = 0;
  bool valid = false;
};

static CamConfig gCfg;
static bool gCameraReady = false;
static bool gNetReady = false;

static String gLineBuf;

// ================== WS EVENT ==================
void wsEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      // Tránh spam log vì đang dùng UART0 chung đường cấu hình
      ws.sendTXT("type:cam");
      break;
    case WStype_DISCONNECTED:
      break;
    default:
      break;
  }
}

// ================== PARSE CFG LINE ==================
static bool extractValue(const String& line, const char* key, String& out) {
  // tìm "key=" và lấy tới ký tự '|' hoặc hết chuỗi
  String pat = String(key) + "=";
  int s = line.indexOf(pat);
  if (s < 0) return false;
  s += pat.length();
  int e = line.indexOf('|', s);
  if (e < 0) e = line.length();
  out = line.substring(s, e);
  out.trim();
  return true;
}

static bool parseCfgLine(String line, CamConfig& out) {
  line.trim();
  if (!line.startsWith("CFG|")) return false;

  CamConfig c;
  if (!extractValue(line, "ssid", c.ssid)) return false;
  // pass có thể rỗng (wifi open), nên không bắt buộc extractValue pass
  extractValue(line, "pass", c.pass);
  if (!extractValue(line, "host", c.host)) return false;

  String portStr;
  if (!extractValue(line, "port", portStr)) return false;
  int p = portStr.toInt();
  if (p <= 0 || p > 65535) return false;
  c.port = (uint16_t)p;

  c.valid = (c.ssid.length() > 0 && c.host.length() > 0 && c.port > 0);
  if (!c.valid) return false;

  out = c;
  return true;
}

// Đọc 1 dòng kết thúc bằng '\n' từ UART0, non-blocking
static bool readCfgLineNonBlocking(String& outLine) {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;

    if (ch == '\n') {
      outLine = gLineBuf;
      gLineBuf = "";
      outLine.trim();
      return outLine.length() > 0;
    } else {
      // giới hạn buffer để tránh tràn RAM nếu nhiễu
      if (gLineBuf.length() < 256) gLineBuf += ch;
    }
  }
  return false;
}

// ================== WIFI / WS ==================
static void disconnectNet() {
  ws.disconnect();
  WiFi.disconnect(true, true);
  gNetReady = false;
}

static bool connectWiFiWithCfg(const CamConfig& c, uint32_t timeoutMs) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(c.ssid.c_str(), c.pass.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    if (millis() - start >= timeoutMs) return false;
  }
  return true;
}

static void setupWebSocketWithCfg(const CamConfig& c) {
  // arduinoWebSockets hỗ trợ begin(String) và nội bộ gọi begin(host.c_str(), ...) [web:19]
  ws.begin(c.host.c_str(), c.port, WS_PATH);
  ws.onEvent(wsEvent);
  ws.setReconnectInterval(3000);
  ws.enableHeartbeat(15000, 3000, 2);
}

// ================== CAMERA ==================
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_QQVGA;  // 160x120
    config.jpeg_quality = 16;
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_QQVGA;
    config.jpeg_quality = 18;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    delay(500);
    ESP.restart();
  }
  gCameraReady = true;
}

// ================== APPLY NEW CFG ==================
static bool sameCfg(const CamConfig& a, const CamConfig& b) {
  return a.ssid == b.ssid && a.pass == b.pass && a.host == b.host && a.port == b.port;
}

static void applyConfigAndConnect(const CamConfig& c) {
  // Nếu đang chạy rồi mà nhận cấu hình mới => reconnect
  disconnectNet();

  bool ok = connectWiFiWithCfg(c, 20000);
  if (!ok) {
    // Nếu WiFi fail thì quay lại trạng thái chờ config mới
    gNetReady = false;
    return;
  }

  if (!gCameraReady) setupCamera();
  setupWebSocketWithCfg(c);
  gNetReady = true;
}

// ================== SETUP / LOOP ==================
void setup() {
  // Dùng UART0 để nhận cấu hình từ ESP32-S3 (GPIO3 RX, GPIO1 TX) [web:30][web:27]
  Serial.begin(CAM_BAUD, SERIAL_8N1, CAM_UART_RX, CAM_UART_TX);
  delay(200);

  // Không connect WiFi ở đây nữa.
  // Chỉ chờ cấu hình từ ESP32-S3.
}

void loop() {
  // 1) Luôn lắng nghe cấu hình từ ESP32-S3
  String line;
  if (readCfgLineNonBlocking(line)) {
    CamConfig newCfg;
    if (parseCfgLine(line, newCfg)) {
      if (!gCfg.valid || !sameCfg(gCfg, newCfg) || !gNetReady) {
        gCfg = newCfg;
        applyConfigAndConnect(gCfg);
      }
    }
  }

  // 2) Nếu chưa có cấu hình hoặc chưa kết nối mạng => không stream
  if (!gCfg.valid || !gNetReady) {
    delay(10);
    return;
  }

  // 3) Loop websocket + stream frame
  ws.loop();

  if (!ws.isConnected()) {
    delay(5);
    return;
  }

  if (millis() - lastFrameMs < FRAME_INTERVAL_MS) return;
  lastFrameMs = millis();

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;

  ws.sendBIN(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}
