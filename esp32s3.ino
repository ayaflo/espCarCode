struct AppConfig;
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <WebSocketsClient.h>
#include <driver/i2s.h>

// PIN L298N (ESP32-S3)
#define IN1 15
#define IN2 16
#define IN3 17
#define IN4 18
#define ENB 42 // motor sau (kênh B)
#define ENA 41 // motor trước (kênh A)

// PWM settings
static const uint32_t DRIVE_PWM_FREQ = 20000; // 20 kHz
static const uint8_t  DRIVE_PWM_RES  = 8;     // 0..255
static const uint8_t  DRIVE_SPEED_BACK  = 210;
static const uint8_t  DRIVE_SPEED_FRONT = 255;

// I2S MIC (INMP441)
static const i2s_port_t I2S_MIC_PORT = I2S_NUM_0;
#define MIC_SD  4
#define MIC_SCK 2 // BCLK
#define MIC_WS  3 // LRCK/WS

// I2S AMP (MAX98357)
static const i2s_port_t I2S_SPK_PORT = I2S_NUM_1;
#define SPK_DIN  8
#define SPK_BCLK 7
#define SPK_LRC  6

// AUDIO SETTINGS
static const int AUDIO_SR = 16000;
static const size_t MIC_SAMPLES_PER_CHUNK = 2048;
static int16_t mic_pcm16[MIC_SAMPLES_PER_CHUNK];
static int32_t mic_raw32[MIC_SAMPLES_PER_CHUNK];

/** Convert 32-bit sample to 16-bit PCM with clamping */
static inline int16_t mic32_to_pcm16(int32_t v) {
  v >>= 14;
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}

// WEBSOCKET PATHS
const char* WS_PATH_CAR     = "/car";
const char* WS_PATH_MIC     = "/mic";
const char* WS_PATH_SPEAKER = "/speaker";

WebSocketsClient wsCar;
WebSocketsClient wsMic;
WebSocketsClient wsSpeaker;

// CONFIG STORAGE (NVS)
Preferences prefs;

struct AppConfig {
  String wifiSsid;
  String wifiPass;
  String host;
  uint16_t port = 0;
  bool valid = false;
};

static AppConfig gCfg;

// CAPTIVE PORTAL
static const byte DNS_PORT = 53;
DNSServer dnsServer;
WebServer server(80);

static bool gPortalMode = false;

static String apSsid() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[32];
  snprintf(buf, sizeof(buf), "ESP32S3-SETUP-%04X", (uint16_t)(mac & 0xFFFF));
  return String(buf);
}

static const char PORTAL_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <meta name="viewport" content="width=device-width,initial-scale=1"/>
  <title>ESP32-S3 Setup</title>
  <style>
    body{font-family:Arial,Helvetica,sans-serif;margin:16px;max-width:560px}
    label{display:block;margin-top:12px;font-weight:600}
    input{width:100%;padding:10px;font-size:16px}
    button{margin-top:16px;padding:12px 16px;font-size:16px}
    .hint{color:#444;font-size:13px;margin-top:8px}
    .box{border:1px solid #ddd;border-radius:10px;padding:14px}
  </style>
</head>
<body>
  <h2>WiFi / Server Config</h2>
  <div class="box">
    <form method="POST" action="/save">
      <label>WiFi SSID</label>
      <input name="ssid" maxlength="32" required>

      <label>WiFi Password</label>
      <input name="pass" maxlength="64" type="password">

      <label>Server Host (IP/domain)</label>
      <input name="host" maxlength="64" required>

      <label>Server Port</label>
      <input name="port" type="number" min="1" max="65535" required>

      <button type="submit">Save & Reboot</button>
      <div class="hint">Lưu ý: không kết nối được với WiFi 5G!</div>
    </form>
  </div>
</body>
</html>
)HTML";

static bool isIp(const String& str) {
  for (size_t i = 0; i < str.length(); i++) {
    char c = str[i];
    if (!(c == '.' || (c >= '0' && c <= '9'))) return false;
  }
  return true;
}

static String toStringIp(const IPAddress& ip) {
  return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

// Redirect mọi request (không phải IP) về trang portal để OS nhận diện captive portal
static bool captivePortalRedirect() {
  String hostHeader = server.hostHeader();
  if (!isIp(hostHeader)) {
    server.sendHeader("Location", String("http://") + toStringIp(server.client().localIP()), true);
    server.send(302, "text/plain", "");
    server.client().stop();
    return true;
  }
  return false;
}

static void handleRoot() {
  if (captivePortalRedirect()) return;
  server.send(200, "text/html", PORTAL_HTML);
}

static void handleSave() {
  if (captivePortalRedirect()) return;

  if (!server.hasArg("ssid") || !server.hasArg("host") || !server.hasArg("port")) {
    server.send(400, "text/plain", "Missing fields");
    return;
  }

  String ssid = server.arg("ssid");
  String pass = server.arg("pass");
  String host = server.arg("host");
  uint16_t port = (uint16_t)server.arg("port").toInt();

  if (ssid.length() == 0 || host.length() == 0 || port == 0) {
    server.send(400, "text/plain", "Invalid fields");
    return;
  }

  prefs.begin("cfg", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.putString("host", host);
  prefs.putUShort("port", port);
  prefs.end();

  server.send(200, "text/plain", "Saved. Rebooting...");
  delay(600);
  ESP.restart();
}

static void startCaptivePortal() {
  gPortalMode = true;
}

static void setupCaptivePortal() {
  gPortalMode = true;

  WiFi.disconnect(true, true);
  delay(200);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSsid().c_str()); // AP mở, dễ kết nối cấu hình

  // DNS: trả IP AP cho mọi domain -> captive portal
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  // Routes portal
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);

  // Một số URL “connectivity check” hay gặp
  server.on("/generate_204", HTTP_GET, handleRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRoot);
  server.on("/fwlink", HTTP_GET, handleRoot);
  server.onNotFound(handleRoot);

  server.begin();
}

static AppConfig loadConfig() {
  AppConfig c;
  prefs.begin("cfg", true);
  c.wifiSsid = prefs.getString("ssid", "");
  c.wifiPass = prefs.getString("pass", "");
  c.host     = prefs.getString("host", "");
  c.port     = prefs.getUShort("port", 0);
  prefs.end();
  c.valid = (c.wifiSsid.length() > 0 && c.host.length() > 0 && c.port > 0);
  return c;
}

// UART TO ESP32-CAM (GPIO43/12)
static const int CAM_UART_TX = 43; // ESP32-S3 GPIO43 -> ESP32-CAM GPIO3 (RX)
static const int CAM_UART_RX = 44; // ESP32-S3 GPIO44 -> ESP32-CAM GPIO1 (TX)
static const uint32_t CAM_BAUD = 115200;

static void sendConfigToCam(const AppConfig& c) {
  // Format đơn giản để CAM parse:
  // CFG|ssid=...|pass=...|host=...|port=...\n
  Serial1.printf("CFG|ssid=%s|pass=%s|host=%s|port=%u\n",
                 c.wifiSsid.c_str(),
                 c.wifiPass.c_str(),
                 c.host.c_str(),
                 (unsigned)c.port);
  Serial1.flush();
}

// MOTOR CONTROL
void setSteering(int dir) {
  if (dir < 0) {         // trái
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (dir > 0) {  // phải
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, 0);
  }
}

void setDrive(int dir) {
  if (dir > 0) {         // tiến
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else if (dir < 0) {  // lùi
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {               // dừng
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB, 0);
  }
}

void goStop()          { setSteering(0);  setDrive(0); }
void goForward()       { setSteering(0);  setDrive(1);  ledcWrite(ENB, DRIVE_SPEED_BACK-10); }
void goBackward()      { setSteering(0);  setDrive(-1); ledcWrite(ENB, DRIVE_SPEED_BACK-10); }
void goForwardLeft()   { ledcWrite(ENA, DRIVE_SPEED_FRONT); ledcWrite(ENB, DRIVE_SPEED_BACK+10); setSteering(-1); setDrive(1);  }
void goForwardRight()  { ledcWrite(ENA, DRIVE_SPEED_FRONT); ledcWrite(ENB, DRIVE_SPEED_BACK+10); setSteering(1);  setDrive(1);  }
void goBackwardLeft()  { ledcWrite(ENA, DRIVE_SPEED_FRONT); ledcWrite(ENB, DRIVE_SPEED_BACK+10); setSteering(-1); setDrive(-1); }
void goBackwardRight() { ledcWrite(ENA, DRIVE_SPEED_FRONT); ledcWrite(ENB, DRIVE_SPEED_BACK+10); setSteering(1);  setDrive(-1); }

void handleCmdByte(uint8_t c) {
  switch (c) {
    case '1': goStop(); break;
    case '2': goForward(); break;
    case '3': goBackward(); break;
    case '4': goForwardLeft(); break;
    case '5': goForwardRight(); break;
    case '6': goBackwardLeft(); break;
    case '7': goBackwardRight(); break;
    default:  goStop(); break;
  }
}

// I2S SETUP
void setupI2SMic() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = AUDIO_SR;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S_MSB;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 6;
  cfg.dma_buf_len = 256;
  //cfg.dma_buf_count = 12;
  //cfg.dma_buf_len = 512;
  cfg.use_apll = false;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = MIC_SCK;
  pins.ws_io_num = MIC_WS;
  pins.data_in_num = MIC_SD;
  pins.data_out_num = I2S_PIN_NO_CHANGE;

  i2s_driver_install(I2S_MIC_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_MIC_PORT, &pins);
  i2s_zero_dma_buffer(I2S_MIC_PORT);
}

void setupI2SSpeaker() {
  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = AUDIO_SR;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  // cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  //cfg.communication_format = I2S_COMM_FORMAT_I2S_MSB;

  cfg.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);

  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;

  i2s_pin_config_t pins = {};
  pins.bck_io_num = SPK_BCLK;
  pins.ws_io_num = SPK_LRC;
  pins.data_out_num = SPK_DIN;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  i2s_driver_install(I2S_SPK_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_SPK_PORT, &pins);
  i2s_zero_dma_buffer(I2S_SPK_PORT);
}

// WS EVENTS
void wsCarEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      goStop();
      break;
    case WStype_CONNECTED:
      wsCar.sendTXT("type:car");
      break;
    case WStype_TEXT:
      if (length >= 1) handleCmdByte(payload[0]);
      else goStop();
      break;
    default:
      break;
  }
}

void wsMicEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      wsMic.sendTXT("type:mic");
      break;
    default:
      break;
  }
}

static const size_t SPEAKER_SAMPLES_PER_CHUNK = 2048;

void wsSpeakerEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      wsSpeaker.sendTXT("type:speaker");
      break;

    case WStype_BIN: {
      size_t samples = length / 2;
      // static int16_t stereo[SPEAKER_SAMPLES_PER_CHUNK * 2];
      if (samples > SPEAKER_SAMPLES_PER_CHUNK) samples = SPEAKER_SAMPLES_PER_CHUNK;

      // const int16_t* mono = (const int16_t*)payload;
      // for (size_t i = 0; i < samples; i++) {
        // int16_t v = mono[i];
        // stereo[2*i]   = v;
        // stereo[2*i+1] = v;
      // }

      size_t w = 0;
      i2s_write(I2S_SPK_PORT, payload, samples * sizeof(int16_t), &w, pdMS_TO_TICKS(5));
      //size_t written = 0;
      //i2s_write(I2S_SPK_PORT, stereo, samples * 2 * sizeof(int16_t), &written, 0);
      // size_t total = samples * 2 * sizeof(int16_t);
      // size_t offset = 0, w;
      // while (offset < total) {
        // i2s_write(I2S_SPK_PORT, (uint8_t*)stereo + offset, total - offset, &w, portMAX_DELAY);
        // auto ok = i2s_write(I2S_SPK_PORT, (uint8_t*)stereo + offset, total - offset, &w, pdMS_TO_TICKS(5));
        // if (ok != ESP_OK) break;
        // offset += w;
      // }
      break;
    }

    default:
      break;
  }
}

// WIFI + WS CONNECT
static bool connectWiFiWithConfig(const AppConfig& c, uint32_t timeoutMs) {
  if (!c.valid) return false;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(c.wifiSsid.c_str(), c.wifiPass.c_str());

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    if (millis() - start >= timeoutMs) {
      WiFi.disconnect(true, true);
      return false;
    }
  }
  return true;
}

void connectWebSockets(const AppConfig& c) {
  wsCar.begin(c.host.c_str(), c.port, WS_PATH_CAR);
  wsCar.onEvent(wsCarEvent);
  wsCar.setReconnectInterval(3000);
  wsCar.enableHeartbeat(15000, 3000, 2);

  wsMic.begin(c.host.c_str(), c.port, WS_PATH_MIC);
  wsMic.onEvent(wsMicEvent);
  wsMic.setReconnectInterval(5000);
  wsMic.enableHeartbeat(15000, 3000, 2);

  wsSpeaker.begin(c.host.c_str(), c.port, WS_PATH_SPEAKER);
  wsSpeaker.onEvent(wsSpeakerEvent);
  wsSpeaker.setReconnectInterval(7000);
  wsSpeaker.enableHeartbeat(15000, 3000, 2);
}

// MIC STREAM
void streamMicToServer() {
  if (!wsMic.isConnected()) return;
  if (ESP.getFreeHeap() < 40 * 1024) return;

  size_t bytes_read = 0;
  esp_err_t err = i2s_read(I2S_MIC_PORT, mic_raw32, sizeof(mic_raw32), &bytes_read, 0);
  if (err != ESP_OK || bytes_read == 0) return;

  size_t samples = bytes_read / sizeof(int32_t);
  if (samples > MIC_SAMPLES_PER_CHUNK) samples = MIC_SAMPLES_PER_CHUNK;

  for (size_t i = 0; i < samples; i++) {
    mic_pcm16[i] = mic32_to_pcm16(mic_raw32[i]);
  }

  wsMic.sendBIN((uint8_t*)mic_pcm16, samples * sizeof(int16_t));
}

// SETUP / LOOP
void setup() {
  delay(400);

  // UART sang ESP32-CAM
  Serial1.begin(CAM_BAUD, SERIAL_8N1, CAM_UART_RX, CAM_UART_TX);

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENB, DRIVE_PWM_FREQ, DRIVE_PWM_RES);
  ledcWrite(ENB, 0);

  ledcAttach(ENA, DRIVE_PWM_FREQ, DRIVE_PWM_RES);
  ledcWrite(ENA, 0);

  goStop();

  // Audio
  setupI2SMic();
  setupI2SSpeaker();

  // Load config + connect
  gCfg = loadConfig();

  bool ok = connectWiFiWithConfig(gCfg, 20000); // 20s timeout
  if (!ok) {
    // Vào captive portal để nhập cấu hình
    goStop();
    setupCaptivePortal();
    return;
  }

  // Kết nối WiFi OK => gửi cấu hình sang ESP32-CAM rồi connect WS
  sendConfigToCam(gCfg);
  connectWebSockets(gCfg);
}

void loop() {
  if (gPortalMode) {
    dnsServer.processNextRequest();
    server.handleClient();
    delay(5);
    return;
  }

  wsCar.loop();
  wsMic.loop();
  wsSpeaker.loop();

  streamMicToServer();
  delay(2);
}
