#include <ESP8266WiFi.h>
#include <WebSocketsClient.h>
#include <string.h>

// ====== PIN L298N + ESP8266 NodeMCU v3 ======
// Steering motor
#define IN1 D1   // GPIO5
#define IN2 D2   // GPIO4

// Drive motor
#define IN3 D6   // GPIO14
#define IN4 D7   // GPIO12
#define ENB D5   // GPIO13 (PWM)

// ====== PWM (ESP8266 analogWrite) ======
const uint32_t PWM_FREQ  = 5000;   // có thể chỉnh 1k..20k tùy driver [web:68]
const uint16_t PWM_RANGE = 1023;   // 0..1023 mặc định [web:68]
const uint16_t DRIVE_SPEED = 800;  // 0..PWM_RANGE

// ====== WIFI + WEBSOCKET ======
const char* WIFI_SSID     = "Ayaflo";
const char* WIFI_PASSWORD = "12345678tan";

//const char* WS_HOST = "esp32-car-server.onrender.com";
const char* WS_HOST = "27.71.20.43";
//const uint16_t WS_PORT = 443;
const uint16_t WS_PORT = 8080;
const char* WS_PATH = "/car";

WebSocketsClient webSocket;

// ====== MOTOR CONTROL ======
void setSteering(int dir) {
  if (dir < 0) {         // trái
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (dir > 0) {  // phải
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {               // dừng lái
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}

void setDrive(int dir) {
  if (dir > 0) {         // tiến
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, DRIVE_SPEED);
  } else if (dir < 0) {  // lùi
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, DRIVE_SPEED);
  } else {               // dừng
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}

void goStop()          { setSteering(0);  setDrive(0);  }
void goForward()       { setSteering(0);  setDrive(1);  }
void goBackward()      { setSteering(0);  setDrive(-1); }
void goForwardLeft()   { setSteering(-1); setDrive(1);  }
void goForwardRight()  { setSteering(1);  setDrive(1);  }
void goBackwardLeft()  { setSteering(-1); setDrive(-1); }
void goBackwardRight() { setSteering(1);  setDrive(-1); }

// ====== PARSE LỆNH DẠNG SỐ '1'..'7' ======
// 1=stop, 2=fwd, 3=back, 4=fwdL, 5=fwdR, 6=backL, 7=backR
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

// ====== WEBSOCKET EVENT ======
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("[WS] Disconnected");
      goStop();
      break;

    case WStype_CONNECTED:
      Serial.printf("[WS] Connected: %s\n", payload);
      webSocket.sendTXT("type:car");
      break;

    case WStype_TEXT:
      // Không tạo String; xử lý cực nhẹ để tránh heap fragmentation [web:53]

        // Log thời gian + raw payload
  Serial.printf("[RX %lu ms] len=%u data=",
                millis(), (unsigned)length);
  for (size_t i = 0; i < length; i++) {
    Serial.write(payload[i]);   // in đúng ký tự nhận được
  }
  Serial.println();

      if (length >= 1) {
        handleCmdByte(payload[0]);     // chỉ lấy ký tự đầu
      } else {
        goStop();
      }
      break;

    case WStype_BIN:
      // Nếu bên điều khiển gửi nhị phân 0x01..0x07 thì dùng đoạn này:
      if (length >= 1) {
        uint8_t b = payload[0];
        if (b >= 1 && b <= 7) handleCmdByte((uint8_t)('0' + b));
        else goStop();
      }
      break;

    default:
      break;
  }
}

// ====== WIFI ======
void connectWiFi() {
  WiFi.setSleepMode(WIFI_NONE_SLEEP);  // giảm trễ do modem-sleep/light-sleep
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint8_t retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    yield(); // tránh WDT khi đang connect lâu
    Serial.print(".");
    if (++retries >= 150) { // ~30s
      Serial.println("\nWiFi connect failed, restarting...");
      ESP.restart();
    }
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void connectWebSocket() {
  Serial.println("Connecting WebSocket...");

  //webSocket.beginSSL(WS_HOST, WS_PORT, WS_PATH);
  webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
  webSocket.onEvent(webSocketEvent);

  webSocket.setReconnectInterval(5000);

  // Heartbeat/ping-pong để phát hiện socket chết sớm; tham số ms [web:60]
  webSocket.enableHeartbeat(15000, 3000, 2);
}

// ====== SETUP/LOOP ======
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);
  analogWrite(ENB, 0);

  goStop();

  connectWiFi();
  connectWebSocket();
}

void loop() {
  webSocket.loop();
  yield();  // nhường CPU cho WiFi stack, giúp ổn định khi tải cao
}
