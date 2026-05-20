#include <WiFi.h>

// =========================
// User configuration
// =========================
static const char* WIFI_SSID = "hts";
static const char* WIFI_PASS = "12345678";

static const uint16_t TCP_PORT = 3333;

// UART link to MSPM0
static const int UART_RX_PIN = 16;  // ESP32 RX2, connect to MSPM0 TX
static const int UART_TX_PIN = 17;  // ESP32 TX2, connect to MSPM0 RX
static const uint32_t UART_BAUD = 115200;

// USB debug serial
static const uint32_t USB_BAUD = 115200;

// If STA connect fails for long time, create AP for emergency connection.
static const bool ENABLE_AP_FALLBACK = true;
static const char* AP_SSID = "ESP32_CAR_LOG";
static const char* AP_PASS = "12345678";

static const uint32_t WIFI_RETRY_INTERVAL_MS = 10000;
static const uint32_t STATUS_PRINT_INTERVAL_MS = 2000;

// =========================
// Internal state
// =========================
HardwareSerial& mcuSerial = Serial2;
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

static bool serverStarted = false;
static bool apStarted = false;
static uint32_t lastWifiRetryMs = 0;
static uint32_t lastStatusPrintMs = 0;
static uint32_t staConnectStartMs = 0;

static uint64_t bytesUartToNet = 0;
static uint64_t bytesNetToUart = 0;

static void printWiFiInfo() {
  Serial.print("[NET] mode=");
  Serial.print((int)WiFi.getMode());
  Serial.print(" status=");
  Serial.print((int)WiFi.status());
  Serial.print(" sta_ip=");
  Serial.print(WiFi.localIP());
  if (apStarted) {
    Serial.print(" ap_ip=");
    Serial.print(WiFi.softAPIP());
  }
  Serial.println();
}

static void tryConnectSta() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  const uint32_t now = millis();
  if ((now - lastWifiRetryMs) < WIFI_RETRY_INTERVAL_MS) {
    return;
  }
  lastWifiRetryMs = now;

  Serial.print("[NET] Connecting STA to SSID: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  staConnectStartMs = now;
}

static void ensureApFallback() {
  if (!ENABLE_AP_FALLBACK || apStarted || WiFi.status() == WL_CONNECTED) {
    return;
  }

  // Give STA some time before AP fallback.
  if ((millis() - staConnectStartMs) < 15000) {
    return;
  }

  if (WiFi.getMode() != WIFI_AP_STA) {
    WiFi.mode(WIFI_AP_STA);
  }

  if (WiFi.softAP(AP_SSID, AP_PASS)) {
    apStarted = true;
    Serial.print("[NET] AP fallback started, SSID=");
    Serial.print(AP_SSID);
    Serial.print(" IP=");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("[NET] AP fallback start failed");
  }
}

static void ensureServer() {
  if (serverStarted) {
    return;
  }
  tcpServer.begin();
  tcpServer.setNoDelay(true);
  serverStarted = true;
  Serial.print("[NET] TCP server listening on port ");
  Serial.println(TCP_PORT);
}

static void acceptClientIfNeeded() {
  if (tcpClient && tcpClient.connected()) {
    return;
  }

  WiFiClient incoming = tcpServer.available();
  if (!incoming) {
    return;
  }

  if (tcpClient && !tcpClient.connected()) {
    tcpClient.stop();
  }
  tcpClient = incoming;
  tcpClient.setNoDelay(true);

  Serial.print("[NET] client connected: ");
  Serial.print(tcpClient.remoteIP());
  Serial.print(":");
  Serial.println(tcpClient.remotePort());
  tcpClient.println("ESP32_MSPM0_UART_BRIDGE_READY");
  tcpClient.println("Examples: PARAM,SHOW | PARAM,AC,-387 | GIMBAL,1500,1500 | K230,UI,VIEW,BLACK");
}

static void bridgeUartToNet() {
  if (!(tcpClient && tcpClient.connected())) {
    return;
  }

  uint8_t buf[256];
  while (mcuSerial.available() > 0) {
    size_t want = (size_t)mcuSerial.available();
    if (want > sizeof(buf)) {
      want = sizeof(buf);
    }
    size_t n = mcuSerial.readBytes(buf, want);
    if (n == 0) {
      break;
    }
    size_t wn = tcpClient.write(buf, n);
    bytesUartToNet += wn;
    Serial.write(buf, n);  // mirror log to USB monitor
  }
}

static void bridgeNetToUart() {
  if (!(tcpClient && tcpClient.connected())) {
    return;
  }

  uint8_t buf[256];
  while (tcpClient.available() > 0) {
    size_t want = (size_t)tcpClient.available();
    if (want > sizeof(buf)) {
      want = sizeof(buf);
    }
    size_t n = tcpClient.read(buf, want);
    if (n == 0) {
      break;
    }
    size_t wn = mcuSerial.write(buf, n);
    bytesNetToUart += wn;
  }
}

static void handleUsbCommands() {
  if (!Serial.available()) {
    return;
  }

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  String rawCmd = cmd;
  cmd.toLowerCase();

  if (cmd == "ip") {
    printWiFiInfo();
    return;
  }
  if (cmd == "client") {
    if (tcpClient && tcpClient.connected()) {
      Serial.print("[NET] connected ");
      Serial.print(tcpClient.remoteIP());
      Serial.print(":");
      Serial.println(tcpClient.remotePort());
    } else {
      Serial.println("[NET] no client");
    }
    return;
  }
  if (cmd == "stats") {
    Serial.print("[STAT] uart_to_net=");
    Serial.print((unsigned long)bytesUartToNet);
    Serial.print(" net_to_uart=");
    Serial.println((unsigned long)bytesNetToUart);
    return;
  }
  if (cmd.startsWith("send ")) {
    String payload = rawCmd.substring(5);
    mcuSerial.print(payload);
    mcuSerial.print("\r\n");
    Serial.print("[UART] sent: ");
    Serial.println(payload);
    return;
  }
  if (cmd == "reboot") {
    Serial.println("[SYS] reboot...");
    delay(100);
    ESP.restart();
    return;
  }

  if (cmd.length() > 0) {
    Serial.print("[CMD] unknown: ");
    Serial.println(cmd);
    Serial.println("[CMD] available: ip, client, stats, send <line>, reboot");
  }
}

void setup() {
  Serial.begin(USB_BAUD);
  mcuSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  delay(200);
  Serial.println();
  Serial.println("[SYS] ESP32 UART<->WiFi bridge start");
  Serial.print("[SYS] UART2 RX=");
  Serial.print(UART_RX_PIN);
  Serial.print(" TX=");
  Serial.print(UART_TX_PIN);
  Serial.print(" baud=");
  Serial.println(UART_BAUD);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  tryConnectSta();
  ensureServer();

  Serial.println("[HINT] TCP connects to port 3333 and is transparent to MSPM0 UART");
  Serial.println("[HINT] USB commands: ip, client, stats, send <line>, reboot");
}

void loop() {
  tryConnectSta();
  ensureApFallback();
  ensureServer();
  acceptClientIfNeeded();

  bridgeUartToNet();
  bridgeNetToUart();
  handleUsbCommands();

  if ((millis() - lastStatusPrintMs) >= STATUS_PRINT_INTERVAL_MS) {
    lastStatusPrintMs = millis();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("[NET] STA IP=");
      Serial.println(WiFi.localIP());
    }
  }

  delay(1);
}
