// C++ Standard Libraries
#include <vector>

// Core Arduino Libraries
#include <WiFi.h>
#include <Wire.h>
#include <LittleFS.h>
#include <EEPROM.h>
#include <SPI.h>
#include <ArduinoOTA.h>
#include <time.h>

// Third-party Libraries
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <PCF8574.h>

// ============================================================
// CONDITIONAL COMPILATION - Firmware Type
// ============================================================
#ifdef FIRMWARE_WIFI_ONLY
// WiFi-Only Firmware
#include <WebSocketsClient.h>
const String program_version = VERSION_STRING;
#define NETWORK_MODE_NAME "WiFi"
#pragma message("═════════════════════════════════════════")
#pragma message("  BUILDING: WiFi-Only Firmware")
#pragma message("  Version: " VERSION_STRING)
#pragma message("  Network: WiFi STA → AP Fallback")
#pragma message("  WebSocket: links2004 (NETWORK_ESP32)")
#pragma message("  Transport: WiFiClient")
#pragma message("═════════════════════════════════════════")

#elif defined(FIRMWARE_ETHERNET_ONLY)
// Ethernet-Only Firmware
#include <Ethernet.h>
const String program_version = VERSION_STRING;
const int LAN_LED_PIN = 2; 
#define NETWORK_MODE_NAME "Ethernet"
#pragma message("═════════════════════════════════════════")
#pragma message("  BUILDING: Ethernet-Only Firmware")
#pragma message("  Version: " VERSION_STRING)
#pragma message("  Network: Ethernet W5500 → AP Fallback")
#pragma message("  WebSocket: links2004 (NETWORK_W5100)")
#pragma message("  Transport: EthernetClient")
#pragma message("═════════════════════════════════════════")

#else
#error "Must define FIRMWARE_WIFI_ONLY or FIRMWARE_ETHERNET_ONLY"
#endif

// =================================================================
// --- Hardware & Pin Definitions ---
// =================================================================
const int eth_cs_pin = 5;
const int eth_sclk_pin = 18;
const int eth_mosi_pin = 23;
const int eth_miso_pin = 19;
const int eth_int_pin = -1;
const int eth_rst_pin = -1;
const int eth_power_pin = -1;

const int NUM_PCF_INPUTS = 8;
const int NUM_GPIO_INPUTS = 8;
const uint8_t PCF_PIN_MAP[NUM_PCF_INPUTS] = {0, 1, 2, 3, 4, 5, 6, 7};
const int INPUT_GPIO_PINS[NUM_GPIO_INPUTS] = {32, 33, 25, 26, 27, 14, 34, 13};
const int OUTPUT_PINS[] = {17, 16, 4, 15};

const int NUM_INPUTS = NUM_PCF_INPUTS + NUM_GPIO_INPUTS;
const int NUM_OUTPUTS = sizeof(OUTPUT_PINS) / sizeof(OUTPUT_PINS[0]);

// =================================================================
// --- Network & Server Configuration ---
// =================================================================
String ap_ssid = "IoT-Node ";
const char *ap_password = "12345678";
const char *default_username = "contoh";
const char *default_password = "password";
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600;
const int daylightOffset_sec = 0;

// =================================================================
// --- EEPROM Configuration ---
// =================================================================
#define EEPROM_SIZE 1536  
#define HARDWARE_ID_ADDR 0
#define SERVER_IP_ADDR 32
#define WIFI_SSID_ADDR 64
#define WIFI_PASS_ADDR 128
#define SERVER_PORT_ADDR 192
#define PATH_ADDR 200
#define COMM_MODE_ADDR 232
#define NETWORK_MODE_ADDR 248
#define ACCESS_TOKEN_ADDR 280
#define IO_STATE_START_ADDR 350
#define OUTPUT_STATE_START_ADDR 366
#define TIMESTAMP_ADDR 370
#define IO_STATE_SIGNATURE_ADDR 374
#define IO_STATE_SIGNATURE 0xA5

// =================================================================
// --- Short Detection & Offline Buffer Configuration ---
// =================================================================
#define SHORT_FILTER_TIME 1000          // Waktu filtering 1 detik (ms)
#define SHORT_BUFFER_START_ADDR 400     // Alamat awal buffer di EEPROM
#define SHORT_BUFFER_SIZE 30            // Maksimal 30 event yang bisa disimpan
#define SHORT_BUFFER_COUNT_ADDR 398     // Alamat untuk menyimpan jumlah buffer
#define SHORT_BUFFER_SIGNATURE_ADDR 396 // Signature untuk validasi buffer
#define SHORT_BUFFER_SIGNATURE 0xB5     // Signature value

// =================================================================
// --- Timing & Control ---
// =================================================================
#ifndef CONNECT_TIMEOUT
#define CONNECT_TIMEOUT 15000
#endif

#ifndef AP_TIMEOUT
#define AP_TIMEOUT 150000
#endif

const unsigned long connectTimeout = CONNECT_TIMEOUT;
const unsigned long apTimeout = AP_TIMEOUT;
const unsigned long reconnectInterval = 10 * 1000;
const unsigned long singleAttemptTimeout = 8000;
const unsigned long bufferedSendInterval = 5000;
unsigned long lastBufferedSendAttempt = 0;
unsigned long mqttReconnectInterval = 1000;
const long monitoringInterval = 200;
const unsigned long forceSendInterval = 20 * 60 * 1000;
const size_t MAX_BUFFER_SIZE = 100;

int reconnectAttemptCount = 0;
const int maxReconnectAttempts = 2;

// =================================================================
// --- Configuration Structure ---
// =================================================================
struct Config
{
  String hardwareId, networkMode, serverIP, path, commMode, wifiSSID, wifiPass, accessTokenMQTT;
  int monitoringPort;
};
Config config;

struct ShortEvent {
  uint8_t inputIndex;           // Index input yang trigger (0-15) = 1 byte
  uint8_t triggerState;         // State trigger (selalu 0 untuk short) = 1 byte
  uint32_t timestamp;           // Timestamp dalam detik = 4 bytes
  uint8_t inputStates[16];      // State semua input saat event = 16 bytes
  uint8_t outputStates[4];      // State semua output saat event = 4 bytes
};

    
struct InputFilterState {
  uint8_t lastStableState;       // State stabil terakhir
  uint8_t pendingState;          // State yang sedang di-filter
  unsigned long changeStartTime; // Waktu mulai perubahan terdeteksi
  bool isFiltering;              // Sedang dalam proses filtering
};

// =================================================================
// --- Timing & Shoot ---
// =================================================================

int lastShortTriggerIndex = -1;
bool hasOfflineData = false; 
bool shortSendPending = false; 

unsigned long lastShortCheckTime = 0; 
unsigned long lastShortConfirmTime = 0;
const unsigned long httpPollInterval = 5000;
const unsigned long shortCheckInterval = 50;
const unsigned long SHORT_GROUP_DELAY = 50;

InputFilterState inputFilters[NUM_INPUTS];
std::vector<ShortEvent> shortBuffer; 

// =================================================================
// --- Globals ---
// =================================================================
WebServer server(80);
PCF8574 pcf8574(0x20);

// Network client (conditional)
#ifdef FIRMWARE_WIFI_ONLY
WiFiClient netClient;
WebSocketsClient wsClient;
#define ACTIVE_CLIENT netClient
#else
EthernetClient netClient;
#define ACTIVE_CLIENT netClient
#endif

PubSubClient mqttClient;

// Simplified network status

static bool network_connected = false; 
static bool network_ready = false;
bool isAPMode = false;
bool isAuthenticated = false;
bool isReconnecting = false;
bool readyToSend = false;
bool ws_connected = false;

unsigned long apStartTime = 0;
unsigned long lastCountdownPrint = 0;
unsigned long lastMonitoringTime = 0;
unsigned long lastForceSendTime = 0;
unsigned long lastReconnectAttempt = 0;
unsigned long reconnectAttemptStartTime = 0;
unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastHttpPollTime = 0;

String httpSessionId = "";

uint8_t lastInputStates[NUM_INPUTS] = {0};
std::vector<String> monitoringBuffer;
// =================================================================
// --- Forward Declarations ---
// =================================================================
void publishStatus();
void publishTelemetry();
bool saveConfig();
void sendMonitoringData();
void setupCommunication();
void setupOTA();
void mqttCallback(char *topic, byte *payload, unsigned int length);
void reconnectMQTT();
void processCommand(JsonDocument &doc);

// Short Detection Functions
void initShortDetection();
void checkShortInputs();
void processShortEvent(int inputIndex, uint8_t fromState, uint8_t toState);
void saveShortToBuffer(int inputIndex, uint8_t previousState);
void saveBufferToEEPROM();
void loadBufferFromEEPROM();
void sendBufferedShorts();
void clearShortBuffer();
String buildShortEventJSON(ShortEvent &event);
String buildBatchShortJSON();

// Web Server Handlers
void handleRoot();
void handleAuth();
void handleDashboard();
void handleConfigPage();
void handleControlPage();
void handleStatusPage();
void handleSaveConfig();
void handleToggle();
void handleGetOutputs();
void handleDownloadConfig();
void handleLogout();
void handleApiControl();
void handleApiStatus();
void handleApiSet();
void setupApiRoutes();
void serveErrorPage(String errorMessage);

// Other functions
String readFromEEPROM(int address);
void writeToEEPROM(int address, const String &value);
void startAPMode();
void tryReconnect();
bool initEthernet();
void wsDisconnect();
bool wsConnect();
bool wsSendText(const String &s);
void wsPump();
bool loadConfig();
bool sendViaHTTP(const String &jsonData, String &resp);
bool ethHttpPost(const String &host, uint16_t port, const String &path, const String &body, String &response);

// =================================================================
// --- EEPROM Helpers ---
// =================================================================
String readFromEEPROM(int address)
{
  String value = "";
  char ch;
  int currentAddr = address;
  while (currentAddr < EEPROM_SIZE)
  {
    ch = EEPROM.read(currentAddr);
    if (ch == '\0')
      break;
    value += ch;
    currentAddr++;
    if (value.length() > 64)
      break;
  }
  return value;
}
void writeToEEPROM(int address, const String &value)
{
  int i = 0;
  for (; i < value.length() && (address + i) < (EEPROM_SIZE - 1); i++)
  {
    EEPROM.write(address + i, value[i]);
  }
  EEPROM.write(address + i, '\0');
}

// =================================================================
// --- EEPROM I/O STATE ---
// =================================================================
void saveIOStateToEEPROM()
{
  for (int i = 0; i < NUM_INPUTS; i++)
    EEPROM.write(IO_STATE_START_ADDR + i, lastInputStates[i]);
  for (int i = 0; i < NUM_OUTPUTS; i++)
  {
    uint8_t outputState = digitalRead(OUTPUT_PINS[i]);
    EEPROM.write(OUTPUT_STATE_START_ADDR + i, outputState);
  }
  unsigned long timestamp = millis();
  EEPROM.write(TIMESTAMP_ADDR + 0, (timestamp >> 24) & 0xFF);
  EEPROM.write(TIMESTAMP_ADDR + 1, (timestamp >> 16) & 0xFF);
  EEPROM.write(TIMESTAMP_ADDR + 2, (timestamp >> 8) & 0xFF);
  EEPROM.write(TIMESTAMP_ADDR + 3, timestamp & 0xFF);
  EEPROM.write(IO_STATE_SIGNATURE_ADDR, IO_STATE_SIGNATURE);
  EEPROM.commit();
}
void loadIOStateFromEEPROM()
{
  if (EEPROM.read(IO_STATE_SIGNATURE_ADDR) != IO_STATE_SIGNATURE)
    return;
  for (int i = 0; i < NUM_INPUTS; i++)
    lastInputStates[i] = EEPROM.read(IO_STATE_START_ADDR + i);
  for (int i = 0; i < NUM_OUTPUTS; i++)
    digitalWrite(OUTPUT_PINS[i], EEPROM.read(OUTPUT_STATE_START_ADDR + i));
}

// =================================================================
// --- Short Detection Implementation ---
// =================================================================
void initShortDetection()
{
  Serial.println("[Short] Initializing short detection system...");
  
  // Inisialisasi filter state untuk setiap input
  for (int i = 0; i < NUM_INPUTS; i++)
  {
    inputFilters[i].lastStableState = 1;  // Default HIGH 
    inputFilters[i].pendingState = 1;
    inputFilters[i].changeStartTime = 0;
    inputFilters[i].isFiltering = false;
  }
  
  // Load buffer dari EEPROM jika ada
  loadBufferFromEEPROM();
  
  Serial.printf("[Short] System ready. Buffer size: %d events\n", shortBuffer.size());
}

/**
 * Baca state input saat ini (gabungan PCF8574 + GPIO)
 */
uint8_t readInputState(int index)
{
  if (index < NUM_PCF_INPUTS)
  {
    // PCF8574 input
    uint8_t pcfData = pcf8574.read8();
    return (pcfData >> PCF_PIN_MAP[index]) & 0x01;
  }
  else
  {
    // GPIO input
    int gpioIndex = index - NUM_PCF_INPUTS;
    return digitalRead(INPUT_GPIO_PINS[gpioIndex]);
  }
}

/**
 * Check semua input untuk short detection dengan filtering
 */
void checkShortInputs()
{
  unsigned long currentTime = millis();
  
  // Check interval
  if (currentTime - lastShortCheckTime < shortCheckInterval)
  {
    return;
  }
  lastShortCheckTime = currentTime;
  
  // Baca semua input sekaligus untuk konsistensi
  uint8_t currentStates[NUM_INPUTS];
  
  // Baca PCF8574
  uint8_t pcfData = pcf8574.read8();
  for (int i = 0; i < NUM_PCF_INPUTS; i++)
  {
    currentStates[i] = (pcfData >> PCF_PIN_MAP[i]) & 0x01;
  }
  
  // Baca GPIO inputs
  for (int i = 0; i < NUM_GPIO_INPUTS; i++)
  {
    currentStates[NUM_PCF_INPUTS + i] = digitalRead(INPUT_GPIO_PINS[i]);
  }
  
  // Process setiap input dengan filtering
  for (int i = 0; i < NUM_INPUTS; i++)
  {
    uint8_t currentState = currentStates[i];
    InputFilterState &filter = inputFilters[i];
    
    if (!filter.isFiltering)
    {
      if (currentState != filter.lastStableState)
      {
        // Perubahan terdeteksi - mulai filtering
        filter.pendingState = currentState;
        filter.changeStartTime = currentTime;
        filter.isFiltering = true;
        // Serial.printf("[Short] I%d: Change detected %d -> %d, filtering...\n", 
        //               i + 1, filter.lastStableState, currentState);
      }
    }
    else
    {
      // Sedang filtering - validasi perubahan
      if (currentState != filter.pendingState)
      {
        // State berubah lagi sebelum filter selesai - batalkan
        filter.isFiltering = false;
        // Serial.printf("[Short] I%d: Filter cancelled (bounce detected)\n", i + 1);
      }
      else if (currentTime - filter.changeStartTime >= SHORT_FILTER_TIME)
      {
        // Filter time tercapai dan state konsisten
        uint8_t previousState = filter.lastStableState;
        filter.lastStableState = currentState;
        filter.isFiltering = false;
      
        // Simpan state stabil baru
        lastInputStates[i] = currentState;
      
        // Hanya proses jika perubahan dari 1 -> 0 (SHORT)
        if (previousState == 1 && currentState == 0)
        {
          Serial.printf("[Short] I%d: SHORT CONFIRMED (1 -> 0)\n", i + 1);
        
          // JANGAN kirim langsung. Tandai dulu, akan dikirim setelah
          // semua input yang ikut short pada burst ini dikonfirmasi.
          shortSendPending    = true;
          lastShortConfirmTime = currentTime;
          lastShortTriggerIndex = i;  // hanya dipakai sebagai label "trigger"
        }
        else
        {
          // Perubahan 0 -> 1 (release) - tidak perlu kirim
          Serial.printf("[Short] I%d: Release detected (0 -> 1), ignored\n", i + 1);
        }
      }
    }
  }
    // Setelah semua input diproses, kirim 1 paket jika ada short yang pending
  if (shortSendPending)
  {
    if (currentTime - lastShortConfirmTime >= SHORT_GROUP_DELAY)
    {
      processShortEvent(lastShortTriggerIndex, 1, 0);

      shortSendPending = false;
    }
  }
}

void processShortEvent(int inputIndex, uint8_t fromState, uint8_t toState)
{
  Serial.println("\n========== SHORT EVENT ==========");
  Serial.printf("[Short] Input: I%d, Change: %d -> %d\n", inputIndex + 1, fromState, toState);
  
  // Tentukan apakah ThingsBoard
  bool isThingsBoard = (config.serverIP.indexOf("thingsboard") >= 0);
  
  // Build JSON dengan format sama seperti buildMonitoringJSON
  StaticJsonDocument<512> doc;
  
  // Tambahkan box_id jika bukan ThingsBoard
  if (!isThingsBoard)
  {
    doc["box_id"] = config.hardwareId;
  }
  
  // Tambahkan semua Input states (I1 - I16)
  for (int i = 0; i < NUM_PCF_INPUTS; i++)
  {
    String key = "I" + String(i + 1);
    uint8_t state = inputFilters[i].lastStableState;
    // Inverted: hardware 1 = tidak aktif (kirim "0"), hardware 0 = aktif (kirim "1")
    doc[key] = (state == 1) ? "0" : "1";
  }
  
  for (int i = 0; i < NUM_GPIO_INPUTS; i++)
  {
    String key = "I" + String(NUM_PCF_INPUTS + i + 1);
    uint8_t state = inputFilters[NUM_PCF_INPUTS + i].lastStableState;
    doc[key] = (state == 1) ? "0" : "1";
  }
  
  // Tambahkan semua Output states (Q1 - Q4)
  for (int i = 0; i < NUM_OUTPUTS; i++)
  {
    String key = "Q" + String(i + 1);
    doc[key] = digitalRead(OUTPUT_PINS[i]);
  }
  
  // Tambahkan info trigger (untuk tracking)
  if (!isThingsBoard)
  {
    doc["trigger"] = "I" + String(inputIndex + 1);
    doc["event"] = "short";
  }
  
  String jsonData;
  serializeJson(doc, jsonData);
  
  // Serial.println("[Short] JSON: " + jsonData);
  // Serial.printf("[Short] JSON Length: %d bytes\n", jsonData.length());
  // 
  // Check koneksi dan kirim
  bool sent = false;
  
  if (network_connected && network_ready)
  {
    if (config.commMode == "ws")
    {
#ifdef FIRMWARE_WIFI_ONLY
      if (ws_connected)
      {
        sent = wsSendText(jsonData);
        if (sent) Serial.println("[Short] ✓ Sent via WebSocket");
      }
      else
      {
        Serial.println("[Short] ✗ WebSocket not connected");
      }
#endif
    }
    else if (config.commMode == "mqtt")
    {
      // Pastikan MQTT masih connected
      if (!mqttClient.connected())
      {
        Serial.println("[Short] MQTT disconnected, attempting reconnect...");
        reconnectMQTT();
        delay(100);
      }
      
      if (mqttClient.connected())
      {
        String topic = isThingsBoard ? "v1/devices/me/telemetry" : "iot-node/" + config.hardwareId + "/telemetry";
        
        Serial.printf("[Short] Publishing to: %s\n", topic.c_str());
        Serial.printf("[Short] Payload size: %d bytes\n", jsonData.length());
        
        for (int i = 0; i < 3; i++) {
          mqttClient.loop();
          delay(10);
        }
            
        // Publish dengan QoS check
        bool publishResult = mqttClient.publish(topic.c_str(), jsonData.c_str(), false);
        
        if (publishResult)
        {
          Serial.println("[Short] ✓ MQTT publish returned true");
          
          // Verify connection masih ada setelah publish
          if (mqttClient.connected())
          {
            Serial.println("[Short] ✓ MQTT still connected after publish");
            sent = true;
          }
          else
          {
            Serial.println("[Short] ✗ MQTT disconnected after publish!");
            sent = false;
          }
        }
        else
        {
          Serial.println("[Short] ✗ MQTT publish returned false");
          Serial.printf("[Short] MQTT State: %d\n", mqttClient.state());
        }
      }
      else
      {
        Serial.println("[Short] ✗ MQTT reconnect failed");
        Serial.printf("[Short] MQTT State: %d\n", mqttClient.state());
      }
    }
    else if (config.commMode == "httppost")
    {
      String response;
      sent = sendViaHTTP(jsonData, response);
      if (sent) 
      {
        Serial.println("[Short] ✓ Sent via HTTP POST");
      }
      else
      {
        Serial.println("[Short] ✗ HTTP POST failed");
      }
    }
  }
  else
  {
    Serial.printf("[Short] ✗ Network not ready (connected=%d, ready=%d)\n", 
                  network_connected, network_ready);
  }
  
  // Jika tidak bisa kirim, simpan ke buffer
  if (!sent)
  {
    Serial.println("[Short] Saving to offline buffer...");
    saveShortToBuffer(inputIndex, fromState);
  }
  
}

/**
 * Simpan short event ke buffer dengan state I/O lengkap
 */
void saveShortToBuffer(int inputIndex, uint8_t previousState)
{
  // Check apakah buffer masih ada ruang
  if (shortBuffer.size() >= SHORT_BUFFER_SIZE)
  {
    Serial.println("[Short] Buffer full! Removing oldest event...");
    shortBuffer.erase(shortBuffer.begin());
  }
  
  // Buat event baru dengan state lengkap
  ShortEvent event;
  event.inputIndex = inputIndex;
  event.triggerState = 0; // Short = 1->0, jadi state akhir = 0
  event.timestamp = millis() / 1000;
  
  // Simpan state semua input saat ini
  for (int i = 0; i < NUM_INPUTS && i < 16; i++)
  {
    event.inputStates[i] = inputFilters[i].lastStableState;
  }
  
  // Simpan state semua output saat ini
  for (int i = 0; i < NUM_OUTPUTS && i < 4; i++)
  {
    event.outputStates[i] = digitalRead(OUTPUT_PINS[i]);
  }
  
  shortBuffer.push_back(event);
  hasOfflineData = true;
  
  Serial.printf("[Short] Buffered with full I/O state: I%d, ts=%lu. Total: %d\n",
                inputIndex + 1, event.timestamp, shortBuffer.size());
  
  saveBufferToEEPROM();
}

/**
 * Simpan seluruh buffer ke EEPROM (dengan state I/O lengkap)
 */
void saveBufferToEEPROM()
{
  Serial.println("[Short] Saving buffer to EEPROM...");
  
  EEPROM.write(SHORT_BUFFER_SIGNATURE_ADDR, SHORT_BUFFER_SIGNATURE);
  
  uint8_t count = shortBuffer.size();
  EEPROM.write(SHORT_BUFFER_COUNT_ADDR, count);
  
  // Setiap event = 26 bytes
  int addr = SHORT_BUFFER_START_ADDR;
  
  for (size_t i = 0; i < count && i < SHORT_BUFFER_SIZE; i++)
  {
    ShortEvent &event = shortBuffer[i];
    
    EEPROM.write(addr++, event.inputIndex);
    EEPROM.write(addr++, event.triggerState);
    
    // Timestamp (4 bytes)
    EEPROM.write(addr++, (event.timestamp >> 0) & 0xFF);
    EEPROM.write(addr++, (event.timestamp >> 8) & 0xFF);
    EEPROM.write(addr++, (event.timestamp >> 16) & 0xFF);
    EEPROM.write(addr++, (event.timestamp >> 24) & 0xFF);
    
    // Input states (16 bytes)
    for (int j = 0; j < 16; j++)
    {
      EEPROM.write(addr++, event.inputStates[j]);
    }
    
    // Output states (4 bytes)
    for (int j = 0; j < 4; j++)
    {
      EEPROM.write(addr++, event.outputStates[j]);
    }
  }
  
  EEPROM.commit();
  Serial.printf("[Short] Saved %d events to EEPROM (%d bytes)\n", count, count * 26);
}

/**
 * Load buffer dari EEPROM (dengan state I/O lengkap)
 */
void loadBufferFromEEPROM()
{
  Serial.println("[Short] Loading buffer from EEPROM...");
  
  uint8_t sig = EEPROM.read(SHORT_BUFFER_SIGNATURE_ADDR);
  if (sig != SHORT_BUFFER_SIGNATURE)
  {
    Serial.println("[Short] No valid buffer signature found");
    return;
  }
  
  uint8_t count = EEPROM.read(SHORT_BUFFER_COUNT_ADDR);
  
  if (count == 0 || count > SHORT_BUFFER_SIZE)
  {
    Serial.println("[Short] Invalid buffer count");
    return;
  }
  
  shortBuffer.clear();
  int addr = SHORT_BUFFER_START_ADDR;
  
  for (int i = 0; i < count; i++)
  {
    ShortEvent event;
    
    event.inputIndex = EEPROM.read(addr++);
    event.triggerState = EEPROM.read(addr++);
    
    // Timestamp
    uint32_t ts = 0;
    ts |= ((uint32_t)EEPROM.read(addr++) << 0);
    ts |= ((uint32_t)EEPROM.read(addr++) << 8);
    ts |= ((uint32_t)EEPROM.read(addr++) << 16);
    ts |= ((uint32_t)EEPROM.read(addr++) << 24);
    event.timestamp = ts;
    
    // Input states
    for (int j = 0; j < 16; j++)
    {
      event.inputStates[j] = EEPROM.read(addr++);
    }
    
    // Output states
    for (int j = 0; j < 4; j++)
    {
      event.outputStates[j] = EEPROM.read(addr++);
    }
    
    if (event.inputIndex < NUM_INPUTS)
    {
      shortBuffer.push_back(event);
    }
  }
  
  if (shortBuffer.size() > 0)
  {
    hasOfflineData = true;
    Serial.printf("[Short] Loaded %d buffered events from EEPROM\n", shortBuffer.size());
  }
}

/**
 * Kirim semua data yang di-buffer ke server
 */
void sendBufferedShorts()
{
  if (shortBuffer.empty())
  {
    return;
  }

  if (!network_connected || !network_ready)
  {
    return;
  }

  Serial.printf("[Short] Sending buffered events one by one (total: %d)...\n", shortBuffer.size());

  bool anySent = false;

  while (!shortBuffer.empty())
  {
    // Selalu ambil event paling awal (kronologis)
    ShortEvent &event = shortBuffer.front();

    // Build JSON untuk satu event (I1..I16 & Q1..Q4)
    String jsonData = buildShortEventJSON(event);

    bool sent = false;

    if (config.commMode == "ws")
    {
#ifdef FIRMWARE_WIFI_ONLY
      if (ws_connected)
      {
        sent = wsSendText(jsonData);
      }
#endif
    }
    else if (config.commMode == "mqtt")
    {
      if (!mqttClient.connected())
      {
        reconnectMQTT();
      }

      if (mqttClient.connected())
      {
        bool isThingsBoard = (config.serverIP.indexOf("thingsboard") >= 0);
        String topic = isThingsBoard
                         ? "v1/devices/me/telemetry"
                         : "iot-node/" + config.hardwareId + "/telemetry";

        sent = mqttClient.publish(topic.c_str(), jsonData.c_str());
      }
    }
    else if (config.commMode == "httppost")
    {
      String response;
      sent = sendViaHTTP(jsonData, response);
    }

    if (!sent)
    {
      Serial.println("[Short] Failed to send current buffered event, will retry later");
      // Jangan hapus event ini, biar nanti dicoba lagi
      break;
    }

    anySent = true;
    Serial.printf("[Short] One buffered event sent. Remaining in buffer: %d\n", shortBuffer.size() - 1);

    // Hapus event yang sudah terkirim dari buffer RAM
    shortBuffer.erase(shortBuffer.begin());
  }

  // Kalau ada yang berhasil terkirim, sync ke EEPROM
  if (anySent)
  {
    if (shortBuffer.empty())
    {
      // Semua sudah terkirim
      clearShortBuffer();      // ini juga mengosongkan EEPROM + hasOfflineData = false
    }
    else
    {
      // Masih ada sisa, simpan sisa buffer ke EEPROM
      saveBufferToEEPROM();
      hasOfflineData = true;
    }
  }
}


String buildShortEventJSON(ShortEvent &event)
{
  bool isThingsBoard = (config.serverIP.indexOf("thingsboard") >= 0);

  StaticJsonDocument<512> doc;

  // Untuk server biasa: sertakan box_id
  if (!isThingsBoard)
  {
    doc["box_id"] = config.hardwareId;
  }

  // Input I1 - I16 (pakai state yang tersimpan di event, dan tetap dibalik 1→"0", 0→"1")
  for (int j = 0; j < NUM_INPUTS && j < 16; j++)
  {
    String key = "I" + String(j + 1);
    // Inverted logic seperti buildMonitoringJSON / processShortEvent
    doc[key] = (event.inputStates[j] == 1) ? "0" : "1";
  }

  // Output Q1 - Q4 (pakai state yang tersimpan di event)
  for (int j = 0; j < NUM_OUTPUTS && j < 4; j++)
  {
    String key = "Q" + String(j + 1);
    doc[key] = event.outputStates[j];   // 0 atau 1
  }

  // Info tambahan khusus server non-ThingsBoard
  if (!isThingsBoard)
  {
    doc["trigger"]   = "I" + String(event.inputIndex + 1);
    doc["event"]     = "short";
    doc["timestamp"] = event.timestamp;  // optional, kalau backend mau pakai
  }

  String jsonData;
  serializeJson(doc, jsonData);
  return jsonData;
}

/**
 * Build JSON untuk batch short events dengan state I/O lengkap
 */
String buildBatchShortJSON()
{
  bool isThingsBoard = (config.serverIP.indexOf("thingsboard") >= 0);
  
  DynamicJsonDocument doc(4096);
  
  if (!isThingsBoard)
  {
    doc["box_id"] = config.hardwareId;
    doc["event_type"] = "buffered_shorts";
    doc["count"] = shortBuffer.size();
    doc["sent_at"] = millis() / 1000;
  }
  
  JsonArray events = doc.createNestedArray("events");
  
  for (size_t i = 0; i < shortBuffer.size(); i++)
  {
    ShortEvent &event = shortBuffer[i];
    
    JsonObject eventObj = events.createNestedObject();
    
    // Timestamp
    eventObj["timestamp"] = event.timestamp;
    eventObj["trigger"] = "I" + String(event.inputIndex + 1);
    
    // Semua Input states (dengan inverted logic)
    for (int j = 0; j < NUM_INPUTS && j < 16; j++)
    {
      String key = "I" + String(j + 1);
      eventObj[key] = (event.inputStates[j] == 1) ? "0" : "1";
    }
    
    // Semua Output states
    for (int j = 0; j < NUM_OUTPUTS && j < 4; j++)
    {
      String key = "Q" + String(j + 1);
      eventObj[key] = event.outputStates[j];
    }
  }
  
  String jsonData;
  serializeJson(doc, jsonData);
  
  return jsonData;
}

/**
 * Hapus buffer setelah berhasil kirim
 */
void clearShortBuffer()
{
  shortBuffer.clear();
  hasOfflineData = false;
  
  // Clear EEPROM buffer
  EEPROM.write(SHORT_BUFFER_SIGNATURE_ADDR, 0x00); // Invalidate signature
  EEPROM.write(SHORT_BUFFER_COUNT_ADDR, 0);
  EEPROM.commit();
  
  Serial.println("[Short] Buffer cleared");
}

/**
 * Sinkronisasi state input saat startup
 */
void syncInputStates()
{
  Serial.println("[Short] Syncing input states...");
  
  // Baca PCF8574
  uint8_t pcfData = pcf8574.read8();
  for (int i = 0; i < NUM_PCF_INPUTS; i++)
  {
    uint8_t state = (pcfData >> PCF_PIN_MAP[i]) & 0x01;
    inputFilters[i].lastStableState = state;
    inputFilters[i].pendingState = state;
    inputFilters[i].isFiltering = false;
    lastInputStates[i] = state;
  }
  
  // Baca GPIO inputs
  for (int i = 0; i < NUM_GPIO_INPUTS; i++)
  {
    uint8_t state = digitalRead(INPUT_GPIO_PINS[i]);
    int idx = NUM_PCF_INPUTS + i;
    inputFilters[idx].lastStableState = state;
    inputFilters[idx].pendingState = state;
    inputFilters[idx].isFiltering = false;
    lastInputStates[idx] = state;
  }
  
  Serial.println("[Short] Input states synchronized");
}

// =================================================================
// --- Ethernet (W5500) ---
// =================================================================
bool initEthernet()
{
#ifndef FIRMWARE_ETHERNET_ONLY
  return false;
#else
  Serial.println("\n[Ethernet] Initializing...");
  if (eth_rst_pin >= 0)
  {
    pinMode(eth_rst_pin, OUTPUT);
    digitalWrite(eth_rst_pin, LOW);
    delay(50);
    digitalWrite(eth_rst_pin, HIGH);
    delay(200);
  }

  SPI.begin(eth_sclk_pin, eth_miso_pin, eth_mosi_pin, eth_cs_pin);
  Ethernet.init(eth_cs_pin);

  uint64_t chipid = ESP.getEfuseMac();
  byte mac[6];
  mac[0] = 0xDE;
  mac[1] = 0xAD;
  mac[2] = (chipid >> 32) & 0xFF;
  mac[3] = (chipid >> 24) & 0xFF;
  mac[4] = (chipid >> 16) & 0xFF;
  mac[5] = (chipid >> 8) & 0xFF;

  Serial.print("  MAC: ");
  for (int i = 0; i < 6; i++)
  {
    if (mac[i] < 16)
      Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5)
      Serial.print(":");
  }
  Serial.println();

  Serial.print("  Trying DHCP... ");
  if (Ethernet.begin(mac, 5000) == 0) // ← Simplified timeout
  {
    Serial.println("FAILED");
    Serial.print("  Trying Static IP... ");
    IPAddress ip(192, 168, 0, 100);
    IPAddress dns(8, 8, 8, 8);
    IPAddress gateway(192, 168, 0, 1);
    IPAddress subnet(255, 255, 255, 0);
    Ethernet.begin(mac, ip, dns, gateway, subnet);
    delay(1000);
  }
  else
  {
    Serial.println("OK");
  }

  delay(500);
  if (Ethernet.linkStatus() == LinkOFF)
  {
    Serial.println("  No cable detected!");
    return false;
  }

  Serial.println("  Cable connected");
  Serial.print("  IP: ");
  Serial.println(Ethernet.localIP());
  Serial.print("  GW: ");
  Serial.println(Ethernet.gatewayIP());
  Serial.print("  MASK: ");
  Serial.println(Ethernet.subnetMask());
  Serial.print("  DNS: ");
  Serial.println(Ethernet.dnsServerIP());
  return true;
#endif
}

// =================================================================
// --- File System & Config ---
// =================================================================
bool loadConfig()
{
  bool loadedFromJson = false;
  File file = LittleFS.open("/config.json", "r");
  if (file)
  {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    if (!error)
    {
      config.hardwareId = doc["hardwareId"] | "unconfigured_json";
      JsonObject serverObj = doc["server"];
      config.serverIP = serverObj["serverIP"] | "";
      config.path = serverObj["path"] | "/ws";
      config.monitoringPort = doc["monitoringPort"] | 80;
      config.commMode = doc["commMode"] | "ws";
      JsonObject wifiObj = doc["wifi"];
      config.wifiSSID = wifiObj["ssid"] | "";
      config.wifiPass = wifiObj["password"] | "";
      config.accessTokenMQTT = doc["accessTokenMQTT"] | "";
      config.networkMode = doc["networkMode"] | "Hybrid";
      Serial.println("Configuration loaded from LittleFS (JSON).");
      loadedFromJson = true;
    }
    else
    {
      Serial.println("ERR: Parse JSON: " + String(error.c_str()) + ". Trying EEPROM...");
    }
  }
  else
  {
    Serial.println("config.json not found. Trying EEPROM...");
  }

  if (!loadedFromJson)
  {
    Serial.println("Loading from EEPROM...");
    config.hardwareId = readFromEEPROM(HARDWARE_ID_ADDR);
    config.serverIP = readFromEEPROM(SERVER_IP_ADDR);
    config.path = readFromEEPROM(PATH_ADDR);
    config.monitoringPort = readFromEEPROM(SERVER_PORT_ADDR).toInt();
    config.commMode = readFromEEPROM(COMM_MODE_ADDR);
    config.wifiSSID = readFromEEPROM(WIFI_SSID_ADDR);
    config.wifiPass = readFromEEPROM(WIFI_PASS_ADDR);
    config.networkMode = readFromEEPROM(NETWORK_MODE_ADDR);
    config.accessTokenMQTT = readFromEEPROM(ACCESS_TOKEN_ADDR);
    if (config.hardwareId.length() == 0)
      config.hardwareId = "unconfigured_eeprom";
    if (config.path.length() == 0)
      config.path = "/wsiot";
    if (config.monitoringPort <= 0)
      config.monitoringPort = 80;
    if (config.commMode.length() == 0)
      config.commMode = "ws";
    if (config.networkMode.length() == 0)
      config.networkMode = "Hybrid";
  }

  config.hardwareId.trim();
  config.networkMode.trim();
  config.serverIP.trim();
  config.path.trim();
  config.commMode.trim();
  config.wifiSSID.trim();
  config.wifiPass.trim();
  config.accessTokenMQTT.trim();

  if (!config.networkMode.equalsIgnoreCase("WiFi") &&
      !config.networkMode.equalsIgnoreCase("Ethernet") &&
      !config.networkMode.equalsIgnoreCase("Hybrid"))
  {
    Serial.println("[WARN] Invalid networkMode, setting to Hybrid");
    config.networkMode = "Hybrid";
  }

  Serial.println("Syncing initial config to LittleFS...");
  saveConfig();
  return true;
}

bool saveConfig()
{
  bool successJson = false;
  bool successEeprom = false;
  StaticJsonDocument<512> doc;
  doc["hardwareId"] = config.hardwareId;
  JsonObject serverObj = doc.createNestedObject("server");
  serverObj["serverIP"] = config.serverIP;
  serverObj["path"] = config.path;
  doc["monitoringPort"] = config.monitoringPort;
  doc["commMode"] = config.commMode;
  doc["networkMode"] = config.networkMode;
  doc["accessTokenMQTT"] = config.accessTokenMQTT;
  JsonObject wifiObj = doc.createNestedObject("wifi");
  wifiObj["ssid"] = config.wifiSSID;
  wifiObj["password"] = config.wifiPass;
  File file = LittleFS.open("/config.json", "w");
  if (file)
  {
    successJson = (serializeJson(doc, file) > 0);
    file.close();
  }
  writeToEEPROM(HARDWARE_ID_ADDR, config.hardwareId);
  writeToEEPROM(SERVER_IP_ADDR, config.serverIP);
  writeToEEPROM(WIFI_SSID_ADDR, config.wifiSSID);
  writeToEEPROM(WIFI_PASS_ADDR, config.wifiPass);
  writeToEEPROM(SERVER_PORT_ADDR, String(config.monitoringPort));
  writeToEEPROM(PATH_ADDR, config.path);
  writeToEEPROM(COMM_MODE_ADDR, config.commMode);
  writeToEEPROM(NETWORK_MODE_ADDR, config.networkMode);
  writeToEEPROM(ACCESS_TOKEN_ADDR, config.accessTokenMQTT);
  successEeprom = EEPROM.commit();
  return successJson && successEeprom;
}

String loadFile(const char *path)
{
  File file = LittleFS.open(path, "r");
  if (!file)
    return "";
  String content = file.readString();
  file.close();
  return content;
}

// =================================================================
// --- JSON Builders ---
// =================================================================
String buildStatusJSON()
{
  StaticJsonDocument<512> doc;
  doc["hardwareId"] = config.hardwareId;
  doc["firmware_version"] = program_version;
  doc["commMode"] = config.commMode;
  doc["serverIP"] = config.serverIP;
  doc["monitoringPort"] = config.monitoringPort;
  doc["networkMode"] = config.networkMode;
#ifdef FIRMWARE_WIFI_ONLY
  if (network_connected)
  {
    doc["connection_type"] = "WiFi";
    doc["ip_address"] = WiFi.localIP().toString();
    doc["mac_address"] = WiFi.macAddress();
    doc["wifiSSID"] = config.wifiSSID;
  }
  else
  {
    doc["connection_type"] = "None";
  }
#else
  if (network_connected)
  {
    doc["connection_type"] = "Ethernet";
    doc["ip_address"] = Ethernet.localIP().toString();
  }
  else
  {
    doc["connection_type"] = "None";
  }
#endif
  // =============================================

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

String buildMonitoringJSON(bool forThingsBoard = false)
{
  StaticJsonDocument<512> doc;
  if (!forThingsBoard)
  {
    doc["box_id"] = config.hardwareId;
  }
  for (int i = 0; i < NUM_PCF_INPUTS; i++)
  {
    String kondisi_input = String(lastInputStates[i]);
    kondisi_input = (kondisi_input == "1") ? "0" : "1";
    doc["I" + String(i + 1)] = kondisi_input;
  }
  for (int i = 0; i < NUM_GPIO_INPUTS; i++)
  {
    String kondisi_input = String(lastInputStates[NUM_PCF_INPUTS + i]);
    kondisi_input = (kondisi_input == "1") ? "0" : "1";
    doc["I" + String(NUM_PCF_INPUTS + i + 1)] = kondisi_input;
  }
  for (int i = 0; i < NUM_OUTPUTS; i++)
    doc["Q" + String(i + 1)] = digitalRead(OUTPUT_PINS[i]);
  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

// =================================================================
// --- MQTT ---
// =================================================================
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("[MQTT] Message received on topic: ");
  Serial.println(topic);

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error)
  {
    Serial.println("[MQTT] JSON parse error");
    return;
  }

  if (!doc.containsKey("box_id") || doc["box_id"].as<String>() != config.hardwareId)
  {
    Serial.println("[MQTT] box_id mismatch, ignoring");
    return;
  }

  if (doc.containsKey("cmd"))
  {
    String cmd = doc["cmd"].as<String>();

    if (cmd == "config")
    {
      StaticJsonDocument<400> resp;
      resp["box_id"] = config.hardwareId;
      resp["serverIP"] = config.serverIP;
      resp["port"] = config.monitoringPort;
      resp["wifiSSID"] = config.wifiSSID;
      resp["networkMode"] = config.networkMode;

#ifdef FIRMWARE_WIFI_ONLY
      if (network_connected)
      {
        resp["ip"] = WiFi.localIP().toString();
        resp["mac"] = WiFi.macAddress();
      }
#else
      if (network_connected)
      {
        resp["ip"] = Ethernet.localIP().toString();
      }
#endif

      String jsonOut;
      serializeJson(resp, jsonOut);
      String topic = "iot-node/" + config.hardwareId + "/config_response";
      mqttClient.publish(topic.c_str(), jsonOut.c_str());
    }
    else if (cmd == "restart")
    {
      StaticJsonDocument<100> resp;
      resp["box_id"] = config.hardwareId;
      resp["status"] = "restarting";
      String jsonOut;
      serializeJson(resp, jsonOut);
      String topic = "iot-node/" + config.hardwareId + "/status";
      mqttClient.publish(topic.c_str(), jsonOut.c_str());
      delay(500);
      ESP.restart();
    }
  }
  else if (doc.containsKey("serverIP") || doc.containsKey("wifiSSID") ||
           doc.containsKey("monitoringPort") || doc.containsKey("wifiPass") ||
           doc.containsKey("networkMode"))
  {

    bool updated = false;
    if (doc.containsKey("serverIP") && doc["serverIP"].as<String>() != config.serverIP)
    {
      config.serverIP = doc["serverIP"].as<String>();
      updated = true;
    }
    if (doc.containsKey("monitoringPort") && doc["monitoringPort"].as<int>() != config.monitoringPort)
    {
      config.monitoringPort = doc["monitoringPort"].as<int>();
      updated = true;
    }
    if (doc.containsKey("wifiSSID") && doc["wifiSSID"].as<String>() != config.wifiSSID)
    {
      config.wifiSSID = doc["wifiSSID"].as<String>();
      updated = true;
    }
    if (doc.containsKey("wifiPass") && doc["wifiPass"].as<String>().length() > 0)
    {
      config.wifiPass = doc["wifiPass"].as<String>();
      updated = true;
    }
    if (doc.containsKey("networkMode"))
    {
      String newMode = doc["networkMode"].as<String>();
      if ((newMode == "Ethernet" || newMode == "WiFi" || newMode == "Hybrid") &&
          newMode != config.networkMode)
      {
        config.networkMode = newMode;
        updated = true;
      }
    }

    if (updated)
    {
      saveConfig();
      StaticJsonDocument<100> resp;
      resp["box_id"] = config.hardwareId;
      resp["status"] = "config_updated";
      String jsonOut;
      serializeJson(resp, jsonOut);
      String topic = "iot-node/" + config.hardwareId + "/status";
      mqttClient.publish(topic.c_str(), jsonOut.c_str());
      delay(1000);
      ESP.restart();
    }
  }
  else if (doc.containsKey("O1") || doc.containsKey("O2") ||
           doc.containsKey("O3") || doc.containsKey("O4"))
  {
    for (int i = 0; i < NUM_OUTPUTS; i++)
    {
      String key = "O" + String(i + 1);
      if (doc.containsKey(key))
      {
        int state = doc[key].as<int>();
        digitalWrite(OUTPUT_PINS[i], state == 1 ? HIGH : LOW);
      }
    }
    saveIOStateToEEPROM();
  }
}

void reconnectMQTT()
{
  if (millis() - lastMqttReconnectAttempt < mqttReconnectInterval)
    return;
  lastMqttReconnectAttempt = millis();

  mqttClient.setClient(ACTIVE_CLIENT);
  mqttClient.setServer(config.serverIP.c_str(), config.monitoringPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(8192);
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(15);

  Serial.println("\n═══ MQTT CONNECTION ═══");
  Serial.println("Server: " + config.serverIP + ":" + String(config.monitoringPort));
  Serial.printf("Transport: %s\n", NETWORK_MODE_NAME);

  String clientId = "IoTNode-" + config.hardwareId;
  Serial.println("ClientID: " + clientId);

  bool isThingsBoard = (config.serverIP.indexOf("thingsboard") >= 0);
  
  if (isThingsBoard) {
    Serial.println("Mode: ThingsBoard Cloud");
    Serial.println("Token: " + config.accessTokenMQTT.substring(0, 8) + "***");
  }
  
  Serial.print("Connecting... ");

  bool ok = false;

  if (isThingsBoard && config.accessTokenMQTT.length() > 0) {
    ok = mqttClient.connect(clientId.c_str(), 
                            config.accessTokenMQTT.c_str(), 
                            NULL);
  } else {
    ok = mqttClient.connect(clientId.c_str());
  }

  if (ok)
  {
    Serial.println("SUCCESS!");
    
    if (isThingsBoard) {
      String rpcTopic = "v1/devices/me/rpc/request/+";
      mqttClient.subscribe(rpcTopic.c_str());
      Serial.println("Subscribed: " + rpcTopic);

      String attrTopic = "v1/devices/me/attributes";
      String attrData = "{\"firmware\":\"" + program_version + "\",\"hardware\":\"" + config.hardwareId + "\"}";
      mqttClient.publish(attrTopic.c_str(), attrData.c_str());
    } else {
      String cmdTopic = "iot-node/" + config.hardwareId + "/cmd";
      mqttClient.subscribe(cmdTopic.c_str());
      Serial.println("Subscribed: " + cmdTopic);
    }
    
    publishStatus();
    readyToSend = true;
    mqttReconnectInterval = 5000;
  }
  else
  {
    Serial.print("✗ FAILED! RC=");
    Serial.println(mqttClient.state());
    
    // Decode error
    switch(mqttClient.state()) {
      case -4: Serial.println("   → Connection timeout"); break;
      case -3: Serial.println("   → Connection lost"); break;
      case -2: Serial.println("   → Connect failed"); break;
      case -1: Serial.println("   → Disconnected"); break;
      case 1:  Serial.println("   → Bad protocol"); break;
      case 2:  Serial.println("   → Bad client ID"); break;
      case 3:  Serial.println("   → Server unavailable"); break;
      case 4:  Serial.println("   → Bad credentials ← CHECK TOKEN!"); break;
      case 5:  Serial.println("   → Unauthorized"); break;
    }
    
    readyToSend = false;
    mqttReconnectInterval = 5000; 
  }
}

void publishTelemetry()
{
  if (!mqttClient.connected())
    return;

  bool isThingsBoard = (config.serverIP.indexOf("thingsboard") >= 0);
  String data = buildMonitoringJSON(isThingsBoard);

  String telemetryTopic = isThingsBoard ? "v1/devices/me/telemetry" : "iot-node/" + config.hardwareId + "/telemetry";

  if (mqttClient.publish(telemetryTopic.c_str(), data.c_str()))
  {
    Serial.println("[MQTT] Sent to: " + telemetryTopic);
    Serial.println("Sending : " + data);
  }
  else
  {
    Serial.println("[MQTT] Failed to publish");
  }
}

/// =================================================================
// --- WebSocket Connect ---
// =================================================================
bool wsConnect()
{
#ifdef FIRMWARE_WIFI_ONLY
  Serial.printf("[WS] Connecting to ws://%s:%d%s\n",
                config.serverIP.c_str(),
                config.monitoringPort,
                config.path.c_str());
  Serial.printf("[WS] Transport: %s\n", NETWORK_MODE_NAME);

  wsClient.onEvent([](WStype_t type, uint8_t *payload, size_t length)
                   {
    switch (type) {
    case WStype_DISCONNECTED:
      {
        Serial.println("[WS] Disconnected!");
        ws_connected = false;
      }
      break;

    case WStype_CONNECTED:
      {
        Serial.printf("[WS] ✓ Connected to: %s\n", payload);
        ws_connected = true;
        publishStatus();
      }
      break;

    case WStype_TEXT:
      {
        Serial.printf("[WS] ← Received: %s\n", payload);
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, payload, length);
        
        if (!error) {
          processCommand(doc);
        } else {
          Serial.printf("[WS] JSON parse error: %s\n", error.c_str());
        }
      }
      break;

    case WStype_BIN:
      Serial.printf("[WS] Binary data (%u bytes)\n", length);
      break;

    case WStype_PING:
      Serial.println("[WS] Ping");
      break;

    case WStype_PONG:
      Serial.println("[WS] Pong");
      break;

    case WStype_ERROR:
      {
        Serial.printf("[WS]  Error: %s\n", payload);
        ws_connected = false;
      }
      break;
      
    default:
      break;
    } });

  wsClient.setReconnectInterval(5000);

  IPAddress serverIP;
  if (serverIP.fromString(config.serverIP))
  {
    wsClient.begin(serverIP, config.monitoringPort, config.path.c_str());
  }
  else
  {
    wsClient.begin(config.serverIP, config.monitoringPort, config.path);
  }

  Serial.println("[WS] Connection initiated...");
  return true;
#endif
}

void wsDisconnect()
{
#ifdef FIRMWARE_WIFI_ONLY
  Serial.println("[WS] Disconnecting...");
  wsClient.disconnect();
  ws_connected = false;
#endif
}

bool wsSendText(const String &s)
{
#ifdef FIRMWARE_WIFI_ONLY
  if (!ws_connected)
  {
    Serial.println("[WS] Not connected, cannot send");
    return false;
  }

  String payload = s;
  wsClient.sendTXT(payload);
  return true;
#endif
}

void wsPump()
{
#ifdef FIRMWARE_WIFI_ONLY
  wsClient.loop();
#endif
}

// =================================================================
// --- HTTP POST Helper ---
// =================================================================
bool ethHttpPost(const String &host, uint16_t port, const String &path, const String &body, String &response)
{
#ifndef FIRMWARE_ETHERNET_ONLY
  // Ethernet tidak tersedia di WiFi firmware
  Serial.println("[HTTP] Ethernet not available in WiFi firmware");
  return false;

#else
  // ============== ETHERNET IMPLEMENTATION ==============
  if (!netClient.connect(host.c_str(), port))
  {
    Serial.println("[HTTP] Ethernet connect failed");
    return false;
  }

  netClient.print("POST ");
  netClient.print(path);
  netClient.print(" HTTP/1.1\r\n");
  netClient.print("Host: ");
  netClient.print(host);
  netClient.print("\r\n");
  netClient.print("Content-Type: application/json\r\n");
  netClient.print("Content-Length: ");
  netClient.print(body.length());
  netClient.print("\r\n");
  netClient.print("Connection: close\r\n\r\n");
  netClient.print(body);

  unsigned long t0 = millis();
  while (netClient.connected() && !netClient.available() && millis() - t0 < 5000)
    delay(10);

  response = "";
  while (netClient.available())
    response += (char)netClient.read();
  netClient.stop();

  int status = -1;
  int sp1 = response.indexOf(' ');
  if (sp1 > 0)
  {
    int sp2 = response.indexOf(' ', sp1 + 1);
    if (sp2 > sp1)
      status = response.substring(sp1 + 1, sp2).toInt();
  }
  return status >= 200 && status < 300;
#endif
}

bool sendViaHTTP(const String &jsonData, String &resp)
{
#ifdef FIRMWARE_WIFI_ONLY
  // ============== WIFI FIRMWARE ==============
  HTTPClient http;
  String url = "http://" + config.serverIP + ":" +
               String(config.monitoringPort) + config.path;
  http.begin(netClient, url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-ID", config.hardwareId);

  int code = http.POST(jsonData);
  resp = (code > 0) ? http.getString() : "";

  if (code <= 0)
  {
    Serial.printf("[HTTP] ✗ WiFi Error: %s\n", http.errorToString(code).c_str());
  }

  http.end();
  return code > 0;

#else
  // ============== ETHERNET FIRMWARE ==============
  return ethHttpPost(config.serverIP, config.monitoringPort,
                     config.path, jsonData, resp);
#endif
}

// =================================================================
// --- HTTP POST Command Polling ---
// =================================================================
void pollHTTPCommands()
{
  if (millis() - lastHttpPollTime < httpPollInterval)
    return;
  lastHttpPollTime = millis();

  String cmdUrl = config.path;
  if (!cmdUrl.endsWith("/"))
    cmdUrl += "/";
  cmdUrl += "commands";

#ifdef FIRMWARE_WIFI_ONLY
  HTTPClient http;
  String fullUrl = "http://" + config.serverIP + ":" + String(config.monitoringPort) + cmdUrl;
  http.begin(netClient, fullUrl); // ← netClient
  int code = http.GET();
  if (code == HTTP_CODE_OK)
  {
    String payload = http.getString();
    if (payload.length() && payload != "null" && payload != "[]")
    {
      StaticJsonDocument<512> doc;
      if (!deserializeJson(doc, payload))
        processCommand(doc);
    }
  }
  http.end();

#else
  String resp;
  if (netClient.connect(config.serverIP.c_str(), config.monitoringPort)) // ← netClient (bukan ethClient)
  {
    netClient.print("GET ");
    netClient.print(cmdUrl);
    netClient.print(" HTTP/1.1\r\n");
    netClient.print("Host: ");
    netClient.print(config.serverIP);
    netClient.print("\r\n");
    netClient.print("Connection: close\r\n\r\n");
    unsigned long t0 = millis();
    while (netClient.connected() && !netClient.available() && millis() - t0 < 5000)
      delay(10);
    while (netClient.available())
      resp += (char)netClient.read();
    netClient.stop();

    int sep = resp.indexOf("\r\n\r\n");
    String body = (sep >= 0) ? resp.substring(sep + 4) : resp;
    if (body.length() && body != "null" && body != "[]")
    {
      StaticJsonDocument<512> doc;
      if (!deserializeJson(doc, body))
        processCommand(doc);
    }
  }
#endif
}

// =================================================================
// --- Universal Command Processor ---
// =================================================================
void processCommand(JsonDocument &doc)
{
  if (doc.containsKey("box_id") && doc["box_id"].as<String>() != config.hardwareId)
  {
    Serial.println("[CMD] box_id mismatch, ignoring");
    return;
  }

  if (doc.containsKey("cmd"))
  {
    String cmd = doc["cmd"].as<String>();
    Serial.println("[CMD] Received: " + cmd);

    if (cmd == "config")
    {
      // Send configuration info
      publishStatus();
    }
    else if (cmd == "restart")
    {
      Serial.println("[CMD] Restart requested");

      // Send acknowledgment
      StaticJsonDocument<100> resp;
      resp["box_id"] = config.hardwareId;
      resp["status"] = "restarting";
      String jsonOut;
      serializeJson(resp, jsonOut);

      if (config.commMode == "ws")
      {
        wsSendText(jsonOut);
      }
      else if (config.commMode == "mqtt")
      {
        String topic = "iot-node/" + config.hardwareId + "/status";
        mqttClient.publish(topic.c_str(), jsonOut.c_str());
      }
      else if (config.commMode == "httppost")
      {
        String response;
        sendViaHTTP(jsonOut, response);
      }

      delay(500);
      ESP.restart();
    }
    else if (cmd == "get_status")
    {
      publishStatus();
    }
    else if (cmd == "get_io")
    {
      sendMonitoringData();
    }
  }

  // Handle configuration update
  else if (doc.containsKey("serverIP") || doc.containsKey("wifiSSID") ||
           doc.containsKey("monitoringPort") || doc.containsKey("wifiPass") ||
           doc.containsKey("networkMode"))
  {
    bool updated = false;

    if (doc.containsKey("serverIP") && doc["serverIP"].as<String>() != config.serverIP)
    {
      config.serverIP = doc["serverIP"].as<String>();
      updated = true;
    }
    if (doc.containsKey("monitoringPort") && doc["monitoringPort"].as<int>() != config.monitoringPort)
    {
      config.monitoringPort = doc["monitoringPort"].as<int>();
      updated = true;
    }
    if (doc.containsKey("wifiSSID") && doc["wifiSSID"].as<String>() != config.wifiSSID)
    {
      config.wifiSSID = doc["wifiSSID"].as<String>();
      updated = true;
    }
    if (doc.containsKey("wifiPass") && doc["wifiPass"].as<String>().length() > 0)
    {
      config.wifiPass = doc["wifiPass"].as<String>();
      updated = true;
    }
    if (doc.containsKey("networkMode"))
    {
      String newMode = doc["networkMode"].as<String>();
      if ((newMode == "Ethernet" || newMode == "WiFi" || newMode == "Hybrid") &&
          newMode != config.networkMode)
      {
        config.networkMode = newMode;
        updated = true;
      }
    }

    if (updated)
    {
      Serial.println("[CMD] Configuration updated");
      saveConfig();

      // Send acknowledgment
      StaticJsonDocument<100> resp;
      resp["box_id"] = config.hardwareId;
      resp["status"] = "config_updated";
      String jsonOut;
      serializeJson(resp, jsonOut);

      if (config.commMode == "ws")
      {
        wsSendText(jsonOut);
      }
      else if (config.commMode == "mqtt")
      {
        String topic = "iot-node/" + config.hardwareId + "/status";
        mqttClient.publish(topic.c_str(), jsonOut.c_str());
      }
      else if (config.commMode == "httppost")
      {
        String response;
        sendViaHTTP(jsonOut, response);
      }

      delay(1000);
      ESP.restart();
    }
  }

  // Handle output control
  else if (doc.containsKey("O1") || doc.containsKey("O2") ||
           doc.containsKey("O3") || doc.containsKey("O4"))
  {
    Serial.println("[CMD] Output control received");

    for (int i = 0; i < NUM_OUTPUTS; i++)
    {
      String key = "O" + String(i + 1);
      if (doc.containsKey(key))
      {
        int state = doc[key].as<int>();
        digitalWrite(OUTPUT_PINS[i], state == 1 ? HIGH : LOW);
        Serial.printf("[CMD] O%d = %d\n", i + 1, state);
      }
    }

    saveIOStateToEEPROM();

    // Send acknowledgment with current I/O state
    delay(100);
    sendMonitoringData();
  }
}

// =================================================================
// --- HTTP/WS/MQTT Send Helpers ---
// =================================================================
void sendMonitoringData()
{
  if (!network_ready)
    return;

  bool isThingsBoard = (config.serverIP.indexOf("thingsboard") >= 0);
  String data = buildMonitoringJSON(isThingsBoard);

  // Serial.println("Sending : " + data);

  if (config.commMode == "ws")
  {
    if (ws_connected)
    {
      wsSendText(data);
    }
    else
    {
      Serial.println("[WS] Not connected, skipping send");
    }
  }
  else if (config.commMode == "mqtt")
  {
    if (mqttClient.connected())
    {
      String topic = isThingsBoard ? "v1/devices/me/telemetry" : "iot-node/" + config.hardwareId + "/telemetry";
      mqttClient.publish(topic.c_str(), data.c_str());
    }
  }
  else if (config.commMode == "httppost")
  {
    String response;
    if (sendViaHTTP(data, response))
    {
      Serial.println("[HTTP] Telemetry sent successfully");

      // Check if server sent command in response
      if (response.length() > 0 && response != "OK" && response != "null")
      {
        Serial.println("[HTTP] Server response: " + response);

        StaticJsonDocument<512> doc;
        if (deserializeJson(doc, response) == DeserializationError::Ok)
        {
          processCommand(doc);
        }
      }
    }
    else
    {
      Serial.println("[HTTP] Failed to send telemetry");
    }
  }
}

void publishStatus()
{
  if (!network_ready)
    return;

  String statusData = buildStatusJSON();
  Serial.println("Publishing Status: " + statusData);

  if (config.commMode == "ws")
  {
    if (ws_connected)
    {
      wsSendText(statusData);
    }
  }
  else if (config.commMode == "mqtt")
  {
    if (mqttClient.connected())
    {
      String topic = "iot-node/" + config.hardwareId + "/status";
      mqttClient.publish(topic.c_str(), statusData.c_str());
    }
  }
  else if (config.commMode == "httppost")
  {
    String response;
    if (sendViaHTTP(statusData, response))
    {
      Serial.println("[HTTP] Status published successfully");
    }
  }
}

// =================================================================
// --- I/O & WEB SERVER (ringkas) ---
// =================================================================
bool checkInputChanges()
{
  bool changed = false;
  uint8_t s = pcf8574.read8();
  for (int i = 0; i < NUM_PCF_INPUTS; i++)
  {
    uint8_t current = (s >> PCF_PIN_MAP[i]) & 0x01;
    if (current != lastInputStates[i])
    {
      lastInputStates[i] = current;
      changed = true;
    }
  }
  for (int i = 0; i < NUM_GPIO_INPUTS; i++)
  {
    uint8_t current = digitalRead(INPUT_GPIO_PINS[i]);
    int index = NUM_PCF_INPUTS + i;
    if (current != lastInputStates[index])
    {
      lastInputStates[index] = current;
      changed = true;
    }
  }
  if (changed)
    saveIOStateToEEPROM();
  return changed;
}

String htmlPage(String body)
{
  return String("<html><body>") + body + "</body></html>";
}

// Handler untuk Dashboard
void handleDashboard()
{
  if (!isAuthenticated)
  {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
    return;
  }

  File file = LittleFS.open("/dashboard.html", "r");
  if (file)
  {
    String html = file.readString();
    file.close();

    String commModeUpper = config.commMode;
    commModeUpper.toUpperCase();

    // Replace placeholders
    html.replace("%HARDWARE_ID%", config.hardwareId);
    html.replace("%COMM_MODE%", commModeUpper);
    html.replace("%SERVER_IP%", config.serverIP);
    html.replace("%SERVER_PORT%", String(config.monitoringPort));

    // Determine connection type
    String connectionType = "Not Connected";
    
#ifdef FIRMWARE_WIFI_ONLY
    if (network_connected) {
      connectionType = "WiFi";
    }
#else
    if (network_connected) {
      connectionType = "Ethernet";
    }
#endif

    html.replace("%NETWORK_MODE%", connectionType);

    server.send(200, "text/html", html);
  }
  else
  {
    serveErrorPage("Dashboard not found");
  }
}

// Handler untuk Config Page

void handleConfigPage()
{
  if (!isAuthenticated)
  {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
    return;
  }

  File file = LittleFS.open("/config.html", "r");
  if (file)
  {
    String html = file.readString();
    file.close();

    // Replace placeholders
    html.replace("%HARDWARE_ID%", config.hardwareId);
    html.replace("%SERVER_IP%", config.serverIP);
    html.replace("%SERVER_PORT%", String(config.monitoringPort));
    html.replace("%PATH%", config.path);
    html.replace("%WIFI_SSID%", config.wifiSSID);
    html.replace("%WIFI_PASS%", config.wifiPass);
    html.replace("%ACCESS_TOKEN%", config.accessTokenMQTT);

    // Determine connection type and available comm modes
    String connectionType = "Unknown";
    String commOptions = "";

#ifdef FIRMWARE_WIFI_ONLY
    connectionType = "WiFi";
    // WiFi: All 3 modes available
    commOptions = "<option value=\"ws\"" + String(config.commMode == "ws" ? " selected" : "") + ">WebSocket</option>";
    commOptions += "<option value=\"mqtt\"" + String(config.commMode == "mqtt" ? " selected" : "") + ">MQTT</option>";
    commOptions += "<option value=\"httppost\"" + String(config.commMode == "httppost" ? " selected" : "") + ">HTTP POST</option>";
#else
    connectionType = "Ethernet";
    commOptions = "<option value=\"mqtt\"" + String(config.commMode == "mqtt" ? " selected" : "") + ">MQTT</option>";
    commOptions += "<option value=\"httppost\"" + String(config.commMode == "httppost" ? " selected" : "") + ">HTTP POST</option>";
    
    if (config.commMode == "ws") {
      commOptions.replace("value=\"mqtt\"", "value=\"mqtt\" selected");
    }
#endif

    html.replace("%CONNECTION_TYPE%", connectionType);
    html.replace("%COMM_OPTIONS%", commOptions);

    server.send(200, "text/html", html);
  }
  else
  {
    serveErrorPage("Config page not found");
  }
}

// Handler untuk Control Page
void handleControlPage()
{
  if (!isAuthenticated)
  {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
    return;
  }

  File file = LittleFS.open("/control.html", "r");
  if (file)
  {
    server.streamFile(file, "text/html");
    file.close();
  }
  else
  {
    serveErrorPage("Control page not found");
  }
}

// Handler untuk Status Page
void handleStatusPage()
{
  if (!isAuthenticated)
  {
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
    return;
  }

  File file = LittleFS.open("/status.html", "r");
  if (file)
  {
    String html = file.readString();
    file.close();

    // Replace placeholders
    html.replace("%HARDWARE_ID%", config.hardwareId);
    html.replace("%NETWORK_MODE%", config.networkMode);
    html.replace("%WIFI_SSID%", config.wifiSSID);
    html.replace("%COMM_MODE%", config.commMode);
    html.replace("%SERVER_IP%", config.serverIP);
    html.replace("%SERVER_PORT%", String(config.monitoringPort));
    html.replace("%PATH%", config.path);

    // ✅ FIX: Conditional compilation
#ifdef FIRMWARE_WIFI_ONLY
    if (network_connected)
    {
      html.replace("%IP_ADDRESS%", WiFi.localIP().toString());
      html.replace("%MAC_ADDRESS%", WiFi.macAddress());
    }
    else
    {
      html.replace("%IP_ADDRESS%", "Not Connected");
      html.replace("%MAC_ADDRESS%", "N/A");
    }
#else
    if (network_connected)
    {
      html.replace("%IP_ADDRESS%", Ethernet.localIP().toString());
      html.replace("%MAC_ADDRESS%", "N/A");
    }
    else
    {
      html.replace("%IP_ADDRESS%", "Not Connected");
      html.replace("%MAC_ADDRESS%", "N/A");
    }
#endif

    server.send(200, "text/html", html);
  }
  else
  {
    serveErrorPage("Status page not found");
  }
}

// Update handleRoot - redirect to dashboard after login
void handleRoot()
{
  if (!isAuthenticated)
  {
    File file = LittleFS.open("/login.html", "r");
    if (file)
    {
      server.streamFile(file, "text/html");
      file.close();
    }
    else
    {
      server.send(404, "text/plain", "Login page not found");
    }
    return;
  }

  // Redirect to dashboard
  server.sendHeader("Location", "/dashboard");
  server.send(302, "text/plain", "");
}

// Update handleAuth - redirect to dashboard on success
void handleAuth()
{
  if (server.arg("username") == default_username &&
      server.arg("password") == default_password)
  {
    isAuthenticated = true;
    // Serial.println("[Auth] Login successful");
    server.sendHeader("Location", "/dashboard");
    server.send(302, "text/plain", "");
  }
  else
  {
    // Serial.println("[Auth] Login failed - Invalid credentials");
    server.sendHeader("Location", "/?error=1");
    server.send(302, "text/plain", "");
  }
}

void handleSaveConfig()
{
  // Check authentication
  if (!isAuthenticated)
  {
    serveErrorPage("Unauthorized access. Please login first.");
    return;
  }

  // ===== VALIDATION =====

  // Validate Hardware ID
  if (!server.hasArg("hardwareId") || server.arg("hardwareId").length() == 0)
  {
    serveErrorPage("Hardware ID cannot be empty.");
    return;
  }

  // Validate Server IP
  if (!server.hasArg("serverIP") || server.arg("serverIP").length() == 0)
  {
    serveErrorPage("Server IP cannot be empty.");
    return;
  }

  // Validate Server Port
  if (!server.hasArg("serverPort") || server.arg("serverPort").toInt() <= 0 || server.arg("serverPort").toInt() > 65535)
  {
    serveErrorPage("Invalid server port number. Must be between 1-65535.");
    return;
  }

  // ===== CHECK FOR CHANGES =====
  bool changed = false;

  Serial.println("[Config] Checking for changes...");

  // Hardware ID
  if (server.hasArg("hardwareId") && server.arg("hardwareId") != config.hardwareId)
  {
    config.hardwareId = server.arg("hardwareId");
    config.hardwareId.trim();
    changed = true;
    Serial.println("[Config] Hardware ID changed");
  }

  // Server IP
  if (server.hasArg("serverIP") && server.arg("serverIP") != config.serverIP)
  {
    config.serverIP = server.arg("serverIP");
    config.serverIP.trim();
    changed = true;
    Serial.println("[Config] Server IP changed");
  }

  // Server Port
  if (server.hasArg("serverPort") && server.arg("serverPort").toInt() != config.monitoringPort)
  {
    config.monitoringPort = server.arg("serverPort").toInt();
    changed = true;
    Serial.println("[Config] Server Port changed");
  }

  // Server Path
  if (server.hasArg("path") && server.arg("path") != config.path)
  {
    config.path = server.arg("path");
    config.path.trim();
    if (config.path.length() == 0)
    {
      config.path = "/wsiot";
    }
    changed = true;
    Serial.println("[Config] Path changed");
  }

  // Communication Mode
  if (server.hasArg("commMode") && server.arg("commMode") != config.commMode)
  {
    String newCommMode = server.arg("commMode");
    newCommMode.trim();
    
    // Validate comm mode based on firmware type
#ifdef FIRMWARE_WIFI_ONLY
    // WiFi firmware: allow ws, mqtt, httppost
    if (newCommMode == "ws" || newCommMode == "mqtt" || newCommMode == "httppost") {
      config.commMode = newCommMode;
      changed = true;
      Serial.println("[Config] Comm Mode changed to: " + newCommMode);
    }
#else
    // Ethernet firmware: only allow mqtt, httppost (no websocket)
    if (newCommMode == "mqtt" || newCommMode == "httppost") {
      config.commMode = newCommMode;
      changed = true;
      Serial.println("[Config] Comm Mode changed to: " + newCommMode);
    } else if (newCommMode == "ws") {
      Serial.println("[Config] WebSocket not supported on Ethernet, defaulting to MQTT");
      config.commMode = "mqtt";
      changed = true;
    }
#endif
  }

  // MQTT Access Token
  if (server.hasArg("accessToken") && server.arg("accessToken") != config.accessTokenMQTT)
  {
    config.accessTokenMQTT = server.arg("accessToken");
    config.accessTokenMQTT.trim();
    changed = true;
    Serial.println("[Config] Access Token changed");
  }

  // WiFi SSID
  if (server.hasArg("ssid") && server.arg("ssid") != config.wifiSSID)
  {
    config.wifiSSID = server.arg("ssid");
    config.wifiSSID.trim();
    changed = true;
    Serial.println("[Config] WiFi SSID changed");
  }

  // WiFi Password
  if (server.hasArg("password") && server.arg("password").length() > 0 &&
      server.arg("password") != config.wifiPass)
  {
    config.wifiPass = server.arg("password");
    config.wifiPass.trim();
    changed = true;
    Serial.println("[Config] WiFi Password changed");
  }

  // NOTE: Network Mode is no longer configurable via web interface
  // It's determined by firmware type (FIRMWARE_WIFI_ONLY or FIRMWARE_ETHERNET_ONLY)

  // ===== SAVE OR NO CHANGES =====

  if (!changed)
  {
    Serial.println("[Config] No changes detected");

    server.send(200, "text/html",
                "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
                "<meta http-equiv='refresh' content='2;url=/config'>"
                "<link href='https://fonts.googleapis.com/css2?family=Inter:wght@400;600&display=swap' rel='stylesheet'>"
                "<link rel='stylesheet' href='/style.css'>"
                "</head><body>"
                "<div class='message-wrapper'>"
                "<div class='message-container'>"
                "<span class='message-icon'>ℹ️</span>"
                "<h1>No Changes Detected</h1>"
                "<p>Configuration is already up to date.<br>Redirecting back...</p>"
                "</div>"
                "</div>"
                "</body></html>");
    return;
  }

  // ===== SAVE CONFIGURATION =====

  bool saveSuccess = saveConfig();

  if (!saveSuccess)
  {
    serveErrorPage("Failed to save configuration. Please try again.");
    return;
  }

  Serial.println("[Config] Configuration saved successfully!");

  // ===== SERVE SUCCESS PAGE =====

  File file = LittleFS.open("/success.html", "r");
  if (file)
  {
    Serial.println("[Config] Serving success page");
    String html = file.readString();
    file.close();

    // Send response
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", html);

    // Ensure response is fully sent
    delay(100);

    for (int i = 5; i > 0; i--)
    {
      Serial.printf("[System] Restarting in %d...\n", i);
      delay(1000);
    }
    delay(100);

    ESP.restart();
  }
  else
  {
    // Fallback jika success.html tidak ditemukan
    server.send(200, "text/html",
                "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
                "<link rel='stylesheet' href='/style.css'>"
                "<script>setTimeout(()=>location.href='/',5000)</script>"
                "</head><body>"
                "<div class='message-wrapper'>"
                "<div class='message-container'>"
                "<span class='message-icon'>✅</span>"
                "<h1>Configuration Saved!</h1>"
                "<p>Device will restart in 5 seconds...</p>"
                "</div>"
                "</div>"
                "</body></html>");

    delay(5100);
    ESP.restart();
  }
}

// =================================================================
// --- Toggle Output Handler ---
// =================================================================
void handleToggle()
{
  if (!isAuthenticated)
  {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  if (!server.hasArg("pin") || !server.hasArg("state"))
  {
    server.send(400, "text/plain", "Missing parameters");
    return;
  }

  int pin = server.arg("pin").toInt();
  String state = server.arg("state");

  if (pin < 0 || pin >= NUM_OUTPUTS)
  {
    server.send(400, "text/plain", "Invalid pin number");
    return;
  }

  if (state != "on" && state != "off")
  {
    server.send(400, "text/plain", "Invalid state value");
    return;
  }

  digitalWrite(OUTPUT_PINS[pin], state == "on" ? HIGH : LOW);
  saveIOStateToEEPROM();

  Serial.printf("[Control] Output O%d set to %s\n", pin + 1, state.c_str());
  server.send(200, "text/plain", "OK");
}

// =================================================================
// --- Get Outputs Handler ---
// =================================================================
void handleGetOutputs()
{
  if (!isAuthenticated)
  {
    server.send(401, "application/json", "{\"error\":\"Unauthorized\"}");
    return;
  }

  StaticJsonDocument<200> doc;
  JsonArray outputs = doc.createNestedArray("outputs");

  for (int i = 0; i < NUM_OUTPUTS; i++)
  {
    outputs.add(digitalRead(OUTPUT_PINS[i]));
  }

  String jsonString;
  serializeJson(doc, jsonString);
  server.send(200, "application/json", jsonString);
}

// =================================================================
// --- Download Config Handler ---
// =================================================================
void handleDownloadConfig()
{
  if (!isAuthenticated)
  {
    serveErrorPage("Unauthorized access. Please login first.");
    return;
  }

  StaticJsonDocument<512> doc;
  doc["hardwareId"] = config.hardwareId;
  doc["serverIP"] = config.serverIP;
  doc["monitoringPort"] = config.monitoringPort;
  doc["path"] = config.path;
  doc["commMode"] = config.commMode;
  doc["networkMode"] = config.networkMode;
  doc["wifiSSID"] = config.wifiSSID;
  doc["wifiPass"] = config.wifiPass;
  doc["accessTokenMQTT"] = config.accessTokenMQTT;

  String jsonString;
  serializeJsonPretty(doc, jsonString);

  if (jsonString.length() == 0)
  {
    serveErrorPage("Failed to generate configuration file.");
    return;
  }

  String filename = "config_" + config.hardwareId + ".json";
  server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
  server.send(200, "application/json", jsonString);

  Serial.println("[Config] Configuration downloaded: " + filename);
}

// =================================================================
// --- Logout Handler ---
// =================================================================
void handleLogout()
{
  isAuthenticated = false;
  Serial.println("[Auth] User logged out");
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}

// =================================================================
// --- API Handlers untuk Kontrol Output ---
// =================================================================

/**
 * POST /api/control
 * Menerima perintah kontrol output via JSON
 * Body: {"O1": 1, "O2": 0, "O3": 1, "O4": 0}
 * atau: {"output1": 1, "output2": 0}
 */
void handleApiControl()
{
  Serial.println("[API] POST /api/control");

  // Cek content type
  if (server.hasHeader("Content-Type"))
  {
    String contentType = server.header("Content-Type");
    if (contentType.indexOf("application/json") < 0)
    {
      Serial.println("[API] Warning: Content-Type bukan application/json");
    }
  }

  // Parse JSON body
  String body = server.arg("plain");
  Serial.println("[API] Body: " + body);

  if (body.length() == 0)
  {
    server.send(400, "application/json", "{\"error\":\"Empty request body\"}");
    return;
  }

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, body);

  if (error)
  {
    Serial.println("[API] JSON parse error: " + String(error.c_str()));
    server.send(400, "application/json", "{\"error\":\"Invalid JSON: " + String(error.c_str()) + "\"}");
    return;
  }

  // Process output commands
  bool updated = false;
  int updatedCount = 0;

  // Format 1: O1, O2, O3, O4
  for (int i = 0; i < NUM_OUTPUTS; i++)
  {
    String key = "O" + String(i + 1);

    if (doc.containsKey(key))
    {
      int state = doc[key].as<int>();
      state = constrain(state, 0, 1); // Pastikan hanya 0 atau 1
      digitalWrite(OUTPUT_PINS[i], state == 1 ? HIGH : LOW);
      Serial.printf("[API] Output O%d = %d\n", i + 1, state);
      updated = true;
      updatedCount++;
    }
  }

  // Format 2: output1, output2, output3, output4
  for (int i = 0; i < NUM_OUTPUTS; i++)
  {
    String key = "output" + String(i + 1);

    if (doc.containsKey(key))
    {
      int state = doc[key].as<int>();
      state = constrain(state, 0, 1);
      digitalWrite(OUTPUT_PINS[i], state == 1 ? HIGH : LOW);
      Serial.printf("[API] Output %d = %d\n", i + 1, state);
      updated = true;
      updatedCount++;
    }
  }

  // Format 3: Q1, Q2, Q3, Q4 (kompatibel dengan format monitoring)
  for (int i = 0; i < NUM_OUTPUTS; i++)
  {
    String key = "Q" + String(i + 1);

    if (doc.containsKey(key))
    {
      int state = doc[key].as<int>();
      state = constrain(state, 0, 1);
      digitalWrite(OUTPUT_PINS[i], state == 1 ? HIGH : LOW);
      Serial.printf("[API] Output Q%d = %d\n", i + 1, state);
      updated = true;
      updatedCount++;
    }
  }

  if (updated)
  {
    // Simpan state ke EEPROM
    saveIOStateToEEPROM();

    // Build response dengan current states
    StaticJsonDocument<256> response;
    response["status"] = "ok";
    response["message"] = "Outputs updated";
    response["updated_count"] = updatedCount;

    JsonObject outputs = response.createNestedObject("outputs");
    for (int i = 0; i < NUM_OUTPUTS; i++)
    {
      String key = "O" + String(i + 1);
      outputs[key] = digitalRead(OUTPUT_PINS[i]);
    }

    String jsonResponse;
    serializeJson(response, jsonResponse);
    Serial.println("[API] Response: " + jsonResponse);
    server.send(200, "application/json", jsonResponse);
  }
  else
  {
    server.send(400, "application/json", "{\"error\":\"No valid output commands found. Use O1-O4, Q1-Q4, or output1-output4\"}");
  }
}

void handleApiStatus()
{
  Serial.println("[API] GET /api/status");

  StaticJsonDocument<768> doc;

  // Device info
  doc["hardwareId"] = config.hardwareId;
  doc["firmware"] = program_version;
  doc["uptime"] = millis() / 1000;
  doc["freeHeap"] = ESP.getFreeHeap();

  // Connection info
#ifdef FIRMWARE_WIFI_ONLY
  doc["connectionType"] = "WiFi";
  doc["connected"] = network_connected;
  if (network_connected)
  {
    doc["ip"] = WiFi.localIP().toString();
    doc["mac"] = WiFi.macAddress();
    doc["rssi"] = WiFi.RSSI();
    doc["ssid"] = config.wifiSSID;
  }
#else
  doc["connectionType"] = "Ethernet";
  doc["connected"] = network_connected;
  if (network_connected)
  {
    doc["ip"] = Ethernet.localIP().toString();
  }
#endif

  // Communication info
  doc["commMode"] = config.commMode;
  doc["serverIP"] = config.serverIP;
  doc["serverPort"] = config.monitoringPort;

  // Inputs (I1-I16)
  JsonObject inputs = doc.createNestedObject("inputs");
  for (int i = 0; i < NUM_INPUTS; i++)
  {
    String key = "I" + String(i + 1);
    // Inverted logic (1 = tidak aktif, 0 = aktif)
    inputs[key] = (lastInputStates[i] == 1) ? 0 : 1;
  }

  // Outputs (O1-O4)
  JsonObject outputs = doc.createNestedObject("outputs");
  for (int i = 0; i < NUM_OUTPUTS; i++)
  {
    String key = "O" + String(i + 1);
    outputs[key] = digitalRead(OUTPUT_PINS[i]);
  }

  String jsonResponse;
  serializeJson(doc, jsonResponse);
  Serial.println("[API] Status response sent");
  server.send(200, "application/json", jsonResponse);
}

void handleApiSet()
{
  Serial.println("[API] GET /api/set");

  // Validasi parameter
  if (!server.hasArg("output") || !server.hasArg("state"))
  {
    server.send(400, "application/json", 
      "{\"error\":\"Missing parameters\",\"usage\":\"/api/set?output=1&state=1\"}");
    return;
  }

  int output = server.arg("output").toInt();
  int state = server.arg("state").toInt();

  // Validasi output number
  if (output < 1 || output > NUM_OUTPUTS)
  {
    String error = "{\"error\":\"Invalid output number. Must be 1-" + String(NUM_OUTPUTS) + "\"}";
    server.send(400, "application/json", error);
    return;
  }

  // Validasi state
  if (state != 0 && state != 1)
  {
    server.send(400, "application/json", "{\"error\":\"Invalid state. Must be 0 or 1\"}");
    return;
  }

  // Set output
  digitalWrite(OUTPUT_PINS[output - 1], state == 1 ? HIGH : LOW);
  saveIOStateToEEPROM();

  Serial.printf("[API] Output O%d set to %d via GET\n", output, state);

  // Response
  StaticJsonDocument<128> response;
  response["status"] = "ok";
  response["output"] = output;
  response["state"] = state;
  response["pin"] = OUTPUT_PINS[output - 1];

  String jsonResponse;
  serializeJson(response, jsonResponse);
  server.send(200, "application/json", jsonResponse);
}

void handleApiOutputs()
{
  Serial.println("[API] GET /api/outputs");

  StaticJsonDocument<128> doc;

  for (int i = 0; i < NUM_OUTPUTS; i++)
  {
    String key = "O" + String(i + 1);
    doc[key] = digitalRead(OUTPUT_PINS[i]);
  }

  String jsonResponse;
  serializeJson(doc, jsonResponse);
  server.send(200, "application/json", jsonResponse);
}

void handleApiInputs()
{
  Serial.println("[API] GET /api/inputs");

  StaticJsonDocument<256> doc;

  for (int i = 0; i < NUM_INPUTS; i++)
  {
    String key = "I" + String(i + 1);
    // Inverted logic
    doc[key] = (lastInputStates[i] == 1) ? 0 : 1;
  }

  String jsonResponse;
  serializeJson(doc, jsonResponse);
  server.send(200, "application/json", jsonResponse);
}

void handleApiToggle()
{
  Serial.println("[API] POST /api/toggle");

  if (!server.hasArg("output"))
  {
    server.send(400, "application/json", 
      "{\"error\":\"Missing output parameter\",\"usage\":\"/api/toggle?output=1\"}");
    return;
  }

  int output = server.arg("output").toInt();

  if (output < 1 || output > NUM_OUTPUTS)
  {
    String error = "{\"error\":\"Invalid output number. Must be 1-" + String(NUM_OUTPUTS) + "\"}";
    server.send(400, "application/json", error);
    return;
  }

  // Toggle: baca state saat ini, lalu flip
  int currentState = digitalRead(OUTPUT_PINS[output - 1]);
  int newState = (currentState == HIGH) ? LOW : HIGH;
  digitalWrite(OUTPUT_PINS[output - 1], newState);
  saveIOStateToEEPROM();

  Serial.printf("[API] Output O%d toggled: %d -> %d\n", output, currentState, newState);

  StaticJsonDocument<128> response;
  response["status"] = "ok";
  response["output"] = output;
  response["previous_state"] = currentState;
  response["new_state"] = newState;

  String jsonResponse;
  serializeJson(response, jsonResponse);
  server.send(200, "application/json", jsonResponse);
}

void setupApiRoutes()
{
  Serial.println("[API] Setting up API routes...");

  // Control endpoints
  server.on("/api/control", HTTP_POST, handleApiControl);
  server.on("/api/set", HTTP_GET, handleApiSet);
  server.on("/api/toggle", HTTP_POST, handleApiToggle);

  // Status endpoints
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/outputs", HTTP_GET, handleApiOutputs);
  server.on("/api/inputs", HTTP_GET, handleApiInputs);

  // CORS preflight handler (untuk akses dari browser/frontend)
  server.on("/api/control", HTTP_OPTIONS, []() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(204);
  });

  Serial.println("[API] API routes configured:");
  Serial.println("  POST /api/control  - Control outputs via JSON");
  Serial.println("  GET  /api/set      - Set single output");
  Serial.println("  POST /api/toggle   - Toggle single output");
  Serial.println("  GET  /api/status   - Get full device status");
  Serial.println("  GET  /api/outputs  - Get outputs only");
  Serial.println("  GET  /api/inputs   - Get inputs only");
}

// =================================================================
// --- Error Page Handler ---
// =================================================================
void serveErrorPage(String errorMessage)
{
  Serial.println("[Error] " + errorMessage);

  File file = LittleFS.open("/error.html", "r");
  if (file)
  {
    String html = file.readString();
    file.close();

    // Replace placeholder dengan pesan error
    html.replace("%ERROR_MESSAGE%", errorMessage);

    server.send(400, "text/html", html);
  }
  else
  {
    // Fallback jika error.html tidak ditemukan
    server.send(400, "text/html",
                "<!DOCTYPE html><html><head><meta charset='UTF-8'>"
                "<style>"
                "body{font-family:'Inter',sans-serif;background:linear-gradient(135deg,#667eea,#764ba2);"
                "min-height:100vh;display:flex;align-items:center;justify-content:center;padding:20px;}"
                ".box{background:white;padding:40px;border-radius:20px;text-align:center;max-width:400px;box-shadow:0 20px 60px rgba(0,0,0,0.3);}"
                "h2{color:#ef4444;margin-bottom:15px;font-size:24px;}"
                "p{color:#64748b;line-height:1.6;}"
                "button{margin-top:20px;padding:12px 24px;background:#667eea;color:white;border:none;border-radius:10px;font-weight:600;cursor:pointer;}"
                "</style>"
                "</head><body>"
                "<div class='box'>"
                "<div style='font-size:60px;margin-bottom:20px;'>⚠️</div>"
                "<h2>Error Occurred</h2>"
                "<p>" +
                    errorMessage + "</p>"
                                   "<button onclick='history.back()'>← Go Back</button>"
                                   "</div>"
                                   "</body></html>");
  }
}

// =================================================================
// --- OTA ---
// =================================================================
void setupOTA()
{
#ifdef FIRMWARE_WIFI_ONLY
  if (!network_connected)
  {
    Serial.println("[OTA] Skipped - WiFi not connected");
    return;
  }

  ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname(config.hardwareId.c_str());
  ArduinoOTA.setPassword(default_password);
  ArduinoOTA.begin();
  Serial.println("[OTA] Service started (WiFi mode)");
#else
  Serial.println("[OTA] Not available in Ethernet firmware");
#endif
}

// =================================================================
// --- NETWORK SETUP (WiFi/Ethernet/Hybrid + AP Fallback) ---
// =================================================================
void setupNetwork()
{
  isAPMode = false;
  network_connected = false;
  network_ready = false;

  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.printf("║ NETWORK INITIALIZATION - %s Mode%*s║\n",
                NETWORK_MODE_NAME,
                24 - strlen(NETWORK_MODE_NAME), "");
  Serial.println("╠════════════════════════════════════════════════════╣");

#ifdef FIRMWARE_WIFI_ONLY
  Serial.printf("║ Timeout: %d seconds%*s║\n",
                CONNECT_TIMEOUT / 1000, 31, "");
  Serial.println("╚════════════════════════════════════════════════════╝");

  if (config.wifiSSID.length() == 0)
  {
    Serial.println("[WiFi] ✗ SSID not configured!");
    Serial.println("[WiFi] Starting AP Mode...");
    startAPMode();
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);

  Serial.printf("[WiFi] Connecting to: %s\n", config.wifiSSID.c_str());
  WiFi.begin(config.wifiSSID.c_str(), config.wifiPass.c_str());

  unsigned long startTime = millis();
  int dots = 0;

  while (WiFi.status() != WL_CONNECTED &&
         (millis() - startTime < connectTimeout))
  {
    delay(500);
    Serial.print(".");
    dots++;
    if (dots % 40 == 0)
      Serial.println();
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("[WiFi] Connected!");
    Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("[WiFi] MAC: %s\n", WiFi.macAddress().c_str());
    Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
    network_connected = true;
    network_ready = true;
    setupCommunication();
  }
  else
  {
    Serial.println("[WiFi] Connection FAILED!");
    Serial.println("[WiFi] Starting AP Mode...");
    startAPMode();
  }

#else // FIRMWARE_ETHERNET_ONLY
  Serial.printf("║ Timeout: %d seconds%*s║\n",
                CONNECT_TIMEOUT / 1000, 31, "");
  Serial.println("╚════════════════════════════════════════════════════╝");

  WiFi.mode(WIFI_OFF);

  if (initEthernet())
  {
    unsigned long startTime = millis();
    Serial.print("[Ethernet] Waiting for link");

    while (Ethernet.linkStatus() == LinkOFF &&
           (millis() - startTime < connectTimeout))
    {
      delay(500);
      Serial.print(".");
    }
    Serial.println();

    if (Ethernet.linkStatus() == LinkON)
    {
      Serial.println("[Ethernet] Connected!");
      Serial.printf("[Ethernet] IP: %s\n", Ethernet.localIP().toString().c_str());
      Serial.printf("[Ethernet] Gateway: %s\n", Ethernet.gatewayIP().toString().c_str());
      Serial.printf("[Ethernet] Subnet: %s\n", Ethernet.subnetMask().toString().c_str());
      network_connected = true;
      network_ready = true;
      setupCommunication();
    }
    else
    {
      Serial.println("[Ethernet] No cable detected!");
      Serial.println("[Ethernet] Starting AP Mode...");
      startAPMode();
    }
  }
  else
  {
    Serial.println("[Ethernet] Initialization FAILED!");
    Serial.println("[Ethernet] Starting AP Mode...");
    startAPMode();
  }
#endif
}

void setupCommunication()
{
  if (isAPMode || readyToSend)
    return;

  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.printf("║ COMMUNICATION SETUP - %s Mode%*s║\n",
                config.commMode.c_str(),
                21 - config.commMode.length(), "");
  Serial.println("╚════════════════════════════════════════════════════╝");

  if (config.commMode == "ws")
  {
#ifndef FIRMWARE_WIFI_ONLY
    wsConnect();
    readyToSend = true;
#endif
  }
  else if (config.commMode == "mqtt")
  {
    mqttClient.setClient(ACTIVE_CLIENT); // ← Macro yang resolve ke netClient
    mqttClient.setServer(config.serverIP.c_str(), config.monitoringPort);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(8192);
    mqttClient.setKeepAlive(60);
    mqttClient.setSocketTimeout(15);

    Serial.printf("[MQTT] Initialized via %s\n", NETWORK_MODE_NAME); // ← Macro
  }
  else if (config.commMode == "httppost")
  {
    String statusData = buildStatusJSON();
    String resp;

    if (sendViaHTTP(statusData, resp))
    {
      Serial.println("[HTTP] Initial status sent");
      if (resp.length())
        Serial.println("Response: " + resp);
    }
    else
    {
      Serial.println("[HTTP] Failed to send initial status");
    }

    readyToSend = true;
  }
}

void startAPMode()
{
  isAPMode = true;
  network_connected = false;
  network_ready = false;

  String finalApSsid = ap_ssid + config.hardwareId;

  Serial.println("\n╔════════════════════════════════════════════════════╗");
  Serial.println("║              AP MODE ACTIVATED                     ║");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.printf("║ SSID:     %-40s ║\n", finalApSsid.c_str());
  Serial.printf("║ Password: %-40s ║\n", ap_password);
  Serial.printf("║ Timeout:  %d seconds%*s║\n",
                AP_TIMEOUT / 1000, 29, "");
  Serial.println("╠════════════════════════════════════════════════════╣");
  Serial.println("║ 1. Connect to WiFi AP above                        ║");
  Serial.println("║ 2. Open browser: http://192.168.4.1                ║");
  Serial.println("║ 3. Configure network settings                      ║");
  Serial.println("║ 4. Device will restart automatically               ║");
  Serial.println("╚════════════════════════════════════════════════════╝\n");

  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(finalApSsid.c_str(), ap_password);

  apStartTime = millis();

  Serial.printf("[AP] IP Address: %s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("[AP] Waiting for configuration...\n");

  if (ws_connected)
    wsDisconnect();
  if (mqttClient.connected())
    mqttClient.disconnect();
  readyToSend = false;
}

void tryReconnect()
{
#ifdef FIRMWARE_WIFI_ONLY
  if (config.wifiSSID.length() > 0)
  {
    Serial.println("[Reconnect] Attempting WiFi...");
    WiFi.disconnect(true);
    delay(300);
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.wifiSSID.c_str(), config.wifiPass.c_str());
  }
  else
  {
    startAPMode();
    return;
  }
#else // FIRMWARE_ETHERNET_ONLY
  Serial.println("[Reconnect] Re-initializing Ethernet...");
  if (initEthernet() && Ethernet.linkStatus() == LinkON)
  {
    network_connected = true;
  }
#endif

  isReconnecting = true;
  reconnectAttemptStartTime = millis();
}

// =================================================================
// --- SETUP ---
// =================================================================
void setupIO()
{
  if (pcf8574.begin())
    Serial.println("PCF8574: Connected.");
  else
    Serial.println("PCF8574: Failed!");
  pcf8574.write8(0xFF);
  for (int pin : INPUT_GPIO_PINS){
    if (pin == 34)
    {
      pinMode(pin, INPUT);
    }else{
      pinMode(pin, INPUT_PULLUP);
    }
  }
  for (int pin : OUTPUT_PINS)
  {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

#ifdef FIRMWARE_ETHERNET_ONLY
  pinMode(LAN_LED_PIN, OUTPUT);
  digitalWrite(LAN_LED_PIN, LOW);
#endif
  loadIOStateFromEEPROM();
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Serial.println("\n\n==> Starting IoT Node " + program_version + " <==");
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("[FATAL] EEPROM init fail");
    delay(1000);
    ESP.restart();
  }
  if (!LittleFS.begin())
  {
    Serial.println("[FATAL] LittleFS mount failed.");
    while (1)
    {
      delay(1000);
    }
  }

  loadConfig();

  Serial.println("box_id: " + config.hardwareId);
  Serial.println("serverIP: " + config.serverIP);
  Serial.println("path: " + config.path);
  Serial.println("monitoringPort: " + String(config.monitoringPort));
  Serial.println("commMode: " + config.commMode);
  Serial.println("wifiSSID: " + config.wifiSSID);

  setupIO();
  initShortDetection();
  syncInputStates();
  setupNetwork();

  if (!isAPMode)
  {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

#ifdef FIRMWARE_WIFI_ONLY
    if (network_connected)
    {
      setupOTA();
    }
#endif
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/auth", HTTP_POST, handleAuth);
  server.on("/dashboard", HTTP_GET, handleDashboard);
  server.on("/config", HTTP_GET, handleConfigPage);
  server.on("/control", HTTP_GET, handleControlPage);
  server.on("/status", HTTP_GET, handleStatusPage);
  server.on("/save-config", HTTP_POST, handleSaveConfig);
  server.on("/toggle", HTTP_POST, handleToggle);
  server.on("/get-outputs", HTTP_GET, handleGetOutputs);
  server.on("/download-config", HTTP_GET, handleDownloadConfig);
  server.on("/logout", HTTP_GET, handleLogout);

  server.on("/style.css", HTTP_GET, []() {
    File file = LittleFS.open("/style.css", "r");
    if (file) {
      server.streamFile(file, "text/css");
      file.close();
    } else {
      server.send(404, "text/plain", "CSS file not found");
    }
  });

  server.on("/script.js", HTTP_GET, []() {
    File file = LittleFS.open("/script.js", "r");
    if (file) {
      server.streamFile(file, "application/javascript");
      file.close();
    } else {
      server.send(404, "text/plain", "JS file not found");
    }
  });

  setupApiRoutes();

  // 404 handler
  server.onNotFound([]()
                    {
  Serial.println("[Web] 404 - Page not found: " + server.uri());
  serveErrorPage("Page not found: " + server.uri()); });
                      
  server.begin();
}

// =================================================================
// --- LOOP ---
// =================================================================
void loop()
{
  server.handleClient();
  checkShortInputs();


  if (isAPMode)
  {
    unsigned long now = millis();
    unsigned long elapsed = now - apStartTime;

    if (apStartTime > 0 && now - lastCountdownPrint >= 1000)
    {
      lastCountdownPrint = now;
      unsigned long remaining = (apTimeout > elapsed) ? (apTimeout - elapsed) / 1000 : 0;
      Serial.printf("[AP MODE] Active. Restart in %lu seconds...\n", remaining);
    }

    if (elapsed >= apTimeout)
    {
      Serial.println("[AP MODE] Timeout! Restarting...");
      delay(200);
      ESP.restart();
    }
    return;
  }

#ifdef FIRMWARE_WIFI_ONLY
  bool isConnected = (WiFi.status() == WL_CONNECTED);
#else
  Ethernet.maintain();
  bool isConnected = (Ethernet.linkStatus() == LinkON);
#ifdef FIRMWARE_ETHERNET_ONLY
  digitalWrite(LAN_LED_PIN, isConnected ? HIGH : LOW);
#endif

#endif

  if (isConnected)
  {
    if (!network_connected)
    {
      network_connected = true;
      network_ready = true;
      Serial.printf("[%s] Reconnected!\n", NETWORK_MODE_NAME);
      
      if (hasOfflineData && shortBuffer.size() > 0)
      {
        Serial.println("[Short] Connection restored, will send buffered data...");
        delay(1000);
      }

    }

    if (isReconnecting)
    {
      isReconnecting = false;
      reconnectAttemptCount = 0;
      reconnectAttemptStartTime = 0;
    }

    if (!readyToSend)
    {
      setupCommunication();
    }

#ifdef FIRMWARE_WIFI_ONLY
    ArduinoOTA.handle();
#endif
    if (config.commMode == "ws")
    {
      wsPump();
      if (ws_connected && hasOfflineData && shortBuffer.size() > 0)
      {
        unsigned long now = millis();
        if (now - lastBufferedSendAttempt >= bufferedSendInterval)
        {
          lastBufferedSendAttempt = now;
          sendBufferedShorts();
        }
      }
      
    }
    else if (config.commMode == "mqtt")
    {
      if (!mqttClient.connected())
      {
        reconnectMQTT();
      }
      else
      {
        mqttClient.loop();

        if (hasOfflineData && shortBuffer.size() > 0)
        {
          unsigned long now = millis();
          if (now - lastBufferedSendAttempt >= bufferedSendInterval)
          {
            lastBufferedSendAttempt = now;
            sendBufferedShorts();
          }
        }
      
      }
    }
    else if (config.commMode == "httppost")
    {
      pollHTTPCommands();

      if (hasOfflineData && shortBuffer.size() > 0)
      {
        unsigned long now = millis();
        if (now - lastBufferedSendAttempt >= bufferedSendInterval)
        {
          lastBufferedSendAttempt = now;
          sendBufferedShorts();
        }
      }
      
    }
    unsigned long currentTime = millis();
    if (currentTime - lastForceSendTime >= forceSendInterval)
    {
      lastForceSendTime = currentTime;
      sendMonitoringData(); 
    }
  }
  else
  {
    // Disconnected handling
    if (network_connected)
    {
      network_connected = false;
      network_ready = false;
      readyToSend = false;
      Serial.printf("[%s] Disconnected!\n", NETWORK_MODE_NAME);

      if (ws_connected) wsDisconnect();
      if (mqttClient.connected()) mqttClient.disconnect();

      isReconnecting = false;
      reconnectAttemptCount = 0;
      reconnectAttemptStartTime = 0;
    }


    if (!isReconnecting)
    {
      if (reconnectAttemptCount >= maxReconnectAttempts)
      {
        Serial.println("\n[Reconnect] Max attempts reached. Starting AP Mode...");
        startAPMode();
        return;
      }

      if (reconnectAttemptCount > 0) delay(2000);
      tryReconnect();
      reconnectAttemptCount++;
    }
    else
    {
      unsigned long attemptElapsed = millis() - reconnectAttemptStartTime;
      if (attemptElapsed >= singleAttemptTimeout)
      {
        Serial.println("\n[Reconnect] Attempt timeout");
        isReconnecting = false;
      }
      else
      {
        static unsigned long lastDot = 0;
        if (millis() - lastDot >= 500)
        {
          Serial.print(".");
          lastDot = millis();
        }
        delay(100);
      }
    }
  }
}