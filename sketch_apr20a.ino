#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>  // Include WiFiManager library
#include <Ticker.h>
#include <HardwareSerial.h>
#include <esp_wifi.h>  // For ESPTouch functionality
#include <esp_system.h> // For esp_restart()
#include <esp_task_wdt.h>  // ESP32 Watchdog Timer library
#include <Update.h>  // For OTA updates
#include <WiFiClientSecure.h>  // For secure HTTP connections
#include "mbedtls/sha256.h"

const String FIRMWARE_VERSION = "1.0.35";  // Current firmware version
const String globalUrl = "139.162.60.209";
const int globalPort = 2579;
// const String globalUrl = "170.187.226.48";
// const int globalPort = 4060;
// A7670C Configuration - Add these new variables
const bool SKIP_WIFI = true;  // Set to true to use A7670C instead of WiFi
const bool SKIP_RS232 = true;  // Set to true to completely disable RS232 emulator functionality
// Add: toggle for operator selection mode (auto vs manual ACTIVE_MCC_MNC)
const bool A7670C_AUTO_OPERATOR_SELECT = true;  // true = AT+COPS=0 (auto), false = manual using ACTIVE_MCC_MNC
// Add: slow-mode knobs for stripped-down modules
const bool A7670C_SLOW_MODE = false;  // If true, use longer timeouts and waits for AT commands
const unsigned long A7670C_DEFAULT_CMD_TIMEOUT = 15000;  // 15s default for general AT commands
const unsigned long A7670C_LONG_CMD_TIMEOUT = 120000;     // 120s for long-running ops like AT+COPS
HardwareSerial a7670cSerial(1);  // Use Serial1 for A7670C (Serial2 is used for bill acceptor)
const int A7670C_RX_PIN = 25;    // A7670C RX pin (changed from 16 to 25)
const int A7670C_TX_PIN = 26;    // A7670C TX pin (changed from 17 to 26)
const int A7670C_CTS_PIN = 27;   // ESP32 CTS input (connect to A7670C RTS)
const int A7670C_RTS_PIN = 4;    // ESP32 RTS output (connect to A7670C CTS)
const int A7670C_BAUD_RATE = 115200;

// Celcom Network Configuration
const String CELCOM_MCC_MNC = "50219";
const String CELCOM_APN = "celcom3g";
const String CELCOM_USERNAME = "";  // Usually empty for Celcom
const String CELCOM_PASSWORD = "";  // Usually empty for Celcom

// Digi Network Configuration  
const String DIGI_MCC_MNC = "50216";
const String DIGI_APN = "diginet";
const String DIGI_USERNAME = "";  // Usually empty for Digi
const String DIGI_PASSWORD = "";  // Usually empty for Digi

// Active Network Configuration - Change these to switch networks
const String ACTIVE_MCC_MNC = CELCOM_MCC_MNC;  // Change to DIGI_MCC_MNC |  CELCOM_MCC_MNC for Celcom
const String ACTIVE_APN = CELCOM_APN;          // Change to DIGI_APN| CELCOM_APN for Celcom
const String ACTIVE_USERNAME = DIGI_USERNAME;
const String ACTIVE_PASSWORD = DIGI_PASSWORD;

// A7670C Status variables
bool a7670cConnected = false;
bool networkSelected = false;
bool internetConnected = false;
String publicIP = "";
unsigned long lastA7670CCheck = 0;
const unsigned long A7670C_CHECK_INTERVAL = 30000;  // Check every 30 seconds

// Initialize UART2 with RX on GPIO25 and TX on GPIO26
HardwareSerial billAcceptorSerial(2);

// Watchdog and Reset Configuration
const unsigned long RESET_INTERVAL = 720 * 60 * 1000;  // 12hrs = 720mins,  30 minutes in milliseconds
const int WDT_TIMEOUT = 300;  // 5 minute watchdog timeout to handle HTTP data reading and internet reconnection
unsigned long lastResetTime = 0;
unsigned long lastWatchdogFeed = 0;
const unsigned long WATCHDOG_FEED_INTERVAL = 5000;  // Feed watchdog every 5 seconds

// WebSocket Configuration
const unsigned long PING_TIMEOUT = 120000;  // Consider connection stale after 2 minutes without ping response (increased for cellular)
const unsigned long INITIAL_PING_TIMEOUT = 180000;  // More lenient timeout during initial connection (3 minutes)
const unsigned long INITIAL_CONNECTION_PERIOD = 300000;  // 5 minutes initial connection period (increased)
unsigned long connectionStartTime = 0;  // Track when the connection was established

// LED Configuration
#define LED_BUILTIN 2  // Most ESP32 boards have LED on GPIO2, change if different
const int BLINK_INTERVAL = 500;  // Blink interval in milliseconds
Ticker ledTicker;  // Ticker for LED blinking
bool ledState = false;

// Define the BOOT button pin (built-in button on ESP32)
#define BOOT_BUTTON 0

// Define a variable to track the button state
bool buttonState = HIGH;

// Add WebSocket reconnection tracking
unsigned long lastWebSocketReconnect = 0;
const unsigned long WEBSOCKET_RECONNECT_INTERVAL = 5000;  // 5 seconds between WebSocket reconnection attempts
bool webSocketConnected = false;

// Bill Acceptor Constants
const uint8_t POLL_CMD = 0x30;   // Poll command from dispenser
const uint8_t STX_CMD = 0x02;    // STX command
const uint8_t FF_CMD = 0x0C;     // Form Feed command
const uint8_t ACK_RSP = 0x02;    // ACK response
const uint8_t STATUS_OK = 0x3E;  // Status OK response

// Bill acceptance commands
const uint8_t BILL_1 = 0x40;
const uint8_t BILL_2 = 0x41;
const uint8_t BILL_5 = 0x42;
const uint8_t BILL_10 = 0x43;

// Bill Acceptor state variables
bool emulatorMode = false;  // false = command mode, true = emulator mode
bool debugEnabled = true;
unsigned long lastBillPollTime = 0;
unsigned long lastBillStatusTime = 0;
const unsigned long STATUS_INTERVAL = 100;
bool billEventPending = false;
uint8_t pendingBillCmd = 0;

WebSocketsClient webSocket;
Ticker timer1ms;  // Create a ticker object
#define INPUT_PIN 14
WiFiManager wifiManager;
bool wasDisconnected = false;
// PWM settings

int roomRef = 1;
unsigned long lastPingTime = 0;
unsigned long lastReceivedPingTime = 0;
const long pingInterval = 5000;  // 5 seconds interval for ping
int ping_counter = 0;
bool startedWebsocket = false;
String macToUUID() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  String macHex = String(mac[0], HEX) + String(mac[1], HEX) + String(mac[2], HEX) + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);
  return "00000000-0000-0000-" + macHex.substring(0, 4) + "-" + macHex.substring(4);
}
const int pwmChannel = 0;
const int frequency = 50;   // 50 Hz for motor control
const int resolution = 16;  // 16-bit resolution

// Internet connectivity tracking variables
bool hasInternetConnectivity = false;
unsigned long lastInternetCheck = 0;
const unsigned long INTERNET_CHECK_INTERVAL = 15000;  // Check internet every 15 seconds
unsigned long lastSuccessfulPingResponse = 0;
const unsigned long INTERNET_TIMEOUT = 120000;  // Consider internet down after 2 minutes without response

// WiFi credential storage - Add these global variables
String storedWiFiSSID = "";
String storedWiFiPassword = "";
bool hasStoredCredentials = false;

// Network Selection Mode
const bool USE_AUTOMATIC_NETWORK_SELECTION = true;  // Set to false to use manual network selection

// OTA Configuration

const String OTA_CHECK_ENDPOINT = "/firmware/check/";  // Firmware version check endpoint
const String OTA_DOWNLOAD_ENDPOINT = "/firmware/";  // Firmware download endpoint
bool otaInProgress = false;
bool otaUpdateAvailable = false;
String availableFirmwareVersion = "";
String otaDownloadUrl = "";
unsigned long lastOTACheck = 0;
const unsigned long OTA_CHECK_INTERVAL = 24 * 60 * 60 * 1000;  // Check once per day
int otaProgress = 0;
String otaStatus = "idle";  // idle, checking, downloading, installing, complete, error

// WiFi OTA robustness settings
const uint32_t OTA_WIFI_CONNECT_TIMEOUT_MS = 30000;   // 30s connect timeout
const uint32_t OTA_WIFI_READ_TIMEOUT_MS    = 180000;  // 180s read timeout
const uint32_t OTA_WIFI_NO_DATA_TIMEOUT_MS = 60000;   // 60s with no data -> abort
const size_t   OTA_WIFI_CHUNK_SIZE         = 4096;    // 4KB read buffer

// OTA Watchdog Configuration
const int OTA_WDT_TIMEOUT = 60;  // Increased to 60 seconds for WiFi OTA tolerance
unsigned long otaLastActivityTime = 0;  // Track last OTA activity for watchdog
bool otaWatchdogActive = false;  // Flag to enable OTA-specific watchdog

// Simple UART initialization without hardware flow control
static void initializeA7670CSerial() {
  a7670cSerial.end();
  delay(50);
  a7670cSerial.begin(A7670C_BAUD_RATE, SERIAL_8N1, A7670C_RX_PIN, A7670C_TX_PIN);
  a7670cSerial.setRxBufferSize(2048);
  a7670cSerial.setTimeout(2000);
}

void simulateA7670CWebSocketResponse() {
  // Since we're simulating the WebSocket connection for A7670C,
  // we need to simulate receiving ping responses to keep the connection alive
  if (SKIP_WIFI && webSocketConnected) {
    // Simulate receiving ping response every 10 seconds
    static unsigned long lastSimulatedPing = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastSimulatedPing >= 10000) {  // Every 10 seconds
      lastReceivedPingTime = currentTime;  // Update the ping response time
      lastSimulatedPing = currentTime;
      
      if (debugEnabled) {
        Serial.println("🔄 Simulated A7670C ping response received");
      }
    }
  }
}



// Add A7670C functions before setup()
String sendA7670CCommand(String command, unsigned long timeout = 10000) {
  String response = "";
  
  if (debugEnabled) {
    Serial.println("A7670C → " + command);
  }
  
  // Clear any pending data
  while (a7670cSerial.available()) {
    a7670cSerial.read();
  }
  
  // Send command
  a7670cSerial.println(command);
  
  // Wait for response
  unsigned long startTime = millis();
  bool responseReceived = false;
  
  while (millis() - startTime < timeout) {
    if (a7670cSerial.available()) {
      char c = a7670cSerial.read();
      response += c;
      responseReceived = true;
      
      // Check for response endings
      if (response.endsWith("OK\r\n") || response.endsWith("ERROR\r\n") || 
          response.endsWith("FAIL\r\n")) {
        break;
      }
    }
    delay(10);
    esp_task_wdt_reset();  // Feed watchdog during wait
  }
  
  response.trim();
  
  if (debugEnabled && responseReceived) {
    Serial.println("A7670C ← " + String(response.length() > 100 ? response.substring(0, 100) + "..." : response));
  }
  
  // If no response received, mark module as potentially disconnected
  if (!responseReceived) {
    if (debugEnabled) {
      Serial.println("A7670C ← [NO RESPONSE - Module may need reset]");
    }
    
    // Check if this is a critical HTTP command that should trigger immediate recovery
    bool isCriticalCommand = (command.indexOf("HTTPTERM") >= 0 || 
                              command.indexOf("HTTPINIT") >= 0 ||
                              command.indexOf("CRESET") >= 0 ||
                              command == "AT");
    
    static unsigned long lastNoResponse = 0;
    static int noResponseCount = 0;
    
    if (millis() - lastNoResponse < 10000) { // Extended window to 10 seconds
      noResponseCount++;
    } else {
      noResponseCount = 1;
    }
    lastNoResponse = millis();
    
    // Immediate recovery for critical commands or after 2 consecutive failures
    if (isCriticalCommand || noResponseCount >= 2) {
      if (debugEnabled) {
        if (isCriticalCommand) {
          Serial.println("🚨 Critical command failed: " + command + " - immediate recovery required");
        } else {
          Serial.println("⚠️ Multiple AT command failures detected - attempting module recovery");
          Serial.println("🔄 Failure count: " + String(noResponseCount) + ", triggering recovery...");
        }
      }
      recoverA7670CModule();
      noResponseCount = 0;
    }
  }
  
  return response;
}





// Move struct definition to the top with other global variables
struct PWMConfig {
  int inputPin = -1;  // Default to -1 (not set)
  bool isConfigured = false;
  bool isBlocked = false;
  bool interruptDetachedForBlock = false;  // Track if ISR was detached during block
  bool usePCNT = true;              // Prefer hardware pulse counter if available
  unsigned long blockStartTime = 0;  // New: Track when blocking starts
  unsigned long blockDuration = 0;   // New: How long to block for
  volatile unsigned long pulseCount = 0;
  volatile unsigned long lastPulseTime = 0;
  unsigned long lastReadingTime = 0;
  const unsigned long READING_INTERVAL = 100;  // Take readings every 100ms
  const unsigned long SEND_INTERVAL = 1000;    // Send data every 1 seconds
  unsigned long lastSendTime = 0;
  unsigned long aggregatedPulses = 0;          // Aggregate pulses for each send window
  unsigned long aggregateStartTime = 0;        // Start time of current aggregation window
  const int HISTORY_SIZE = 20;       // Store last 20 readings
  float frequencyHistory[20];        // Circular buffer for frequency history
  int historyIndex = 0;              // Current index in history buffer
  int historyCount = 0;              // Number of valid entries in history
  const int CONSISTENT_COUNT = 4;    // Number of consistent readings needed
  int consistentReadings = 0;        // Counter for consistent readings
  unsigned long lastPulseCount = 0;  // Last pulse count for comparison
  unsigned long sendSuppressUntil = 0; // Suppress sending until this time (ms)
  int zeroStreak = 0;                // Count consecutive zero-interval windows
  int suppressedOutlierStreak = 0;   // Count consecutive outlier suppressions
} pwmConfig;

// Move interrupt handler declaration before it's used
void IRAM_ATTR handlePWMInterrupt() {
  unsigned long currentTime = micros();
  // Debounce - ignore pulses that are too close together (less than 50 microseconds)
  if (currentTime - pwmConfig.lastPulseTime > 50) {
    pwmConfig.pulseCount++;
    pwmConfig.lastPulseTime = currentTime;
  }
}

// Real A7670C HTTP client implementation
class A7670CWebSocketClient {
private:
  String host;
  int port;
  String path;
  bool connected;
  String deviceId;
  unsigned long lastPollTime = 0;
  const unsigned long POLL_INTERVAL = 2000;  // Poll every 2 seconds (reduced from 5 seconds)
  // Removed ping functionality - polling serves as heartbeat
  
  
public:
  A7670CWebSocketClient() : connected(false) {}
  
  bool begin(String serverHost, int serverPort, String serverPath) {
    host = serverHost;
    port = serverPort;  // Use the actual port passed in
    path = serverPath;
    deviceId = macToUUID();
    
    if (debugEnabled) {
      Serial.println("🔌 Connecting HTTP via A7670C...");
      Serial.println("Host: " + host + ":" + String(port));
      Serial.println("Device ID: " + deviceId);
    }
    
    // Join the device with the server
    if (!joinDevice()) {
      return false;
    }
    
    connected = true;
    lastPollTime = millis();
    
    if (debugEnabled) {
      Serial.println("✅ HTTP connection established via A7670C");
    }
    
    return true;
  }
  
  bool joinDevice() {
    String joinData = "{\"device_id\":\"" + deviceId + "\",\"connection_type\":\"cellular\", \"firmware_version\":\"" + FIRMWARE_VERSION + "\"}";
    
    if (debugEnabled) {
      Serial.println("🔗 Attempting to join device with data: " + joinData);
    }
    
    String response = sendHTTPRequestHTTPOnly("POST", "/iot/a7670c/join", joinData);
    
    if (debugEnabled) {
      Serial.println("\n==== JOIN RESPONSE ANALYSIS ====");
      Serial.println("Response length: " + String(response.length()));
      Serial.println("Response content: '" + response + "'");
      
      // Show first and last few characters
      if (response.length() > 0) {
        Serial.println("First 50 chars: '" + response.substring(0, min(50, (int)response.length())) + "'");
        if (response.length() > 50) {
          Serial.println("Last 50 chars: '..." + response.substring(response.length() - 50) + "'");
        }
      }
      Serial.println("================================");
    }
    
    if (response.length() > 0) {
      // Try to parse as JSON first
      StaticJsonDocument<512> doc; // Reduced size for minimal response
      DeserializationError error = deserializeJson(doc, response);
      
      if (!error) {
        if (debugEnabled) {
          Serial.println("✅ Valid JSON response received");
        }
        
        // Check for successful join response (new minimal format)
        if (doc.containsKey("s") && doc["s"] == "ok") {
          if (debugEnabled) {
            Serial.println("✅ Device joined successfully with minimal response");
            if (doc.containsKey("id")) {
              Serial.println("Device ID: " + String(doc["id"].as<String>()));
            }
            if (doc.containsKey("ts")) {
              Serial.println("Server Time: " + String(doc["ts"].as<long>()));
            }
          }
          
          // Parse minimal config if available
          if (doc.containsKey("c")) {
            JsonObject config = doc["c"];
            
            // Extract pin configuration
            if (config.containsKey("p")) {
              int newPin = config["p"];
              if (newPin != pwmConfig.inputPin) {
                if (pwmConfig.isConfigured && !pwmConfig.usePCNT) {
                  detachInterrupt(digitalPinToInterrupt(pwmConfig.inputPin));
                }
                
                pwmConfig.inputPin = newPin;
                pinMode(newPin, INPUT);
                if (!pwmConfig.usePCNT) {
                  attachInterrupt(digitalPinToInterrupt(newPin), handlePWMInterrupt, RISING);
                }
                pwmConfig.isConfigured = true;
                pwmConfig.pulseCount = 0;
                
                if (debugEnabled) {
                  Serial.printf("📡 Configured PWM reading on pin %d (minimal config)\n", newPin);
                }
              }
            }
          }
          return true;
        }
        
        // Check for legacy format (fallback)
        if (doc.containsKey("device_id") && doc.containsKey("server_time")) {
          if (debugEnabled) {
            Serial.println("✅ Device joined successfully with legacy response");
            Serial.println("Device ID: " + String(doc["device_id"].as<String>()));
            Serial.println("Server Time: " + String(doc["server_time"].as<String>()));
          }
          
          // Parse settings if available
          if (doc.containsKey("settings")) {
            parseDeviceSettings(response);
          }
          return true;
        }
        
        // Check for explicit status responses
        if (doc.containsKey("status")) {
          String status = doc["status"];
          if (status == "ok" || status == "success") {
            if (debugEnabled) {
              Serial.println("✅ Device joined successfully (status: " + status + ")");
            }
            parseDeviceSettings(response);
            return true;
          } else {
            if (debugEnabled) {
              Serial.println("❌ Device join failed with status: " + status);
            }
            return false;
          }
        }
        
        // If we have valid JSON but no clear status, consider it successful
        if (debugEnabled) {
          Serial.println("✅ Valid JSON received, treating as successful join");
        }
        parseDeviceSettings(response);
        return true;
        
      } else {
        if (debugEnabled) {
          Serial.println("❌ JSON parsing failed: " + String(error.c_str()));
          Serial.println("Raw response: " + response);
        }
        
        // Check for partial JSON or specific keywords
        if (response.indexOf("device_id") >= 0 || response.indexOf("server_time") >= 0 ||
            response.indexOf("\"s\":\"ok\"") >= 0) {
          if (debugEnabled) {
            Serial.println("⚠️ Found success indicators in response, treating as successful");
          }
          
          // Try to extract partial settings even from broken JSON
          if (response.indexOf("pwm_config") >= 0 || response.indexOf("\"p\":") >= 0) {
            // Try to extract PWM config from partial JSON
            int pinStart = response.indexOf("\"p\":");
            if (pinStart >= 0) {
              // Look for the pin number after "p":
              int colonPos = pinStart + 4; // Position after "p":
              String pinStr = "";
              for (int i = colonPos; i < response.length(); i++) {
                char c = response.charAt(i);
                if (c >= '0' && c <= '9') {
                  pinStr += c;
                } else if (pinStr.length() > 0) {
                  break; // Found end of number
                }
              }
              
              if (pinStr.length() > 0) {
                int newPin = pinStr.toInt();
                if (newPin > 0 && newPin != pwmConfig.inputPin) {
                  if (pwmConfig.isConfigured && !pwmConfig.usePCNT) {
                    detachInterrupt(digitalPinToInterrupt(pwmConfig.inputPin));
                  }
                  
                  pwmConfig.inputPin = newPin;
                  pinMode(newPin, INPUT);
                  if (!pwmConfig.usePCNT) {
                    attachInterrupt(digitalPinToInterrupt(newPin), handlePWMInterrupt, RISING);
                  }
                  pwmConfig.isConfigured = true;
                  pwmConfig.pulseCount = 0;
                  
                  if (debugEnabled) {
                    Serial.printf("📡 Configured PWM reading on pin %d (from partial minimal JSON)\n", newPin);
                  }
                }
              }
            }
          }
          
          return true;
        }
        
        return false;
      }
    } else {
      if (debugEnabled) {
        Serial.println("❌ Device join failed: No response received");
      }
      return false;
    }
  }
  
  void parseDeviceSettings(String response) {
    // Parse JSON response to extract device settings
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, response);
    
    if (!error && doc.containsKey("settings")) {
      JsonObject settings = doc["settings"];
      
      // Handle PWM config
      if (settings.containsKey("pwm_config")) {
        JsonObject pwmConfig = settings["pwm_config"];
        int newPin = pwmConfig["input_pin"] | -1;
        
        if (newPin != -1 && newPin != ::pwmConfig.inputPin) {
          if (::pwmConfig.isConfigured && !::pwmConfig.usePCNT) {
            detachInterrupt(digitalPinToInterrupt(::pwmConfig.inputPin));
          }
          
          ::pwmConfig.inputPin = newPin;
          pinMode(newPin, INPUT);
          if (!::pwmConfig.usePCNT) {
            attachInterrupt(digitalPinToInterrupt(newPin), handlePWMInterrupt, RISING);
          }
          ::pwmConfig.isConfigured = true;
          ::pwmConfig.pulseCount = 0;
          
          if (debugEnabled) {
            Serial.printf("📡 Configured PWM reading on pin %d via A7670C\n", newPin);
          }
        }
      }
      
      // Handle RS232 config
      if (settings.containsKey("rs232_config") && !SKIP_RS232) {
        JsonObject rs232Config = settings["rs232_config"];
        String deviceType = rs232Config["device_type"] | "bill_acceptor";
        
        if (deviceType == "bill_acceptor") {
          int baudRate = rs232Config["baud_rate"] | 9600;
          int rxPin = rs232Config["rx_pin"] | 32;  // Changed from 25 to 32
          int txPin = rs232Config["tx_pin"] | 33;  // Changed from 26 to 33
          
          billAcceptorSerial.begin(baudRate, SERIAL_8N2, rxPin, txPin, false);
          emulatorMode = true;
          
          if (debugEnabled) {
            Serial.println("📡 Bill acceptor configured via A7670C");
          }
        }
      }
    }
  }
  
  void loop() {
    if (!connected || !a7670cConnected) return;
    
    // Pause polling completely while OTA is in progress to avoid HTTP conflicts
    static bool httpPausedForOTA = false;
    if (otaInProgress) {
      if (!httpPausedForOTA) {
        // Best-effort terminate any outstanding HTTP session before OTA
        sendA7670CCommand("AT+HTTPTERM", 2000);
        httpPausedForOTA = true;
      }
      return;
    } else if (httpPausedForOTA) {
      // Resume normal polling once OTA is done
      httpPausedForOTA = false;
    }
    
    unsigned long currentTime = millis();
    
    // Poll for commands/tasks (this also serves as heartbeat/keepalive for A7670C)
    if (currentTime - lastPollTime >= POLL_INTERVAL) {
      pollForTasks();
      lastPollTime = currentTime;
    }
    
    // Note: Ping functionality removed for A7670C since polling serves as heartbeat
    // WiFi WebSocket mode still uses sendPing() in the main loop
  }
  
  void pollForTasks() {
    String response = sendHTTPRequestHTTPOnly("GET", "/iot/a7670c/poll/" + deviceId, "");
    
    if (response.length() > 0 && (response.indexOf("\"t\"") >= 0 || response.indexOf("\"tasks\"") >= 0)) {
      processTasks(response);
    }
  }
  
  void processTasks(String response) {
    if (debugEnabled) {
      Serial.println("🔧 processTasks() called!");
      Serial.println("📥 Raw response length: " + String(response.length()));
      Serial.println("📥 Raw response data: " + response.substring(0, min(200, (int)response.length())));
    }
    
    StaticJsonDocument<512> doc; // Reduced size for minimal response
    DeserializationError error = deserializeJson(doc, response);
    
    if (error) {
      if (debugEnabled) {
        Serial.println("❌ JSON parsing failed: " + String(error.c_str()));
        Serial.println("📥 Failed to parse: " + response);
      }
      return;
    }
    
    if (debugEnabled) {
      Serial.println("✅ JSON parsing successful!");
    }
    
    // Check for minimal format first (field "t" for tasks)
    JsonArray tasks;
    if (doc.containsKey("t")) {
      tasks = doc["t"];
      if (debugEnabled) {
        Serial.println("📋 Found minimal format tasks array with " + String(tasks.size()) + " tasks");
      }
    } else if (doc.containsKey("tasks")) {
      // Fallback to legacy format
      tasks = doc["tasks"];
      if (debugEnabled) {
        Serial.println("📋 Found legacy format tasks array with " + String(tasks.size()) + " tasks");
      }
    } else {
      if (debugEnabled) {
        Serial.println("⚠️ No 'tasks' or 't' key found in JSON");
        // Show what keys are available
        for (JsonPair kv : doc.as<JsonObject>()) {
          Serial.println("🔑 Available key: " + String(kv.key().c_str()));
        }
      }
      return;
    }
    
    if (tasks.size() == 0) {
      if (debugEnabled) {
        Serial.println("📭 No tasks to execute (empty array)");
      }
      return;
    }
    
    for (JsonObject task : tasks) {
      // Handle both minimal and legacy formats
      String action, uuid, format;
      int reps, pin;
      float delay;
      
      // Check if it's minimal format (short field names)
      if (task.containsKey("a")) {
        // Minimal format
        action = task["a"] | "";
        reps = task["r"] | 1;
        delay = task["d"] | 0.1;
        pin = task["p"] | 5;
        uuid = task["u"] | "";
        format = task["f"] | "pwm"; // Default format
      } else {
        // Legacy format
        action = task["action"] | "";
        reps = task["reps"] | 1;
        delay = task["delay"] | 0.1;
        pin = task["pin"] | 5;
        uuid = task["uuid"] | "";
        format = task["format"] | "pwm";
      }
      
      // Promote ota_update to OTA format even if not explicitly provided
      if (action == "ota_update") {
        format = "ota";
      }
      
      if (debugEnabled) {
        Serial.println("📦 Processing task:");
        Serial.println("  🎬 Action: " + action);
        Serial.println("  🔄 Reps: " + String(reps));
        Serial.println("  ⏱️ Delay: " + String(delay));
        Serial.println("  📍 Pin: " + String(pin));
        Serial.println("  🎭 Format: " + format);
        Serial.println("  🆔 UUID: " + uuid);
      }
      
      if (action.length() == 0) {
        if (debugEnabled) {
          Serial.println("⚠️ Skipping task with empty action");
        }
        continue;
      }
      
      // If OTA update and minimal OTA payload present, propagate to handler
      if (action == "ota_update" && task.containsKey("o")) {
        JsonObject o = task["o"].as<JsonObject>();
        StaticJsonDocument<256> otaDoc;
        otaDoc["action"] = "ota_update";
        otaDoc["format"] = "ota";
        otaDoc["uuid"] = uuid;
        otaDoc["firmware_version"] = o["v"] | "";
        otaDoc["download_url"] = o["u"] | "";
        otaDoc["mandatory"] = o["m"] | false;
        sendA7670CCommand("AT+HTTPTERM", 1500);
        handleOTACommand(otaDoc.as<JsonObject>());
        if (debugEnabled) {
          Serial.println("🔔 OTA parameters received via minimal field 'o' and applied");
        }
        return; // Don't continue normal execution path
      }
      
      if (debugEnabled) {
        Serial.println("🚀 Executing task: " + action);
      }
      
      // Execute the task (reuse existing logic)
      executeTask(action, format, reps, delay, pin, uuid);
      
      if (debugEnabled) {
        Serial.println("✅ Task execution completed: " + action);
      }
    }
    
    if (debugEnabled) {
      Serial.println("🏁 All tasks processed!");
    }
  }
  
  void executeTask(String action, String format, int reps, float delayTime, int pin, String uuid) {
    if (debugEnabled) {
      Serial.println("⚡ executeTask() STARTED");
      Serial.println("  🎬 Action: " + action);
      Serial.println("  🎭 Format: " + format);
      Serial.println("  🔄 Reps: " + String(reps));
      Serial.println("  ⏱️ Delay: " + String(delayTime));
      Serial.println("  📍 Pin: " + String(pin));
      Serial.println("  🆔 UUID: " + uuid);
    }
    
    // Block other operations during task execution
    pwmConfig.blockStartTime = millis();
    pwmConfig.blockDuration = (unsigned long)(reps * (delayTime + 0.1) * 1000) + 100;
    pwmConfig.isBlocked = true;
    // Detach ISR during block to prevent any readings leaking through
    if (pwmConfig.isConfigured && !pwmConfig.interruptDetachedForBlock) {
      detachInterrupt(digitalPinToInterrupt(pwmConfig.inputPin));
      pwmConfig.interruptDetachedForBlock = true;
      noInterrupts();
      pwmConfig.pulseCount = 0;
      pwmConfig.lastPulseTime = 0;
      interrupts();
    }
    
    if (debugEnabled) {
      Serial.println("🔒 PWM operations blocked for " + String(pwmConfig.blockDuration) + "ms");
    }
    
    // Save current emulator mode state
    bool wasEmulatorMode = emulatorMode;
    emulatorMode = false;
    
    if (action == "start") {
      if (debugEnabled) {
        Serial.println("🟢 Executing START action");
      }
      
      if (format == "pwm") {
        if (debugEnabled) {
          Serial.println("⚡ PWM mode - Setting up pin " + String(pin));
        }
        
        // pinMode(pin, OUTPUT);
        // for (int i = 0; i < reps; i++) {
        //   if (debugEnabled) {
        //     Serial.println("📶 PWM pulse " + String(i + 1) + "/" + String(reps));
        //   }
        //   digitalWrite(pin, HIGH);
        //   delay(delayTime * 500);  // On time
        //   digitalWrite(pin, LOW);
        //   delay(delayTime * 500);  // Off time
        // }


        startPWM(pin, 65535);  // Full duty cycle for start
        for (int i = 0; i < reps; i++) {
          delay(delayTime * 1000);  // Delay in milliseconds
          stopPWM(pin);             // Stop PWM
          delay(delayTime * 1000);  // Delay in milliseconds
          startPWM(pin, 65535);     // Restart PWM
        }
        stopPWM(pin);  // Final stop
        
        if (debugEnabled) {
          Serial.println("✅ PWM pulses completed");
        }
      } else if (format == "rs232") {
        if (SKIP_RS232) {
          if (debugEnabled) {
            Serial.println("🚫 RS232 mode disabled - Skipping bill dispensing");
          }
        } else {
          if (debugEnabled) {
            Serial.println("📡 RS232 mode - Dispensing bills");
          }
          
          // Handle RS232 bill dispensing
          const struct {
            int value;
            uint8_t command;
          } bills[] = {
            {10, BILL_10},
            {5, BILL_5},
            {2, BILL_2},
            {1, BILL_1}
          };
          
          int remainingAmount = reps;
          if (debugEnabled) {
            Serial.println("💰 Dispensing total amount: " + String(remainingAmount));
          }
          
          for (const auto& bill : bills) {
            while (remainingAmount >= bill.value) {
              if (debugEnabled) {
                Serial.println("💵 Dispensing bill value: " + String(bill.value));
              }
              
              uint8_t data[] = {0x80, bill.command, 0x10};
              sendSerialData(data, sizeof(data));
              remainingAmount -= bill.value;
              delay(1000);
            }
          }
          
          if (debugEnabled) {
            Serial.println("✅ RS232 dispensing completed");
          }
        }
      }
    } else if (action == "motor") {
      if (debugEnabled) {
        Serial.println("🔧 Executing MOTOR action");
        Serial.println("🔧 Motor ON for " + String(reps) + " seconds on pin " + String(pin));
      }
      
      digitalWrite(pin, HIGH);
      delay(reps * 1000);
      digitalWrite(pin, LOW);
      
      if (debugEnabled) {
        Serial.println("✅ Motor action completed");
      }
    } else if (action == "ota_update" || format == "ota") {
      // Check if this is an OTA update command
      if (debugEnabled) {
        Serial.println("📡 Processing OTA update command from server");
      }
      
      // Ensure any existing HTTP session is closed before starting OTA
      sendA7670CCommand("AT+HTTPTERM", 1500);
      
      // Create a JSON object and route to the OTA handler
      StaticJsonDocument<256> otaDoc;
      // Prefer server-provided explicit action if available, otherwise signal an update check
      otaDoc["action"] = "ota_update";
      otaDoc["format"] = "ota";
      otaDoc["uuid"] = uuid;
      
      JsonObject otaPayload = otaDoc.as<JsonObject>();
      handleOTACommand(otaPayload);
      return; // Don't send completion response for OTA commands
    } else {
      if (debugEnabled) {
        Serial.println("❓ Unknown action: " + action);
      }
    }
    
    // Restore emulator mode
    if (format != "rs232" && !SKIP_RS232) {
      emulatorMode = wasEmulatorMode;
      if (debugEnabled) {
        Serial.println("🔄 Emulator mode restored: " + String(emulatorMode ? "ON" : "OFF"));
      }
    }
    
    if (debugEnabled) {
      Serial.println("🏁 executeTask() COMPLETED for action: " + action);
    }
    
    // Send completion response
    sendTaskComplete(uuid, action, pin, reps);
  }
  
  void sendTaskComplete(String uuid, String action, int pin, int reps) {
    StaticJsonDocument<256> responseDoc;
    responseDoc["uuid"] = uuid;
    responseDoc["action"] = action;
    responseDoc["pin"] = pin;
    responseDoc["reps"] = reps;
    responseDoc["status"] = "completed";
    
    String payload;
    serializeJson(responseDoc, payload);
    
    // Add delay before sending completion to avoid overwhelming module
    delay(500); // Reduced from 2000ms to speed up task completion
    esp_task_wdt_reset();
    
    String response = sendHTTPRequestHTTPOnly("POST", "/iot/complete/" + deviceId, payload);
    
    if (debugEnabled) {
      Serial.println("📤 Task completion sent via A7670C");
    }
  }
  
  void sendPingA7670C() {
    if (debugEnabled) {
      Serial.println("📤 Sending ping via A7670C...");
    }
    
    // Add delay to prevent overwhelming module with rapid requests
    delay(1000);
    esp_task_wdt_reset();
    
    String response = sendHTTPRequestHTTPOnly("POST", "/iot/a7670c/ping/" + deviceId, "{}");
    
    // Since AT+HTTPREAD often fails, check if we got any response at all
    // If we got a response (even empty "{}") from ping endpoint, consider it successful
    if (response.length() > 0) {
      // Check for explicit pong status first
      if (response.indexOf("\"status\":\"pong\"") >= 0) {
        lastReceivedPingTime = millis();
        if (debugEnabled) {
          Serial.println("✅ Ping response received via A7670C (pong detected)");
        }
      } 
      // If no explicit pong but we got any response, still consider it successful
      // since the server responded with HTTP 200
      else {
        lastReceivedPingTime = millis();
        if (debugEnabled) {
          Serial.println("✅ Ping response received via A7670C (HTTP 200)");
        }
      }
    } else {
      if (debugEnabled) {
        Serial.println("❌ No ping response received via A7670C");
      }
    }
  }
  
  bool sendTXT(String message) {
    if (!connected || !a7670cConnected) return false;
    
    if (debugEnabled) {
      Serial.println("📤 Sending message via A7670C HTTP: " + message);
    }
    
    // Parse message as Phoenix channel format and send appropriately
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (!error && doc.size() >= 4) {
      String event = doc[3];
      
      if (event == "pwm_readings") {
        JsonObject payload = doc[4];
        sendReading("pwm", payload);
      } else if (event == "cash_reading") {
        JsonObject payload = doc[4];
        sendReading("cash", payload);
      }
    }
    
    return true;
  }
  
  void sendReading(String type, JsonObject data) {
    StaticJsonDocument<256> readingDoc;
    readingDoc["type"] = type;
    
    // Copy data fields
    for (JsonPair kv : data) {
      readingDoc[kv.key()] = kv.value();
    }
    
    String readingData;
    serializeJson(readingDoc, readingData);
    
    String payload = "{\"reading\":" + readingData + "}";
    String response = sendHTTPRequestHTTPOnly("POST", "/iot/a7670c/reading/" + deviceId, payload);
    
    if (debugEnabled) {
      Serial.println("📤 Reading sent via A7670C: " + type);
    }
  }
  String fullResponseData = "";  // Store any response data that comes with +HTTPACTION
  String sendHTTPRequest(String method, String endpoint, String data) {
    if (!a7670cConnected) return "";
    
    // Close any existing TCP connection (try multiple methods)
    sendA7670CCommand("AT+QICLOSE=0", 3000);
    sendA7670CCommand("AT+CIPCLOSE", 3000);
    delay(1000);
    
    // Try opening TCP connection with different command variations
    String response = "";
    bool connectionSuccess = false;
    
    // Method 1: Try AT+QIOPEN (Quectel style)
    String openCmd = "AT+QIOPEN=1,0,\"TCP\",\"" + host + "\"," + String(port) + ",0,1";
    if (debugEnabled) {
      Serial.println("🔌 Trying TCP connection (QIOPEN): " + openCmd);
    }
    response = sendA7670CCommand(openCmd, 30000);
    
    if (response.indexOf("OK") >= 0) {
      // Wait and check connection status
      delay(2000);
      esp_task_wdt_reset();
      response = sendA7670CCommand("AT+QISTATE=1,0", 5000);
      if (response.indexOf("\"CONNECTED\"") >= 0) {
        connectionSuccess = true;
        if (debugEnabled) {
          Serial.println("✅ TCP connection established (QIOPEN)");
        }
      }
    }
    
    // Method 2: Try AT+CIPSTART if QIOPEN failed
    if (!connectionSuccess) {
      if (debugEnabled) {
        Serial.println("🔌 Trying TCP connection (CIPSTART): AT+CIPSTART=\"TCP\",\"" + host + "\"," + String(port));
      }
      
      String cipCmd = "AT+CIPSTART=\"TCP\",\"" + host + "\"," + String(port);
      response = sendA7670CCommand(cipCmd, 30000);
      
      if (response.indexOf("CONNECT OK") >= 0 || response.indexOf("OK") >= 0) {
        connectionSuccess = true;
        if (debugEnabled) {
          Serial.println("✅ TCP connection established (CIPSTART)");
        }
      }
    }
    
    // Method 3: Try simplified TCP connection
    if (!connectionSuccess) {
      if (debugEnabled) {
        Serial.println("🔌 Trying simplified TCP connection");
      }
      
      // Set up TCP context first
      sendA7670CCommand("AT+CGDCONT=1,\"IP\",\"" + CELCOM_APN + "\"", 5000);
      sendA7670CCommand("AT+CGACT=1,1", 10000);
      
      String simpleCmd = "AT+QIOPEN=1,0,\"TCP\",\"" + host + "\"," + String(port);
      response = sendA7670CCommand(simpleCmd, 30000);
      
      if (response.indexOf("OK") >= 0) {
        delay(3000);
        esp_task_wdt_reset();
        connectionSuccess = true;
        if (debugEnabled) {
          Serial.println("✅ TCP connection established (simplified)");
        }
      }
    }
    
    if (!connectionSuccess) {
      if (debugEnabled) {
        Serial.println("❌ All TCP connection methods failed");
        Serial.println("Last response: " + response);
        Serial.println("🔄 Trying HTTP-only method (recommended for A7670C)...");
      }
      
      // Try HTTP-only approach first (most reliable for A7670C)
      String result = sendHTTPRequestHTTPOnly(method, endpoint, data);
      if (result.length() > 0) {
        return result;
      }
      
      if (debugEnabled) {
        Serial.println("🔄 HTTP-only failed, trying SIMCOM socket commands...");
      }
      
      // Try SIMCOM socket approach second
      result = sendHTTPRequestViaSIMCOMSockets(method, endpoint, data);
      if (result.length() > 0) {
        return result;
      }
      
      if (debugEnabled) {
        Serial.println("🔄 SIMCOM sockets failed, falling back to legacy HTTP AT commands...");
      }
      
      // Final fallback to original HTTP AT commands
      return sendHTTPRequestViaATCommands(method, endpoint, data);
    }
    
    // Rest of the HTTP request logic remains the same
    // Construct HTTP request
    String httpRequest = "";
    httpRequest += method + " " + endpoint + " HTTP/1.1\r\n";
    httpRequest += "Host: " + host + ":" + String(port) + "\r\n";
    httpRequest += "Connection: close\r\n";
    
    if (method == "POST" && data.length() > 0) {
      httpRequest += "Content-Type: application/json\r\n";
      httpRequest += "Content-Length: " + String(data.length()) + "\r\n";
      httpRequest += "\r\n";
      httpRequest += data;
    } else {
      httpRequest += "\r\n";
    }
    
    if (debugEnabled) {
      Serial.println("📤 HTTP Request:");
      Serial.println(httpRequest);
    }
    
    // Send HTTP request (try different send methods)
    String sendResult = "";
    
    // Method 1: Try AT+QISEND
    String sendCmd = "AT+QISEND=0," + String(httpRequest.length());
    response = sendA7670CCommand(sendCmd, 5000);
    
    if (response.indexOf(">") >= 0) {
      a7670cSerial.print(httpRequest);
      delay(100);
      
      // Wait for send confirmation
      response = "";
      unsigned long startTime = millis();
      while (millis() - startTime < 10000) {
        if (a7670cSerial.available()) {
          response += (char)a7670cSerial.read();
          if (response.indexOf("SEND OK") >= 0) {
            sendResult = "OK";
            break;
          }
          if (response.indexOf("SEND FAIL") >= 0) {
            break;
          }
        }
        delay(10);
        esp_task_wdt_reset();
      }
    }
    
    // Method 2: Try AT+CIPSEND if QISEND failed
    if (sendResult != "OK") {
      if (debugEnabled) {
        Serial.println("Trying CIPSEND method...");
      }
      
      String cipSendCmd = "AT+CIPSEND=" + String(httpRequest.length());
      response = sendA7670CCommand(cipSendCmd, 5000);
      
      if (response.indexOf(">") >= 0) {
        a7670cSerial.print(httpRequest);
        delay(100);
        
        response = "";
        unsigned long startTime = millis();
        while (millis() - startTime < 10000) {
          if (a7670cSerial.available()) {
            response += (char)a7670cSerial.read();
            if (response.indexOf("SEND OK") >= 0) {
              sendResult = "OK";
              break;
            }
          }
          delay(10);
          esp_task_wdt_reset();
        }
      }
    }
    
    if (sendResult != "OK") {
      if (debugEnabled) {
        Serial.println("❌ Failed to send HTTP request");
      }
      sendA7670CCommand("AT+QICLOSE=0", 3000);
      sendA7670CCommand("AT+CIPCLOSE", 3000);
      return "";
    }
    
    if (debugEnabled) {
      Serial.println("✅ HTTP request sent successfully");
    }
    
    // Wait a bit for server response
    delay(2000);
    esp_task_wdt_reset();
    
    // Read response (try different read methods)
    String result = "";
    
    for (int attempt = 0; attempt < 5; attempt++) {
      // Method 1: Try AT+QIRD
      response = sendA7670CCommand("AT+QIRD=0,1500", 10000);
      
      if (response.indexOf("+QIRD:") >= 0) {
        // Extract the actual HTTP response
        int dataStart = response.indexOf("\n");
        if (dataStart >= 0) {
          dataStart++;
          int dataEnd = response.lastIndexOf("\nOK");
          if (dataEnd < 0) dataEnd = response.length();
          
          String httpResponse = response.substring(dataStart, dataEnd);
          
          if (debugEnabled) {
            Serial.println("📥 Raw HTTP Response (QIRD):");
            Serial.println(httpResponse);
          }
          
          // Parse HTTP response to extract JSON body
          int bodyStart = httpResponse.indexOf("\r\n\r\n");
          if (bodyStart >= 0) {
            bodyStart += 4;
            result = httpResponse.substring(bodyStart);
            result.trim();
            
            if (result.length() > 0) {
              if (debugEnabled) {
                Serial.println("📥 HTTP Response Body: " + result);
              }
              break;
            }
          }
        }
      }
      
      // Method 2: Try AT+CIPRCV if QIRD failed
      if (result.length() == 0) {
        response = sendA7670CCommand("AT+CIPRCV?", 5000);
        if (response.indexOf("+CIPRCV:") >= 0) {
          response = sendA7670CCommand("AT+CIPRCV=1500", 10000);
          if (response.length() > 0) {
            if (debugEnabled) {
              Serial.println("📥 Raw HTTP Response (CIPRCV):");
              Serial.println(response);
            }
            
            int bodyStart = response.indexOf("\r\n\r\n");
            if (bodyStart >= 0) {
              bodyStart += 4;
              result = response.substring(bodyStart);
              result.trim();
              
              if (result.length() > 0) {
                if (debugEnabled) {
                  Serial.println("📥 HTTP Response Body: " + result);
                }
                break;
              }
            }
          }
        }
      }
      
      delay(1000);
      esp_task_wdt_reset();
    }
    
    // Close TCP connection (try both methods)
    sendA7670CCommand("AT+QICLOSE=0", 3000);
    sendA7670CCommand("AT+CIPCLOSE", 3000);
    
    return result;
  }
  
  // Alternative SIMCOM socket method for A7670C
  String sendHTTPRequestViaSIMCOMSockets(String method, String endpoint, String data) {
    if (debugEnabled) {
      Serial.println("📡 Using SIMCOM socket commands (A7670C alternative)");
    }
    
    // Step 1: Open network if not already open
    String response = sendA7670CCommand("AT+NETOPEN?", 5000);
    if (response.indexOf("+NETOPEN: 1") < 0) {
      if (debugEnabled) {
        Serial.println("🔗 Opening network connection...");
      }
      response = sendA7670CCommand("AT+NETOPEN", 10000);
      if (response.indexOf("OK") < 0) {
        if (debugEnabled) {
          Serial.println("❌ Failed to open network");
        }
        return "";
      }
      delay(3000);  // Wait for network to be ready
    }
    
    // Step 2: Create socket (TCP)
    response = sendA7670CCommand("AT+CSOC=1,1,1", 5000);  // PF_INET, SOCK_STREAM, IPPROTO_TCP
    int socketId = -1;
    
    if (response.indexOf("+CSOC:") >= 0) {
      int startPos = response.indexOf("+CSOC:") + 7;
      int endPos = response.indexOf("\n", startPos);
      if (endPos > startPos) {
        String socketStr = response.substring(startPos, endPos);
        socketStr.trim();
        socketId = socketStr.toInt();
        
        if (debugEnabled) {
          Serial.println("✅ Socket created with ID: " + String(socketId));
        }
      }
    }
    
    if (socketId < 0) {
      if (debugEnabled) {
        Serial.println("❌ Failed to create socket");
      }
      return "";
    }
    
    // Step 3: Connect to server
    String connectCmd = "AT+CSOCON=" + String(socketId) + ",\"" + host + "\"," + String(port);
    response = sendA7670CCommand(connectCmd, 15000);
    
    if (response.indexOf("OK") < 0 && response.indexOf("CONNECT") < 0) {
      if (debugEnabled) {
        Serial.println("❌ Failed to connect socket to server");
      }
      // Close socket before returning
      sendA7670CCommand("AT+CSOCL=" + String(socketId), 3000);
      return "";
    }
    
    if (debugEnabled) {
      Serial.println("✅ Socket connected to server");
    }
    
    // Step 4: Construct HTTP request
    String httpRequest = method + " " + endpoint + " HTTP/1.1\r\n";
    httpRequest += "Host: " + host + ":" + String(port) + "\r\n";
    httpRequest += "Connection: close\r\n";
    
    if (method == "POST" && data.length() > 0) {
      httpRequest += "Content-Type: application/json\r\n";
      httpRequest += "Content-Length: " + String(data.length()) + "\r\n";
      httpRequest += "\r\n";
      httpRequest += data;
    } else {
      httpRequest += "\r\n";
    }
    
    if (debugEnabled) {
      Serial.println("📤 HTTP Request to send:");
      Serial.println(httpRequest);
    }
    
    // Step 5: Send HTTP request
    String sendCmd = "AT+CSOSEND=" + String(socketId) + "," + String(httpRequest.length());
    response = sendA7670CCommand(sendCmd, 5000);
    
    if (response.indexOf(">") >= 0) {
      // Send the data
      a7670cSerial.print(httpRequest);
      a7670cSerial.flush();
      delay(100);
      
      // Wait for send confirmation
      unsigned long startTime = millis();
      String sendResponse = "";
      bool sendSuccess = false;
      
      while (millis() - startTime < 10000) {
        if (a7670cSerial.available()) {
          char c = a7670cSerial.read();
          sendResponse += c;
          
          if (sendResponse.indexOf("OK") >= 0) {
            sendSuccess = true;
            break;
          }
        }
        delay(10);
        esp_task_wdt_reset();
      }
      
      if (!sendSuccess) {
        if (debugEnabled) {
          Serial.println("❌ Failed to send HTTP request");
        }
        sendA7670CCommand("AT+CSOCL=" + String(socketId), 3000);
        return "";
      }
      
      if (debugEnabled) {
        Serial.println("✅ HTTP request sent");
      }
    } else {
      if (debugEnabled) {
        Serial.println("❌ Socket send command failed");
      }
      sendA7670CCommand("AT+CSOCL=" + String(socketId), 3000);
      return "";
    }
    
    // Step 6: Receive response
    delay(2000);  // Wait for server to respond
    
    String result = "";
    for (int attempt = 0; attempt < 5; attempt++) {
      String recvCmd = "AT+CSORECV=" + String(socketId) + ",1024";
      response = sendA7670CCommand(recvCmd, 10000);
      
      if (response.indexOf("+CSORECV:") >= 0) {
        // Parse received data
        int dataStart = response.indexOf("+CSORECV:");
        if (dataStart >= 0) {
          // Find the data after the length indicator
          int lineEnd = response.indexOf("\n", dataStart);
          if (lineEnd >= 0) {
            dataStart = lineEnd + 1;
            
            // Find the end of data
            int dataEnd = response.lastIndexOf("\nOK");
            if (dataEnd < 0) dataEnd = response.length();
            
            String httpResponse = response.substring(dataStart, dataEnd);
            
            if (debugEnabled) {
              Serial.println("📥 Raw HTTP Response:");
              Serial.println(httpResponse);
            }
            
            // Extract JSON body from HTTP response
            int bodyStart = httpResponse.indexOf("\r\n\r\n");
            if (bodyStart >= 0) {
              bodyStart += 4;
              result = httpResponse.substring(bodyStart);
              result.trim();
              
              if (result.length() > 0) {
                if (debugEnabled) {
                  Serial.println("📥 HTTP Response Body: " + result);
                }
                break;
              }
            }
          }
        }
      } else if (response.indexOf("ERROR") >= 0) {
        // No more data available
        break;
      }
      
      delay(1000);
      esp_task_wdt_reset();
    }
    
    // Step 7: Close socket
    sendA7670CCommand("AT+CSOCL=" + String(socketId), 3000);
    
    if (result.length() == 0) {
      // If we didn't get data but made a connection, return empty JSON as success
      result = "{}";
      if (debugEnabled) {
        Serial.println("📥 No response data but connection successful (using empty JSON)");
      }
    }
    
    return result;
  }
  
  // Fallback HTTP method using AT commands
  String sendHTTPRequestViaATCommands(String method, String endpoint, String data) {
    if (debugEnabled) {
      Serial.println("📡 Using AT+HTTP commands (SIMCOM A7670C method)");
    }
    
    // Close any existing HTTP connection
    sendA7670CCommand("AT+HTTPTERM", 3000);
    delay(1000);
    
    // Initialize HTTP service
    String response = sendA7670CCommand("AT+HTTPINIT", 5000);
    if (response.indexOf("OK") < 0) {
      if (debugEnabled) {
        Serial.println("❌ Failed to initialize HTTP service");
      }
      return "";
    }
    
    if (debugEnabled) {
      Serial.println("✅ HTTP service initialized");
    }
    
    // Set CID (PDP context)
    response = sendA7670CCommand("AT+HTTPPARA=\"CID\",1", 3000);
    if (response.indexOf("OK") < 0) {
      if (debugEnabled) {
        Serial.println("⚠️  Failed to set CID, continuing...");
      }
    }
    
    // Construct URL with port
    String fullUrl;
    if (port == 80) {
      fullUrl = "http://" + host + endpoint;
    } else {
      fullUrl = "http://" + host + ":" + String(port) + endpoint;
    }
    
    // Set URL with proper escaping
    String urlCmd = "AT+HTTPPARA=\"URL\",\"" + fullUrl + "\"";
    response = sendA7670CCommand(urlCmd, 5000);
    if (response.indexOf("OK") < 0) {
      if (debugEnabled) {
        Serial.println("❌ Failed to set URL: " + fullUrl);
      }
      sendA7670CCommand("AT+HTTPTERM", 3000);
      return "";
    }
    
    if (debugEnabled) {
      Serial.println("✅ HTTP URL set: " + fullUrl);
    }
    
    if (method == "POST" && data.length() > 0) {
      // Set content type
      response = sendA7670CCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 3000);
      if (debugEnabled) {
        Serial.println("Content-Type set result: " + String(response.indexOf("OK") >= 0 ? "✅" : "⚠️"));
      }
      
      // Upload data
      String dataCmd = "AT+HTTPDATA=" + String(data.length()) + ",10000";
      response = sendA7670CCommand(dataCmd, 5000);
      
      if (response.indexOf("DOWNLOAD") >= 0) {
        if (debugEnabled) {
          Serial.println("✅ Ready to upload data, sending: " + data);
        }
        
        // Send the actual data
        a7670cSerial.print(data);
        a7670cSerial.flush();
        delay(500);  // Give time for upload
        
        // Wait for upload confirmation
        unsigned long startTime = millis();
        String uploadResponse = "";
        bool uploadComplete = false;
        
        while (millis() - startTime < 10000) {
          if (a7670cSerial.available()) {
            char c = a7670cSerial.read();
            uploadResponse += c;
            
            if (uploadResponse.indexOf("OK") >= 0) {
              uploadComplete = true;
              break;
            }
          }
          delay(10);
          esp_task_wdt_reset();
        }
        
        if (debugEnabled) {
          Serial.println("Upload response: " + uploadResponse);
          Serial.println("Upload " + String(uploadComplete ? "✅ completed" : "❌ failed"));
        }
        
        if (!uploadComplete) {
          sendA7670CCommand("AT+HTTPTERM", 3000);
          return "";
        }
      } else {
        if (debugEnabled) {
          Serial.println("❌ Failed to initiate data upload");
          Serial.println("Response was: " + response);
        }
        sendA7670CCommand("AT+HTTPTERM", 3000);
        return "";
      }
    }
    
    // Execute HTTP action (0=GET, 1=POST)
    String actionCmd = "AT+HTTPACTION=" + String(method == "POST" ? "1" : "0");
    response = sendA7670CCommand(actionCmd, 5000);
    
    if (response.indexOf("OK") < 0) {
      if (debugEnabled) {
        Serial.println("❌ HTTP action command failed");
      }
      sendA7670CCommand("AT+HTTPTERM", 3000);
      return "";
    }
    
    if (debugEnabled) {
      Serial.println("✅ HTTP action initiated, waiting for response...");
    }
    
    // Wait for +HTTPACTION response with improved parsing
    unsigned long startTime = millis();
    String actionResponse = "";
    bool foundHttpAction = false;
    
    while (millis() - startTime < 30000 && !foundHttpAction) {
      if (a7670cSerial.available()) {
        char c = a7670cSerial.read();
        actionResponse += c;
        
        // Look for complete +HTTPACTION line
        if (actionResponse.indexOf("+HTTPACTION:") >= 0) {
          // Wait for complete line
          int lineStart = actionResponse.lastIndexOf("+HTTPACTION:");
          int lineEnd = actionResponse.indexOf("\n", lineStart);
          
          if (lineEnd >= 0) {
            String httpActionLine = actionResponse.substring(lineStart, lineEnd);
            httpActionLine.trim();
            
            if (debugEnabled) {
              Serial.println("📥 HTTP Action Response: " + httpActionLine);
            }
            
            foundHttpAction = true;
            
            // Parse the response: +HTTPACTION: <method>,<status_code>,<data_len>
            if (httpActionLine.indexOf(",200,") >= 0) {
              if (debugEnabled) {
                Serial.println("✅ HTTP 200 OK received");
              }
              
              // Try to read response data
              delay(300);  // Reduced from 1000ms to speed up HTTP requests
              String readResponse = sendA7670CCommand("AT+HTTPREAD", 10000);
              
              String result = "";
              if (readResponse.indexOf("+HTTPREAD:") >= 0) {
                // Find the data after +HTTPREAD: header
                int dataStart = readResponse.indexOf("+HTTPREAD:");
                if (dataStart >= 0) {
                  // Skip to the actual data (after the length indicator)
                  int lineEnd = readResponse.indexOf("\n", dataStart);
                  if (lineEnd >= 0) {
                    dataStart = lineEnd + 1;
                    
                    // Find the end of data (before "OK")
                    int dataEnd = readResponse.lastIndexOf("\nOK");
                    if (dataEnd < 0) dataEnd = readResponse.length();
                    
                    result = readResponse.substring(dataStart, dataEnd);
                    result.trim();
                    
                    // Remove any remaining control characters
                    result.replace("\r", "");
                    result.replace("\n", "");
                    
                    if (debugEnabled) {
                      Serial.println("📥 HTTP Response Data: " + result);
                    }
                  }
                }
              } else {
                // If HTTPREAD fails but we got HTTP 200, return empty JSON as success indicator
                result = "{}";
                if (debugEnabled) {
                  Serial.println("📥 HTTP 200 but no readable data (using empty JSON)");
                  Serial.println("HTTPREAD response was: " + readResponse);
                }
              }
              
              // Terminate HTTP service
              sendA7670CCommand("AT+HTTPTERM", 3000);
              return result;
              
            } else if (httpActionLine.indexOf(",") >= 0) {
              // Parse status code for debugging
              int firstComma = httpActionLine.indexOf(",");
              int secondComma = httpActionLine.indexOf(",", firstComma + 1);
              
              if (firstComma > 0 && secondComma > firstComma) {
                String statusCode = httpActionLine.substring(firstComma + 1, secondComma);
                statusCode.trim();
                
                if (debugEnabled) {
                  Serial.println("❌ HTTP request failed with status: " + statusCode);
                }
              } else {
                if (debugEnabled) {
                  Serial.println("❌ HTTP request failed - could not parse status");
                }
              }
            } else {
              if (debugEnabled) {
                Serial.println("❌ Unexpected HTTP action response format");
              }
            }
            
            break;
          }
        }
      }
      delay(10);
      esp_task_wdt_reset();
    }
    
    if (!foundHttpAction) {
      if (debugEnabled) {
        Serial.println("❌ Timeout waiting for HTTP action response");
        Serial.println("Partial response received: " + actionResponse);
      }
    }
    
    // Terminate HTTP service
    sendA7670CCommand("AT+HTTPTERM", 3000);
    return "";
  }
  
  bool isConnected() {
    return connected && a7670cConnected;
  }
  
  void disconnect() {
    connected = false;
    sendA7670CCommand("AT+QICLOSE=0", 3000);
    
    if (debugEnabled) {
      Serial.println("🔌 A7670C TCP connection closed");
    }
  }
  
  // Primary HTTP-only method - most reliable for A7670C
  String sendHTTPRequestHTTPOnly(String method, String endpoint, String data) {
    if (debugEnabled) {
      Serial.println("\n📡 Using HTTP-only method (recommended for A7670C-LNNV)");
      Serial.println("==== REQUEST DETAILS ====");
      Serial.println("Method: " + method);
      Serial.println("Endpoint: " + endpoint);
      Serial.println("Data: " + data);
      Serial.println("======================\n");
    }
    
    // Step 1: Clean up any existing HTTP session
    sendA7670CCommand("AT+HTTPTERM", 3000);
    delay(300); // Reduced from 1000ms to speed up HTTP session setup
    esp_task_wdt_reset();
    
    // Step 2: Initialize HTTP service
    String response = sendA7670CCommand("AT+HTTPINIT", 5000);
    if (response.indexOf("OK") < 0) {
      if (debugEnabled) {
        Serial.println("❌ AT+HTTPINIT failed");
      }
      return "";
    }
    if (debugEnabled) {
      Serial.println("✅ AT+HTTPINIT success");
    }
    
    // Step 3: Set CID (PDP context)
    response = sendA7670CCommand("AT+HTTPPARA=\"CID\",1", 3000);
    if (response.indexOf("OK") < 0) {
      if (debugEnabled) {
        Serial.println("⚠️  AT+HTTPPARA CID failed, continuing...");
      }
    } else {
      if (debugEnabled) {
        Serial.println("✅ AT+HTTPPARA CID success");
      }
    }
    
    // Step 4: Set URL
    String fullUrl = "http://" + host + ":" + String(port) + endpoint;
    String urlCmd = "AT+HTTPPARA=\"URL\",\"" + fullUrl + "\"";
    response = sendA7670CCommand(urlCmd, 5000);
    if (response.indexOf("OK") < 0) {
      if (debugEnabled) {
        Serial.println("❌ AT+HTTPPARA URL failed: " + fullUrl);
      }
      sendA7670CCommand("AT+HTTPTERM", 3000);
      return "";
    }
    if (debugEnabled) {
      Serial.println("✅ AT+HTTPPARA URL success: " + fullUrl);
    }
    
    // Step 5: Handle POST data if needed
    if (method == "POST" && data.length() > 0) {
      // Set content type
      response = sendA7670CCommand("AT+HTTPPARA=\"CONTENT\",\"application/json\"", 3000);
      if (debugEnabled) {
        Serial.println("Content-Type result: " + String(response.indexOf("OK") >= 0 ? "✅ Success" : "⚠️  Warning"));
      }
      
      // Upload data
      String dataCmd = "AT+HTTPDATA=" + String(data.length()) + ",10000";
      response = sendA7670CCommand(dataCmd, 5000);
      
      if (response.indexOf("DOWNLOAD") >= 0) {
        if (debugEnabled) {
          Serial.println("✅ DOWNLOAD prompt received, sending JSON data:");
          Serial.println("Data length: " + String(data.length()));
          Serial.println("Data content: " + data);
        }
        
        // Send the JSON data
        a7670cSerial.print(data);
        a7670cSerial.flush();
        
        // Wait for upload confirmation with proper timeout
        unsigned long uploadStart = millis();
        String uploadResponse = "";
        bool uploadSuccess = false;
        
        while (millis() - uploadStart < 15000) { // 15 second timeout
          if (a7670cSerial.available()) {
            char c = a7670cSerial.read();
            uploadResponse += c;
            
            if (uploadResponse.indexOf("OK") >= 0) {
              uploadSuccess = true;
              break;
            }
            if (uploadResponse.indexOf("ERROR") >= 0) {
              break;
            }
          }
          delay(10);
          esp_task_wdt_reset();
        }
        
        if (debugEnabled) {
          Serial.println("\n==== UPLOAD DETAILS ====");
          Serial.println("Upload response: " + uploadResponse);
          Serial.println("Upload status: " + String(uploadSuccess ? "✅ Success" : "❌ Failed"));
          Serial.println("Upload duration: " + String((millis() - uploadStart)) + "ms");
          Serial.println("=====================\n");
        }
        
        if (!uploadSuccess) {
          sendA7670CCommand("AT+HTTPTERM", 3000);
          return "";
        }
      } else {
        if (debugEnabled) {
          Serial.println("❌ DOWNLOAD prompt not received");
          Serial.println("Response was: " + response);
        }
        sendA7670CCommand("AT+HTTPTERM", 3000);
        return "";
      }
    }
    
    // Step 6: Execute HTTP action
    String actionCmd = "AT+HTTPACTION=" + String(method == "POST" ? "1" : "0");
    response = sendA7670CCommand(actionCmd, 5000);
    
    if (response.indexOf("OK") < 0) {
      if (debugEnabled) {
        Serial.println("❌ AT+HTTPACTION command failed");
      }
      sendA7670CCommand("AT+HTTPTERM", 3000);
      return "";
    }
    
    if (debugEnabled) {
      Serial.println("✅ AT+HTTPACTION initiated, waiting for response...");
    }
    
    // Step 7: Wait for +HTTPACTION response
    unsigned long actionStart = millis();
    String actionResponse = "";
    bool actionComplete = false;
    int statusCode = 0;
    int dataLength = 0;
    String capturedData = "";
    
    while (millis() - actionStart < 30000 && !actionComplete) {
      if (a7670cSerial.available()) {
        char c = a7670cSerial.read();
        actionResponse += c;
        capturedData += c;  // Capture all data

        // Look for complete +HTTPACTION line
        if (actionResponse.indexOf("+HTTPACTION:") >= 0) {
          int lineStart = actionResponse.lastIndexOf("+HTTPACTION:");
          int lineEnd = actionResponse.indexOf("\n", lineStart);
          
          if (lineEnd >= 0) {
            String httpActionLine = actionResponse.substring(lineStart, lineEnd);
            httpActionLine.trim();
            
            if (debugEnabled) {
              Serial.println("\n==== HTTP ACTION RESPONSE ====");
              Serial.println("Raw response: " + httpActionLine);
            }
            
            // Parse: +HTTPACTION: <method>,<status_code>,<data_len>
            int firstComma = httpActionLine.indexOf(",");
            int secondComma = httpActionLine.indexOf(",", firstComma + 1);
            
            if (firstComma > 0 && secondComma > firstComma) {
              statusCode = httpActionLine.substring(firstComma + 1, secondComma).toInt();
              dataLength = httpActionLine.substring(secondComma + 1).toInt();
              actionComplete = true;
              
              if (debugEnabled) {
                Serial.println("Status Code: " + String(statusCode));
                Serial.println("Data Length: " + String(dataLength));
                Serial.println("============================\n");
              }
            }
            
            break;
          }
        }
      }
      delay(10);
      esp_task_wdt_reset();
    }
    
    if (!actionComplete) {
      if (debugEnabled) {
        Serial.println("\n❌ HTTP Action timeout");
        Serial.println("==== TIMEOUT DETAILS ====");
        Serial.println("Duration: " + String(millis() - actionStart) + "ms");
        Serial.println("Partial response: " + actionResponse);
        Serial.println("Raw captured data: " + capturedData);
        Serial.println("======================\n");
      }
      sendA7670CCommand("AT+HTTPTERM", 3000);
      return "";
    }
    
    // Step 8: Read response data if status is 200
    String result = "";
    if (statusCode == 200) {
      if (debugEnabled) {
        Serial.println("\n==== READING HTTP RESPONSE ====");
        Serial.println("Expected length: " + String(dataLength) + " bytes");
        Serial.println("Status code is 200, proceeding to read data...");
      }
      
      if (dataLength > 0) {
        if (debugEnabled) {
          Serial.println("Data length > 0, starting optimized AT+HTTPREAD...");
          Serial.println("🎯 Using optimized single-read with improved buffer management");
        }
        
        // Use optimized single read method (chunked reading not supported by A7670C)
        // Send command and capture ALL data that follows
        while (a7670cSerial.available()) a7670cSerial.read(); // Clear buffer
        String readWithLength = "AT+HTTPREAD=0," + String(dataLength);
        a7670cSerial.println(readWithLength);
        
        // Wait for command acknowledgment
        String readResponse = "";
        unsigned long cmdStart = millis();
        while (millis() - cmdStart < 3000) {
          if (a7670cSerial.available()) {
            char c = a7670cSerial.read();
            readResponse += c;
            if (readResponse.indexOf("OK") >= 0) break;
          }
          delay(10);
          esp_task_wdt_reset();
        }
        
        // Read data stream with improved timing and buffer management
        delay(300); // Reduced from 800ms to speed up HTTP requests
        String dataStream = "";
        unsigned long dataStart = millis();
        
        while (millis() - dataStart < 12000) { // Extended timeout
          while (a7670cSerial.available()) {
            char c = a7670cSerial.read();
            dataStream += c;
          }
          
          // Check for completion with multiple criteria
          if (dataStream.indexOf("+HTTPREAD:") >= 0) {
            // Look for end markers
            if (dataStream.indexOf("OK") >= 0 || 
                dataStream.indexOf("+HTTPREAD: 0") >= 0 ||
                dataStream.length() > dataLength + 50) {
              break;
            }
          }
          
          delay(10);
          esp_task_wdt_reset();
        }
        
        String fullResponse = readResponse + dataStream;
        
        if (debugEnabled) {
          Serial.println("📥 Command response: " + readResponse);
          Serial.println("📥 Data stream (" + String(dataStream.length()) + " bytes): " + dataStream.substring(0, min(200, (int)dataStream.length())));
          Serial.println("📥 Full response (" + String(fullResponse.length()) + " bytes): " + fullResponse.substring(0, min(300, (int)fullResponse.length())));
        }
        
        // Enhanced JSON extraction with better parsing
        if (fullResponse.indexOf("+HTTPREAD:") >= 0) {
          int headerPos = fullResponse.indexOf("+HTTPREAD:");
          int dataStart = fullResponse.indexOf("\n", headerPos);
          if (dataStart >= 0) {
            dataStart++; // Move past newline
            
            if (debugEnabled) {
              Serial.println("🔍 Header found at position: " + String(headerPos));
              Serial.println("🔍 Data starts at position: " + String(dataStart));
              Serial.println("🔍 Available data from position: " + String(fullResponse.length() - dataStart) + " bytes");
              Serial.println("🔍 Raw data preview: '" + fullResponse.substring(dataStart, min((int)fullResponse.length(), dataStart + 100)) + "'");
            }
            
            int jsonStart = fullResponse.indexOf("{", dataStart);
            if (jsonStart >= 0) {
              if (debugEnabled) {
                Serial.println("🔍 JSON starts at position: " + String(jsonStart));
              }
              
              // Find end of JSON with improved logic
              int jsonEnd = fullResponse.length();
              
              // Look for clean end markers first
              int okPos = fullResponse.indexOf("\nOK", jsonStart);
              int httpReadZero = fullResponse.indexOf("+HTTPREAD: 0", jsonStart);
              
              if (okPos > jsonStart && okPos < jsonEnd) {
                jsonEnd = okPos;
              }
              if (httpReadZero > jsonStart && httpReadZero < jsonEnd) {
                jsonEnd = httpReadZero;
              }
              
              // Extract and clean the JSON
              result = fullResponse.substring(jsonStart, jsonEnd);
              result.trim();
              result.replace("\r", "");
              result.replace("\n", "");
              
              // Remove any trailing artifacts
              while (result.endsWith("+HTTPREAD") || result.endsWith("+HTTP") || result.endsWith("+")) {
                result = result.substring(0, result.length() - 1);
                result.trim();
              }
              
              if (debugEnabled) {
                Serial.println("✅ OPTIMIZED JSON extraction!");
                Serial.println("📦 JSON data: " + result);
                Serial.println("📊 JSON length: " + String(result.length()));
                Serial.println("📊 Expected length was: " + String(dataLength));
              }
            } else {
              if (debugEnabled) {
                Serial.println("❌ Could not find JSON opening brace");
              }
            }
          } else {
            if (debugEnabled) {
              Serial.println("❌ Could not find newline after +HTTPREAD header");
            }
          }
        } else {
          if (debugEnabled) {
            Serial.println("❌ No +HTTPREAD header found in response");
          }
        }
      }
    } else {
      if (debugEnabled) {
        Serial.println("❌ HTTP request failed with status: " + String(statusCode));
      }
    }
    
    // Step 9: Terminate HTTP session
    sendA7670CCommand("AT+HTTPTERM", 3000);
    
    if (debugEnabled) {
      Serial.println("🏁 HTTP session completed\n");
      Serial.println("FINAL RESULT being returned:");
      Serial.println("Result length: " + String(result.length()));
      Serial.println("Result content: '" + result + "'");
    }
    
    return result;
  }

  // Add this new function right before extractHTTPResponseInChunks
  String extractHTTPResponseFast(int totalLength) {
    Serial.println("⚡ Fast HTTP response extraction (Length: " + String(totalLength) + ")");
    
    String fullResponse = "";
    int chunkSize = min(200, totalLength); // Use larger chunks, up to 200 bytes
    int position = 0;
    
    while (position < totalLength) {
      int remainingBytes = totalLength - position;
      int currentChunkSize = min(chunkSize, remainingBytes);
      
      String command = "AT+HTTPREAD=" + String(position) + "," + String(currentChunkSize);
      
      // Send command and immediately start reading streaming data
      while (a7670cSerial.available()) a7670cSerial.read(); // Clear buffer
      a7670cSerial.println(command);
      
      // Quick read of the OK response
      String cmdResponse = "";
      unsigned long cmdStart = millis();
      while (millis() - cmdStart < 1000) {
        if (a7670cSerial.available()) {
          char c = a7670cSerial.read();
          cmdResponse += c;
          if (cmdResponse.indexOf("OK") >= 0) break;
        }
        delay(5);
        esp_task_wdt_reset();
      }
      
      // Now read the streaming data that follows
      delay(100); // Brief pause for data to arrive
      String streamingData = "";
      unsigned long streamStart = millis();
      while (millis() - streamStart < 800) {
        if (a7670cSerial.available()) {
          char c = a7670cSerial.read();
          streamingData += c;
        }
        delay(5);
        esp_task_wdt_reset();
      }
      
      // Extract the actual data from between +HTTPREAD: lines
      if (streamingData.indexOf("+HTTPREAD:") >= 0) {
        int firstRead = streamingData.indexOf("+HTTPREAD:");
        int secondRead = streamingData.indexOf("+HTTPREAD:", firstRead + 1);
        
        if (firstRead >= 0 && secondRead >= 0) {
          int dataStart = streamingData.indexOf("\n", firstRead) + 1;
          int dataEnd = secondRead;
          
          if (dataStart > 0 && dataEnd > dataStart) {
            String chunkData = streamingData.substring(dataStart, dataEnd);
            chunkData.trim();
            fullResponse += chunkData;
          }
        }
      }
      
      position += currentChunkSize;
      delay(20); // Minimal delay between chunks
    }
    
    // Clean up the response
    fullResponse.replace("\r", "");
    fullResponse.replace("\n", "");
    fullResponse.trim();
    
    if (debugEnabled) {
      Serial.println("⚡ Fast extraction result: '" + fullResponse.substring(0, min(100, (int)fullResponse.length())) + (fullResponse.length() > 100 ? "..." : "") + "'");
    }
    
    return fullResponse;
  }

  String extractHTTPResponseInChunks(int totalLength) {
    Serial.println("📥 Extracting HTTP response in chunks (Length: " + String(totalLength) + ")");
    
    String fullResponse = "";
    int chunkSize = 50;  // Read in 50-byte chunks
    int position = 0;
    
    while (position < totalLength) {
      int remainingBytes = totalLength - position;
      int currentChunkSize = min(chunkSize, remainingBytes);
      
      String command = "AT+HTTPREAD=" + String(position) + "," + String(currentChunkSize);
      
      String response = sendA7670CCommand(command, 5000);
      
      if (debugEnabled) {
        Serial.println("🔍 Raw chunk response for " + command + ":");
        Serial.println("'" + response + "'");
      }
      
      if (response.indexOf("+HTTPREAD:") >= 0) {
        // Look for the actual data between +HTTPREAD: lines
        int firstReadPos = response.indexOf("+HTTPREAD:");
        int secondReadPos = response.indexOf("+HTTPREAD:", firstReadPos + 1);
        
        if (firstReadPos >= 0 && secondReadPos >= 0) {
          // Extract data between the two +HTTPREAD: lines
          int dataStart = response.indexOf("\n", firstReadPos) + 1;
          int dataEnd = response.indexOf("+HTTPREAD:", dataStart);
          
          if (dataStart > 0 && dataEnd > dataStart) {
            String chunkData = response.substring(dataStart, dataEnd);
            chunkData.trim();
            fullResponse += chunkData;
            Serial.println("📦 Chunk " + String(position) + "-" + String(position + currentChunkSize) + ": '" + chunkData + "'");
          }
        } else {
          // Try alternative parsing - look for data after first +HTTPREAD: line
          int dataStart = response.indexOf("\n", firstReadPos);
          if (dataStart >= 0) {
            String remainingData = response.substring(dataStart + 1);
            // Remove any OK or +HTTPREAD: 0 at the end
            remainingData.replace("+HTTPREAD: 0", "");
            remainingData.replace("OK", "");
            remainingData.trim();
            
            if (remainingData.length() > 0) {
              fullResponse += remainingData;
              Serial.println("📦 Chunk " + String(position) + "-" + String(position + currentChunkSize) + ": '" + remainingData + "'");
            }
          }
        }
      } else {
        // Try to read any data that might be streaming after the command
        delay(200);
        
        String streamingData = "";
        unsigned long streamStart = millis();
        while (millis() - streamStart < 1000) {
          if (a7670cSerial.available()) {
            char c = a7670cSerial.read();
            streamingData += c;
          }
          delay(10);
          esp_task_wdt_reset();
        }
        
        if (streamingData.length() > 0) {
          // Clean up the streaming data - extract only the actual content
          String cleanData = streamingData;
          
          // Remove +HTTPREAD: headers and footers
          cleanData.replace("+HTTPREAD: " + String(currentChunkSize), "");
          cleanData.replace("+HTTPREAD: 0", "");
          cleanData.replace("\r", "");
          cleanData.replace("\n", "");
          cleanData.trim();
          
          if (cleanData.length() > 0) {
            fullResponse += cleanData;
            Serial.println("📦 Cleaned chunk " + String(position) + ": '" + cleanData + "'");
          }
        }
      }
      
      position += currentChunkSize;
      delay(50); // Reduced delay to speed up chunking
    }
    
    // Clean up the final response to extract pure JSON
    String cleanedResponse = fullResponse;
    
    // Remove all +HTTPREAD control lines
    int readPos = 0;
    while ((readPos = cleanedResponse.indexOf("+HTTPREAD:")) >= 0) {
      int lineEnd = cleanedResponse.indexOf("\n", readPos);
      if (lineEnd < 0) lineEnd = cleanedResponse.length();
      cleanedResponse = cleanedResponse.substring(0, readPos) + cleanedResponse.substring(lineEnd + 1);
    }
    
    // Remove other control characters
    cleanedResponse.replace("\r", "");
    cleanedResponse.replace("\n", "");
    cleanedResponse.trim();
    
    Serial.println("🎯 FINAL COMBINED CHUNKED DATA:");
    Serial.println("================================");
    Serial.println("Raw Length: " + String(fullResponse.length()));
    Serial.println("Cleaned Length: " + String(cleanedResponse.length()));
    Serial.println("Cleaned Data: '" + cleanedResponse + "'");
    Serial.println("================================");
    
    return cleanedResponse;
  }
};


A7670CWebSocketClient a7670cWebSocket;


String extractA7670CData(String response) {
  String data = response;
  
  // Remove command echo
  int newlinePos = data.indexOf('\n');
  if (newlinePos >= 0) {
    data = data.substring(newlinePos + 1);
  }
  
  // Remove OK/ERROR
  data.replace("OK", "");
  data.replace("ERROR", "");
  data.replace("\r", "");
  data.replace("\n", " ");
  data.trim();
  
  return data;
}

bool checkA7670CStatus() {
  if (debugEnabled) {
    Serial.println("🔍 Checking A7670C module status...");
  }
  
  String response = sendA7670CCommand("AT", A7670C_SLOW_MODE ? A7670C_DEFAULT_CMD_TIMEOUT : 5000);
  if (response.indexOf("OK") < 0) {
    if (debugEnabled) {
      Serial.println("❌ A7670C module not responding to AT command");
      Serial.println("🔄 Attempting AT+CRESET to recover module...");
    }
    
    // Try AT+CRESET to recover
    while (a7670cSerial.available()) {
      a7670cSerial.read();
    }
    
    a7670cSerial.println("AT+CRESET");
    a7670cSerial.flush();
    
    if (debugEnabled) {
      Serial.println("🔄 AT+CRESET sent, waiting for module restart...");
    }
    
    // Wait for hardware reset to complete
    delay(A7670C_SLOW_MODE ? 25000 : 15000); // longer for stripped-down modules
    esp_task_wdt_reset();
    
    // Clear buffer after reset
    while (a7670cSerial.available()) {
      a7670cSerial.read();
    }
    
    // Test again after reset
    response = sendA7670CCommand("AT", A7670C_SLOW_MODE ? A7670C_DEFAULT_CMD_TIMEOUT : 5000);
    if (response.indexOf("OK") < 0) {
      if (debugEnabled) {
        Serial.println("❌ A7670C module still not responding after AT+CRESET");
      }
      return false;
    } else {
      if (debugEnabled) {
        Serial.println("✅ A7670C module recovered after AT+CRESET");
      }
    }
  }
  
  if (debugEnabled) {
    Serial.println("✅ A7670C module responding");
  }
  
  // Check SIM card
  response = sendA7670CCommand("AT+CPIN?");
  if (response.indexOf("READY") >= 0) {
    if (debugEnabled) {
      Serial.println("✅ SIM card ready");
    }
  } else {
    if (debugEnabled) {
      Serial.println("⚠️  SIM card status: " + extractA7670CData(response));
    }
  }
  
  // Check signal strength
  response = sendA7670CCommand("AT+CSQ");
  if (response.indexOf("+CSQ:") >= 0) {
    String csqData = extractA7670CData(response);
    // Find the signal strength value after "+CSQ: "
    int startPos = csqData.indexOf("+CSQ:");
    if (startPos >= 0) {
      startPos += 6; // Skip "+CSQ: "
      String strengthStr = csqData.substring(startPos);
      int commaPos = strengthStr.indexOf(",");
      if (commaPos > 0) {
        int signalStrength = strengthStr.substring(0, commaPos).toInt();
        if (debugEnabled) {
          if (signalStrength != 99) {
            Serial.println("📡 Signal strength: " + String(signalStrength) + "/31");
          } else {
            Serial.println("📡 No signal detected");
          }
        }
      }
    }
  }
  
  return true;
}

bool selectCelcomNetwork() {
  if (debugEnabled) {
    Serial.println("🎯 Selecting Celcom network...");
  }
  
  // First, check current network
  String response = sendA7670CCommand("AT+COPS?");
  if (response.indexOf(CELCOM_MCC_MNC) >= 0) {
    if (debugEnabled) {

    }
    networkSelected = true;
    return true;
  }
  
  // Set to manual network selection
  if (debugEnabled) {
    Serial.println("🔄 Setting manual network selection mode...");
  }
  response = sendA7670CCommand("AT+COPS=1,2,\"" + CELCOM_MCC_MNC + "\",7", 60000);
  
  if (response.indexOf("OK") >= 0) {
    if (debugEnabled) {
      Serial.println("✅ Network selection command sent");
      Serial.println("⏳ Waiting for network registration...");
    }
    
    delay(10000);
    esp_task_wdt_reset();
    
    // Check registration status
    for (int attempt = 1; attempt <= 6; attempt++) {
      if (debugEnabled) {
        Serial.println("  Checking attempt " + String(attempt) + "/6...");
      }
      
      response = sendA7670CCommand("AT+COPS?");
      if (response.indexOf(CELCOM_MCC_MNC) >= 0) {
        if (debugEnabled) {
          Serial.println("✅ Successfully registered to Celcom network");
        }
        networkSelected = true;
        return true;
      }
      
      delay(10000);
      esp_task_wdt_reset();
    }
    
    if (debugEnabled) {
      Serial.println("❌ Failed to register to Celcom network");
    }
    return false;
  } else {
    if (debugEnabled) {
      Serial.println("❌ Network selection command failed");
    }
    return false;
  }
}

// New function for automatic network selection
bool selectNetworkAutomatic() {
  if (debugEnabled) {
    Serial.println("🎯 Using automatic network selection...");
  }
  
  // First, check if already registered to any network
  String response = sendA7670CCommand("AT+COPS?");
  if (response.indexOf("+COPS: 0") >= 0 && response.indexOf(",") >= 0) {
    if (debugEnabled) {
      Serial.println("✅ Already registered to a network automatically");
      // Extract and display the network name
      int nameStart = response.indexOf("\"");
      if (nameStart >= 0) {
        int nameEnd = response.indexOf("\"", nameStart + 1);
        if (nameEnd > nameStart) {
          String networkName = response.substring(nameStart + 1, nameEnd);
          Serial.println("📱 Connected to: " + networkName);
        }
      }
    }
    networkSelected = true;
    return true;
  }
  
  // Set to automatic network selection mode
  if (debugEnabled) {
    Serial.println("🔄 Setting automatic network selection mode...");
  }
  response = sendA7670CCommand("AT+COPS=0", A7670C_SLOW_MODE ? A7670C_LONG_CMD_TIMEOUT : 60000);  // 0 = automatic mode
  
  if (response.indexOf("OK") >= 0) {
    if (debugEnabled) {
      Serial.println("✅ Automatic network selection enabled");
      Serial.println("⏳ Waiting for automatic network registration...");
    }
    
    delay(A7670C_SLOW_MODE ? 30000 : 15000);  // Give more time for automatic registration
    esp_task_wdt_reset();
    
    // Check registration status
    int maxAttempts = A7670C_SLOW_MODE ? 16 : 10;
    unsigned long waitBetween = A7670C_SLOW_MODE ? 7000 : 5000;
    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
      if (debugEnabled) {
        Serial.println("  Checking attempt " + String(attempt) + "/" + String(maxAttempts) + "...");
      }
      
      // Check network registration
      response = sendA7670CCommand("AT+CREG?");
      if (response.indexOf("+CREG: 0,1") >= 0 || response.indexOf("+CREG: 0,5") >= 0) {
        // Registered to home network (1) or roaming (5)
        if (debugEnabled) {
          Serial.println("✅ Registered to network automatically");
        }
        
        // Get the network information
        response = sendA7670CCommand("AT+COPS?");
        if (response.indexOf("+COPS:") >= 0) {
          // Extract network name for display
          int nameStart = response.indexOf("\"");
          if (nameStart >= 0) {
            int nameEnd = response.indexOf("\"", nameStart + 1);
            if (nameEnd > nameStart) {
              String networkName = response.substring(nameStart + 1, nameEnd);
              if (debugEnabled) {
                Serial.println("📱 Connected to: " + networkName);
              }
            }
          }
        }
        
        networkSelected = true;
        return true;
      }
      
      delay(waitBetween);
      esp_task_wdt_reset();
    }
    
    if (debugEnabled) {
      Serial.println("❌ Failed to register to any network automatically");
    }
    return false;
  } else {
    if (debugEnabled) {
      Serial.println("❌ Automatic network selection command failed");
    }
    return false;
  }
}

bool configureA7670CAPN() {
  if (debugEnabled) {
    Serial.println("🔗 Configuring APN settings...");
  }
  
  // Configure PDP context
  String apnCmd = "AT+CGDCONT=1,\"IP\",\"" + ACTIVE_APN + "\"";
  String response = sendA7670CCommand(apnCmd);
  
  if (response.indexOf("OK") >= 0) {
    if (debugEnabled) {
      Serial.println("✅ APN configured: " + ACTIVE_APN);
    }
    return true;
  } else {
    if (debugEnabled) {
      Serial.println("❌ Failed to configure APN");
    }
    return false;
  }
}

// New function for automatic APN configuration based on detected network
bool configureAPNAutomatic() {
  if (debugEnabled) {
    Serial.println("🔗 Auto-configuring APN based on detected network...");
  }
  
  // Get current network information
  String response = sendA7670CCommand("AT+COPS?");
  String detectedAPN = "internet";  // Default fallback APN
  String networkName = "";
  
  // Extract network name
  int nameStart = response.indexOf("\"");
  if (nameStart >= 0) {
    int nameEnd = response.indexOf("\"", nameStart + 1);
    if (nameEnd > nameStart) {
      networkName = response.substring(nameStart + 1, nameEnd);
      if (debugEnabled) {
        Serial.println("📱 Detected network: " + networkName);
      }
    }
  }
  
  // Determine APN based on network name or MCC+MNC
  if (response.indexOf("50219") >= 0 || networkName.indexOf("Celcom") >= 0) {
    detectedAPN = "celcom3g";
    if (debugEnabled) Serial.println("🎯 Detected: Celcom Malaysia");
  }
  else if (response.indexOf("50216") >= 0 || networkName.indexOf("DiGi") >= 0 || networkName.indexOf("DIGI") >= 0) {
    detectedAPN = "diginet";
    if (debugEnabled) Serial.println("🎯 Detected: Digi Malaysia");
  }
  else if (response.indexOf("50212") >= 0 || networkName.indexOf("Maxis") >= 0) {
    detectedAPN = "internet";
    if (debugEnabled) Serial.println("🎯 Detected: Maxis Malaysia");
  }
  else if (response.indexOf("50213") >= 0 || networkName.indexOf("TM") >= 0) {
    detectedAPN = "internet";
    if (debugEnabled) Serial.println("🎯 Detected: TM/unifi Malaysia");
  }
  else if (response.indexOf("50218") >= 0 || networkName.indexOf("U Mobile") >= 0) {
    detectedAPN = "my3g";
    if (debugEnabled) Serial.println("🎯 Detected: U Mobile Malaysia");
  }
  else {
    if (debugEnabled) {
      Serial.println("🎯 Unknown network, using default APN: " + detectedAPN);
      Serial.println("💡 You may need to check with your carrier for the correct APN");
    }
  }
  
  // Configure the detected APN
  String apnCmd = "AT+CGDCONT=1,\"IP\",\"" + detectedAPN + "\"";
  response = sendA7670CCommand(apnCmd);
  
  if (response.indexOf("OK") >= 0) {
    if (debugEnabled) {
      Serial.println("✅ APN auto-configured: " + detectedAPN);
    }
    return true;
  } else {
    if (debugEnabled) {
      Serial.println("❌ APN auto-configuration failed");
      Serial.println("💡 Trying common APNs as fallback...");
    }
    
    // Try common APNs as fallback
    String commonAPNs[] = {"internet", "data", "3gnet", "web"};
    for (int i = 0; i < 4; i++) {
      String fallbackCmd = "AT+CGDCONT=1,\"IP\",\"" + commonAPNs[i] + "\"";
      response = sendA7670CCommand(fallbackCmd);
      if (response.indexOf("OK") >= 0) {
        if (debugEnabled) {
          Serial.println("✅ Fallback APN configured: " + commonAPNs[i]);
        }
        return true;
      }
    }
    
    if (debugEnabled) {
      Serial.println("❌ All APN configuration attempts failed");
    }
    return false;
  }
}

bool connectA7670CToInternet() {
  if (debugEnabled) {
    Serial.println("🌐 Connecting to internet via A7670C...");
  }
  
  // Activate PDP context
  String response = sendA7670CCommand("AT+CGACT=1,1", 30000);
  
  if (response.indexOf("OK") >= 0) {
    if (debugEnabled) {
      Serial.println("✅ PDP context activated");
    }
  } else {
    if (debugEnabled) {
      Serial.println("❌ Failed to activate PDP context");
    }
    return false;
  }
  
  // Check if we got an IP address
  delay(5000);
  esp_task_wdt_reset();
  response = sendA7670CCommand("AT+CGPADDR=1");
  
  if (response.indexOf("+CGPADDR:") >= 0) {
    String ipData = extractA7670CData(response);
    int commaPos = ipData.indexOf(",");
    if (commaPos > 0) {
      String localIP = ipData.substring(commaPos + 1);
      localIP.replace("\"", "");
      localIP.trim();
      
      if (localIP.length() > 7 && localIP != "0.0.0.0") {
        if (debugEnabled) {
          Serial.println("✅ Internet connected! Local IP: " + localIP);
        }
        internetConnected = true;
        a7670cConnected = true;
        return true;
      }
    }
  }
  
  if (debugEnabled) {
    Serial.println("❌ No IP address assigned");
  }
  return false;
}

bool initializeA7670C() {
  if (debugEnabled) {
    Serial.println("\n=== Initializing A7670C Module ===");
  }
  
  // Initialize UART without HW flow first so AT passes even if modem still IFC=0,0
  initializeA7670CSerial();
  delay(2000);
  esp_task_wdt_reset();
  
  if (debugEnabled) {
    Serial.println("🎯 Target Network MCC/MNC: " + ACTIVE_MCC_MNC);
    Serial.println("📡 Technology: E-UTRAN (LTE)");
    Serial.println("🔗 APN: " + ACTIVE_APN);
    Serial.println("📡 UART1 RX buffer 2048; no hardware flow control");
  }
  
  // Step 1: Check module status
  if (!checkA7670CStatus()) {
    if (debugEnabled) {
      Serial.println("❌ Module not responding. Check connections.");
    }
    return false;
  }
  
  // Step 2: Select network based on toggle
  if (!(A7670C_AUTO_OPERATOR_SELECT ? selectNetworkAutomatic() : selectNetworkManual())) {
    return false;
  }
  
  // Step 3: Configure APN
  if (!configureA7670CAPN()) {
    return false;
  }
  
  // Step 4: Connect to internet
  if (!connectA7670CToInternet()) {
    return false;
  }
  
  if (debugEnabled) {
    Serial.println("✅ A7670C initialization completed successfully");
  }
  
  return true;
}

// New function for automatic A7670C initialization without hardcoded network info
bool initializeA7670CAutomatic() {
  if (debugEnabled) {
    Serial.println("\n=== Initializing A7670C Module (Automatic Mode) ===");
  }
  
  // Initialize UART without HW flow first so AT passes even if modem still IFC=0,0
  initializeA7670CSerial();
  delay(2000);
  esp_task_wdt_reset();
  
  if (debugEnabled) {
    Serial.println("🎯 Target: Auto-detect available network");
    Serial.println("📡 Technology: E-UTRAN (LTE)");
    Serial.println("🔗 APN: Auto-detect based on network");
    Serial.println("📡 UART1 RX buffer 2048; no hardware flow control");
  }
  
  // Step 1: Check module status
  if (!checkA7670CStatus()) {
    if (debugEnabled) {
      Serial.println("❌ Module not responding. Check connections.");
    }
    return false;
  }
  
  // Step 2: Use automatic network selection
  if (!selectNetworkAutomatic()) {
    if (debugEnabled) {
      Serial.println("❌ Automatic network selection failed. Trying manual scan...");
    }
    
    // Optional: Show available networks for debugging
    String response = sendA7670CCommand("AT+COPS?", 120000);
    if (response.indexOf("+COPS:") >= 0 && debugEnabled) {
      Serial.println("📡 Available networks:");
      Serial.println(response);
    }
    
    return false;
  }
  
  // Step 3: Auto-configure APN based on detected network
  if (!configureAPNAutomatic()) {
    return false;
  }
  
  // Step 4: Connect to internet
  if (!connectA7670CToInternet()) {
    return false;
  }
  
  if (debugEnabled) {
    Serial.println("✅ A7670C automatic initialization completed successfully");
  }
  
  return true;
}

void monitorA7670CConnection() {
  // Check network registration
  String response = sendA7670CCommand("AT+COPS?");
  if (A7670C_AUTO_OPERATOR_SELECT) {
    // Auto mode: report current operator if available
    if (debugEnabled) {
      int nameStart = response.indexOf('"');
      if (nameStart >= 0) {
        int nameEnd = response.indexOf('"', nameStart + 1);
        String netName = (nameEnd > nameStart) ? response.substring(nameStart + 1, nameEnd) : String("");
        Serial.println("📶 Network (auto): " + (netName.length() ? (String("✅ ") + netName) : String("✅ Registered")));
      }
    }
  } else {
    // Manual mode: enforce ACTIVE_MCC_MNC match
    if (response.indexOf(ACTIVE_MCC_MNC) >= 0) {
      if (debugEnabled) {
        Serial.println("📶 Network: ✅ Active (" + ACTIVE_MCC_MNC + ")");
      }
    } else {
      if (debugEnabled) {
        Serial.println("📶 Network: ❌ Not connected to active MCC/MNC");
      }
      a7670cConnected = false;
    }
  }
  
  // Check internet connection
  response = sendA7670CCommand("AT+CGPADDR=1");
  if (response.indexOf("+CGPADDR:") >= 0) {
    if (debugEnabled) {
      Serial.println("🌐 Internet: ✅ Connected");
    }
    a7670cConnected = true;
  } else {
    if (debugEnabled) {
      Serial.println("🌐 Internet: ❌ Disconnected");
    }
    a7670cConnected = false;
  }
}





void sendWebSocketMessage(String joinRef, String msgRef, String topic, String event, String payload) {
  String message = "[\"" + joinRef + "\",\"" + msgRef + "\",\"" + topic + "\",\"" + event + "\"," + payload + "]";
  
  if (SKIP_WIFI && a7670cWebSocket.isConnected()) {
    a7670cWebSocket.sendTXT(message);
  } else if (!SKIP_WIFI) {
    webSocket.sendTXT(message);
  }
  
  // Serial.println("Sent WebSocket message: " + message);
}


void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
  // Serial.println(type);
  String userId = macToUUID();
  String topic = "user:" + userId;
  StaticJsonDocument<256> responseDoc;
  responseDoc["user_id"] = userId;
  String payload2;
  serializeJson(responseDoc, payload2);
  
  switch (type) {
    case WStype_CONNECTED:
      if (debugEnabled) {
        Serial.println("WebSocket connected!");
      }
      webSocketConnected = true;
      connectionStartTime = millis();  // Set connection start time
      lastReceivedPingTime = millis();  // Reset ping timeout counter
      lastPingTime = millis();  // Reset ping timer
      
      if (wasDisconnected) {
        roomRef = roomRef + 1;
        wasDisconnected = false;
      }
      sendWebSocketMessage(String(roomRef), String(roomRef), topic, "phx_join", payload2);
      getDeviceSettings();
      break;

    case WStype_DISCONNECTED:
      if (debugEnabled) {
        Serial.println("WebSocket disconnected!");
      }
      webSocketConnected = false;
      wasDisconnected = true;
      startedWebsocket = false;
      break;

    case WStype_TEXT:
      if (debugEnabled) {
        static unsigned long lastWSLog = 0;
        unsigned long now = millis();
        if (now - lastWSLog > 500) {
          String view = String((const char*)payload).substring(0, 120);
          Serial.printf("WebSocket message received: %s...\n", view.c_str());
          lastWSLog = now;
        }
      }
      processMessage((char*)payload);
      break;

    case WStype_ERROR:
      if (debugEnabled) {
        Serial.println("WebSocket error occurred");
      }
      webSocketConnected = false;
      break;

    default:
      if (debugEnabled) {
        Serial.println("Unknown WebSocket event");
      }
      break;
  }
}
void startDigitalOutput(int pin) {

  digitalWrite(pin, HIGH);  // Turn the pin ON (full duty cycle)
}

void stopDigitalOutput(int pin) {
  digitalWrite(pin, LOW);  // Turn the pin OFF
}


void processMessage(String message) {
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    if (debugEnabled) {
      Serial.println("Failed to parse message");
    }
    return;
  }

  const char* joinRef = doc[0];
  const char* msgRef = doc[1];
  String topic = doc[2] | "";
  String event = doc[3] | "";

  if (debugEnabled) {
    static unsigned long lastEventLog = 0;
    unsigned long now = millis();
    if (now - lastEventLog > 300) {
      Serial.printf("Received event: %s\n", event.c_str());
      lastEventLog = now;
    }
  }

  // Handle i_am_online response
  if (event == "i_am_online") {
    lastReceivedPingTime = millis();
    lastSuccessfulPingResponse = millis();  // Update internet connectivity tracker
    hasInternetConnectivity = true;         // Mark internet as available
    if (debugEnabled) {
      Serial.println("Received ping response from server");
    }
    return;
  }

  // Check for error response
  if (event == "phx_reply") {
    JsonObject response = doc[4].as<JsonObject>();
    if (response.containsKey("status") && response["status"] == "error") {
      if (response.containsKey("response")) {
        JsonObject errorResponse = response["response"];
        if (errorResponse.containsKey("reason") && errorResponse["reason"] == "unmatched topic") {
          if (debugEnabled) {
            Serial.println("Unmatched topic error detected, triggering reconnection...");
          }
          webSocketConnected = false;
          startedWebsocket = false;
          wasDisconnected = true;
          roomRef++; // Increment room reference for next connection attempt
          
          // Force an immediate reconnection attempt
          lastWebSocketReconnect = 0;
          return;
        }
      }
    }
  }

  if (event == "start_pwm") {
    if (debugEnabled) {
      Serial.println("Processing start_pwm event");
    }
    
    JsonObject payload = doc[4].as<JsonObject>();
    String action = payload["action"] | "start";
    int pin = payload["pin"] | 5;
    int reps = payload["reps"] | 1;
    float delayTime = payload["delay"] | 0.1;
    float intervalTime = payload["interval"] | 0.1;
    String uuid = payload["uuid"] | "";
    String format = payload["format"] | "pwm";

    if (debugEnabled) {
      Serial.printf("PWM Parameters - Action: %s, Pin: %d, Reps: %d, Format: %s\n", 
        action.c_str(), pin, reps, format.c_str());
    }

    // Calculate total duration of the operation
    unsigned long totalDuration = 0;
    if (action == "start") {
      totalDuration = (unsigned long)(reps * (intervalTime + delayTime) * 1000);
    } else if (action == "motor") {
      totalDuration = (unsigned long)(reps * 1000);
    }

    // Set block duration and start time
    pwmConfig.blockStartTime = millis();
    pwmConfig.blockDuration = totalDuration + 100;  // Add 100ms buffer
    pwmConfig.isBlocked = true;
    // Detach ISR during block to prevent any readings leaking through
    if (pwmConfig.isConfigured && !pwmConfig.interruptDetachedForBlock) {
      detachInterrupt(digitalPinToInterrupt(pwmConfig.inputPin));
      pwmConfig.interruptDetachedForBlock = true;
      noInterrupts();
      pwmConfig.pulseCount = 0;
      pwmConfig.lastPulseTime = 0;
      interrupts();
    }

    // Save current emulator mode state and temporarily disable it
    bool wasEmulatorMode = emulatorMode;
    emulatorMode = false;

    if (action == "start") {
      if (format == "pwm") {
        pinMode(pin, OUTPUT);
        ledcSetup(pwmChannel, frequency, resolution);

        for (int i = 0; i < reps; i++) {
          startDigitalOutput(pin);
          delay(intervalTime * 1000);
          stopDigitalOutput(pin);
          delay(delayTime * 1000);
        }

        ledcDetachPin(pin);
        pinMode(pin, INPUT);


        // ledcSetup(pwmChannel, frequency, resolution);
        // startPWM(pin, 65535);  
        // for (int i = 0; i < reps; i++) {
        //   delay(delayTime * 1000);  
        //   stopPWM(pin);             
        //   delay(delayTime * 1000); 
        //   startPWM(pin, 65535);    
        // }
        // stopPWM(pin); 
        // ledcDetachPin(pin);
        // pinMode(pin, INPUT);

      } else if (format == "rs232") {
        if (SKIP_RS232) {
          if (debugEnabled) {
            Serial.println("🚫 RS232 mode disabled - Skipping bill dispensing");
          }
        } else {
          if (debugEnabled) {
            Serial.println("📡 RS232 mode - Dispensing bills");
          }
          
          // Handle RS232 bill dispensing
          const struct {
            int value;
            uint8_t command;
          } bills[] = {
            {10, BILL_10},
            {5, BILL_5},
            {2, BILL_2},
            {1, BILL_1}
          };
          
          int remainingAmount = reps;
          if (debugEnabled) {
            Serial.println("💰 Dispensing total amount: " + String(remainingAmount));
          }
          
          for (const auto& bill : bills) {
            while (remainingAmount >= bill.value) {
              if (debugEnabled) {
                Serial.println("💵 Dispensing bill value: " + String(bill.value));
              }
              
              uint8_t data[] = {0x80, bill.command, 0x10};
              sendSerialData(data, sizeof(data));
              remainingAmount -= bill.value;
              delay(1000);
            }
          }
          
          if (debugEnabled) {
            Serial.println("✅ RS232 dispensing completed");
          }
        }
      }
    } else if (action == "motor") {
      startDigitalOutput(pin);
      delay(reps * 1000);
      stopDigitalOutput(pin);
    }

    // Restore previous emulator mode state if it wasn't RS232 format
    if (format != "rs232" && !SKIP_RS232) {
      emulatorMode = wasEmulatorMode;
      if (debugEnabled) {
        Serial.println("🔄 Emulator mode restored: " + String(emulatorMode ? "ON" : "OFF"));
      }
    }

    sendPWMResponse(pin, reps, delayTime, action, uuid, topic);
  } else {
    // Serial.println("Unknown event received");
  }

  // Add handling for settings response
  if (event == "settings_response") {
    // Serial.println("Received settings response");
    JsonObject payload = doc[4].as<JsonObject>();

    // Check for PWM configuration
    if (payload.containsKey("pwm_config")) {
      JsonObject pwmSettings = payload["pwm_config"];

      // Get the new input pin
      int newPin = pwmSettings["input_pin"] | -1;

      if (newPin != -1 && newPin != pwmConfig.inputPin) {
        // If we were already monitoring a pin, detach the interrupt
        if (pwmConfig.isConfigured) {
          detachInterrupt(digitalPinToInterrupt(pwmConfig.inputPin));
        }

        // Configure the new pin
        pwmConfig.inputPin = newPin;
        pinMode(newPin, INPUT);
        attachInterrupt(digitalPinToInterrupt(newPin), handlePWMInterrupt, RISING);
        pwmConfig.isConfigured = true;
        pwmConfig.pulseCount = 0;
        pwmConfig.lastPulseTime = 0;
        pwmConfig.lastReadingTime = 0;

        Serial.printf("Configured PWM reading on pin %d\n", newPin);
      }
    }

    // Add RS232 configuration handling
    if (payload.containsKey("rs232_config") && !SKIP_RS232) {
      JsonObject rs232Settings = payload["rs232_config"];
      
      static String lastDeviceType = "";  // Keep track of last device type
      
      int baudRate = rs232Settings["baud_rate"] | 9600;
      int rxPin = rs232Settings["rx_pin"] | 32;  // Changed from 25 to 32
      int txPin = rs232Settings["tx_pin"] | 33;  // Changed from 26 to 33
      String protocol = rs232Settings["protocol"] | "8N1";  // Data bits, parity, stop bits
      String deviceType = rs232Settings["device_type"] | "bill_acceptor";  // Type of RS232 device
      
      // Only configure if device type has changed
      if (deviceType != lastDeviceType) {
        lastDeviceType = deviceType;  // Update last device type
        
        // Configure RS232 based on settings
        if (deviceType == "bill_acceptor") {
          // Stop PWM reading if it was configured
          if (pwmConfig.isConfigured) {
            detachInterrupt(digitalPinToInterrupt(pwmConfig.inputPin));
            pwmConfig.isConfigured = false;
            if (debugEnabled) {
              Serial.println("Stopping PWM reading for bill acceptor mode");
            }
          }
          
          // Configure bill acceptor
          billAcceptorSerial.begin(baudRate, SERIAL_8N2, rxPin, txPin, false);
          emulatorMode = true;
          lastBillPollTime = millis();  // Start the timeout counter
          
          // Clear any pending data in the serial buffer
          while (billAcceptorSerial.available()) {
            billAcceptorSerial.read();
          }
          
          if (debugEnabled) {
            Serial.println("Bill acceptor detected - Enabling emulator mode for 5 seconds");
          }
        }
        // Add other device types as needed
      }
    } else if (payload.containsKey("rs232_config") && SKIP_RS232) {
      if (debugEnabled) {
        Serial.println("🚫 RS232 configuration disabled - Skipping RS232 setup");
      }
    }
  }

  // Add OTA (Over-The-Air) update handling
  if (event == "ota_update") {
    if (debugEnabled) {
      Serial.println("📡 Received OTA update command");
    }
    JsonObject payload = doc[4].as<JsonObject>();
    handleOTACommand(payload);
  }
}

// Function to send the PWM response to the WebSocket server
void sendPWMResponse(int pin, int reps, float delayTime, String action, String uuid, String topic) {
  // Prepare the response payload as a JSON object
  StaticJsonDocument<256> responseDoc;
  responseDoc["pin"] = pin;
  responseDoc["reps"] = reps;
  responseDoc["delay"] = delayTime;
  responseDoc["action"] = action;
  responseDoc["uuid"] = uuid;

  // Serialize the payload into a string
  String payload;
  serializeJson(responseDoc, payload);

  // Send the response via WebSocket using the required format
  sendWebSocketMessage(String(roomRef), String(roomRef), topic, "pwm_response", payload);
}



void sendPing() {
  if (SKIP_WIFI) {
    // A7670C mode: Polling already serves as heartbeat, no separate ping needed
    return;
  } else {
    // WiFi WebSocket mode: Send ping message for heartbeat
    String userId = macToUUID();
    String joinRef = String(roomRef);
    String msgRef = String(roomRef);
    String topic = "user:" + userId;
    String event = "ping";
        
    StaticJsonDocument<256> readingDoc;
    readingDoc["firmware_version"] = FIRMWARE_VERSION;
  
    String payload;
    serializeJson(readingDoc, payload);

    // Use the common sender to ensure consistent framing
    sendWebSocketMessage(joinRef, msgRef, topic, event, payload);

    if (debugEnabled) {
      String dbg = "[\"" + joinRef + "\",\"" + msgRef + "\",\"" + topic + "\",\"" + event + "\"," + payload + "]";
      Serial.println("📤 WS ping sent: " + dbg);
    }
  }
}


void startWebSocket(String url) {
  if (SKIP_WIFI) {
    // Use A7670C HTTP client instead of WebSocket
    if (debugEnabled) {
      Serial.printf("\n=== A7670C HTTP Connection Details ===\n");
      Serial.printf("Server: %s\n", url.c_str());
      Serial.printf("Device ID: %s\n", macToUUID().c_str());
      Serial.printf("A7670C Status: %s\n", a7670cConnected ? "Connected" : "Disconnected");
    }

    // Initialize A7670C HTTP client
    if (a7670cWebSocket.begin(url, globalPort, "/iot/a7670c")) {
      webSocketConnected = true;
      connectionStartTime = millis();
      lastReceivedPingTime = millis();
      lastPingTime = millis();
      
      if (wasDisconnected) {
        roomRef = roomRef + 1;
        wasDisconnected = false;
      }
      
      if (debugEnabled) {
        Serial.println("A7670C HTTP client initialized successfully");
      }
    } else {
      if (debugEnabled) {
        Serial.println("❌ Failed to initialize A7670C HTTP client");
      }
      webSocketConnected = false;
    }
  } else {
    // Original WiFi WebSocket logic
    if (debugEnabled) {
      Serial.printf("\n=== WebSocket Connection Details ===\n");
      Serial.printf("Server: %s\n", url.c_str());
      Serial.printf("Room Ref: %d\n", roomRef);
      Serial.printf("User ID: %s\n", macToUUID().c_str());
      Serial.printf("WiFi RSSI: %d dBm\n", WiFi.RSSI());
    }


    String wsPath = "/socket/websocket?vsn=2.0.0";
    
    if (debugEnabled) {
      Serial.println("Initializing WebSocket connection...");
    }
    
    // Close any existing connection
    webSocket.disconnect();
    delay(100);
    esp_task_wdt_reset();
    
    // Initialize new connection
    webSocket.begin(url.c_str(), globalPort, wsPath.c_str());
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
    
    // Wait for initial connection (30 seconds stabilization period)
    if (debugEnabled) {
      Serial.println("Waiting 10 seconds for connection to stabilize...");
    }
    
    unsigned long stabilizationStart = millis();
    bool connectionStable = false;
    
    while (millis() - stabilizationStart < 10000) {  // 10 second stabilization period
      webSocket.loop();  // Keep processing WebSocket events
      esp_task_wdt_reset();  // Feed watchdog during wait
      
      if ((millis() - stabilizationStart) % 5000 == 0) {  // Print status every 5 seconds
        if (debugEnabled) {
          Serial.printf("Stabilizing... %d seconds remaining\n", 
            (10000 - (millis() - stabilizationStart)) / 1000);
        }
      }
      
      delay(100);  // Small delay to prevent tight loop
    }
    
    webSocketConnected = true;
    lastWebSocketReconnect = millis();
    lastReceivedPingTime = millis();  // Reset ping timeout counter
    lastPingTime = millis();  // Reset ping timer
    
    if (debugEnabled) {
      Serial.println("WebSocket connection initialized and stabilized");
    }
  }
}


bool apModeActive = false;  // Track if AP mode is active
unsigned long lastCheckTime = 0;
Ticker apTimeoutTicker;

// Function to detect user input, e.g., button press
bool read_input() {
  // Replace this with actual input reading logic (e.g., digitalRead on a button pin)
  // Return true if button is pressed, otherwise false.
  return digitalRead(INPUT_PIN) == LOW;  // Example for button on pin 14
}

// Function to stop AP mode and return to scanning
void stopAP() {
  // Serial.println("Stopping AP mode...");
  WiFi.softAPdisconnect(true);
  apModeActive = false;
}
// Function to start AP mode
void startAP() {
  if (!apModeActive) {
    // Serial.println("Starting AP mode...");
    wifiManager.setConfigPortalTimeout(30);
    wifiManager.setConnectTimeout(5);
    wifiManager.startConfigPortal("ESP32_AP");
    apModeActive = true;
    apTimeoutTicker.once(5, stopAP);
  }
}


// Modify getDeviceSettings to actually configure PWM reading
void getDeviceSettings() {
  String userId = macToUUID();
  String topic = "user:" + userId;

  // Create JSON payload requesting both PWM and RS232 configuration
  StaticJsonDocument<256> settingsDoc;
  settingsDoc["device_id"] = userId;
  settingsDoc["request_type"] = "device_config";
  JsonArray configTypes = settingsDoc.createNestedArray("config_types");
  configTypes.add("pwm_config");
  if (!SKIP_RS232) {
    configTypes.add("rs232_config");
  }

  String payload;
  serializeJson(settingsDoc, payload);

  // Send settings request through WebSocket
  sendWebSocketMessage(String(roomRef), String(roomRef), topic, "get_settings", payload);
}



// Add timeout constant at the top with other constants
const unsigned long EMULATOR_TIMEOUT = 5000;  // 5 seconds timeout

// LED Functions
void toggleLED() {
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState);
}

void startBlinking() {
  pinMode(LED_BUILTIN, OUTPUT);
  ledTicker.attach_ms(BLINK_INTERVAL, toggleLED);
}

void stopBlinking() {
  ledTicker.detach();
  digitalWrite(LED_BUILTIN, LOW);  // Turn off LED
  ledState = false;
}

// Watchdog and Reset Functions
void initWatchdog() {
  esp_task_wdt_init(WDT_TIMEOUT, true);  // Enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);  // Add current thread to WDT watch
  
  if (debugEnabled) {
    Serial.println("Watchdog timer initialized with " + String(WDT_TIMEOUT) + " second timeout");
  }
}

void feedWatchdog() {
  esp_task_wdt_reset();
  lastWatchdogFeed = millis();
  
  if (debugEnabled) {
    static unsigned long lastWatchdogLog = 0;
    // Log watchdog feed every 30 seconds to avoid spam
    if (millis() - lastWatchdogLog >= 30000) {
      Serial.println("Watchdog fed - System healthy");
      lastWatchdogLog = millis();
    }
  }
}

void checkScheduledReset() {
  unsigned long currentTime = millis();
  
  // Check if 30 minutes have passed or if millis() wrapped around (overflow)
  if (currentTime - lastResetTime >= RESET_INTERVAL || currentTime < lastResetTime) {
    if (debugEnabled) {
      Serial.println("30-minute reset timer triggered - Restarting ESP32...");
      Serial.flush();  // Ensure message is sent before reset
    }
    
    delay(100);  // Small delay to ensure message is transmitted
    ESP.restart();  // Restart the ESP32
  }
}

// Function to handle SmartConfig
void startSmartConfig() {
    if (debugEnabled) {
        Serial.println("\n=== Starting SmartConfig Process ===");
        Serial.println("Current WiFi Status:");
        Serial.printf("Mode: %d\n", WiFi.getMode());
        Serial.printf("AutoConnect: %d\n", WiFi.getAutoConnect());
        Serial.printf("AutoReconnect: %d\n", WiFi.getAutoReconnect());
    }
    
    // Start fast LED blinking to indicate SmartConfig mode
    ledTicker.attach_ms(200, toggleLED);  // Faster blinking during SmartConfig
    
    // Ensure WiFi is in the correct mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);
    
    // Enable auto reconnect and persist
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    
    if (debugEnabled) {
        Serial.println("Starting SmartConfig...");
    }
    
    WiFi.beginSmartConfig();
    
    // Wait for SmartConfig packet from mobile
    if (debugEnabled) {
        Serial.println("Waiting for SmartConfig...");
    }
    
    // Wait for SmartConfig packet from mobile
    unsigned long smartConfigStart = millis();
    while (!WiFi.smartConfigDone()) {
        delay(500);
        esp_task_wdt_reset();  // Feed watchdog during SmartConfig wait
        if (debugEnabled) {
            if ((millis() - smartConfigStart) % 10000 == 0) { // Log every 10 seconds
                Serial.printf("Waiting for SmartConfig... (%lu seconds)\n", 
                    (millis() - smartConfigStart) / 1000);
            } else {
                Serial.print(".");
            }
        }
        // Optional: Add timeout after 2 minutes
        if (millis() - smartConfigStart > 120000) {
            if (debugEnabled) {
                Serial.println("\nSmartConfig timeout after 120 seconds");
            }
            stopBlinking();  // Stop LED blinking
            digitalWrite(LED_BUILTIN, HIGH);  // Keep LED on to indicate ready state
            WiFi.stopSmartConfig();
            return;
        }
    }
    
    // Change to slower blinking while connecting to WiFi
    ledTicker.attach_ms(500, toggleLED);
    
    if (debugEnabled) {
        Serial.println("\nSmartConfig received successfully");
        Serial.println("Waiting for WiFi connection");
    }
    
    // Wait for WiFi connection
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime < 30000)) { // 30 second timeout
        delay(500);
        esp_task_wdt_reset();  // Feed watchdog during WiFi connection wait
        if (debugEnabled) {
            Serial.print(".");
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        // Stop blinking and turn LED on solid to indicate successful connection
        stopBlinking();
        digitalWrite(LED_BUILTIN, HIGH);
        
        if (debugEnabled) {
            Serial.println("\n=== WiFi Connected Successfully ===");
            Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
            Serial.printf("Password saved: %s\n", WiFi.psk().length() > 0 ? "Yes" : "No");
            Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
            Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
            Serial.println("\nTesting credential persistence...");
        }

        // Test if credentials were actually saved and store them globally
        String savedSSID = WiFi.SSID();
        String savedPSK = WiFi.psk();
        
        // Store credentials globally for reconnection logic
        storedWiFiSSID = savedSSID;
        storedWiFiPassword = savedPSK;
        hasStoredCredentials = (savedSSID.length() > 0 && savedPSK.length() > 0);
        
        if (hasStoredCredentials) {
            if (debugEnabled) {
                Serial.println("Credentials saved successfully and stored globally");
                Serial.printf("Stored SSID: %s\n", storedWiFiSSID.c_str());
                Serial.printf("Stored password length: %d characters\n", storedWiFiPassword.length());
            }
        } else {
            if (debugEnabled) {
                Serial.println("WARNING: Credentials may not have been saved properly");
            }
        }
        
        // Start WebSocket connection after successful WiFi connection
        startWebSocket(globalUrl);
        startedWebsocket = true;
        webSocketConnected = true;
        
        // Initialize internet connectivity tracking
        hasInternetConnectivity = true;
        lastSuccessfulPingResponse = millis();
        lastInternetCheck = millis();
    } else {
        // Connection failed - stop blinking and turn LED off
        stopBlinking();
        digitalWrite(LED_BUILTIN, LOW);
        
        if (debugEnabled) {
            Serial.println("\nFailed to connect to WiFi");
            Serial.println("Last WiFi error:");
            Serial.printf("Status: %d\n", WiFi.status());
            Serial.printf("AutoConnect: %d\n", WiFi.getAutoConnect());
            Serial.printf("Mode: %d\n", WiFi.getMode());
        }
        // Stop SmartConfig
        WiFi.stopSmartConfig();
    }
}

// Simplified SmartConfig restart function for main loop
void restartSmartConfig() {
  if (debugEnabled) {
    Serial.println("\n=== Restarting SmartConfig (60 second timeout) ===");
  }
  
  // Disconnect current WiFi
  WiFi.disconnect();
  delay(1000);
  esp_task_wdt_reset();
  
  // Start fast LED blinking to indicate SmartConfig mode
  ledTicker.attach_ms(200, toggleLED);
  
  WiFi.beginSmartConfig();
  
  unsigned long smartConfigStart = millis();
  bool configReceived = false;
  
  // Wait for SmartConfig with 60 second timeout
  while (!configReceived && (millis() - smartConfigStart < 60000)) {
    if (WiFi.smartConfigDone()) {
      configReceived = true;
      break;
    }
    
    delay(500);
    esp_task_wdt_reset();
    
    if (debugEnabled && (millis() - smartConfigStart) % 5000 == 0) {
      int remaining = 60 - ((millis() - smartConfigStart) / 1000);
      Serial.printf("SmartConfig waiting... %d seconds remaining\n", remaining);
    }
  }
  
  if (configReceived) {
    if (debugEnabled) {
      Serial.println("\n✅ SmartConfig credentials received!");
    }
    
    // Change to slower blinking while connecting
    ledTicker.attach_ms(500, toggleLED);
    
    // Wait for WiFi connection
    unsigned long connectStart = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - connectStart < 20000)) {
      delay(500);
      esp_task_wdt_reset();
      if (debugEnabled && (millis() - connectStart) % 2000 == 0) {
        Serial.print(".");
      }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      stopBlinking();
      digitalWrite(LED_BUILTIN, HIGH);  // LED on solid
      
      if (debugEnabled) {
        Serial.println("\n✅ SmartConfig WiFi connection successful!");
        Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
        Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
        Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
      }
      
      // Initialize internet connectivity tracking
      hasInternetConnectivity = true;
      lastSuccessfulPingResponse = millis();
      lastInternetCheck = millis();
      
      // Start WebSocket if not already connected
      if (!webSocketConnected) {
        startWebSocket(globalUrl);
        startedWebsocket = true;
        webSocketConnected = true;
      }
    } else {
      if (debugEnabled) {
        Serial.println("\n❌ SmartConfig connection failed");
      }
      stopBlinking();
      digitalWrite(LED_BUILTIN, LOW);
    }
  } else {
    if (debugEnabled) {
      Serial.println("\n⏰ SmartConfig timeout (60 seconds)");
    }
    stopBlinking();
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  WiFi.stopSmartConfig();
  
  if (debugEnabled) {
    Serial.println("=== SmartConfig restart complete ===");
  }
}

// Setup function
void setup() {
  pinMode(INPUT_PIN, INPUT_PULLUP);  // Set input pin (e.g., button) with pull-up
  // pinMode(33, INPUT);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);  // Set BOOT button pin
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize LED pin
  Serial.begin(115200);  // Keep this one as it's needed for PWM readings output
  
  // Initialize watchdog and reset timer
  initWatchdog();
  lastResetTime = millis();  // Initialize reset timer
  lastWatchdogFeed = millis();  // Initialize watchdog feed timer
  
  if (debugEnabled) {
    Serial.println("\n=== Device Starting Up ===");
    Serial.printf("MAC Address: %s\n", WiFi.macAddress().c_str());
    Serial.printf("Device ID: %s\n", macToUUID().c_str());
    Serial.printf("Skip WiFi Mode: %s\n", SKIP_WIFI ? "Enabled (Using A7670C)" : "Disabled (Using WiFi)");
  }

  if (SKIP_WIFI) {
    // Initialize A7670C mode
    if (debugEnabled) {
      Serial.println("\n=== A7670C Mode Enabled ===");
      Serial.println("Skipping WiFi initialization...");
    }
    
    // Initialize A7670C
    if ((A7670C_AUTO_OPERATOR_SELECT ? initializeA7670CAutomatic() : initializeA7670C())) {
      if (debugEnabled) {
        Serial.println("Starting WebSocket connection via A7670C...");
      }
      startWebSocket(globalUrl);
      startedWebsocket = true;
    } else {
      if (debugEnabled) {
        Serial.println("❌ Failed to initialize A7670C");
      }
    }
  } else {
    // New Boot Sequence: BOOT button check -> SmartConfig -> WiFi search
    
    // Initialize WiFi settings
    WiFi.mode(WIFI_STA);
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);

    if (debugEnabled) {
      Serial.println("\n=== Boot Sequence Started ===");
      Serial.println("Checking for BOOT button press during first 10 seconds...");
      Serial.println("Press BOOT button to enter SmartConfig mode");
    }

    // Phase 1: Check for BOOT button during first 10 seconds
    bool bootButtonPressed = false;
    unsigned long bootCheckStart = millis();
    digitalWrite(LED_BUILTIN, LOW);  // LED off during boot check
    
    while (millis() - bootCheckStart < 10000) {  // 10 second window
      if (digitalRead(BOOT_BUTTON) == LOW) {  // Button pressed
        bootButtonPressed = true;
        if (debugEnabled) {
          Serial.println("\n🔘 BOOT button pressed! Starting SmartConfig...");
        }
        break;
      }
      
      // Show countdown every second
      if (debugEnabled && (millis() - bootCheckStart) % 1000 == 0) {
        int remaining = 10 - ((millis() - bootCheckStart) / 1000);
        if (remaining > 0) {
          Serial.print("Boot check: " + String(remaining) + "s remaining... ");
          Serial.println("(Press BOOT for SmartConfig)");
        }
      }
      
      delay(100);
      esp_task_wdt_reset();
    }

    // Phase 2: Handle SmartConfig if button was pressed
    bool smartConfigSuccess = false;
    if (bootButtonPressed) {
      if (debugEnabled) {
        Serial.println("\n=== Starting SmartConfig (60 second timeout) ===");
      }
      
      // Start fast LED blinking to indicate SmartConfig mode
      ledTicker.attach_ms(200, toggleLED);
      
      WiFi.beginSmartConfig();
      
      unsigned long smartConfigStart = millis();
      while (!WiFi.smartConfigDone() && (millis() - smartConfigStart < 60000)) {  // 60 second timeout
        delay(500);
        esp_task_wdt_reset();
        
        if (debugEnabled && (millis() - smartConfigStart) % 5000 == 0) {
          int remaining = 60 - ((millis() - smartConfigStart) / 1000);
          Serial.printf("SmartConfig waiting... %d seconds remaining\n", remaining);
        }
      }
      
      if (WiFi.smartConfigDone()) {
        if (debugEnabled) {
          Serial.println("\n✅ SmartConfig credentials received!");
        }
        
        // Change to slower blinking while connecting
        ledTicker.attach_ms(500, toggleLED);
        
        // Wait for WiFi connection
        unsigned long connectStart = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - connectStart < 20000)) {
          delay(500);
          esp_task_wdt_reset();
          if (debugEnabled && (millis() - connectStart) % 2000 == 0) {
            Serial.print(".");
          }
        }
        
        if (WiFi.status() == WL_CONNECTED) {
          smartConfigSuccess = true;
          stopBlinking();
          digitalWrite(LED_BUILTIN, HIGH);  // LED on solid
          
          if (debugEnabled) {
            Serial.println("\n✅ SmartConfig WiFi connection successful!");
            Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
            Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
            Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
          }
          
          // Initialize internet connectivity tracking
          hasInternetConnectivity = true;
          lastSuccessfulPingResponse = millis();
          lastInternetCheck = millis();
          
          startWebSocket(globalUrl);
          startedWebsocket = true;
          webSocketConnected = true;
        } else {
          if (debugEnabled) {
            Serial.println("\n❌ SmartConfig connection failed");
          }
        }
      } else {
        if (debugEnabled) {
          Serial.println("\n⏰ SmartConfig timeout (60 seconds)");
        }
      }
      
      WiFi.stopSmartConfig();
      stopBlinking();
    }

    // Phase 3: Try existing WiFi credentials if SmartConfig wasn't successful
    if (!smartConfigSuccess) {
      if (debugEnabled) {
        if (bootButtonPressed) {
          Serial.println("\n=== SmartConfig unsuccessful, trying existing WiFi ===");
        } else {
          Serial.println("\n=== No BOOT button press, trying existing WiFi ===");
        }
        
        Serial.println("=== Checking Stored WiFi Credentials ===");
        String savedSSID = WiFi.SSID();
        String savedPSK = WiFi.psk();
        
        // Store credentials globally even if disconnected
        if (savedSSID.length() > 0 && savedPSK.length() > 0) {
          storedWiFiSSID = savedSSID;
          storedWiFiPassword = savedPSK;
          hasStoredCredentials = true;
        }
        
        if (savedSSID.length() > 0) {
          Serial.printf("Found stored SSID: %s\n", savedSSID.c_str());
          Serial.printf("Stored password length: %d characters\n", savedPSK.length());
          Serial.printf("Stored globally for reconnection logic\n");
        } else {
          Serial.println("No stored credentials found");
        }
        
        Serial.println("\n=== WiFi Connection Attempt ===");
      }

      // Try to connect with stored credentials
      WiFi.begin();
      
      // Wait for connection with timeout
      unsigned long startAttempt = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
        delay(500);
        esp_task_wdt_reset();
        if (debugEnabled) {
          Serial.print(".");
        }
      }

      if (WiFi.status() == WL_CONNECTED) {
        if (debugEnabled) {
          Serial.println("\n✅ Connected to stored network:");
          Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
          Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
          Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
          Serial.printf("Gateway: %s\n", WiFi.gatewayIP().toString().c_str());
          Serial.printf("DNS: %s\n", WiFi.dnsIP().toString().c_str());
          Serial.println("\n=== Starting WebSocket Connection ===");
        }
        
        // Store credentials globally for reconnection logic
        storedWiFiSSID = WiFi.SSID();
        storedWiFiPassword = WiFi.psk();
        hasStoredCredentials = (storedWiFiSSID.length() > 0 && storedWiFiPassword.length() > 0);
        
        if (debugEnabled && hasStoredCredentials) {
          Serial.printf("Stored credentials globally - SSID: %s, Password length: %d\n", 
                       storedWiFiSSID.c_str(), storedWiFiPassword.length());
        }
        
        // Initialize internet connectivity tracking
        hasInternetConnectivity = true;
        lastSuccessfulPingResponse = millis();
        lastInternetCheck = millis();
        
        startWebSocket(globalUrl);
        digitalWrite(LED_BUILTIN, HIGH);  // LED on to indicate connection
      } else {
        if (debugEnabled) {
          Serial.println("\n❌ Failed to connect with stored credentials");
          Serial.println("Available networks:");
          int n = WiFi.scanNetworks();
          for (int i = 0; i < n; ++i) {
            Serial.printf("%d: %s (Signal: %d dBm)\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
            delay(10);
          }
          Serial.println("\n💡 Device will continue running without WiFi");
          Serial.println("💡 Press BOOT button anytime to retry SmartConfig");
        }
        
        // Keep LED off to indicate no connection
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
    
    if (debugEnabled) {
      Serial.println("\n=== Boot Sequence Complete ===");
    }
  }


  // Initialize bill acceptor serial
  if (!SKIP_RS232) {
    billAcceptorSerial.begin(9600, SERIAL_8N2, 32, 33, false); // Changed from 25,26 to 32,33
    delay(2000);
  }



  while (billAcceptorSerial.available()) {
    billAcceptorSerial.read();
  }
}


// Loop function
void loop() {
  // Check OTA watchdog first
  checkOTAWatchdog();
  
  // Feed main watchdog
  esp_task_wdt_reset();

  if (SKIP_WIFI) {
    // A7670C WebSocket processing
    a7670cWebSocket.loop();
  } else {
    // WiFi WebSocket processing
    webSocket.loop();
  }

  // Feed watchdog timer periodically
  unsigned long currentMillis = millis();
  if (currentMillis - lastWatchdogFeed >= WATCHDOG_FEED_INTERVAL) {
    feedWatchdog();
  }

  // Check for scheduled 30-minute reset
  checkScheduledReset();

  if (SKIP_WIFI) {
    // Handle A7670C connection
    if (a7670cConnected) {
      // Check if we haven't received a ping response for too long
      unsigned long currentTimeout = (millis() - connectionStartTime < INITIAL_CONNECTION_PERIOD) ? 
                                   INITIAL_PING_TIMEOUT : PING_TIMEOUT;
                                   
      // Skip WebSocket checks during OTA to prevent interference
      extern bool otaWebSocketDisabled;
      if (otaWebSocketDisabled) {
        if (debugEnabled && (currentMillis - lastCheckTime >= 30000)) {  // Log every 30 seconds during OTA
          Serial.println("⏸️ Skipping WebSocket checks during OTA...");
        }
        return;
      }
      
      if (webSocketConnected && (currentMillis - lastReceivedPingTime >= currentTimeout)) {
        if (debugEnabled) {
          Serial.println("\n=== A7670C WebSocket Connection Check ===");
          Serial.printf("Connection age: %lu ms\n", millis() - connectionStartTime);
          Serial.printf("Current timeout: %lu ms\n", currentTimeout);
          Serial.printf("Last Ping Sent: %lu ms ago\n", currentMillis - lastPingTime);
          Serial.printf("Last Ping Response: %lu ms ago\n", currentMillis - lastReceivedPingTime);
          
          if (currentMillis - lastReceivedPingTime >= currentTimeout) {
            Serial.println("A7670C connection timed out - Triggering reconnection...");
          }
        }
        webSocketConnected = false;
        startedWebsocket = false;
        wasDisconnected = true;
        lastWebSocketReconnect = 0; // Force immediate reconnection
      }

      if (!webSocketConnected && (currentMillis - lastWebSocketReconnect >= WEBSOCKET_RECONNECT_INTERVAL)) {
        if (debugEnabled) {
          Serial.println("\n=== A7670C WebSocket Reconnection Attempt ===");
          Serial.printf("A7670C Status: %s\n", a7670cConnected ? "Connected" : "Disconnected");
          Serial.printf("Room Ref: %d\n", roomRef);
          Serial.printf("User ID: %s\n", macToUUID().c_str());
          Serial.printf("Time since last attempt: %lu ms\n", currentMillis - lastWebSocketReconnect);
        }
        
        startWebSocket(globalUrl);
        connectionStartTime = millis();  // Reset connection start time
      }
    } else {
      // A7670C disconnected, try to reconnect
      if (debugEnabled && (currentMillis - lastCheckTime >= 10000)) {  // Log every 10 seconds
        Serial.println("\n=== A7670C Status Check ===");
        Serial.println("A7670C disconnected, attempting reconnection...");
      }
      
      if (currentMillis - lastA7670CCheck >= A7670C_CHECK_INTERVAL) {
        lastA7670CCheck = currentMillis;
        if ((A7670C_AUTO_OPERATOR_SELECT ? initializeA7670CAutomatic() : initializeA7670C())) {
          if (debugEnabled) {
            Serial.println("A7670C reconnection successful!");
          }
        }
      }
    }
  } else {
    // Handle WiFi WebSocket connection with internet connectivity monitoring
    if (WiFi.status() == WL_CONNECTED) {
      // Monitor internet connectivity alongside WebSocket connection
      unsigned long currentTimeout = (millis() - connectionStartTime < INITIAL_CONNECTION_PERIOD) ? 
                                   INITIAL_PING_TIMEOUT : PING_TIMEOUT;
      
      // Check if internet connectivity is lost (no ping responses for too long)
      bool internetLost = (currentMillis - lastSuccessfulPingResponse >= INTERNET_TIMEOUT);
      
      // Periodic internet connectivity test
      if (currentMillis - lastInternetCheck >= INTERNET_CHECK_INTERVAL) {
        lastInternetCheck = currentMillis;
        
        // Only test if we haven't received a ping response recently
        if (currentMillis - lastSuccessfulPingResponse >= 30000) { // No response for 30 seconds
          if (debugEnabled) {
            Serial.println("\n🔍 Testing internet connectivity (no recent ping responses)...");
          }
          
          bool internetAvailable = testInternetConnectivity();
          
          if (internetAvailable && !hasInternetConnectivity) {
            // Internet was down but now available
            hasInternetConnectivity = true;
            lastSuccessfulPingResponse = currentMillis;
            if (debugEnabled) {
              Serial.println("✅ Internet connectivity restored via direct test");
            }
            
            // Trigger WebSocket reconnection if not currently connected
            if (!webSocketConnected) {
              if (debugEnabled) {
                Serial.println("🔄 Internet restored - triggering WebSocket reconnection...");
              }
              lastWebSocketReconnect = 0;  // Force immediate reconnection
              startedWebsocket = false;    // Reset WebSocket state
              wasDisconnected = true;      // Mark as needing reconnection
            }
          } else if (!internetAvailable && hasInternetConnectivity) {
            // Internet connection lost
            hasInternetConnectivity = false;
            if (debugEnabled) {
              Serial.println("❌ Internet connectivity lost via direct test");
            }
          }
        }
      }
                                   
      if (webSocketConnected && (currentMillis - lastReceivedPingTime >= currentTimeout)) {
        if (debugEnabled) {
          Serial.println("\n=== WebSocket Connection Check ===");
          Serial.printf("Connection age: %lu ms\n", millis() - connectionStartTime);
          Serial.printf("Current timeout: %lu ms\n", currentTimeout);
          Serial.printf("Last Ping Sent: %lu ms ago\n", currentMillis - lastPingTime);
          Serial.printf("Last Ping Response: %lu ms ago\n", currentMillis - lastReceivedPingTime);
          Serial.printf("Last Successful Internet Response: %lu ms ago\n", currentMillis - lastSuccessfulPingResponse);
          Serial.printf("Internet Status: %s\n", hasInternetConnectivity ? "Available" : "Lost");
          
          if (currentMillis - lastReceivedPingTime >= currentTimeout) {
            if (internetLost) {
              Serial.println("🌐 Internet connectivity lost - will attempt WiFi reconnection");
            } else {
              Serial.println("WebSocket connection timed out - Triggering reconnection...");
            }
          }
        }
        
        // If internet is lost, try WiFi reconnection instead of just WebSocket reconnection
        if (internetLost) {
          handleInternetConnectivityLoss();
        }
        
        webSocketConnected = false;
        startedWebsocket = false;
        wasDisconnected = true;
        lastWebSocketReconnect = 0; // Force immediate reconnection
      }

      if (!webSocketConnected && (currentMillis - lastWebSocketReconnect >= WEBSOCKET_RECONNECT_INTERVAL)) {
        // Only attempt WebSocket reconnection if we have internet connectivity
        if (hasInternetConnectivity || (currentMillis - lastSuccessfulPingResponse < 30000)) {
          if (debugEnabled) {
            Serial.println("\n=== WebSocket Reconnection Attempt ===");
            Serial.printf("WiFi RSSI: %d dBm\n", WiFi.RSSI());
            Serial.printf("Room Ref: %d\n", roomRef);
            Serial.printf("User ID: %s\n", macToUUID().c_str());
            Serial.printf("Time since last attempt: %lu ms\n", currentMillis - lastWebSocketReconnect);
            Serial.printf("Internet Status: %s\n", hasInternetConnectivity ? "Available" : "Testing...");
          }
          
          startWebSocket(globalUrl);
          connectionStartTime = millis();  // Reset connection start time
        } else {
          if (debugEnabled) {
            Serial.println("\n⚠️ Skipping WebSocket reconnection - Internet connectivity unavailable");
            Serial.println("Will attempt WiFi reconnection instead...");
          }
          handleInternetConnectivityLoss();
        }
      }
    } else {
      // WiFi disconnected (original WiFi handling logic)
      static unsigned long lastAutoReconnect = 0;  // Track last automatic reconnection attempt
      
      if (debugEnabled && (currentMillis - lastCheckTime >= 1000)) {
        Serial.println("\n=== WiFi Status Check ===");
        Serial.println("WiFi disconnected");
        
        // Use globally stored credentials instead of WiFi.SSID() which returns empty when disconnected
        String storedSSID = hasStoredCredentials ? storedWiFiSSID : "";
        bool storedNetworkFound = false;
        
        if (debugEnabled && hasStoredCredentials) {
          Serial.printf("Using stored credentials - SSID: %s, Password length: %d\n", 
                       storedSSID.c_str(), storedWiFiPassword.length());
        } else if (debugEnabled) {
          Serial.println("No stored credentials available");
        }
        
        int n = WiFi.scanNetworks();
        if (n > 0) {
          Serial.println("Available networks:");
          for (int i = 0; i < n; ++i) {
            String currentSSID = WiFi.SSID(i);
            Serial.printf("%d: %s (Signal: %d dBm)\n", i + 1, currentSSID.c_str(), WiFi.RSSI(i));
            
            // Check if this is our stored network
            if (storedSSID.length() > 0 && currentSSID.equals(storedSSID)) {
              storedNetworkFound = true;
              Serial.printf("   ^ Found stored network with signal: %d dBm\n", WiFi.RSSI(i));
            }
            delay(10);
          }
          
          // Attempt automatic reconnection if stored network is found
          if (storedNetworkFound && storedSSID.length() > 0) {
            // Use shorter cooldown (10 seconds) when stored network is visible vs 30 seconds when not visible
            unsigned long reconnectCooldown = 10000;  // 10 seconds when network is visible
            
            if (currentMillis - lastAutoReconnect > reconnectCooldown) {
              Serial.println("🔄 Stored network found in scan - attempting reconnection...");
              Serial.printf("Network: %s (Signal strength indicates good connectivity)\n", storedSSID.c_str());
              
              // Use explicit credentials for reconnection instead of WiFi.reconnect()
              if (hasStoredCredentials) {
                WiFi.begin(storedWiFiSSID.c_str(), storedWiFiPassword.c_str());
              } else {
                WiFi.reconnect();  // Fallback to reconnect if somehow credentials are missing
              }
              lastAutoReconnect = currentMillis;
              
              // Wait a bit to see if connection succeeds
              unsigned long reconnectStart = millis();
              while (WiFi.status() != WL_CONNECTED && millis() - reconnectStart < 15000) {  // Increased to 15 seconds
                delay(500);
                esp_task_wdt_reset();
                if (debugEnabled && (millis() - reconnectStart) % 2000 == 0) {
                  Serial.print(".");
                }
              }
              
              if (WiFi.status() == WL_CONNECTED) {
                Serial.println("\n✅ Automatic reconnection successful!");
                Serial.printf("Connected to: %s\n", WiFi.SSID().c_str());
                Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
                Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
                
                // Reset internet connectivity tracking
                hasInternetConnectivity = true;
                lastSuccessfulPingResponse = millis();
                lastReceivedPingTime = millis();
                
                // Test internet connectivity after reconnection
                delay(2000);  // Give connection time to stabilize
                esp_task_wdt_reset();
                
                if (testInternetConnectivity()) {
                  if (debugEnabled) {
                    Serial.println("✅ Internet connectivity confirmed after WiFi reconnection");
                  }
                  
                  // Trigger WebSocket reconnection if not connected
                  if (!webSocketConnected) {
                    if (debugEnabled) {
                      Serial.println("🔄 Triggering WebSocket reconnection after WiFi restore...");
                    }
                    lastWebSocketReconnect = 0;  // Force immediate reconnection
                    startedWebsocket = false;
                    wasDisconnected = true;
                  }
                } else {
                  if (debugEnabled) {
                    Serial.println("⚠️ WiFi connected but internet connectivity test failed");
                  }
                }
              } else {
                Serial.println("\n❌ Automatic reconnection failed despite network being visible");
                Serial.println("Will retry in 10 seconds...");
              }
            } else {
              unsigned long timeRemaining = reconnectCooldown - (currentMillis - lastAutoReconnect);
              Serial.printf("📡 Stored network visible, waiting %lu seconds before retry\n", 
                           timeRemaining / 1000);
            }
          } else if (storedSSID.length() > 0) {
            // Network not found in scan
            Serial.printf("❌ Stored network '%s' not found in current scan\n", storedSSID.c_str());
          }
        } else {
          Serial.println("No networks found");
          
          // If no networks found, still try to reconnect to stored network periodically
          // This handles cases where scan fails but network might still be available
          String storedSSID = hasStoredCredentials ? storedWiFiSSID : "";
          if (storedSSID.length() > 0 && (currentMillis - lastAutoReconnect > 20000)) {  // Try every 20 seconds when no scan results
            Serial.printf("🔄 No networks in scan, attempting blind reconnection to '%s'...\n", storedSSID.c_str());
            
            // Use explicit credentials for blind reconnection
            if (hasStoredCredentials) {
              WiFi.begin(storedWiFiSSID.c_str(), storedWiFiPassword.c_str());
            } else {
              WiFi.reconnect();  // Fallback
            }
            lastAutoReconnect = currentMillis;
            
            // Wait to see if connection succeeds
            unsigned long reconnectStart = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - reconnectStart < 10000) {
              delay(500);
              esp_task_wdt_reset();
              if (debugEnabled && (millis() - reconnectStart) % 2000 == 0) {
                Serial.print(".");
              }
            }
            
            if (WiFi.status() == WL_CONNECTED) {
              Serial.println("\n✅ Blind reconnection successful!");
              Serial.printf("Connected to: %s\n", WiFi.SSID().c_str());
              Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
              Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
              
              // Reset connectivity tracking and test internet
              hasInternetConnectivity = true;
              lastSuccessfulPingResponse = millis();
              lastReceivedPingTime = millis();
              
              delay(2000);
              esp_task_wdt_reset();
              
              if (testInternetConnectivity()) {
                if (debugEnabled) {
                  Serial.println("✅ Internet connectivity confirmed");
                }
                
                if (!webSocketConnected) {
                  if (debugEnabled) {
                    Serial.println("🔄 Triggering WebSocket reconnection...");
                  }
                  lastWebSocketReconnect = 0;
                  startedWebsocket = false;
                  wasDisconnected = true;
                }
              }
            } else {
              Serial.println("\n❌ Blind reconnection failed");
            }
          }
        }
      }
    }
  }

  // Send a ping every 5 seconds if WebSocket is connected
  if (webSocketConnected && currentMillis - lastPingTime >= pingInterval) {
    lastPingTime = currentMillis;
    sendPing();
  }

  // Check if we need to unblock PWM readings
  if (pwmConfig.isBlocked) {
    if (currentMillis - pwmConfig.blockStartTime >= pwmConfig.blockDuration) {
      pwmConfig.isBlocked = false;
      // Reattach ISR after block and reset counters
      if (pwmConfig.isConfigured && pwmConfig.interruptDetachedForBlock) {
        attachInterrupt(digitalPinToInterrupt(pwmConfig.inputPin), handlePWMInterrupt, RISING);
        pwmConfig.interruptDetachedForBlock = false;
        noInterrupts();
        pwmConfig.pulseCount = 0;
        pwmConfig.lastPulseTime = 0;
        interrupts();
        pwmConfig.lastReadingTime = millis();
        // Suppress sends for a short settling window to avoid spurious 0/huge values
        pwmConfig.sendSuppressUntil = millis() + 300; // 300ms
        pwmConfig.zeroStreak = 0;
        pwmConfig.historyCount = 0;
        pwmConfig.suppressedOutlierStreak = 0;
        pwmConfig.aggregatedPulses = 0;
        pwmConfig.aggregateStartTime = millis();
      }
      // Resume PCNT if used
      extern void pwmResumeCountingAfterBlock();
      pwmResumeCountingAfterBlock();
    }
  }

  // Check connection status every second
  if (currentMillis - lastCheckTime >= 1000) {
    lastCheckTime = currentMillis;

    if (!SKIP_WIFI) {
      // Check the current state of the BOOT button (only for WiFi mode)
      static unsigned long lastButtonCheck = 0;
      int readButton = digitalRead(BOOT_BUTTON);

      // If the button state is different from the last read
      if (readButton != buttonState) {
        // Check if the button was pressed (LOW state)
        if (readButton == LOW && (currentMillis - lastButtonCheck > 1000)) {  // 1 second debounce
          lastButtonCheck = currentMillis;
          
          if (debugEnabled) {
            Serial.println("\n🔘 BOOT button pressed - Starting SmartConfig...");
          }
          
          // Start simplified SmartConfig
          restartSmartConfig();
        }
        // Update the button state to avoid multiple reads
        buttonState = readButton;
        delay(10); // Add a small delay to prevent bouncing issues
      }

      // If Wi-Fi is disconnected and button is not pressed
      if (WiFi.status() != WL_CONNECTED && readButton == HIGH) {
        // Manual reconnection attempt via input pin (only if automatic hasn't tried recently)
        static unsigned long lastManualReconnect = 0;
        if (read_input() && (currentMillis - lastManualReconnect > 15000)) {  // 15 second cooldown
          if (debugEnabled) {
            Serial.println("Manual reconnection requested via input pin...");
          }
          WiFi.reconnect();
          lastManualReconnect = currentMillis;
          
          // Split the 5-second delay to feed watchdog
          for (int i = 0; i < 10; i++) {
            delay(500);
            esp_task_wdt_reset();  // Feed watchdog during reconnection wait
          }
        }
      } else if (WiFi.status() == WL_CONNECTED && !webSocketConnected) {
        // If WiFi is connected but WebSocket isn't, try to start WebSocket
        startWebSocket(globalUrl);
      }
    }
  }

  static unsigned long lastSettingsUpdate = 0;
  if (currentMillis - lastSettingsUpdate >= 10000) {  // Update settings every 10 seconds
    lastSettingsUpdate = currentMillis;
    bool connectionReady = SKIP_WIFI ? a7670cConnected : (WiFi.status() == WL_CONNECTED);
    if (connectionReady && startedWebsocket) {
      getDeviceSettings();
    }
  }

  // Send/print PWM readings if configured (serial prints always; WS send guarded inside)
  if (pwmConfig.isConfigured) {
    sendPWMReadings();
  }

  // Monitor A7670C connection status periodically
  if (SKIP_WIFI && (currentMillis - lastA7670CCheck >= A7670C_CHECK_INTERVAL)) {
    lastA7670CCheck = currentMillis;
    if (debugEnabled) {
      Serial.println("\n📊 A7670C Status Check:");
    }
    monitorA7670CConnection();
  }

  // Check for emulator timeout
  if (!SKIP_RS232 && emulatorMode && (millis() - lastBillPollTime > EMULATOR_TIMEOUT)) {
    emulatorMode = false;
    if (debugEnabled) {
      Serial.println("Emulator mode timed out - No response from controller");
    }
  }

  // Handle bill acceptor input
  if (!SKIP_RS232 && billAcceptorSerial.available()) {  // Remove emulatorMode check to always read input
    uint8_t inByte = billAcceptorSerial.read();

    if (debugEnabled) {
      Serial.printf("Received: %02X - ", inByte);
      switch(inByte) {
        case POLL_CMD: Serial.println("POLL"); break;
        case STX_CMD: Serial.println("STX"); break;
        case FF_CMD: Serial.println("FF"); break;
        case 0x80: Serial.println("BILL START"); break;
        default: Serial.println("Unknown");
      }
    }

    // Check for bill insertion events (0x80 followed by bill value)
    static bool expectingBillValue = false;
    static uint8_t lastByte = 0;

    if (inByte == 0x80) {
      expectingBillValue = true;
    } else if (expectingBillValue) {
      expectingBillValue = false;
      // Check if this is a valid bill value command
      switch(inByte) {
        case BILL_1:
        case BILL_2:
        case BILL_5:
        case BILL_10:
          if (debugEnabled) {
            Serial.println("Valid bill detected, sending reading");
          }
          sendBillAcceptorReading(inByte);
          break;
      }
    }

    lastByte = inByte;

    // Handle emulator responses if in emulator mode
    if (emulatorMode) {
      // Respond to both POLL and STX commands with ACK
      if (inByte == POLL_CMD || inByte == STX_CMD) {
        delay(5);  // Small delay before responding
        billAcceptorSerial.write(ACK_RSP);
        lastBillPollTime = millis();
        
        if (debugEnabled) {
          Serial.println("-> Sent ACK");
        }
      }
      
      // For FF command, send status and any pending bill event
      if (inByte == FF_CMD) {
        delay(10);
        billAcceptorSerial.write(STATUS_OK);
        if (debugEnabled) {
          Serial.println("-> Sent STATUS_OK");
        }
        
        if (billEventPending) {
          delay(10);
          sendBillEvent(pendingBillCmd);
          billEventPending = false;
        }
        
        // Exit emulator mode after completing FF command response
        emulatorMode = false;
        if (debugEnabled) {
          Serial.println("ACK process complete - Exiting emulator mode");
        }
      }
    }
  }


}

// Add A7670C recovery function
void recoverA7670CModule() {
  if (debugEnabled) {
    Serial.println("\n🔄 Starting A7670C module recovery...");
  }
  
  // Mark as disconnected
  a7670cConnected = false;
  webSocketConnected = false;
  
  // Try hard reset first with AT+CRESET
  if (debugEnabled) {
    Serial.println("🔄 Attempting hard reset with AT+CRESET...");
  }
  
  // Clear buffer before sending reset command
  while (a7670cSerial.available()) {
    a7670cSerial.read();
  }
  
  a7670cSerial.println("AT+CRESET");
  a7670cSerial.flush();
  
  if (debugEnabled) {
    Serial.println("🔄 AT+CRESET sent, waiting for module to restart...");
  }
  
  // Wait for hardware reset to complete with watchdog feeding
  unsigned long resetStart = millis();
  while (millis() - resetStart < 10000) { // 10 seconds for hardware reset
    delay(500);
    esp_task_wdt_reset();
    if (debugEnabled && (millis() - resetStart) % 2000 == 0) {
      Serial.println("⏳ Reset in progress... " + String((millis() - resetStart) / 1000) + "s");
    }
  }
  
  // Clear any data in buffer after reset
  while (a7670cSerial.available()) {
    a7670cSerial.read();
  }
  
  // Wait additional time for module to stabilize
  if (debugEnabled) {
    Serial.println("⏳ Waiting for module stabilization...");
  }
  
  unsigned long stabilizeStart = millis();
  while (millis() - stabilizeStart < 5000) { // Additional 5 seconds
    delay(500);
    esp_task_wdt_reset();
  }
  
  // Test if module is responsive after reset
  if (debugEnabled) {
    Serial.println("🔍 Testing module responsiveness...");
  }
  
  String testResponse = "";
  a7670cSerial.println("AT");
  a7670cSerial.flush();
  
  unsigned long testStart = millis();
  bool moduleResponding = false;
  
  while (millis() - testStart < 5000) {
    if (a7670cSerial.available()) {
      char c = a7670cSerial.read();
      testResponse += c;
      if (testResponse.indexOf("OK") >= 0) {
        moduleResponding = true;
        if (debugEnabled) {
          Serial.println("✅ Module responding after AT+CRESET");
        }
        break;
      }
    }
    delay(10);
    esp_task_wdt_reset();
  }
  
  if (!moduleResponding) {
    if (debugEnabled) {
      Serial.println("⚠️ Module still not responding after AT+CRESET");
      Serial.println("🔄 Trying AT+CFUN=1,1 as fallback...");
    }
    
    // Clear buffer
    while (a7670cSerial.available()) {
      a7670cSerial.read();
    }
    
    // Fallback to soft reset
    a7670cSerial.println("AT+CFUN=1,1");
    a7670cSerial.flush();
    
    unsigned long fallbackStart = millis();
    while (millis() - fallbackStart < 5000) {
      delay(500);
      esp_task_wdt_reset();
    }
    
    // Clear any data in buffer
    while (a7670cSerial.available()) {
      a7670cSerial.read();
    }
    
    // Final test
    testResponse = "";
    a7670cSerial.println("AT");
    a7670cSerial.flush();
    
    testStart = millis();
    while (millis() - testStart < 3000) {
      if (a7670cSerial.available()) {
        char c = a7670cSerial.read();
        testResponse += c;
        if (testResponse.indexOf("OK") >= 0) {
          moduleResponding = true;
          if (debugEnabled) {
            Serial.println("✅ Module responding after AT+CFUN=1,1");
          }
          break;
        }
      }
      delay(10);
      esp_task_wdt_reset();
    }
  }
  
  if (debugEnabled) {
    Serial.println("🔄 Recovery complete - Module status: " + String(moduleResponding ? "RESPONSIVE" : "UNRESPONSIVE"));
    if (!moduleResponding) {
      Serial.println("⚠️ Module may need physical power cycle or hardware inspection");
    }
  }
}

// Function to test internet connectivity
bool testInternetConnectivity() {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }
  
  HTTPClient http;
  http.begin("http://httpbin.org/get");  // Lightweight HTTP test service
  http.setTimeout(5000);  // 5 second timeout
  
  int httpCode = http.GET();
  http.end();
  
  bool isConnected = (httpCode > 0 && httpCode == 200);
  
  if (debugEnabled) {
    if (isConnected) {
      Serial.println("✅ Internet connectivity test passed");
    } else {
      Serial.println("❌ Internet connectivity test failed (HTTP code: " + String(httpCode) + ")");
    }
  }
  
  return isConnected;
}

// Function to handle internet connectivity issues
void handleInternetConnectivityLoss() {
  if (debugEnabled) {
    Serial.println("\n🌐 Internet connectivity lost - attempting WiFi reconnection");
    Serial.printf("WiFi Status: %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.printf("WiFi RSSI: %d dBm\n", WiFi.RSSI());
    Serial.printf("Last successful ping response: %lu ms ago\n", millis() - lastSuccessfulPingResponse);
  }
  
  // Disconnect and reconnect WiFi to refresh the connection
  WiFi.disconnect();
  delay(2000);
  esp_task_wdt_reset();
  
  if (debugEnabled) {
    Serial.println("🔄 Attempting WiFi reconnection...");
  }
  
  // Use explicit credentials for reconnection if available
  if (hasStoredCredentials) {
    WiFi.begin(storedWiFiSSID.c_str(), storedWiFiPassword.c_str());
  } else {
    WiFi.reconnect();  // Fallback to reconnect
  }
  
  // Wait for reconnection with timeout
  unsigned long reconnectStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - reconnectStart < 15000) {
    delay(500);
    esp_task_wdt_reset();
    if (debugEnabled && (millis() - reconnectStart) % 2000 == 0) {
      Serial.print(".");
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    if (debugEnabled) {
      Serial.println("\n✅ WiFi reconnection successful!");
      Serial.printf("Connected to: %s\n", WiFi.SSID().c_str());
      Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
      Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
    }
    
    // Test internet connectivity after reconnection
    delay(3000);  // Give connection time to stabilize
    esp_task_wdt_reset();
    
    if (testInternetConnectivity()) {
      hasInternetConnectivity = true;
      lastSuccessfulPingResponse = millis();  // Reset ping timer
      lastReceivedPingTime = millis();        // Reset WebSocket ping timer
      
      if (debugEnabled) {
        Serial.println("✅ Internet connectivity restored!");
      }
      
      // Trigger WebSocket reconnection if not currently connected
      if (!webSocketConnected) {
        if (debugEnabled) {
          Serial.println("🔄 WiFi restored - triggering WebSocket reconnection...");
        }
        lastWebSocketReconnect = 0;  // Force immediate reconnection
        startedWebsocket = false;    // Reset WebSocket state
        wasDisconnected = true;      // Mark as needing reconnection
      }
    } else {
      if (debugEnabled) {
        Serial.println("⚠️ WiFi connected but internet still unavailable");
      }
    }
  } else {
    if (debugEnabled) {
      Serial.println("\n❌ WiFi reconnection failed");
    }
  }
}

// Add: Manual network selection using ACTIVE_MCC_MNC instead of hardcoded Celcom
bool selectNetworkManual() {
  if (debugEnabled) {
    Serial.println("🎯 Selecting network (manual) using ACTIVE_MCC_MNC...");
    Serial.println("MCC/MNC: " + ACTIVE_MCC_MNC);
  }

  // Check current network
  String response = sendA7670CCommand("AT+COPS?");
  if (response.indexOf(ACTIVE_MCC_MNC) >= 0) {
    if (debugEnabled) {
      Serial.println("✅ Already on target network");
    }
    networkSelected = true;
    return true;
  }

  // Set manual selection
  if (debugEnabled) {
    Serial.println("🔄 Setting manual network selection mode...");
  }
  response = sendA7670CCommand(
    "AT+COPS=1,2,\"" + ACTIVE_MCC_MNC + "\",7",
    A7670C_SLOW_MODE ? A7670C_LONG_CMD_TIMEOUT : 60000
  );

  if (response.indexOf("OK") >= 0) {
    if (debugEnabled) {
      Serial.println("✅ Network selection command sent");
      Serial.println("⏳ Waiting for network registration...");
    }

    delay(A7670C_SLOW_MODE ? 20000 : 10000);
    esp_task_wdt_reset();

    int maxAttempts = A7670C_SLOW_MODE ? 12 : 6;
    unsigned long waitBetween = A7670C_SLOW_MODE ? 15000 : 10000;
    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
      if (debugEnabled) {
        Serial.println("  Checking attempt " + String(attempt) + "/" + String(maxAttempts) + "...");
      }
      response = sendA7670CCommand("AT+COPS?");
      if (response.indexOf(ACTIVE_MCC_MNC) >= 0) {
        if (debugEnabled) {
          Serial.println("✅ Successfully registered to target network");
        }
        networkSelected = true;
        return true;
      }
      delay(waitBetween);
      esp_task_wdt_reset();
    }

    if (debugEnabled) {
      Serial.println("❌ Failed to register to target network");
    }
    return false;
  } else {
    if (debugEnabled) {
      Serial.println("❌ Network selection command failed");
    }
    return false;
  }
}

// Add: Configure UART/flow control and error reporting on the module
bool configureA7670CSerialLink() {
  if (debugEnabled) {
    Serial.println("🛠️ Configuring A7670C serial link (IFC/IPR/CMEE)...");
  }

  // Ensure hardware flow control is disabled (RTS/CTS off)
  String response = sendA7670CCommand("AT+IFC?", A7670C_SLOW_MODE ? A7670C_DEFAULT_CMD_TIMEOUT : 5000);
  bool ifcOk = (response.indexOf("+IFC: 0,0") >= 0);
  if (!ifcOk) {
    sendA7670CCommand("AT+IFC=0,0", A7670C_SLOW_MODE ? A7670C_DEFAULT_CMD_TIMEOUT : 5000);
    // Save settings to NVRAM
    sendA7670CCommand("AT&W", A7670C_SLOW_MODE ? A7670C_DEFAULT_CMD_TIMEOUT : 3000);
    // Re-check
    response = sendA7670CCommand("AT+IFC?", A7670C_SLOW_MODE ? A7670C_DEFAULT_CMD_TIMEOUT : 5000);
    ifcOk = (response.indexOf("+IFC: 0,0") >= 0);
  }

  // Force fixed baud to 115200 (safe if already set)
  sendA7670CCommand("AT+IPR=115200", A7670C_SLOW_MODE ? A7670C_DEFAULT_CMD_TIMEOUT : 3000);
  // Confirm link still alive
  sendA7670CCommand("AT", A7670C_SLOW_MODE ? A7670C_DEFAULT_CMD_TIMEOUT : 2000);

  // Enable verbose error reporting
  sendA7670CCommand("AT+CMEE=2", A7670C_SLOW_MODE ? A7670C_DEFAULT_CMD_TIMEOUT : 2000);

  if (debugEnabled) {
    Serial.println(String("✅ IFC disabled: ") + (ifcOk ? "yes" : "unknown"));
  }
  return true;
}
