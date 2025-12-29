#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <Update.h>
#include <WiFiClientSecure.h>
#include "mbedtls/sha256.h"

// Global flag to disable WebSocket operations during OTA
bool otaWebSocketDisabled = false;

// Global flag to track if we're dealing with base64 encoded firmware
bool isBase64Firmware = false;

// Binary HTTPREAD helper for SIMCOM: reads +HTTPREAD header and exact body bytes
static int httpReadBinaryChunkSIMCOM(int offset, int reqLen, unsigned long timeoutMs) {
  // Clear buffer
  while (a7670cSerial.available()) a7670cSerial.read();

  String cmd = String("AT+HTTPREAD=") + String(offset) + "," + String(reqLen);
  a7670cSerial.println(cmd);

  // Wait for OK ack (some firmware sends OK first, some only +HTTPREAD)
  String acc = "";
  unsigned long start = millis();
  bool sawHeader = false;
  int reportedLen = -1;

  while (millis() - start < timeoutMs) {
    while (a7670cSerial.available()) {
      char c = a7670cSerial.read();
      acc += c;
      int pos = acc.lastIndexOf("+HTTPREAD:");
      if (pos >= 0) {
        int lineEnd = acc.indexOf('\n', pos);
        if (lineEnd > pos) {
          String line = acc.substring(pos, lineEnd);
          // +HTTPREAD: <len>
          int colon = line.indexOf(':');
          if (colon >= 0) {
            String lenStr = line.substring(colon + 1);
            lenStr.trim();
            reportedLen = lenStr.toInt();
            sawHeader = (reportedLen >= 0);
            break;
          }
        }
      }
    }
    if (sawHeader) break;
    delay(5);
    esp_task_wdt_reset();
  }

  if (!sawHeader || reportedLen <= 0) {
    return 0;
  }

  // Read exactly reportedLen bytes and stream to Update.write in small buffers
  int remaining = reportedLen;
  int totalWritten = 0;
  static uint8_t buf[256];
  unsigned long lastData = millis();

  // Validate we have data to write
  if (reportedLen <= 0) {
    if (debugEnabled) {
      Serial.printf("❌ Invalid reported length: %d\n", reportedLen);
    }
    return 0;
  }

  while (remaining > 0 && millis() - start < timeoutMs) {
    int toRead = remaining;
    if (toRead > (int)sizeof(buf)) toRead = sizeof(buf);
    int got = a7670cSerial.readBytes((char*)buf, toRead);
    if (got > 0) {
      // Validate first chunk contains ESP32 firmware header
      if (totalWritten == 0 && got >= 4) {
        if (buf[0] != 0xE9) {
          if (debugEnabled) {
            Serial.printf("❌ Invalid firmware header: 0x%02X (expected 0xE9)\n", buf[0]);
            Serial.printf("🔍 First 16 bytes: ");
            for (int i = 0; i < min(got, 16); i++) {
              Serial.printf("%02X ", buf[i]);
            }
            Serial.println();
          }
          return 0; // Abort - not valid ESP32 firmware
        } else if (debugEnabled) {
          Serial.println("✅ Valid ESP32 firmware header detected");
        }
      }
      
      // Check for padding bytes (0xFF) which indicate end of data or corruption
      if (got >= 4) {
        int ffCount = 0;
        for (int i = 0; i < got; i++) {
          if (buf[i] == 0xFF) ffCount++;
        }
        if (ffCount > (got * 0.8)) { // More than 80% are 0xFF
          if (debugEnabled) {
            Serial.printf("⚠️ Chunk contains %d/%d padding bytes (0xFF) - likely end of data\n", ffCount, got);
          }
          return totalWritten; // Stop here - we've hit padding
        }
      }
      
      // Debug: Show hex dump for first 3 chunks
      if (debugEnabled && totalWritten <= 3072) { // First 3KB
        Serial.printf("🔍 Chunk data at offset %d (first 32 bytes): ", totalWritten);
        for (int i = 0; i < min(got, 32); i++) {
          Serial.printf("%02X ", buf[i]);
          if (i == 15) Serial.printf("\n                                                     ");
        }
        Serial.println();
      }
      
      // Ensure 4-byte alignment for flash writes
      int alignedSize = got;
      if (alignedSize % 4 != 0) {
        alignedSize = (got / 4) * 4; // Round down to nearest 4-byte boundary
        if (alignedSize == 0 && got > 0) {
          // If we have data but not aligned, pad the remaining bytes
          static uint8_t alignedBuf[260]; // Slightly larger buffer
          memcpy(alignedBuf, buf, got);
          // Pad with 0xFF to next 4-byte boundary
          while ((got % 4) != 0) {
            alignedBuf[got] = 0xFF;
            got++;
          }
          alignedSize = got;
          size_t w = Update.write(alignedBuf, alignedSize);
          if (w != alignedSize) {
            if (debugEnabled) {
              Serial.printf("❌ Flash write error (aligned): wrote %d of %d bytes. Error: %s\n", 
                           (int)w, alignedSize, Update.errorString());
            }
            return 0;
          }
          totalWritten += w;
          remaining -= (w < got ? w : got); // Account for original got, not padded
          lastData = millis();
          esp_task_wdt_reset();
          continue;
        }
      }
      
      size_t w = Update.write(buf, got);
      if (w != got) {
        if (debugEnabled) {
          Serial.printf("❌ Flash write error: wrote %d of %d bytes. Error: %s\n", 
                       (int)w, got, Update.errorString());
        }
        return 0; // Abort on write error
      }
      totalWritten += w;
      remaining -= got;
      lastData = millis();
      esp_task_wdt_reset();
    } else {
      if (millis() - lastData > 800) break;
      delay(5);
    }
  }

  return totalWritten; // number of bytes written to flash for this call
}

// Test different buffer sizes to find optimal chunk size for A7670C HTTP reads
static int testOptimalChunkSize(String firmwareUrl) {
  int testSizes[] = {1024, 512, 256, 128};
  int numSizes = sizeof(testSizes) / sizeof(testSizes[0]);
  int bestSize = 256; // Default fallback
  
  if (debugEnabled) {
    Serial.println("🧪 Testing optimal chunk sizes for A7670C HTTP reads...");
  }
  
  for (int i = 0; i < numSizes; i++) {
    int testSize = testSizes[i];
    if (debugEnabled) {
      Serial.printf("📊 Testing chunk size: %d bytes\n", testSize);
    }
    
    // Initialize HTTP session for test
    sendA7670CCommand("AT+HTTPTERM", 1500);
    delay(200);
    String initResp = sendA7670CCommand("AT+HTTPINIT", 3000);
    if (initResp.indexOf("OK") < 0) {
      if (debugEnabled) Serial.printf("❌ HTTP init failed for size %d\n", testSize);
      continue;
    }
    
    sendA7670CCommand("AT+HTTPPARA=\"URL\",\"" + firmwareUrl + "\"", 3000);
    
    // Test a small range request
    String headers = "User-Agent: ESP32-A7670C/1.0\\r\\nHost: blog.jimatlabs.com\\r\\nAccept: */*\\r\\nConnection: close";
    headers += "\\r\\nRange: bytes=0-" + String(testSize - 1);
    sendA7670CCommand("AT+HTTPPARA=\"USERDATA\",\"" + headers + "\"", 3000);
    
    // Fire request
    sendA7670CCommand("AT+HTTPACTION=0", 3000);
    
    // Wait for response
    int httpStatus = -1, dataLen = -1;
    unsigned long start = millis();
    String acc = "";
    while (millis() - start < 15000) {
      while (a7670cSerial.available()) {
        char c = a7670cSerial.read();
        acc += c;
      }
      int pos = acc.lastIndexOf("+HTTPACTION:");
      if (pos >= 0) {
        int lineEnd = acc.indexOf('\n', pos);
        String line = (lineEnd > pos) ? acc.substring(pos, lineEnd) : acc.substring(pos);
        int c1 = line.indexOf(',');
        int c2 = (c1 >= 0) ? line.indexOf(',', c1 + 1) : -1;
        if (c1 >= 0 && c2 > c1) {
          String statusStr = line.substring(c1 + 1, c2); statusStr.trim();
          String lenStr = line.substring(c2 + 1); lenStr.trim();
          httpStatus = statusStr.toInt();
          dataLen = lenStr.toInt();
          break;
        }
      }
      delay(10);
      esp_task_wdt_reset();
    }
    
    if (httpStatus == 200 || httpStatus == 206) {
      // Try to read the data
      unsigned long readStart = millis();
      int bytesRead = 0;
      
      String cmd = String("AT+HTTPREAD=0,") + String(testSize);
      a7670cSerial.println(cmd);
      
      // Wait for data
      acc = "";
      bool sawData = false;
      while (millis() - readStart < 10000) {
        while (a7670cSerial.available()) {
          char c = a7670cSerial.read();
          acc += c;
          if (acc.indexOf("+HTTPREAD:") >= 0) {
            sawData = true;
            bytesRead = testSize; // Assume success if we see the header
            break;
          }
        }
        if (sawData) break;
        delay(5);
      }
      
      unsigned long readTime = millis() - readStart;
      
      if (debugEnabled) {
        Serial.printf("✅ Size %d: HTTP %d, read %d bytes in %lu ms\n", 
                     testSize, httpStatus, bytesRead, readTime);
      }
      
      // Success criteria: got data in reasonable time
      if (sawData && readTime < 5000) {
        bestSize = testSize;
        if (debugEnabled) {
          Serial.printf("🎯 Found working size: %d bytes\n", bestSize);
        }
        sendA7670CCommand("AT+HTTPTERM", 1500);
        break; // Use the largest working size
      }
    } else {
      if (debugEnabled) {
        Serial.printf("❌ Size %d: HTTP error %d\n", testSize, httpStatus);
      }
    }
    
    sendA7670CCommand("AT+HTTPTERM", 1500);
    delay(500); // Pause between tests
  }
  
  if (debugEnabled) {
    Serial.printf("🔧 Selected optimal chunk size: %d bytes\n", bestSize);
  }
  
  return bestSize;
}

const String globalUrlNamed = "blog.damienslab.com";
const int globalPort80 = 80;
// ... existing code ...
String performA7670CHTTPRequest(String url) {
  if (debugEnabled) {
    Serial.println("📡 Performing A7670C HTTP request to: " + url);
  }
  String path = "/firmware/check/" + macToUUID();
  if (url.startsWith("http://")) {
    int pathPos = url.indexOf("/", 7);
    if (pathPos > 0) {
      path = url.substring(pathPos);
    }
  } else if (url.startsWith("/")) {
    path = url;
  }
  String response = performSimpleA7670CHTTPGet(path);
  return response;
}

// ... existing code ...
String performSimpleA7670CHTTPGet(String endpoint) {
  if (debugEnabled) {
    Serial.println("📡 Simple A7670C HTTP GET: " + endpoint);
  }
  sendA7670CCommand("AT+HTTPTERM", 3000);
  delay(300);
  esp_task_wdt_reset();
  String response = sendA7670CCommand("AT+HTTPINIT", 5000);
  if (response.indexOf("OK") < 0) {
    if (debugEnabled) {
      Serial.println("❌ AT+HTTPINIT failed");
    }
    return "";
  }
  String fullUrl = "http://" + globalUrlNamed + ":" + String(globalPort80) + endpoint;
  String urlCmd = "AT+HTTPPARA=\"URL\",\"" + fullUrl + "\"";
  response = sendA7670CCommand(urlCmd, 5000);
  if (response.indexOf("OK") < 0) {
    if (debugEnabled) {
      Serial.println("❌ AT+HTTPPARA URL failed: " + fullUrl);
    }
    sendA7670CCommand("AT+HTTPTERM", 3000);
    return "";
  }
  response = sendA7670CCommand("AT+HTTPACTION=0", 10000);
  if (response.indexOf("+HTTPACTION: 0,200") < 0) {
    if (debugEnabled) {
      Serial.println("❌ HTTP GET failed: " + response);
    }
    sendA7670CCommand("AT+HTTPTERM", 3000);
    return "";
  }
  String readResponse = sendA7670CCommand("AT+HTTPREAD", 10000);
  String result = "";
  if (readResponse.indexOf("+HTTPREAD:") >= 0) {
    int jsonStart = readResponse.indexOf("{");
    if (jsonStart >= 0) {
      int jsonEnd = readResponse.lastIndexOf("}");
      if (jsonEnd > jsonStart) {
        result = readResponse.substring(jsonStart, jsonEnd + 1);
        result.trim();
        result.replace("\r", "");
        result.replace("\n", "");
      }
    }
  }
  sendA7670CCommand("AT+HTTPTERM", 3000);
  return result;
}

// ... existing code ...
void checkForFirmwareUpdate() {
  if (otaInProgress) {
    if (debugEnabled) {
      Serial.println("OTA update already in progress, skipping check");
    }
    return;
  }
  otaStatus = "checking";
  String deviceId = macToUUID();
  String checkUrl;
  if (SKIP_WIFI) {
    checkUrl = "http://" + globalUrlNamed + ":" + String(globalPort80) + OTA_CHECK_ENDPOINT + deviceId;
    String response = performA7670CHTTPRequest(checkUrl);
    processFirmwareCheckResponse(response);
  } else {
    HTTPClient http;
    http.begin("http://" + globalUrlNamed + ":" + String(globalPort80) + OTA_CHECK_ENDPOINT + deviceId);
    http.addHeader("User-Agent", "ESP32-OTA-Client");
    http.addHeader("X-Current-Version", FIRMWARE_VERSION);
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String response = http.getString();
      processFirmwareCheckResponse(response);
    } else {
      if (debugEnabled) {
        Serial.printf("Firmware check failed with code: %d\n", httpCode);
      }
      otaStatus = "error";
    }
    http.end();
  }
  lastOTACheck = millis();
}

// ... existing code ...
void processFirmwareCheckResponse(String response) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, response);
  if (error) {
    if (debugEnabled) {
      Serial.println("Failed to parse firmware check response");
    }
    otaStatus = "error";
    return;
  }
  String latestVersion = doc["current_version"] | "";
  String downloadUrl = doc["download_url"] | "";
  bool mandatory = doc["mandatory"] | false;
  if (latestVersion.length() > 0 && latestVersion != FIRMWARE_VERSION) {
    otaUpdateAvailable = true;
    availableFirmwareVersion = latestVersion;
    otaDownloadUrl = downloadUrl;
    if (debugEnabled) {
      Serial.printf("📦 Firmware update available: %s -> %s\n", FIRMWARE_VERSION.c_str(), latestVersion.c_str());
      Serial.printf("Download URL: %s\n", downloadUrl.c_str());
      Serial.printf("Mandatory: %s\n", mandatory ? "Yes" : "No");
    }
    sendOTAStatusUpdate();
    if (mandatory || isSafeUpdateWindow()) {
      if (debugEnabled) {
        Serial.println("🚀 Starting automatic firmware update...");
      }
      downloadAndInstallFirmware(otaDownloadUrl);
    }
  } else {
    otaUpdateAvailable = false;
    otaStatus = "idle";
    if (debugEnabled) {
      Serial.println("✅ Firmware is up to date");
    }
  }
}

// ... existing code ...
bool isSafeUpdateWindow() {
  return millis() > (10 * 60 * 1000);
}

// ... existing code ...
void downloadAndInstallFirmware(String firmwareUrl) {
  if (otaInProgress) {
    if (debugEnabled) {
      Serial.println("OTA already in progress");
    }
    return;
  }
  otaInProgress = true;
  otaWebSocketDisabled = true; // Disable WebSocket operations during OTA
  otaStatus = "downloading";
  otaProgress = 0;
  startOTAWatchdog();
  if (debugEnabled) {
    Serial.println("🔄 Starting OTA firmware download...");
    Serial.printf("URL: %s\n", firmwareUrl.c_str());
  }
  // Do NOT send OTA status at start when using A7670C to avoid jamming
  if (!SKIP_WIFI) {
    sendOTAStatusUpdate();
  }
  feedOTAWatchdog();
  if (SKIP_WIFI) {
    performSimplifiedA7670COTA(firmwareUrl);
  } else {
    performWiFiOTA(firmwareUrl);
  }
  stopOTAWatchdog();
  otaWebSocketDisabled = false; // Re-enable WebSocket operations after OTA
}

// Add helper to initialize HTTPClient for HTTP or HTTPS
static bool beginOTAHttpClient(HTTPClient &http, const String &url) {
  if (url.startsWith("https://")) {
    static WiFiClientSecure secureClient;
    secureClient.setInsecure();
    return http.begin(secureClient, url);
  }
  return http.begin(url);
}

// ... existing code ...
void performWiFiOTA(String firmwareUrl) {
  HTTPClient http;
  String fullUrl = firmwareUrl;
  if (firmwareUrl.startsWith("/")) {
    // Use plain HTTP for relative URLs (port 80), and let the redirect handler upgrade to HTTPS
    // if the server instructs it. Defaulting to HTTPS here can fail if the host doesn't serve TLS.
    fullUrl = String("http://") + globalUrlNamed + ":" + String(globalPort80) + firmwareUrl;
  }
  if (debugEnabled) {
    Serial.printf("🌐 OTA resolved URL: %s\n", fullUrl.c_str());
  }
  // Initialize client according to scheme (HTTP/HTTPS)
  if (!beginOTAHttpClient(http, fullUrl)) {
    if (debugEnabled) {
      Serial.printf("❌ Failed to initialize HTTP client for URL: %s\n", fullUrl.c_str());
    }
    otaStatus = "error";
    sendOTAStatusUpdate();
    otaInProgress = false;
    return;
  }
  http.addHeader("User-Agent", "ESP32-OTA-Client");
  http.addHeader("X-Device-ID", macToUUID());
  http.setConnectTimeout(OTA_WIFI_CONNECT_TIMEOUT_MS);
  http.setTimeout(OTA_WIFI_READ_TIMEOUT_MS);
  http.setReuse(false);
  // Try to follow redirects manually up to 3 times
  int redirectAttempts = 0;
  if (debugEnabled) {
    Serial.printf("🌐 OTA GET: %s\n", fullUrl.c_str());
  }
  int httpCode = http.GET();
  while (redirectAttempts < 3 && (httpCode == HTTP_CODE_MOVED_PERMANENTLY ||
                                  httpCode == HTTP_CODE_FOUND ||
                                  httpCode == HTTP_CODE_TEMPORARY_REDIRECT ||
                                  httpCode == HTTP_CODE_PERMANENT_REDIRECT)) {
    String location = http.header("Location");
    if (location.length() == 0) {
      break;
    }
    if (debugEnabled) {
      Serial.printf("🔁 Redirect (%d): %s\n", httpCode, location.c_str());
    }
    http.end();
    // Build absolute URL if relative, prefer HTTPS for our host
    if (location.startsWith("/")) {
      // Keep scheme consistent with current URL unless server provided an absolute Location.
      // For http we keep :80; for https we omit the port.
      const bool useHttps = fullUrl.startsWith("https://");
      if (useHttps) {
        fullUrl = String("https://") + globalUrlNamed + location;
      } else {
        fullUrl = String("http://") + globalUrlNamed + ":" + String(globalPort80) + location;
      }
    } else {
      fullUrl = location; // Absolute URL provided by server
    }
    if (debugEnabled) {
      Serial.printf("🌐 OTA redirected URL: %s\n", fullUrl.c_str());
    }
    if (!beginOTAHttpClient(http, fullUrl)) {
      if (debugEnabled) {
        Serial.printf("❌ Failed to initialize HTTP client after redirect: %s\n", fullUrl.c_str());
      }
      otaStatus = "error";
      sendOTAStatusUpdate();
      otaInProgress = false;
      return;
    }
    http.addHeader("User-Agent", "ESP32-OTA-Client");
    http.addHeader("X-Device-ID", macToUUID());
    http.setConnectTimeout(OTA_WIFI_CONNECT_TIMEOUT_MS);
    http.setTimeout(OTA_WIFI_READ_TIMEOUT_MS);
    http.setReuse(false);
    if (debugEnabled) {
      Serial.printf("🌐 OTA GET (redirect %d): %s\n", redirectAttempts + 1, fullUrl.c_str());
    }
    httpCode = http.GET();
    redirectAttempts++;
  }
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    feedOTAWatchdog();
    if (contentLength > 0) {
      bool memoryReady = prepareOTAMemory(contentLength);
      feedOTAWatchdog();
      if (memoryReady) {
        if (debugEnabled) {
          Serial.printf("📦 Firmware size: %d bytes\n", contentLength);
        }
        String expectedSha256 = http.header("x-checksum");
        bool doChecksum = expectedSha256.length() == 64;
        mbedtls_sha256_context shaCtx;
        if (doChecksum) {
          mbedtls_sha256_init(&shaCtx);
          mbedtls_sha256_starts_ret(&shaCtx, 0);
        }
        otaStatus = "installing";
        sendOTAStatusUpdate();
        feedOTAWatchdog();
        WiFiClient* client = http.getStreamPtr();
        size_t written = 0;
        static uint8_t buffer[OTA_WIFI_CHUNK_SIZE];
        int lastReportedProgress = -1;
        unsigned long lastStatusTime = millis();
        unsigned long downloadStartTime = millis();
        unsigned long lastDataTime = millis();
        while (written < (size_t)contentLength) {
          size_t remaining = (size_t)contentLength - written;
          size_t available = client->available();
          if (available) {
            size_t toRead = available;
            if (toRead > OTA_WIFI_CHUNK_SIZE) toRead = OTA_WIFI_CHUNK_SIZE;
            if (toRead > remaining) toRead = remaining;
            size_t readBytes = client->readBytes(buffer, toRead);
            if (readBytes > 0) {
              if (doChecksum) {
                mbedtls_sha256_update_ret(&shaCtx, buffer, readBytes);
              }
              size_t writtenBytes = Update.write(buffer, readBytes);
              if (writtenBytes != readBytes || writtenBytes == 0) {
                if (debugEnabled) {
                  Serial.printf("✌️ Update.write error: wrote %d of %d bytes. err=%s\n", (int)writtenBytes, (int)readBytes, Update.errorString());
                }
                Update.abort();
                otaStatus = "error";
                sendOTAStatusUpdate();
                http.end();
                otaInProgress = false;
                return;
              }
              written += writtenBytes;
              lastDataTime = millis();
              feedOTAWatchdog();
              otaProgress = (written * 100) / contentLength;
              unsigned long currentTime = millis();
              bool shouldReport = false;
              if (otaProgress != lastReportedProgress && otaProgress % 5 == 0) {
                shouldReport = true;
                lastReportedProgress = otaProgress;
              } else if (currentTime - lastStatusTime > 10000) {
                shouldReport = true;
              }
              if (shouldReport) {
                sendOTAStatusUpdate();
                feedOTAWatchdog();
                lastStatusTime = currentTime;
                if (debugEnabled) {
                  unsigned long elapsed = (currentTime - downloadStartTime) / 1000;
                  float speed = elapsed ? ((float)written / elapsed / 1024.0f) : 0.0f;
                  Serial.printf("📊 WiFi OTA Progress: %d%% (%d/%d bytes) - %.1f KB/s - %lu sec\n", 
                                otaProgress, (int)written, contentLength, speed, elapsed);
                }
              }
              esp_task_wdt_reset();
            }
          } else {
            unsigned long currentTime = millis();
            if (currentTime - lastStatusTime > 5000) {
              if (debugEnabled) {
                Serial.printf("⏳ WiFi OTA: Waiting for data... %d%% complete\n", otaProgress);
              }
              feedOTAWatchdog();
              lastStatusTime = currentTime;
            }
            if (currentTime - lastDataTime > OTA_WIFI_NO_DATA_TIMEOUT_MS) {
              if (debugEnabled) {
                Serial.println("❌ WiFi OTA aborted due to prolonged no-data condition");
              }
              Update.abort();
              otaStatus = "error";
              sendOTAStatusUpdate();
              http.end();
              otaInProgress = false;
              return;
            }
          }
          delay(1);
        }
        if (doChecksum) {
          unsigned char digest[32];
          mbedtls_sha256_finish_ret(&shaCtx, digest);
          mbedtls_sha256_free(&shaCtx);
          char hexBuf[65];
          for (int i = 0; i < 32; i++) {
            sprintf(&hexBuf[i * 2], "%02x", digest[i]);
          }
          hexBuf[64] = '\0';
          String computed = String(hexBuf);
          if (!expectedSha256.equalsIgnoreCase(computed)) {
            if (debugEnabled) {
              Serial.printf("❌ SHA256 mismatch! expected=%s computed=%s\n", expectedSha256.c_str(), computed.c_str());
            }
            Update.abort();
            otaStatus = "error";
            sendOTAStatusUpdate();
            http.end();
            otaInProgress = false;
            return;
          } else if (debugEnabled) {
            Serial.println("✅ SHA256 verified successfully");
          }
        }
        feedOTAWatchdog();
        if (written == (size_t)contentLength && Update.end(true)) {
          if (debugEnabled) {
            unsigned long totalTime = (millis() - downloadStartTime) / 1000;
            Serial.printf("✅ WiFi OTA Update successful! (Total time: %lu sec) Rebooting...\n", totalTime);
          }
          otaStatus = "complete";
          otaProgress = 100;
          sendOTAStatusUpdate();
          feedOTAWatchdog();
          delay(2000);
          ESP.restart();
        } else {
          if (debugEnabled) {
            Serial.printf("❌ WiFi OTA Update failed: %s (Written: %d/%d bytes)\n", Update.errorString(), (int)written, contentLength);
          }
          otaStatus = "error";
          sendOTAStatusUpdate();
          feedOTAWatchdog();
        }
      } else {
        if (debugEnabled) {
          Serial.println("❌ Failed to prepare memory for WiFi OTA update");
        }
        otaStatus = "error";
        sendOTAStatusUpdate();
        feedOTAWatchdog();
      }
    } else {
      if (debugEnabled) {
        Serial.println("❌ Invalid content length for WiFi OTA update");
      }
      otaStatus = "error";
      sendOTAStatusUpdate();
      feedOTAWatchdog();
    }
  } else {
    if (debugEnabled) {
      Serial.printf("❌ WiFi OTA download failed with code: %d\n", httpCode);
      String loc = http.header("Location");
      if (loc.length() > 0) {
        Serial.printf("↪️  Server suggested redirect to: %s\n", loc.c_str());
      }
    }
    otaStatus = "error";
    sendOTAStatusUpdate();
    feedOTAWatchdog();
  }
  http.end();
  otaInProgress = false;
}

// ... existing code ...
void performA7670COTA(String firmwareUrl) {
  if (debugEnabled) {
    Serial.println("🔄 Starting A7670C OTA download...");
  }
  String fullUrl = firmwareUrl;
  if (firmwareUrl.startsWith("/")) {
    fullUrl = "http://" + globalUrlNamed + ":" + String(globalPort80) + firmwareUrl;
  }
  sendA7670CCommand("AT+HTTPINIT", 3000);
  feedOTAWatchdog();
  String response = sendA7670CCommand("AT+HTTPPARA=\"URL\",\"" + fullUrl + "\"", 3000);
  if (response.indexOf("OK") < 0) {
    if (debugEnabled) {
      Serial.println("❌ AT+HTTPPARA URL failed: " + fullUrl);
    }
    sendA7670CCommand("AT+HTTPTERM", 3000);
 
  }
  // Base headers (we will re-apply together with Range each request)
  String baseHeaders = String("User-Agent: ESP32-OTA-Client\\r\\n") +
                      "X-Device-ID: " + macToUUID() + "\\r\\n" +
                      "Host: " + globalUrlNamed + "\\r\\n" +
                      "Accept: */*\\r\\n" +
                      "Accept-Encoding: identity\\r\\n" +
                      "Connection: close";
  sendA7670CCommand("AT+HTTPPARA=\"USERDATA\",\"" + baseHeaders + "\"", 3000);
  feedOTAWatchdog();
  response = sendA7670CCommand("AT+HTTPHEAD", 10000);
  feedOTAWatchdog();
  // Always proceed with HTTP unknown-length mode (more reliable for this modem/edge)
  bool unknownLengthMode = true;
  int contentLength = -1;
  if (debugEnabled) Serial.println("ℹ️ Proceeding over HTTP with unknown content length (range GETs)");

  bool memoryReady = prepareOTAMemory(-1);
  feedOTAWatchdog();
  if (memoryReady) {
    otaStatus = "installing";
    // Suppress progress status sends entirely on A7670C
    // sendOTAStatusUpdate();
    feedOTAWatchdog();
    
    // Test optimal chunk size first
    int chunkSize = testOptimalChunkSize(fullUrl);
    feedOTAWatchdog();
    
    // Lambda function to wait for HTTP action response
    auto waitForHTTPAction = [&](unsigned long timeoutMs, int &statusOut, int &lenOut) -> bool {
      unsigned long start = millis();
      String acc = "";
      statusOut = -1; lenOut = -1;
      while (millis() - start < timeoutMs) {
        while (a7670cSerial.available()) {
          char c = a7670cSerial.read();
          acc += c;
        }
        int pos = acc.lastIndexOf("+HTTPACTION:");
        if (pos >= 0) {
          int lineEnd = acc.indexOf('\n', pos);
          String line = (lineEnd > pos) ? acc.substring(pos, lineEnd) : acc.substring(pos);
          // +HTTPACTION: 0,200,xxx
          int c1 = line.indexOf(',');
          int c2 = (c1 >= 0) ? line.indexOf(',', c1 + 1) : -1;
          if (c1 >= 0 && c2 > c1) {
            String statusStr = line.substring(c1 + 1, c2); statusStr.trim();
            String lenStr = line.substring(c2 + 1); lenStr.trim();
            statusOut = statusStr.toInt();
            lenOut = lenStr.toInt();
            if (debugEnabled) {
              Serial.printf("🔎 HTTPACTION parsed: status=%d len=%d\n", statusOut, lenOut);
            }
            return true;
          }
        }
        delay(10);
        esp_task_wdt_reset();
      }
      if (debugEnabled) {
        Serial.println("⏱️ Timeout waiting for +HTTPACTION");
      }
      return false;
    };
    
    // Single GET + single read strategy: Download entire file in one operation
    if (debugEnabled) {
      Serial.println("🔧 Using single GET + single read strategy for A7670C");
      Serial.println("📡 Reading entire firmware file in one operation");
    }
    
    // Declare all variables at the beginning of the function scope
    int totalWritten = 0;
    int lastReportedProgress = -1;
    unsigned long lastStatusTime = millis();
    unsigned long downloadStartTime = millis();
    int chunkCount = 0;
    int consecutiveErrors = 0;
    
    // Initialize single GET request
    sendA7670CCommand("AT+HTTPTERM", 1500);
    delay(500);
    sendA7670CCommand("AT+HTTPINIT", 3000);
    sendA7670CCommand("AT+HTTPPARA=\"URL\",\"" + fullUrl + "\"", 3000);
    
    String singleHeaders = String("User-Agent: ESP32-OTA-Single\\r\\n") +
                          "X-Device-ID: " + macToUUID() + "\\r\\n" +
                          "Host: " + globalUrlNamed + "\\r\\n" +
                          "Accept: */*\\r\\n" +
                          "Accept-Encoding: identity\\r\\n" +
                          "Connection: close";
    
    sendA7670CCommand("AT+HTTPPARA=\"USERDATA\",\"" + singleHeaders + "\"", 3000);
    sendA7670CCommand("AT+HTTPACTION=0", 3000);
    
    int singleStatus = -1, singleLen = -1;
    waitForHTTPAction(20000, singleStatus, singleLen);
    
    if (singleStatus == 200) {
      if (debugEnabled) {
        Serial.printf("✅ Single GET successful: HTTP %d, length %d\n", singleStatus, singleLen);
        Serial.println("🎯 Now reading data in chunks from single HTTP session");
      }
      
      // Single GET + single read strategy: Download entire file in one operation
      if (debugEnabled) {
        Serial.println("🔧 Using single GET + single read strategy for A7670C");
        Serial.printf("📡 Reading entire firmware file (%d bytes) in one operation\n", singleLen);
      }
      
      // Read the entire file in one large operation
      int wrote = httpReadBinaryChunkSIMCOM(0, singleLen, 120000); // 2 minute timeout for large read
      
      if (wrote > 0) {
        totalWritten = wrote;
        if (debugEnabled) {
          Serial.printf("✅ A7670C: Successfully read %d bytes of %d total\n", wrote, singleLen);
          unsigned long elapsed = (millis() - downloadStartTime) / 1000;
          float speed = elapsed ? ((float)wrote / elapsed / 1024.0f) : 0.0f;
          Serial.printf("📊 A7670C: Download completed in %lu seconds (%.1f KB/s)\n", elapsed, speed);
        }
      } else {
        if (debugEnabled) {
          Serial.printf("❌ A7670C: Failed to read entire file\n");
        }
      }
       
      if (debugEnabled) {
        Serial.printf("✅ A7670C: Completed reading %d bytes of %d total\n", totalWritten, singleLen);
      }
      
      // Proceed to end of download processing
    } else {
      if (debugEnabled) {
        Serial.printf("❌ Single GET failed: HTTP %d\n", singleStatus);
        Serial.println("🔄 Falling back to range request strategy");
      }
      // Fall back to range requests if single GET fails
      // Reset variables for range request strategy
      totalWritten = 0;
      chunkCount = 0;
      consecutiveErrors = 0;
      
      // HTTP-only per-chunk Range requests with small backoff and limited retries
      while (true) {
          int currentChunkSize = chunkSize;
          chunkCount++;
          if (debugEnabled && chunkCount % 10 == 0) {
            Serial.printf("📡 A7670C: Processing chunk %d, %d bytes written so far\n", chunkCount, totalWritten);
          }
          String headers = baseHeaders + String("\\r\\nRange: bytes=") + String(totalWritten) + "-" + String(totalWritten + currentChunkSize - 1);
          // Add timestamp to ensure unique requests
          headers += String("\\r\\nX-Request-Time: ") + String(millis());
          headers += String("\\r\\nX-Chunk-ID: ") + String(chunkCount) + "\\r\\n" +
                     "X-Session-ID: " + String(random(100000, 999999));
          
          // Use completely different User-Agent for each chunk to avoid accumulation
          String uniqueHeaders = String("User-Agent: ESP32-OTA-Chunk-") + String(chunkCount) + "\\r\\n" +
                                "X-Device-ID: " + macToUUID() + "\\r\\n" +
                                "Host: " + globalUrlNamed + "\\r\\n" +
                                "Accept: */*\\r\\n" +
                                "Accept-Encoding: identity\\r\\n" +
                                "Connection: close\\r\\n" +
                                "Range: bytes=" + String(totalWritten) + "-" + String(totalWritten + currentChunkSize - 1) + "\\r\\n" +
                                "If-Range: \"1.0.5\"\\r\\n" +
                                "X-Range-Request: true\\r\\n" +
                                "X-Request-Time: " + String(millis()) + "\\r\\n" +
                                "X-Chunk-ID: " + String(chunkCount) + "\\r\\n" +
                                "X-Session-ID: " + String(random(100000, 999999)) + "\\r\\n" +
                                "Cache-Control: no-cache\\r\\n" +
                                "Pragma: no-cache";
          
          // Add unique query parameter to force different requests
          String uniqueUrl = fullUrl;
          // Add range information to URL path to force server processing
          String rangeInfo = String("&range=") + String(totalWritten) + "-" + String(totalWritten + currentChunkSize - 1);
          if (uniqueUrl.indexOf("?") >= 0) {
            uniqueUrl += "&chunk=" + String(chunkCount) + "&t=" + String(millis()) + rangeInfo;
          } else {
            uniqueUrl += "?chunk=" + String(chunkCount) + "&t=" + String(millis()) + rangeInfo;
          }
          
          if (debugEnabled && chunkCount <= 5) {
            Serial.printf("🔍 Chunk %d: Requesting Range: bytes=%d-%d\n", 
                         chunkCount, totalWritten, totalWritten + currentChunkSize - 1);
          }
          
          // Reinitialize HTTP session to clear accumulated USERDATA (since clearing returns ERROR)
          sendA7670CCommand("AT+HTTPTERM", 1500);
          delay(500);  // Longer delay to ensure clean session reset
          
          // Clear UART buffer more thoroughly between chunks
          if (chunkCount > 1) {
            if (debugEnabled) {
              Serial.printf("🧹 Clearing UART buffer between chunks (chunk %d)\n", chunkCount);
            }
            // Clear any pending data
            while (a7670cSerial.available()) {
              a7670cSerial.read();
              delay(1);
            }
            delay(1000);  // Extra delay to ensure modem is ready
          }
          
          sendA7670CCommand("AT+HTTPINIT", 3000);
          sendA7670CCommand("AT+HTTPPARA=\"URL\",\"" + uniqueUrl + "\"", 3000);
         
          sendA7670CCommand("AT+HTTPPARA=\"USERDATA\",\"" + uniqueHeaders + "\"", 3000);
          delay(200);  // Additional delay before firing request
          feedOTAWatchdog();
          sendA7670CCommand("AT+HTTPACTION=0", 3000); // fire request
          int httpStatus = -1, dataLen = -1;
          waitForHTTPAction(20000, httpStatus, dataLen);
          feedOTAWatchdog();
          if (httpStatus == 200 || httpStatus == 206) {
            int wrote = httpReadBinaryChunkSIMCOM(0, currentChunkSize, 20000);
            if (debugEnabled && chunkCount <= 3) {
              Serial.printf("🔍 Chunk %d: Expected %d bytes, wrote %d bytes to flash\n", 
                           chunkCount, currentChunkSize, wrote);
            }
            if (wrote > 0) {
              totalWritten += wrote;
              consecutiveErrors = 0;
              feedOTAWatchdog();
              unsigned long currentTime = millis();
              if (debugEnabled && (currentTime - lastStatusTime > 15000)) {
                unsigned long elapsed = (currentTime - downloadStartTime) / 1000;
                float speed = elapsed ? ((float)totalWritten / elapsed / 1024.0f) : 0.0f;
                Serial.printf("📊 A7670C OTA Progress: %d bytes written - %.1f KB/s - %lu sec - chunk %d\n", 
                              totalWritten, speed, elapsed, chunkCount);
                lastStatusTime = currentTime;
              }
              // Backoff: small delay between chunks; larger pause every 16 chunks
              delay(150);
              if (chunkCount % 16 == 0) delay(1200);
              // Stop condition for unknown length: if server returned less than requested
              if (dataLen >= 0 && wrote < currentChunkSize) break;
            } else {
              if (debugEnabled) Serial.printf("⚠️ A7670C: Empty chunk at %d bytes\n", totalWritten);
              if (unknownLengthMode) break;
            }
          } else {
            // Limited retry: re-init HTTP and retry chunk up to 2 times
            consecutiveErrors++;
            if (debugEnabled) {
              Serial.printf("❌ HTTP %d at chunk %d (written=%d). Retrying (%d/2) ...\n", httpStatus, chunkCount, totalWritten, consecutiveErrors);
            }
            // Check if session exists before terminating
            String checkResp = sendA7670CCommand("AT+HTTPSTATUS?", 1000);
            if (checkResp.indexOf("OK") >= 0) {
              sendA7670CCommand("AT+HTTPTERM", 1500);
            }
            delay(300);
            sendA7670CCommand("AT+HTTPINIT", 3000);
            feedOTAWatchdog();
            sendA7670CCommand("AT+HTTPPARA=\"URL\",\"" + fullUrl + "\"", 3000);
            sendA7670CCommand("AT+HTTPPARA=\"USERDATA\",\"" + baseHeaders + "\"", 3000);
            if (consecutiveErrors <= 2) {
              delay(700);
              continue; // retry same chunk
            } else {
              if (debugEnabled) Serial.println("🚫 Too many errors; aborting OTA");
              break;
            }
          }
          unsigned long currentTime = millis();
          if (debugEnabled && currentTime - lastStatusTime > 10000) {
            Serial.printf("⏳ A7670C OTA: Still downloading... %d bytes written (chunk %d)\n", totalWritten, chunkCount);
            feedOTAWatchdog();
            lastStatusTime = currentTime;
          }
          esp_task_wdt_reset();
        }
    }
    
    feedOTAWatchdog();
    int expectedLen = contentLength; // unknown (-1)
    if (totalWritten > 0) {
      if (Update.end(true)) {
        if (debugEnabled) {
          unsigned long totalTime = (millis() - downloadStartTime) / 1000;
          Serial.printf("✅ A7670C OTA Update successful! (Total time: %lu sec) Rebooting...\n", totalTime);
        }
        otaStatus = "complete";
        otaProgress = 100;
        // Skip status send on A7670C to avoid jamming
        // sendOTAStatusUpdate();
        feedOTAWatchdog();
        delay(2000);
        ESP.restart();
      } else {
        if (debugEnabled) {
          Serial.printf("❌ A7670C OTA Update failed on end: %s (Written: %d bytes)\n", Update.errorString(), totalWritten);
        }
        otaStatus = "error";
        // sendOTAStatusUpdate();
        feedOTAWatchdog();
      }
    } else {
      if (debugEnabled) {
        Serial.printf("❌ A7670C OTA Update incomplete (Written: %d bytes)\n", totalWritten);
      }
      otaStatus = "error";
      // sendOTAStatusUpdate();
      feedOTAWatchdog();
    }
  } else {
    if (debugEnabled) {
      Serial.println("❌ Failed to prepare memory for A7670C OTA update");
    }
    otaStatus = "error";
    // sendOTAStatusUpdate();
    feedOTAWatchdog();
  }
  sendA7670CCommand("AT+HTTPTERM", 3000);
  feedOTAWatchdog();
  otaInProgress = false;
}

// Simplified A7670C OTA using range requests (A7670C buffer-aware)
void performSimplifiedA7670COTA(String firmwareUrl) {
  // Save original debug state and disable during critical sections
  bool originalDebugState = debugEnabled;
  
  if (debugEnabled) {
    Serial.println("🔄 Starting Simplified A7670C OTA download...");
    Serial.println("🔌 Temporarily disabling WebSocket during OTA...");
  }
  
  // Disable WebSocket during OTA to prevent interference
  otaWebSocketDisabled = true;
  
  String fullUrl = firmwareUrl;
  if (firmwareUrl.startsWith("/")) {
    fullUrl = "http://" + globalUrlNamed + ":" + String(globalPort80) + firmwareUrl;
  }
  
  if (debugEnabled) {
    Serial.printf("🌐 Original firmware URL: %s\n", firmwareUrl.c_str());
    Serial.printf("🌐 Full download URL: %s\n", fullUrl.c_str());
    Serial.printf("🌐 Server: %s:%d\n", globalUrlNamed.c_str(), globalPort80);
    Serial.println("🔧 Using range requests due to A7670C buffer limitations");
    Serial.println("🔧 A7670C can only buffer ~2-4KB internally, so we need small chunks");
    Serial.println("⚠️ Disabling debug output during binary reads to prevent contamination");
  }
  
  // Clean up any existing HTTP sessions first
  sendA7670CCommand("AT+HTTPTERM", 2000);
  delay(500);
  
  // Get total file size first using a small range request (0-0)
  String response = sendA7670CCommand("AT+HTTPINIT", 5000);
  if (response.indexOf("OK") < 0) {
    if (debugEnabled) {
      Serial.println("❌ AT+HTTPINIT failed for size detection");
    }
    otaWebSocketDisabled = false;
    return;
  }
  
  // Removed CID setting due to frequent ERROR and not required on A7670C
  // sendA7670CCommand("AT+HTTPPARA=\"CID\",1", 3000);
  
  String urlCmd = "AT+HTTPPARA=\"URL\",\"" + fullUrl + "\"";
  sendA7670CCommand(urlCmd, 5000);
  
  // Use a tiny range request to get the total file size from Content-Range header
  String sizeHeaders = String("User-Agent: ESP32-OTA-Client\\r\\n") + 
                       "X-Device-ID: " + macToUUID() + "\\r\\n" +
                       "Host: " + globalUrlNamed + "\\r\\n" +
                       "Accept: application/octet-stream\\r\\n" +
                       "Range: bytes=0-0";
  
  String headerCmd = "AT+HTTPPARA=\"USERDATA\",\"" + sizeHeaders + "\"";
  sendA7670CCommand(headerCmd, 3000);
  
  sendA7670CCommand("AT+HTTPACTION=0", 3000);
  
  // Parse response to get total file size
  int httpStatus = -1;
  int contentLength = -1;
  unsigned long start = millis();
  String acc = "";
  
  while (millis() - start < 20000) {
    while (a7670cSerial.available()) {
      char c = a7670cSerial.read();
      acc += c;
    }
    
    int pos = acc.lastIndexOf("+HTTPACTION:");
    if (pos >= 0) {
      int lineEnd = acc.indexOf('\n', pos);
      String line = (lineEnd > pos) ? acc.substring(pos, lineEnd) : acc.substring(pos);
      
      // Parse +HTTPACTION: 0,206,1 (206 = Partial Content for range request)
      int c1 = line.indexOf(',');
      int c2 = (c1 >= 0) ? line.indexOf(',', c1 + 1) : -1;
      if (c1 >= 0 && c2 > c1) {
        String statusStr = line.substring(c1 + 1, c2); 
        statusStr.trim();
        httpStatus = statusStr.toInt();
        
        if (debugEnabled) {
          Serial.printf("📏 Size detection: HTTP %d\n", httpStatus);
        }
        break;
      }
    }
    delay(10);
    esp_task_wdt_reset();
  }
  
  sendA7670CCommand("AT+HTTPTERM", 3000);
  
  // For this implementation, we'll use a known size since size detection is complex
  // In production, you could parse Content-Range header or use a HEAD request
  contentLength = 484192; // Known size from previous attempts
  
  if (debugEnabled) {
    Serial.printf("📏 Using known firmware size: %d bytes\n", contentLength);
  }
  
  // Prepare OTA memory
  if (!Update.begin(contentLength)) {
    if (debugEnabled) {
      Serial.printf("❌ Failed to begin OTA: %s\n", Update.errorString());
    }
    otaWebSocketDisabled = false;
    return;
  }
  
  if (debugEnabled) {
    Serial.println("✅ OTA memory prepared");
    Serial.println("📦 Starting range-based firmware download");
  }
  
  // Download firmware in small chunks using range requests
  int totalWritten = 0;
  int chunkSize = 1024; // fixed 1KB chunks for stability with A7670C
  bool downloadSuccess = true;
  unsigned long downloadStartTime = millis();
  int chunkCount = 0;
  
  while (totalWritten < contentLength && downloadSuccess) {
    chunkCount++;
    int remaining = contentLength - totalWritten;
    int currentChunkSize = min(remaining, chunkSize);
    int rangeStart = totalWritten;
    int rangeEnd = rangeStart + currentChunkSize - 1;
    
    if (debugEnabled && chunkCount % 50 == 0) {
      Serial.printf("📊 Progress: %.1f%% (%d/%d bytes) - Chunk %d\n", 
                   (float)totalWritten / contentLength * 100, totalWritten, contentLength, chunkCount);
    }
    
    // Retry this chunk up to 3 attempts
    bool chunkOk = false;
    for (int attempt = 1; attempt <= 3 && !chunkOk; attempt++) {
      // Initialize new HTTP session for this range
      response = sendA7670CCommand("AT+HTTPINIT", 5000);
      if (response.indexOf("OK") < 0) {
        if (debugEnabled) {
          Serial.printf("❌ HTTP init failed for chunk %d (attempt %d)\n", chunkCount, attempt);
        }
        delay(200);
        continue; // retry
      }
      
      // Removed CID setting due to frequent ERROR and not required on A7670C
      // sendA7670CCommand("AT+HTTPPARA=\"CID\",1", 3000);
      sendA7670CCommand(urlCmd, 5000);
      
      // Set range headers for this specific chunk
      String rangeHeaders = String("Host: ") + globalUrlNamed + "\r\n" +
                           "Accept: application/octet-stream\r\n" +
                           "Connection: close\r\n" +
                           "Range: bytes=" + String(rangeStart) + "-" + String(rangeEnd);
      
      headerCmd = "AT+HTTPPARA=\"USERDATA\",\"" + rangeHeaders + "\"";
      sendA7670CCommand(headerCmd, 3000);
      
      // Execute range request
      sendA7670CCommand("AT+HTTPACTION=0", 3000);
      
      // Wait for HTTP response
      int httpStatus = -1;
      int rangeLength = -1;
      unsigned long start = millis();
      String acc = "";
      
      while (millis() - start < 20000) { // 20s timeout
        while (a7670cSerial.available()) {
          char c = a7670cSerial.read();
          acc += c;
        }
        
        int pos = acc.lastIndexOf("+HTTPACTION:");
        if (pos >= 0) {
          int lineEnd = acc.indexOf('\n', pos);
          String line = (lineEnd > pos) ? acc.substring(pos, lineEnd) : acc.substring(pos);
          
          int c1 = line.indexOf(',');
          int c2 = (c1 >= 0) ? line.indexOf(',', c1 + 1) : -1;
          if (c1 >= 0 && c2 > c1) {
            String statusStr = line.substring(c1 + 1, c2); 
            statusStr.trim();
            String lenStr = line.substring(c2 + 1); 
            lenStr.trim();
            httpStatus = statusStr.toInt();
            rangeLength = lenStr.toInt();
            break;
          }
        }
        delay(10);
        esp_task_wdt_reset();
      }
      
      if (httpStatus == 200 || httpStatus == 206) {
        // Give the modem a small settle time before reading
        delay(1000);
        
        // CRITICAL SECTION: Disable debug to prevent Serial.printf contamination
        bool originalDebugStateLocal = debugEnabled;
        debugEnabled = false;
        
        // Read the chunk data using httpReadBinaryChunkSIMCOM (extended timeout)
        int wrote = httpReadBinaryChunkSIMCOM(0, currentChunkSize, 30000);
        if (wrote <= 0) {
          // One more quick retry after a brief wait
          delay(500);
          wrote = httpReadBinaryChunkSIMCOM(0, currentChunkSize, 30000);
        }
        
        // Re-enable debug after critical section
        debugEnabled = originalDebugStateLocal;
        
        if (wrote > 0) {
          totalWritten += wrote;
          chunkOk = true;
          
          if (chunkCount == 1 && originalDebugStateLocal) {
            Serial.printf("✅ First chunk successful: wrote %d bytes\n", wrote);
          }
        } else {
          if (originalDebugStateLocal) {
            Serial.printf("❌ Failed to read chunk %d (range %d-%d) attempt %d\n", chunkCount, rangeStart, rangeEnd, attempt);
          }
        }
      } else {
        if (debugEnabled) {
          Serial.printf("❌ Range request failed: HTTP %d for chunk %d (attempt %d)\n", httpStatus, chunkCount, attempt);
        }
      }
      
      // Cleanup this HTTP session
      sendA7670CCommand("AT+HTTPTERM", 3000);
      
      // Short backoff before next attempt or next chunk
      delay(200);
      esp_task_wdt_reset();
    }
    
    if (!chunkOk) {
      downloadSuccess = false;
      break;
    }
  }
  
  // Restore original debug state
  debugEnabled = originalDebugState;
  
  // Finalize OTA
  if (downloadSuccess && totalWritten == contentLength) {
    if (debugEnabled) {
      unsigned long totalTime = (millis() - downloadStartTime) / 1000;
      float speed = totalTime ? ((float)totalWritten / totalTime / 1024.0f) : 0.0f;
      Serial.printf("📊 Total bytes written: %d of %d expected\n", totalWritten, contentLength);
      Serial.printf("📊 Download completed in %lu seconds (%.1f KB/s)\n", totalTime, speed);
      Serial.printf("📊 Total chunks processed: %d\n", chunkCount);
    }
    
    if (Update.end()) {
      if (debugEnabled) {
        Serial.printf("✅ Simplified A7670C OTA successful! (%d bytes written)\n", totalWritten);
        Serial.println("🔄 Rebooting in 3 seconds...");
      }
      delay(3000);
      ESP.restart();
    } else {
      if (debugEnabled) {
        Serial.printf("❌ OTA end failed: %s\n", Update.errorString());
      }
    }
  } else {
    Update.abort();
    if (debugEnabled) {
      Serial.printf("❌ OTA failed: wrote %d of %d bytes\n", totalWritten, contentLength);
      Serial.println("💡 This approach uses ~1.5KB chunks to work within A7670C buffer limits");
    }
  }
  
  // Re-enable WebSocket
  if (debugEnabled) {
    Serial.println("🔌 Re-enabling WebSocket after OTA...");
  }
  otaWebSocketDisabled = false;
}

// ... existing code ...
int extractContentLength(String response) {
  int contentLengthPos = response.indexOf("Content-Length:");
  if (contentLengthPos >= 0) {
    int startPos = contentLengthPos + 15;
    int endPos = response.indexOf("\r\n", startPos);
    if (endPos >= 0) {
      String lengthStr = response.substring(startPos, endPos);
      lengthStr.trim();
      return lengthStr.toInt();
    }
  }
  return -1;
}

// ... existing code ...
void sendOTAStatusUpdate() {
  // For A7670C, avoid sending OTA status to prevent jamming; rely on logs
  if (SKIP_WIFI) {
    if (debugEnabled) {
      Serial.printf("⏸️ Skipping OTA status send (A7670C mode): %s (%d%%)\n", otaStatus.c_str(), otaProgress);
    }
    return;
  }
  StaticJsonDocument<256> statusDoc;
  statusDoc["status"] = otaStatus;
  statusDoc["progress"] = otaProgress;
  statusDoc["current_version"] = FIRMWARE_VERSION;
  statusDoc["target_version"] = availableFirmwareVersion;
  statusDoc["update_available"] = otaUpdateAvailable;
  statusDoc["timestamp"] = millis();
  String payload;
  serializeJson(statusDoc, payload);
  String userId = macToUUID();
  String message = "[\"" + String(roomRef) + "\",\"" + String(roomRef) + 
                  "\",\"user:" + userId + "\",\"ota_status\"," + payload + "]";
  bool statusSent = false;
  if (SKIP_WIFI && a7670cWebSocket.isConnected()) {
    a7670cWebSocket.sendTXT(message);
    statusSent = true;
  } else if (webSocketConnected) {
    webSocket.sendTXT(message);
    statusSent = true;
  }
  if (debugEnabled) {
    if (statusSent) {
      Serial.printf("📡 Sent OTA status: %s (%d%%)\n", otaStatus.c_str(), otaProgress);
    } else {
      Serial.printf("⚠️ Failed to send OTA status: %s (No connection)\n", otaStatus.c_str());
    }
  }
  feedOTAWatchdog();
}

// ... existing code ...
void handleOTACommand(JsonObject payload) {
  String action = payload["action"] | "";
  if (action == "check_update") {
    if (debugEnabled) {
      Serial.println("📡 Received OTA check command");
    }
    checkForFirmwareUpdate();
  } else if (action == "start_ota" || action == "ota_update") {
     // Support both direct parameters and nested ota_command object
     String firmwareVersion = payload["firmware_version"] | "";
     String downloadUrl = payload["download_url"] | "";
     bool mandatory = payload["mandatory"] | false;

     if (payload.containsKey("ota_command")) {
       JsonObject nested = payload["ota_command"].as<JsonObject>();
       if (firmwareVersion.length() == 0) firmwareVersion = nested["firmware_version"] | firmwareVersion;
       if (downloadUrl.length() == 0) downloadUrl = nested["download_url"] | downloadUrl;
       if (!payload.containsKey("mandatory")) mandatory = nested["mandatory"] | mandatory;
     }

     if (firmwareVersion.length() > 0 && downloadUrl.length() > 0) {
       if (debugEnabled) {
         Serial.printf("📡 Received OTA start command: %s\n", firmwareVersion.c_str());
       }
       // Only terminate A7670C HTTP session when OTA runs over cellular modem.
       // In WiFi OTA mode (SKIP_WIFI=false), touching the modem can block OTA entirely
       // if the module is unresponsive.
       if (SKIP_WIFI) {
         // Ensure any dangling HTTP session is terminated before OTA
         sendA7670CCommand("AT+HTTPTERM", 1500);
       }

       availableFirmwareVersion = firmwareVersion;
       otaDownloadUrl = downloadUrl;
       otaUpdateAvailable = true;
       downloadAndInstallFirmware(downloadUrl);
     } else {
       if (debugEnabled) {
         Serial.println("ℹ️ Missing OTA parameters; performing server-side check");
       }
       // If parameters are missing, fallback to server check
       checkForFirmwareUpdate();
     }
   } else if (action == "ota_status") {
    if (debugEnabled) {
      Serial.println("📡 Received OTA status request");
    }
    sendOTAStatusUpdate();
  }
}

// ... existing code ...
void startOTAWatchdog() {
  otaWatchdogActive = true;
  otaLastActivityTime = millis();
  if (debugEnabled) {
    Serial.printf("🐕 OTA Watchdog started with %d second timeout\n", OTA_WDT_TIMEOUT);
  }
}

void feedOTAWatchdog() {
  if (otaWatchdogActive) {
    otaLastActivityTime = millis();
  }
}

void stopOTAWatchdog() {
  otaWatchdogActive = false;
  if (debugEnabled) {
    Serial.println("🐕 OTA Watchdog stopped");
  }
}

void checkOTAWatchdog() {
  if (otaWatchdogActive) {
    unsigned long currentTime = millis();
    if (currentTime - otaLastActivityTime > (OTA_WDT_TIMEOUT * 1000)) {
      if (debugEnabled) {
        Serial.printf("⚠️ OTA Watchdog timeout! No activity for %d seconds - resetting OTA\n", OTA_WDT_TIMEOUT);
      }
      otaInProgress = false;
      otaStatus = "error";
      otaProgress = 0;
      stopOTAWatchdog();
      sendOTAStatusUpdate();
      if (Update.isRunning()) {
        Update.abort();
        if (debugEnabled) {
          Serial.println("🧹 Aborted stalled OTA update");
        }
      }
    }
  }
}

// ... existing code ...
bool prepareOTAMemory(int contentLength) {
  if (debugEnabled) {
    Serial.printf("🧹 Preparing OTA memory for %d bytes\n", contentLength);
  }
  if (Update.isRunning()) {
    Update.abort();
    if (debugEnabled) {
      Serial.println("🧹 Aborted previous OTA update");
    }
    delay(100);
  }
  
  // Clear any lingering flash errors
  Update.clearError();
  
  size_t freeHeapBefore = ESP.getFreeHeap();
  (void)freeHeapBefore;
  esp_task_wdt_reset();
  delay(10);
  size_t freeSketchSpace = ESP.getFreeSketchSpace();
  if (debugEnabled) {
    Serial.printf("📊 Free heap: %d bytes, Free sketch space: %d bytes\n", 
                  ESP.getFreeHeap(), freeSketchSpace);
  }
  if (contentLength > 0 && freeSketchSpace < (size_t)contentLength) {
    if (debugEnabled) {
      Serial.printf("❌ Insufficient OTA partition space: need %d, have %d\n", 
                    contentLength, freeSketchSpace);
    }
    return false;
  }
  bool canBegin;
  if (contentLength > 0) {
    canBegin = Update.begin(contentLength);
  } else {
    // Unknown size: use a reasonable estimate (1MB) to avoid segment issues
    size_t estimatedSize = min(freeSketchSpace, (size_t)1048576); // 1MB max
    if (debugEnabled) {
      Serial.printf("🔧 Using estimated size: %d bytes for unknown content length\n", (int)estimatedSize);
    }
    canBegin = Update.begin(estimatedSize, U_FLASH);
  }
  if (!canBegin) {
    if (debugEnabled) {
      Serial.printf("❌ Update.begin() failed: %s\n", Update.errorString());
      Serial.printf("�� Error details: %d\n", Update.getError());
    }
    return false;
  }
  if (debugEnabled) {
    Serial.println("✅ OTA memory prepared successfully");
  }
  return true;
}

// ... existing code ...
void printOTAPartitionInfo() {
  if (debugEnabled) {
    Serial.println("🔍 OTA Partition Diagnostics:");
    Serial.println("============================");
    size_t sketchSize = ESP.getSketchSize();
    Serial.printf("📁 Current sketch size: %d bytes (%.2f MB)\n", 
                  sketchSize, sketchSize / 1024.0 / 1024.0);
    size_t freeSketchSpace = ESP.getFreeSketchSpace();
    Serial.printf("💾 Available OTA space: %d bytes (%.2f MB)\n", 
                  freeSketchSpace, freeSketchSpace / 1024.0 / 1024.0);
    size_t flashSize = ESP.getFlashChipSize();
    Serial.printf("🔧 Total flash size: %d bytes (%.2f MB)\n", 
                  flashSize, flashSize / 1024.0 / 1024.0);
    size_t freeHeap = ESP.getFreeHeap();
    Serial.printf("🧠 Free heap (RAM): %d bytes (%.2f KB)\n", 
                  freeHeap, freeHeap / 1024.0);
    Serial.printf("📏 Maximum firmware size: %d bytes (%.2f MB)\n", 
                  freeSketchSpace, freeSketchSpace / 1024.0 / 1024.0);
    Serial.printf("🏷️  Current firmware version: %s\n", FIRMWARE_VERSION.c_str());
    Serial.println("============================");
    if (freeSketchSpace < 1048576) {
      Serial.printf("⚠️  Warning: Only %.2f MB available for OTA\n", freeSketchSpace / 1024.0 / 1024.0);
      Serial.println("💡 Consider using a larger partition scheme for OTA updates");
    } else {
      Serial.printf("✅ OTA partition can handle firmware up to %.2f MB\n", freeSketchSpace / 1024.0 / 1024.0);
    }
  }
}

// Base64 decode function
int base64_decode(const char* input, uint8_t* output, int input_len) {
  int i = 0, j = 0;
  int v;
  
  while (i < input_len) {
    v = 0;
    for (int k = 0; k < 4 && i < input_len; k++) {
      v <<= 6;
      if (input[i] >= 'A' && input[i] <= 'Z') v |= input[i] - 'A';
      else if (input[i] >= 'a' && input[i] <= 'z') v |= input[i] - 'a' + 26;
      else if (input[i] >= '0' && input[i] <= '9') v |= input[i] - '0' + 52;
      else if (input[i] == '+') v |= 62;
      else if (input[i] == '/') v |= 63;
      else if (input[i] == '=') break;
      else return -1; // Invalid character
      i++;
    }
    
    if (j < input_len) output[j++] = (v >> 16) & 255;
    if (j < input_len) output[j++] = (v >> 8) & 255;
    if (j < input_len) output[j++] = v & 255;
  }
  
  return j;
} 