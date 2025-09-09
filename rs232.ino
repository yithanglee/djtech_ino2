#include <ArduinoJson.h>

// ... existing code ...
void handleBillPoll() {
  if (SKIP_RS232) {
    if (debugEnabled) {
      Serial.println("🚫 RS232 disabled - Cannot handle bill poll!");
    }
    return;
  }
  
  billAcceptorSerial.write(ACK_RSP);
  lastBillPollTime = millis();

  if (debugEnabled) {
    Serial.println("-> Sent ACK");
  }
}

// ... existing code ...
void sendBillEvent(uint8_t billCmd) {
  if (SKIP_RS232) {
    if (debugEnabled) {
      Serial.println("🚫 RS232 disabled - Cannot send bill event!");
    }
    return;
  }
  
  billAcceptorSerial.write(0x80);
  billAcceptorSerial.write(billCmd);
  billAcceptorSerial.write(0x10);

  if (debugEnabled) {
    Serial.printf("-> Sent bill event: %02X\n", billCmd);
  }
}

// ... existing code ...
void simulateBill(uint8_t billCmd) {
  if (SKIP_RS232) {
    if (debugEnabled) {
      Serial.println("🚫 RS232 disabled - Cannot simulate bills!");
    }
    return;
  }
  
  if (!emulatorMode) {
    Serial.println("Must be in emulator mode to simulate bills!");
    return;
  }

  if (millis() - lastBillPollTime < 1000) {
    sendBillEvent(billCmd);
  } else {
    billEventPending = true;
    pendingBillCmd = billCmd;
    if (debugEnabled) {
      Serial.println("Bill event queued for next sequence");
    }
  }
}

// ... existing code ...
bool isBillCombinationPossible(int amount) {
  int remainingAmount = amount;
  const int bills[] = {10, 5, 2, 1};
  
  for (int bill : bills) {
    while (remainingAmount >= bill) {
      remainingAmount -= bill;
    }
  }
  
  return remainingAmount == 0;
}

// ... existing code ...
void logBillDispense(int billValue) {
  if (debugEnabled) {
    Serial.print("Dispensing bill value: $");
    Serial.println(billValue);
  }
}

// ... existing code ...
void sendBillAcceptorReading(uint8_t billCmd) {
  if (SKIP_RS232) {
    if (debugEnabled) {
      Serial.println("🚫 RS232 disabled - Cannot send bill acceptor reading!");
    }
    return;
  }
  
  // Convert bill command to actual value
  int billValue = 0;
  switch (billCmd) {
    case BILL_1: billValue = 1; break;
    case BILL_2: billValue = 2; break;
    case BILL_5: billValue = 5; break;
    case BILL_10: billValue = 10; break;
    default: return; // Invalid bill command
  }

  // Create JSON payload with the bill reading
  StaticJsonDocument<256> readingDoc;
  readingDoc["pin"] = 25; // RS232 RX pin
  readingDoc["type"] = "rs232";
  readingDoc["device"] = "bill_acceptor";
  readingDoc["value"] = billValue;
  readingDoc["timestamp"] = millis();

  String payload;
  serializeJson(readingDoc, payload);

  // Send through WebSocket using the same format as PWM readings
  String userId = macToUUID();
  String message = "[\"" + String(roomRef) + "\",\"" + String(roomRef) + 
                  "\",\"user:" + userId + "\",\"cash_reading\"," + payload + "]";

  webSocket.sendTXT(message);

  if (debugEnabled) {
    Serial.printf("Sent bill acceptor reading - Value: $%d\n", billValue);
  }
}

// ... existing code ...
void handleBillEvent(uint8_t billCmd) {
  // First handle the normal bill acceptor protocol
  sendBillEvent(billCmd);
  
  // Then send the reading through WebSocket
  sendBillAcceptorReading(billCmd);
}

// ... existing code ...
void sendSerialData(const uint8_t* data, size_t len) {
    if (SKIP_RS232) {
        if (debugEnabled) {
            Serial.println("🚫 RS232 disabled - Cannot send serial data!");
        }
        return;
    }
    
    if (debugEnabled) {
        Serial.print("Sending: ");
        for (size_t i = 0; i < len; i++) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
    
    // Clear any pending data
    while(billAcceptorSerial.available()) {
        billAcceptorSerial.read();
    }
    
    // Send command bytes with delays
    for (size_t i = 0; i < len; i++) {
        billAcceptorSerial.write(data[i]);
        delay(10);  // Small delay between bytes
    }
    billAcceptorSerial.flush();
    
    // Wait for response
    delay(50);  // Give device time to respond
    
    if (debugEnabled) {
        Serial.print("Response: ");
        bool gotResponse = false;
        unsigned long timeout = millis() + 500;
        int bytesRead = 0;
        uint8_t responseBuffer[10];
        
        while (millis() < timeout && bytesRead < 10) {
            if (billAcceptorSerial.available()) {
                responseBuffer[bytesRead] = billAcceptorSerial.read();
                Serial.printf("%02X ", responseBuffer[bytesRead]);
                gotResponse = true;
                bytesRead++;
                delay(5);
            }
        }
        
        if (!gotResponse) {
            Serial.println("No response");
        } else {
            Serial.println();
            if (bytesRead == 1 && responseBuffer[0] == 0x02) {
                Serial.println("(ACK received)");
            } else if (bytesRead == 2 && responseBuffer[0] == 0x5E && responseBuffer[1] == 0x3E) {
                Serial.println("(Status response received)");
            }
        }
        Serial.println("------------------------");
    }

    // Send a status request after command
    if (len > 1) {  // Only for multi-byte commands
        delay(500);
        uint8_t status[] = { 0x10 };
        sendSerialData(status, sizeof(status));
    }
} 