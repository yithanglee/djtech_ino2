#include <ArduinoJson.h>
// Removed PCNT-based implementation, reverting to ISR-based pulse counting

// ... existing code ...
void startPWM(int pin, int dutyCycle) {
  ledcAttachPin(pin, pwmChannel);
  ledcSetup(pwmChannel, frequency, resolution);
  ledcWrite(pwmChannel, dutyCycle);
}

// ... existing code ...
void stopPWM(int pin) {
  // ledcDetachPin(pin);
  // pinMode(pin, INPUT);
  ledcWrite(pwmChannel, 1);
}

// ... existing code ...
void sendPWMReadings() {
  if (!pwmConfig.isConfigured || pwmConfig.isBlocked) return;
  unsigned long currentTime = millis();
  if (currentTime - pwmConfig.lastReadingTime >= pwmConfig.READING_INTERVAL) {
    noInterrupts();
    unsigned long pulseCount = pwmConfig.pulseCount;
    interrupts();
    if (Serial.availableForWrite() > 32) Serial.println(pulseCount);
    if (pulseCount == 0) {
      pwmConfig.lastReadingTime = currentTime;
      pwmConfig.consistentReadings = 0;
      pwmConfig.lastPulseCount = pulseCount;
      return;
    }
    if (pulseCount == pwmConfig.lastPulseCount) {
      pwmConfig.consistentReadings++;
      if (pwmConfig.consistentReadings >= pwmConfig.CONSISTENT_COUNT) {
        noInterrupts();
        pwmConfig.pulseCount = 0;
        interrupts();
        float currentFrequency = (float)pulseCount * (1000.0 / pwmConfig.READING_INTERVAL);
        StaticJsonDocument<256> readingDoc;
        readingDoc["pin"] = pwmConfig.inputPin;
        readingDoc["frequency"] = currentFrequency;
        readingDoc["pulse_count"] = pulseCount;
        readingDoc["is_consistent"] = true;
        String payload;
        serializeJson(readingDoc, payload);
        String userId = macToUUID();
        String message = "[\"" + String(roomRef) + "\",\"" + String(roomRef) + "\",\"user:" + userId + "\",\"pwm_readings\"," + payload + "]";
        webSocket.sendTXT(message);
        if (Serial.availableForWrite() > 32) {
          Serial.print("Sent consistent PWM reading - Frequency: ");
          Serial.print(currentFrequency);
          Serial.println(" Hz");
        }
        pwmConfig.consistentReadings = 0;
      }
    } else {
      pwmConfig.consistentReadings = 0;
    }
    pwmConfig.lastPulseCount = pulseCount;
    pwmConfig.lastReadingTime = currentTime;
  }
}