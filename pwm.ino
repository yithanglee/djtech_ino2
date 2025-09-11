#include <ArduinoJson.h>
#include <driver/pcnt.h>
// Removed PCNT-based implementation, reverting to ISR-based pulse counting

// ... existing code ...
// PCNT helpers
static bool pcntConfigured = false;
static pcnt_unit_t pcntUnit = PCNT_UNIT_0;
static pcnt_channel_t pcntChannel = PCNT_CHANNEL_0;

static void pwmInitPCNTIfEnabled() {
  if (!pwmConfig.isConfigured || !pwmConfig.usePCNT || pcntConfigured) return;
  // Configure PCNT on the input pin
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = pwmConfig.inputPin;
  cfg.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  cfg.unit = pcntUnit;
  cfg.channel = pcntChannel;
  cfg.pos_mode = PCNT_COUNT_INC;     // count rising edges
  cfg.neg_mode = PCNT_COUNT_DIS;     // ignore falling
  cfg.lctrl_mode = PCNT_MODE_KEEP;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  cfg.counter_h_lim = 32767;
  cfg.counter_l_lim = 0;
  pcnt_unit_config(&cfg);

  // Glitch filter: ignore pulses shorter than ~5us (APB 80MHz => 400 cycles ~ 5us)
  pcnt_set_filter_value(pcntUnit, 400);
  pcnt_filter_enable(pcntUnit);

  pcnt_counter_pause(pcntUnit);
  pcnt_counter_clear(pcntUnit);
  pcnt_counter_resume(pcntUnit);
  pcntConfigured = true;
}

static int16_t pwmPCNTReadAndClear() {
  if (!pcntConfigured) return 0;
  int16_t cnt = 0;
  pcnt_get_counter_value(pcntUnit, &cnt);
  pcnt_counter_clear(pcntUnit);
  if (cnt < 0) cnt = 0;
  return cnt;
}

void pwmPauseCountingForBlock() {
  if (pcntConfigured) pcnt_counter_pause(pcntUnit);
}

void pwmResumeCountingAfterBlock() {
  if (pcntConfigured) {
    pcnt_counter_clear(pcntUnit);
    pcnt_counter_resume(pcntUnit);
  }
}
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
    // Take delta pulses for the last interval and reset the counter for the next window
    pwmInitPCNTIfEnabled();
    unsigned long intervalPulses = 0;
    if (pwmConfig.usePCNT) {
      intervalPulses = (unsigned long)pwmPCNTReadAndClear();
    } else {
      noInterrupts();
      intervalPulses = pwmConfig.pulseCount;
      pwmConfig.pulseCount = 0;
      interrupts();
    }

    // For accumulate-until-quiescent mode, we only care about whether new pulses arrived

    // Initialize aggregation window if needed and accumulate pulses
    if (pwmConfig.aggregateStartTime == 0) pwmConfig.aggregateStartTime = currentTime;
    pwmConfig.aggregatedPulses += intervalPulses;

    // Debug print raw pulses for this interval
    if (Serial.availableForWrite() > 32) Serial.println(intervalPulses);

    // Track quiescence: if no new pulses this window, increase streak
    if (intervalPulses == 0) pwmConfig.consistentReadings++; else pwmConfig.consistentReadings = 0;

    // Send only when quiescent for N intervals (consistent) to finalize accumulation
    if (currentTime >= pwmConfig.sendSuppressUntil && pwmConfig.consistentReadings >= pwmConfig.CONSISTENT_COUNT && pwmConfig.aggregatedPulses > 0) {
      unsigned long windowMs = (pwmConfig.aggregateStartTime > 0) ? (currentTime - pwmConfig.aggregateStartTime) : (pwmConfig.CONSISTENT_COUNT * pwmConfig.READING_INTERVAL);
      if (windowMs == 0) windowMs = pwmConfig.CONSISTENT_COUNT * pwmConfig.READING_INTERVAL;

      float windowFrequency = (float)pwmConfig.aggregatedPulses * (1000.0 / (float)windowMs);

      StaticJsonDocument<256> readingDoc;
      readingDoc["pin"] = pwmConfig.inputPin;
      readingDoc["frequency"] = windowFrequency;
      readingDoc["pulse_count"] = pwmConfig.aggregatedPulses;
      readingDoc["window_ms"] = windowMs;
      String payload;
      serializeJson(readingDoc, payload);
      String userId = macToUUID();
      String message = "[\"" + String(roomRef) + "\",\"" + String(roomRef) + "\",\"user:" + userId + "\",\"pwm_readings\"," + payload + "]";
      webSocket.sendTXT(message);
      if (Serial.availableForWrite() > 32) {
        Serial.print("Sent accumulated PWM (quiescent) - freq: ");
        Serial.print(windowFrequency);
        Serial.print(" Hz, pulses: ");
        Serial.print(pwmConfig.aggregatedPulses);
        Serial.print(", window: ");
        Serial.println(windowMs);
      }

      pwmConfig.lastSendTime = currentTime;
      pwmConfig.aggregatedPulses = 0;
      pwmConfig.aggregateStartTime = currentTime;
      pwmConfig.consistentReadings = 0;
    }

    pwmConfig.lastReadingTime = currentTime;
  }
}