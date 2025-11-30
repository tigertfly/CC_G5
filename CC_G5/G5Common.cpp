#include "G5Common.h"

// Hardware interface instance
G5_Hardware g5Hardware;

LGFX lcd = LGFX();

// Used in both:
LGFX_Sprite headingBox(&lcd);
LGFX_Sprite gsBox(&lcd);

// Send a pseudo encoder event to MF.
void sendEncoder(String name, int count, bool increase)
{
    // cmdMessenger.sendCmdStart(kButtonChange);
    Serial.printf("sendEncoder: %s count=%d increase=%d\n", name.c_str(), count, increase);
    for (int i = 0; i < count; i++) {
        cmdMessenger.sendCmdStart(kEncoderChange);
        cmdMessenger.sendCmdArg(name);
        cmdMessenger.sendCmdArg(increase ? 0 : 2);
        cmdMessenger.sendCmdEnd();
    }
}

#ifdef USE_GUITION_SCREEN
// GPIO-based implementation for Guition screen with piggyback board using ESP32Encoder library

void G5_Hardware::init()
{
    // Initialize ESP32Encoder library
    ESP32Encoder::useInternalWeakPullResistors = UP; // Enable internal pull-ups
    encoder.attachHalfQuad(ENCODER_A_PIN, ENCODER_B_PIN);
    encoder.setCount(0);

    // Setup button pins with pull-ups
    pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
    pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);

    // Initialize button states
    lastEncoderButtonState = digitalRead(ENCODER_BUTTON_PIN);
    lastPowerButtonState   = digitalRead(POWER_BUTTON_PIN);

    ESP_LOGI("G5_HARDWARE", "ESP32Encoder initialized - A:%d, B:%d, EncBtn:%d, PwrBtn:%d",
             ENCODER_A_PIN, ENCODER_B_PIN, ENCODER_BUTTON_PIN, POWER_BUTTON_PIN);
}

void G5_Hardware::update()
{
    updateButtons();
}

void G5_Hardware::updateButtons()
{
    unsigned long currentTime = millis();
    if (currentTime - lastButtonUpdate < DEBOUNCE_DELAY) {
        return; // Too soon for another update
    }
    lastButtonUpdate = currentTime;

    // Read current button states (active low with pull-ups)
    bool currentEncoderBtn = !digitalRead(ENCODER_BUTTON_PIN);
    bool currentPowerBtn   = !digitalRead(POWER_BUTTON_PIN);

    // Handle encoder button
    if (currentEncoderBtn != lastEncoderButtonState) {
        if (currentEncoderBtn) {
            // Button pressed
            encoderButtonPressTime   = currentTime;
            encoderButtonLongPressed = false;
            encoderButton            = BUTTON_PRESSED;
        } else {
            // Button released
            if (!encoderButtonLongPressed) {
                encoderButton = BUTTON_CLICKED;
            } else {
                encoderButton = BUTTON_RELEASED;
            }
        }
        lastEncoderButtonState = currentEncoderBtn;
    } else if (currentEncoderBtn && !encoderButtonLongPressed) {
        // Check for long press
        if (currentTime - encoderButtonPressTime > LONG_PRESS_TIME) {
            encoderButtonLongPressed = true;
            encoderButton            = BUTTON_LONG_PRESSED;
        }
    } else if (!currentEncoderBtn) {
        encoderButton = BUTTON_IDLE;
    }

    // Handle power button
    if (currentPowerBtn != lastPowerButtonState) {
        if (currentPowerBtn) {
            // Button pressed
            powerButtonPressTime   = currentTime;
            powerButtonLongPressed = false;
            extraButton            = BUTTON_PRESSED;
        } else {
            // Button released
            if (!powerButtonLongPressed) {
                extraButton = BUTTON_CLICKED;
            } else {
                extraButton = BUTTON_RELEASED;
            }
        }
        lastPowerButtonState = currentPowerBtn;
    } else if (currentPowerBtn && !powerButtonLongPressed) {
        // Check for long press
        if (currentTime - powerButtonPressTime > LONG_PRESS_TIME) {
            powerButtonLongPressed = true;
            extraButton            = BUTTON_LONG_PRESSED;
        }
    } else if (!currentPowerBtn) {
        extraButton = BUTTON_IDLE;
    }
}

bool G5_Hardware::readEncoderData(int8_t &outDelta, int8_t &outEncButton, int8_t &outExtraButton)
{
    // Get current encoder count
    int64_t currentCount = encoder.getCount();
    int64_t rawDelta     = currentCount - lastEncoderCount;
    lastEncoderCount     = currentCount;

    // Apply direction setting from build flags
#ifdef ENCODER_DIRECTION_REVERSED
#if ENCODER_DIRECTION_REVERSED == 1
    rawDelta = -rawDelta;
#endif
#endif

    // Handle encoder cycles per detent from build flags
    static int64_t accumulatedRawDelta = 0;
    accumulatedRawDelta += rawDelta;

    // Get cycles per detent from build flag, default to 2 if not defined
#ifndef ENCODER_CYCLES_PER_DETENT
#define ENCODER_CYCLES_PER_DETENT 2
#endif

    int64_t detentDelta = accumulatedRawDelta / ENCODER_CYCLES_PER_DETENT;
    accumulatedRawDelta = accumulatedRawDelta % ENCODER_CYCLES_PER_DETENT; // Keep remainder

    // Update accumulated value
    encoderValue += detentDelta;

    // Check if we have any changes to report
    if (detentDelta == 0 && encoderButton == BUTTON_IDLE && extraButton == BUTTON_IDLE) {
        return false;
    }

    outDelta       = (int8_t)detentDelta;
    outEncButton   = encoderButton;
    outExtraButton = extraButton;

    // Debug output
    if (detentDelta != 0) {
        Serial.printf("readEncoderData: rawDelta=%d, detentDelta=%d, cyc/det=%d, rev=%d (count=%lld)\n",
                      (int)rawDelta, (int)detentDelta, ENCODER_CYCLES_PER_DETENT,
#ifdef ENCODER_DIRECTION_REVERSED
                      ENCODER_DIRECTION_REVERSED,
#else
                      0,
#endif
                      currentCount);
    } // Clear button states after reading
    if (encoderButton == BUTTON_CLICKED || encoderButton == BUTTON_LONG_PRESSED || encoderButton == BUTTON_RELEASED) {
        encoderButton = BUTTON_IDLE;
    }
    if (extraButton == BUTTON_CLICKED || extraButton == BUTTON_LONG_PRESSED || extraButton == BUTTON_RELEASED) {
        extraButton = BUTTON_IDLE;
    }

    return true;
}

#else
// Legacy I2C implementation for non-Guition setups

bool G5_Hardware::readEncoderData(int8_t &outDelta, int8_t &outEncButton, int8_t &outExtraButton)
{
    if (!dataAvailable) {
        return false;
    }

    dataAvailable = false;

    size_t retSize = Wire.requestFrom(RP2040_ADDR, 3); // Request 3 bytes

    if (Wire.available() >= 3) {
        outDelta       = (int8_t)Wire.read();
        outEncButton   = (int8_t)Wire.read();
        outExtraButton = (int8_t)Wire.read();

        // Update internal state
        encoderValue += outDelta;
        encoderButton = outEncButton;
        extraButton   = outExtraButton;

        return true;
    }

    return false;
}

void G5_Hardware::setLedState(bool state)
{
    Wire.beginTransmission(RP2040_ADDR);
    Wire.write(0x01); // LED control command
    Wire.write(state ? 1 : 0);
    Wire.endTransmission();
}

#endif

// This function is an exponential decay that allows changing values with a smooth motion
// Desired response time	α (approx)	Feel (at 20hz)
// 0.2 s (snappy pointer)	0.75	very responsive
// 0.5 s (analog needle)	0.3	smooth, analog feel
// 1.0 s (heavy needle)	0.15	sluggish but realistic
// 2.0 s (slow gauge)	0.075	big thermal or pressure gauge feel
// The snapThreashold is the same unit/magnitude as the input. At a minimum it should be the smallest changable value.
int smoothInput(int input, int current, float alpha, int threashold = 1)
{
    int diff = input - current;
    if (abs(diff) <= threashold) return input;

    int update = (int)(alpha * diff);
    if (update == 0 && abs(diff) > threashold) update = (diff > 0) ? 1 : -1;
    return current + update;
}

float smoothInput(float input, float current, float alpha, float snapThreshold = 0.5f)
{
    float diff   = input - current;
    float update = alpha * diff;

    // When update becomes smaller than snapThreshold * diff, snap to target
    // if (abs(update) < abs(diff) * snapThreshold) {
    if (fabs(update) < snapThreshold) {
        return input;
    }
    return current + update;
}

// Smooth a floating point directional value
float smoothDirection(float input, float current, float alpha, float threshold = 0.5f)
{
    // Handle angle wrapping (crossing 0°/360°)
    float diff = input - current;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    // Snap to target when close enough
    if (abs(diff) < threshold) {
        return input;
    }

    float result = current + alpha * diff;

    // Normalize to 0-360
    if (result < 0.0f) result += 360.0f;
    if (result >= 360.0f) result -= 360.0f;

    return result;
}

float smoothAngle(float input, float current, float alpha, float threshold = 0.5f)
{
    // Handle angle wrapping for -180 to +180 range (for bank angle, etc.)
    float diff = input - current;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    // Snap to target when close enough
    if (abs(diff) < threshold) {
        return input;
    }

    float result = current + alpha * diff;

    // Normalize to -180 to +180
    while (result > 180.0f)
        result -= 360.0f;
    while (result < -180.0f)
        result += 360.0f;

    return result;
}

CC_G5_Settings g5Settings;
G5State        g5State;

bool loadSettings()
{
    if (MFeeprom.read_block(CC_G5_SETTINGS_OFFSET, g5Settings)) {
        if (g5Settings.version != SETTINGS_VERSION) {
            g5Settings = CC_G5_Settings(); // Reset to defaults
                                           //            ESP_LOGE("PREF", "Settings Version mismatch\n");
        }
        ESP_LOGV("PREF", "Settings back. Device: %d\n", g5Settings.deviceType);
        return true;
    } else {
        // EEPROM read failed, defaults already set
        //    ESP_LOGE("PREF", "Load settings failed.\n");
        return false;
    }
}

bool saveSettings()
{
    bool retval;

    retval = MFeeprom.write_block(CC_G5_SETTINGS_OFFSET, g5Settings);
    if (retval) MFeeprom.commit();
    //  ESP_LOGV("PREF", "Settings saved. retval: %d\n", retval);
    return retval;
}
