#include "EncoderReader.h"

// Static instances for ISR routing
EncoderReader* EncoderReader::instances[4] = {nullptr, nullptr, nullptr, nullptr};

void EncoderReader::isrA(void* arg) {
    EncoderReader* encoder = (EncoderReader*)arg;
    if (encoder) encoder->handleA();
}

void EncoderReader::isrB(void* arg) {
    EncoderReader* encoder = (EncoderReader*)arg;
    if (encoder) encoder->handleB();
}

void EncoderReader::handleA() {
    if (digitalRead(pinA) == digitalRead(pinB)) {
        count++;
    } else {
        count--;
    }
}

void EncoderReader::handleB() {
    if (digitalRead(pinA) != digitalRead(pinB)) {
        count++;
    } else {
        count--;
    }
}

EncoderReader::EncoderReader(uint8_t pinA, uint8_t pinB, 
                             uint16_t PPR, uint16_t gearRatio)
    : pinA(pinA), pinB(pinB), PPR(PPR), gearRatio(gearRatio),
      ticksPerRev(PPR * 4 * gearRatio),
      count(0), lastCount(0), lastDelta(0),
      lastUpdateTime(0), speedTicksPerSec(0) {}

void EncoderReader::initialize() {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    
    // Attach interrupts with IRAM_ATTR
    attachInterruptArg(digitalPinToInterrupt(pinA), isrA, (void*)this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(pinB), isrB, (void*)this, CHANGE);
    
    lastUpdateTime = millis();
    Serial.print("Encoder initialized on pins ");
    Serial.print(pinA);
    Serial.print(", ");
    Serial.println(pinB);
}

void EncoderReader::updateSpeed() {
    unsigned long now = millis();
    unsigned long dt = now - lastUpdateTime;
    
    if (dt == 0) return;
    
    lastDelta = count - lastCount;
    lastCount = count;
    
    // Convert to ticks per second
    speedTicksPerSec = (double)lastDelta * 1000.0 / dt;
    
    lastUpdateTime = now;
}

double EncoderReader::getSpeedRPM() const {
    // Convert ticks/sec to RPM
    return (speedTicksPerSec / ticksPerRev) * 60.0;
}

void EncoderReader::reset() {
    count = 0;
    lastCount = 0;
    lastDelta = 0;
}

void EncoderReader::resetSpeed() {
    speedTicksPerSec = 0;
    lastDelta = 0;
}

void EncoderReader::printDebug() {
    Serial.print("Count: ");
    Serial.print(count);
    Serial.print(" | Delta: ");
    Serial.print(lastDelta);
    Serial.print(" | Speed: ");
    Serial.print(speedTicksPerSec, 0);
    Serial.print(" ticks/s (");
    Serial.print(getSpeedRPM(), 1);
    Serial.println(" RPM)");
}
