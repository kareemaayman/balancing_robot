#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>

class EncoderReader {
private:
    // Encoder pins
    const uint8_t pinA;
    const uint8_t pinB;
    
    // Tracking
    volatile int64_t count;
    int64_t lastCount;
    int32_t lastDelta;
    
    // Speed
    unsigned long lastUpdateTime;
    double speedTicksPerSec;
    
    // ISR helper
    static EncoderReader* instances[4];  // Support up to 4 encoders
    static void IRAM_ATTR isrA(void* arg);
    static void IRAM_ATTR isrB(void* arg);

    void handleA();
    void handleB();
    
public:
    EncoderReader(uint8_t pinA, uint8_t pinB);
    
    void initialize();
    
    // Get current count
    int64_t getCount() const { return count; }
    int32_t getLastDelta() { return lastDelta; }
    
    // Update and get speed
    void updateSpeed();
    double getSpeedTicksPerSec() const { return speedTicksPerSec; }
    
    // Reset
    void reset();
    void resetSpeed();
    
    // Debug
    void printDebug();
};

#endif
