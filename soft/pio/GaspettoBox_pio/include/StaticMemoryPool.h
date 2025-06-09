#ifndef STATIC_MEMORY_POOL_H
#define STATIC_MEMORY_POOL_H

#include "TelemetryData.h"

#include <Arduino.h>

#ifndef TELEMETRY_POOL_SIZE
#define TELEMETRY_POOL_SIZE 2
#endif

class StaticMemoryPool {
public:
    StaticMemoryPool();
    TelemetryData *allocate();
    void free(TelemetryData *ptr);

private:
    union FreeBlock {
        TelemetryData data;
        FreeBlock *next;
    };

    FreeBlock _memory[TELEMETRY_POOL_SIZE];
    FreeBlock *_freeListHead;
    void initPool();
};

#endif /* STATIC_MEMORY_POOL_H */
