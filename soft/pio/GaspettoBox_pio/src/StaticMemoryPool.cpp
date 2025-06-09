#include "StaticMemoryPool.h"

StaticMemoryPool::StaticMemoryPool()
        : _freeListHead(nullptr)
{
    initPool();
}

void StaticMemoryPool::initPool()
{
    for (int i = 0; i < TELEMETRY_POOL_SIZE - 1; ++i) {
        _memory[i].next = &_memory[i + 1];
    }
    _memory[TELEMETRY_POOL_SIZE - 1].next = nullptr;

    _freeListHead = &_memory[0];
}

TelemetryData *StaticMemoryPool::allocate()
{
    TelemetryData *allocated_ptr = nullptr;
    noInterrupts();
    if (_freeListHead != nullptr) {
        allocated_ptr = reinterpret_cast<TelemetryData *>(_freeListHead);
        _freeListHead = _freeListHead->next;
    }
    interrupts();
    return allocated_ptr;
}

void StaticMemoryPool::free(TelemetryData *ptr)
{
    if (ptr == nullptr) {
        return;
    }

    if (ptr < reinterpret_cast<TelemetryData *>(&_memory[0].data) ||
        ptr > reinterpret_cast<TelemetryData *>(&_memory[TELEMETRY_POOL_SIZE - 1].data)) {
        return;
    }

    noInterrupts();
    FreeBlock *block_to_free = reinterpret_cast<FreeBlock *>(ptr);
    block_to_free->next = _freeListHead;
    _freeListHead = block_to_free;
    interrupts();
}
