#pragma once

#include <stdint.h>

/* The chip retransmits a few times before it gives up, so this bounds the wait
 * after MAX_RT rather than the whole exchange; txStandBy still gives the real
 * verdict */
constexpr uint32_t kTxTimeoutMs = 10;
