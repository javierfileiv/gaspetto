#pragma once

#ifndef ARDUINO
#include <cassert>
#include <cstdint>
#define __ASSERT_USE_STDERR
#include <assert.h>

#else /* ARDUINO */
#ifndef assert
#define assert(x)                                  \
    do {                                           \
        if (!(x)) {                                \
            Serial.print(F("Assertion failed: ")); \
            Serial.print(F(#x));                   \
            Serial.print(F(" in "));               \
            Serial.print(__FILE__);                \
            Serial.print(F(" at line "));          \
            Serial.println(__LINE__);              \
            while (1) {                            \
                delay(1000);                       \
            }                                      \
        }                                          \
    } while (0)
#endif /* assert */
#endif /* ARDUINO */

#ifndef G_ASSERT
#define G_ASSERT(x) assert(x)
#endif
