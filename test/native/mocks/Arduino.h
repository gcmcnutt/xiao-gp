#pragma once

#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <cmath>

using byte = std::uint8_t;

// Provide minimal Arduino compatibility needed by headers.
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

inline unsigned long millis() { return 0; }
inline unsigned long micros() { return 0; }
inline void delay(unsigned long) {}
