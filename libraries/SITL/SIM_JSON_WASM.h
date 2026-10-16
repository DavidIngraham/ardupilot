/*
 * In-memory transport for the JSON SITL protocol in a WebAssembly host.
 * GPL-3.0-or-later. AI-assisted implementation.
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#if HAL_SITL_WASM_ENABLED
#include <stddef.h>
#include <stdint.h>

namespace SITL {
namespace JSON_WASM {
size_t send_servos(const void *buffer, size_t length);
size_t receive_sensors(void *buffer, size_t capacity);
}
}
#endif
