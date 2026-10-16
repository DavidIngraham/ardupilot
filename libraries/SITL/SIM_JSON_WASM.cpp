/*
 * Copyright 2026. GPL-3.0-or-later.
 * AI-assisted implementation of the browser JSON lockstep transport.
 */
#include "SIM_JSON_WASM.h"

#if HAL_SITL_WASM_ENABLED
#include <atomic>
#include <cstring>
#include <emscripten/emscripten.h>
#include <emscripten/threading.h>

namespace {
// One FDM producer and one serialized JavaScript host. Ownership changes only
// through release/acquire transitions; neither side reads a buffer being written.
enum Phase : uint32_t { IDLE, SERVOS_READY, AWAITING_SENSORS, SENSORS_READY };
std::atomic<uint32_t> phase {Phase::IDLE};
static_assert(sizeof(phase) == sizeof(uint32_t), "JS futex requires a 32-bit atomic");
uint8_t servos[72];
uint8_t sensors[8192];
size_t servo_length;
size_t sensor_length;
uint32_t requested_frame;
}

size_t SITL::JSON_WASM::send_servos(const void *buffer, size_t length)
{
    if (length != 40 && length != 72) {
        return 0;
    }
    // A timeout resend must not overwrite an outstanding frame or its reply.
    if (phase.load(std::memory_order_acquire) != Phase::IDLE) {
        return length;
    }
    memcpy(servos, buffer, length);
    memcpy(&requested_frame, servos + 4, sizeof(requested_frame));
    servo_length = length;
    phase.store(Phase::SERVOS_READY, std::memory_order_release);
    emscripten_futex_wake(&phase, 1);
    return length;
}

size_t SITL::JSON_WASM::receive_sensors(void *buffer, size_t capacity)
{
    const double deadline_ms = emscripten_get_now() + 100;
    while (emscripten_get_now() < deadline_ms) {
        const uint32_t value = phase.load(std::memory_order_acquire);
        if (value == Phase::SENSORS_READY) {
            if (sensor_length > capacity) {
                return 0;
            }
            const size_t length = sensor_length;
            memcpy(buffer, sensors, length);
            phase.store(Phase::IDLE, std::memory_order_release);
            return length;
        }
        // Wake on the reply, rather than quantizing every FDM step to the host
        // OS timer resolution. This runs only on the SITL pthread.
        emscripten_futex_wait(&phase, value, 100);
    }
    return 0;
}

extern "C" {
// The host may use Atomics.waitAsync on this word; it must never modify it.
EMSCRIPTEN_KEEPALIVE uintptr_t ardupilot_json_phase()
{
    return reinterpret_cast<uintptr_t>(&phase);
}

EMSCRIPTEN_KEEPALIVE size_t ardupilot_json_read(uint8_t *buffer, size_t capacity)
{
    if (phase.load(std::memory_order_acquire) != Phase::SERVOS_READY || capacity < servo_length) {
        return 0;
    }
    const size_t length = servo_length;
    memcpy(buffer, servos, length);
    phase.store(Phase::AWAITING_SENSORS, std::memory_order_release);
    return length;
}

EMSCRIPTEN_KEEPALIVE size_t ardupilot_json_write(uint32_t frame, const uint8_t *buffer, size_t length)
{
    if (phase.load(std::memory_order_acquire) != Phase::AWAITING_SENSORS || frame != requested_frame ||
        length < 3 || length > sizeof(sensors) || buffer[0] != '\n' || buffer[length-1] != '\n') {
        return 0;
    }
    memcpy(sensors, buffer, length);
    sensor_length = length;
    phase.store(Phase::SENSORS_READY, std::memory_order_release);
    emscripten_futex_wake(&phase, 1);
    return length;
}
}
#endif
