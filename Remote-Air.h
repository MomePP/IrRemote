#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_attr.h"

// Edge timestamps on the IR receiver pin, so a sender can tell whether anybody else is on the air
// (listen-before-send, Ir-Lbs.h). The RMT RX channel only reports a frame 12 ms after its last edge, so it cannot
// say that one is in progress; a GPIO interrupt on the same pin sees every edge as it happens. The pin stays
// routed to the RMT RX channel (a GPIO interrupt reads the pad, the RMT reads the matrix input), and an edge
// costs a few microseconds of the core that called begin().
class IrRemoteAir
{
public:
    // Call after IrRemoteRx::begin() on the same pin, from the core the IR channels live on: the GPIO interrupt
    // runs on the core that installed the GPIO ISR service, which is this call's core unless the Arduino core's
    // attachInterrupt() installed it first. Never aborts: when the interrupt cannot be set up the air is reported
    // free for ever, so a send never waits for it (the short random slot of Ir-Lbs.h still applies).
    void begin(gpio_num_t rx_gpio_num);

    // esp_timer_get_time() truncated to 32 bits at the last edge of the receiver pin, own frames included.
    // One aligned word written by the interrupt, so a read is never torn. Read it before reading the clock
    // it is compared against.
    uint32_t lastEdgeUs(void) const { return _last_edge_us; }

private:
    static void IRAM_ATTR _onEdge(void *arg);

    volatile uint32_t _last_edge_us = 0;
};
