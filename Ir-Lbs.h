#pragma once

#include <stdint.h>

#include "Ir-Data.h"
#include "Ir-NEC.h"

/**
 * @brief Listen-before-send for IR data frames (carrier sense), pure decision logic.
 *
 * A data frame is lost when another frame overlaps it, or comes closer than the receiver's end-of-frame idle
 * threshold (the receiver then sees one merged burst). Two boards that publish on the same period used to
 * overlap on every cycle. Before a frame goes out the transmitter now waits until the receiver pin has been
 * quiet for IR_LBS_QUIET_US, for at most IR_LBS_MAX_WAIT_US, and then sends whatever it heard.
 *
 * "Heard" is the timestamp of the last edge on the receiver pin (IrRemoteAir). This file only decides; it
 * reads no clock and no pin, so the host test (test/host/test-ir-lbs.cpp) runs the same code the board runs.
 *
 * Where each number comes from (all of it is frame timing, Ir-Data.h and Ir-NEC.h):
 *  - Quiet window, IR_LBS_QUIET_US = 15 ms. It has to be longer than the longest silence inside a frame
 *    (the 4.5 ms leader space) and longer than the 12 ms of idle after which a receiver closes a frame
 *    (IR_NEC_SIGNAL_RANGE_MAX). The 3 ms on top cover the lag between the edge and its timestamp and the
 *    receiver's re-arm. It stays below the 32.767 ms idle tail every data frame ends with (the transmit call
 *    returns only after it), so a board never waits for the echo of its own previous frame.
 *  - Slot, IR_LBS_SLOT_MAX_US = 4 ms. Two boards that become ready in the same instant cannot hear each other
 *    (a frame is heard a few hundred microseconds after it starts), so they would send together, and two boards
 *    on the same period would do it again on every cycle. After finding the air quiet a board therefore waits a
 *    random 0..4 ms (a sleep of up to four 1 ms scheduler ticks) and reads the air again; the earlier one sends, the later one
 *    hears its leader mark and waits for it. The cost is 2 ms on average per frame.
 *  - Jitter, IR_LBS_JITTER_MAX_US = 16 ms. Boards that were all waiting for the same frame would wake in the
 *    same instant and collide. A board that had to wait draws one random extra quiet time in 0..16 ms; the
 *    earliest one sends and the others hear its leader mark and wait again with the same draw. 16 ms is sixteen
 *    1 ms scheduler ticks, so two waiters draw the same tick about one time in sixteen.
 *  - Maximum wait, IR_LBS_MAX_WAIT_US. The longest frame on the air takes at most 333.56 ms (16 payload bytes,
 *    every one of the 144 bits counted as a one, which a real frame cannot reach); the wait covers one such frame that starts just after the slot,
 *    the quiet window and the jitter. Longer than that, the channel is not just busy, it is noisy or saturated,
 *    and waiting more would only add latency.
 */
constexpr uint32_t IR_LBS_RX_IDLE_US      = static_cast<uint32_t>(IR_NEC_SIGNAL_RANGE_MAX / 1000ULL); // receiver closes a frame after this much silence
constexpr uint32_t IR_LBS_QUIET_MARGIN_US = 3000;
constexpr uint32_t IR_LBS_QUIET_US        = IR_LBS_RX_IDLE_US + IR_LBS_QUIET_MARGIN_US;
constexpr uint32_t IR_LBS_SLOT_MAX_US     = 4000;
constexpr uint32_t IR_LBS_JITTER_MAX_US   = 16000;
// Leader, (LEN + payload + CRC) bytes of eight one bits each (a one bit is the longer space), stop mark.
constexpr uint32_t IR_LBS_MAX_FRAME_AIR_US =
    static_cast<uint32_t>(IR_DATA_LEADER_MARK + IR_DATA_LEADER_SPACE
                          + (IR_DATA_MAX_PAYLOAD + 2) * 8 * (IR_DATA_BIT_MARK + IR_DATA_ONE_SPACE) + IR_DATA_STOP_MARK);
constexpr uint32_t IR_LBS_MAX_WAIT_US = IR_LBS_SLOT_MAX_US + IR_LBS_MAX_FRAME_AIR_US + IR_LBS_QUIET_US + IR_LBS_JITTER_MAX_US;

static_assert(IR_LBS_QUIET_US > IR_DATA_LEADER_SPACE, "the quiet window must outlast the longest silence inside a frame");
static_assert(IR_LBS_QUIET_US > IR_LBS_RX_IDLE_US, "two frames closer than the receiver idle threshold merge into one burst");
// Ticks of the stop space are microseconds at the 1 MHz RMT resolution (IR_RESOLUTION_HZ in IrRemote.cpp).
static_assert(IR_LBS_QUIET_US < IR_DATA_STOP_IDLE_TICKS, "a board must not wait for the echo of its own previous frame");

struct ir_lbs_t
{
    uint32_t start_us;  // clock when the wait began
    uint32_t jitter_us; // extra quiet time after a deferral, drawn once at the first busy reading
    bool deferred;      // the air was busy at least once during this wait
    bool slot_taken;    // the random slot after a quiet first reading has been drawn
};

enum ir_lbs_action_t
{
    IR_LBS_SEND,    // the air is clear: transmit now
    IR_LBS_WAIT,    // sleep *wait_us, then call ir_lbs_step again
    IR_LBS_GIVE_UP, // IR_LBS_MAX_WAIT_US passed and the air never cleared: transmit anyway
};

inline void ir_lbs_begin(ir_lbs_t *lbs, uint32_t now_us)
{
    lbs->start_us   = now_us;
    lbs->jitter_us  = 0;
    lbs->deferred   = false;
    lbs->slot_taken = false;
}

// One reading of the air. now_us and last_edge_us are the same free-running 32-bit microsecond clock (it wraps
// every 71.6 minutes; unsigned differences stay right across the wrap, except that an edge more than 71.6 minutes
// old can read as recent, which costs one wait of at most IR_LBS_QUIET_US + IR_LBS_JITTER_MAX_US), and last_edge_us
// must have been read before now_us so it can never be ahead of it. random_us is any fresh 32-bit random value: it is used twice at
// most, for the slot at the first quiet reading and for the jitter at the first busy one. On IR_LBS_WAIT, *wait_us
// is how long to sleep before the next reading (never past the maximum wait).
inline ir_lbs_action_t ir_lbs_step(ir_lbs_t *lbs, uint32_t now_us, uint32_t last_edge_us, uint32_t random_us, uint32_t *wait_us)
{
    const uint32_t quiet_us = now_us - last_edge_us;
    if (!lbs->deferred)
    {
        if (quiet_us >= IR_LBS_QUIET_US)
        {
            // Quiet: take the tie-break slot once, then the next reading decides.
            if (lbs->slot_taken)
                return IR_LBS_SEND;
            lbs->slot_taken     = true;
            const uint32_t slot = random_us % (IR_LBS_SLOT_MAX_US + 1);
            if (slot == 0)
                return IR_LBS_SEND;
            *wait_us = slot;
            return IR_LBS_WAIT;
        }
        lbs->deferred  = true;
        lbs->jitter_us = random_us % (IR_LBS_JITTER_MAX_US + 1);
    }

    const uint32_t needed_us = IR_LBS_QUIET_US + lbs->jitter_us;
    if (quiet_us >= needed_us)
        return IR_LBS_SEND;

    const uint32_t elapsed_us = now_us - lbs->start_us;
    if (elapsed_us >= IR_LBS_MAX_WAIT_US)
        return IR_LBS_GIVE_UP;

    const uint32_t until_clear_us = needed_us - quiet_us;
    const uint32_t until_bound_us = IR_LBS_MAX_WAIT_US - elapsed_us;
    *wait_us = until_clear_us < until_bound_us ? until_clear_us : until_bound_us;
    return IR_LBS_WAIT;
}
