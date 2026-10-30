#pragma once

#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "Ir-NEC.h"
#include "Ir-Sony.h"
#include "Ir-Data.h"
#include "Remote-Air.h"

#define IR_TX_CARRIER_FREQ_HZ_NEC  38000
#define IR_TX_CARRIER_FREQ_HZ_SONY 40000
#define IR_TX_CARRIER_FREQ_HZ_DATA 38000 // data mode shares NEC's 38 kHz (RX band-pass centre)
#define IR_TX_CARRIER_FREQ_HZ_MAX  40000 // raised from 38000 so Sony's 40kHz is not clamped
// Floor of the range override (transmitRange); NEC, Sony and data use their own carriers above it.
// 28 kHz is the most detuned carrier measured (2026-10-30, see IR_RANGE_CALIBRATION): frames still
// decoded at 28.4 kHz; nothing below 28 kHz was measured at a short distance.
#define IR_TX_CARRIER_FREQ_HZ_MIN  28000
#define IR_TX_CARRIER_DUTY         0.33
#define IR_TX_QUEUE_DEPTH         4 // transactions allowed to be pending in the driver
// rmt_transmit() keeps only a pointer to the payload and the encoder reads it later (from the
// ISR once the frame is queued behind another), so every queued frame needs its own slot.
// One more than the queue depth: a slot is rewritten before rmt_transmit() blocks for a
// free transaction, at which point the oldest frame's slot may still be in flight.
#define IR_TX_PAYLOAD_SLOTS       (IR_TX_QUEUE_DEPTH + 1)

// IRM-H638J7 RX is tuned to 38 kHz: TX range peaks when the carrier matches it and
// shrinks as the carrier detunes down toward IR_TX_CARRIER_FREQ_HZ_MIN (28 kHz).
#define IR_TX_RANGE_CARRIER_HZ_MAX 38000 // == RX band-pass centre = maximum range

// The receiver needs at least 10 carrier cycles per mark (Everlight IRM-H6XX datasheet: "min. burst
// length >= 10 pulses/burst"). The shortest mark any sender uses is 560 us (NEC and data bits): 15.7
// cycles at 28 kHz. Detuning below 17.9 kHz would cut marks under the minimum, not just shorten range.
#define IR_RX_MIN_BURST_CYCLES     10
#define IR_TX_US_PER_S             1000000ULL
static_assert(IR_DATA_BIT_MARK * IR_TX_CARRIER_FREQ_HZ_MIN >= IR_RX_MIN_BURST_CYCLES * IR_TX_US_PER_S,
              "a data mark at the carrier floor is shorter than the receiver's minimum burst");
static_assert(NEC_PAYLOAD_ZERO_DURATION_0 * IR_TX_CARRIER_FREQ_HZ_MIN >= IR_RX_MIN_BURST_CYCLES * IR_TX_US_PER_S,
              "a NEC mark at the carrier floor is shorter than the receiver's minimum burst");
static_assert(IR_TX_CARRIER_FREQ_HZ_MIN < IR_TX_RANGE_CARRIER_HZ_MAX, "the range override needs a carrier span");

// range -> carrier calibration: each cm is the distance a frame at that carrier is expected to reach.
// Linearly interpolated; transmit_range below the first cm clamps to it, above the last cm to on-tune.
// Points MUST be sorted STRICTLY ascending by cm (rangeToCarrierHz divides by the cm span) and must not
// fall in Hz as cm grows; IR_RANGE_CALIBRATION_VALID below checks both.
//
// PROVISIONAL v1: ships in the 4.0.0 preview. RECALIBRATE BEFORE 4.0.0 STABLE with direct board-to-board
// distances (tape measure, 10/20/30/50/100 cm). Source: 2026-10-30, board A alone, its own frames
// (30 per step, CRC-checked) after a reflection, so these are reflected round trips, not direct distances:
//   - door, 400 cm round trip: 29.0 kHz 70-80 %, 28.4-28.8 kHz 13-73 %, 28.2 kHz and below 0 % (4 passes);
//   - ceiling, 310 cm round trip: 30.0-30.5 kHz 77-93 %, 31.5 kHz and above 100 % (2 passes);
//   - 38 kHz: 100 % on both paths.
// A reflection loses light, so a carrier's direct reach is longer than the reflected path it survives.
// 30 kHz (the old floor, labelled 10 cm) reaches more than 4 m.
struct ir_range_cal_point_t
{
    uint16_t cm;
    uint32_t hz;
};
static constexpr ir_range_cal_point_t IR_RANGE_CALIBRATION[] = {
    // PROVISIONAL GUESS, low confidence (recalibrate before 4.0.0 stable): the most detuned carrier measured;
    // the 400 cm door echo is 0 of 90 here, the direct reach is unmeasured and may well exceed 30 cm.
    {30, IR_TX_CARRIER_FREQ_HZ_MIN},
    // PROVISIONAL LOWER BOUND, medium confidence (recalibrate before 4.0.0 stable): the 400 cm door echo
    // holds 70-80 % at 29.0 kHz in 4 of 4 passes, so the direct reach is at least 400 cm.
    {400, 29000},
    // PROVISIONAL GUESS, low confidence (recalibrate before 4.0.0 stable): on-tune maximum, 100 % on every
    // reflected path tried; the 800 cm is unmeasured.
    {800, IR_TX_RANGE_CARRIER_HZ_MAX},
};
static constexpr size_t IR_RANGE_CAL_COUNT =
    sizeof(IR_RANGE_CALIBRATION) / sizeof(IR_RANGE_CALIBRATION[0]);
static_assert(IR_RANGE_CAL_COUNT >= 2, "IR_RANGE_CALIBRATION needs at least 2 points to interpolate");
static constexpr bool irRangeCalibrationValid()
{
    for (size_t i = 0; i < IR_RANGE_CAL_COUNT; i++)
    {
        const ir_range_cal_point_t p = IR_RANGE_CALIBRATION[i];
        if (p.hz < IR_TX_CARRIER_FREQ_HZ_MIN || p.hz > IR_TX_RANGE_CARRIER_HZ_MAX)
            return false;
        if (i > 0 && (p.cm <= IR_RANGE_CALIBRATION[i - 1].cm || p.hz < IR_RANGE_CALIBRATION[i - 1].hz))
            return false;
    }
    return true;
}
static constexpr bool IR_RANGE_CALIBRATION_VALID = irRangeCalibrationValid();
static_assert(IR_RANGE_CALIBRATION_VALID,
              "IR_RANGE_CALIBRATION: cm strictly ascending, Hz not falling, every Hz within the range carrier span");

class IrRemoteTx
{
public:
    IrRemoteTx();
    ~IrRemoteTx();

    // `air` is the receiver-pin edge tracker that data frames listen to before they go out (Ir-Lbs.h); null = send at once.
    void begin(gpio_num_t tx_gpio_num, uint32_t resolution_hz, const IrRemoteAir *air = nullptr);

    bool sendNEC(uint16_t address, uint16_t command);
    bool sendSony(uint16_t address, uint16_t command);

    // Send one IR data frame (pulse-distance, CRC-8). len <= IR_DATA_MAX_PAYLOAD.
    // Synchronous: takes the transmit lock, then listens before it sends: it waits, for at most IR_LBS_MAX_WAIT_US,
    // until the receiver has seen no IR activity for IR_LBS_QUIET_US, then takes a random 0..IR_LBS_SLOT_MAX_US slot
    // and reads again, and a board that had to wait adds a random 0..IR_LBS_JITTER_MAX_US of quiet (see Ir-Lbs.h).
    // Then it blocks until the frame is fully transmitted (~frame duration, hundreds of ms at the data bit rate).
    // Returns false if len is too large or the RMT transmit/wait fails.
    bool sendData(const uint8_t *bytes, uint8_t len);

    // Set IR TX usable range: maps cm -> carrier Hz via IR_RANGE_CALIBRATION (clamped
    // to the table's cm endpoints), activates a carrier override that sendNEC, sendSony
    // and sendData honor, applies it, and logs cm -> Hz at INFO (not printed where the IDF
    // runtime log level is ERROR). clearTransmitRange() reverts to the per-protocol
    // carrier (maximum range).
    void transmitRange(uint16_t cm);
    void clearTransmitRange(void);

    // Thread safety: sendNEC, sendSony, sendData and transmitRange take _tx_lock, so any task may
    // call them (Logo VM tasks on core 1, IR Share on the display task on core 0). A second sender
    // waits for the first call to return instead of rewriting the buffers it is still reading, and
    // sendData holds the lock while it listens too, so a queued sender never listens for a frame
    // another task of this board is about to send. sendNEC returns once its frame is queued, so a
    // transmitRange() right after it can still retune the carrier while that frame is on air.
    // clearTransmitRange is one aligned word store and takes no lock.

private:
    // Holds _tx_lock for the rest of the scope. A null lock (begin() not called) is a no-op.
    class TxLock
    {
    public:
        explicit TxLock(SemaphoreHandle_t lock) : _lock(lock)
        {
            if (_lock)
                xSemaphoreTake(_lock, portMAX_DELAY);
        }
        ~TxLock()
        {
            if (_lock)
                xSemaphoreGive(_lock);
        }
        TxLock(const TxLock &)            = delete;
        TxLock &operator=(const TxLock &) = delete;

    private:
        SemaphoreHandle_t _lock;
    };

    void updateCarrierFrequency(uint32_t frequency); // caller holds _tx_lock
    void _listenBeforeSend(void);                    // caller holds _tx_lock; returns when the air is clear or the wait ran out
    SemaphoreHandle_t _tx_lock = NULL;               // created in begin()
    const IrRemoteAir *_air    = nullptr;            // receiver-pin edge tracker; null = do not listen

    rmt_channel_handle_t _channel;
    rmt_carrier_config_t _carrier_cfg;
    rmt_transmit_config_t _transmit_config;
    rmt_encoder_handle_t _encoder;       // NEC custom encoder
    ir_code_t _payloads[IR_TX_PAYLOAD_SLOTS]; // NEC payload slots (rmt_transmit keeps only a pointer)
    uint8_t _next_payload;
    rmt_encoder_handle_t _copy_encoder;  // used to push pre-built Sony frames
    uint32_t _resolution;                // RMT tick resolution, for Sony frame building
    uint32_t _current_carrier_hz;        // cache to avoid redundant rmt_apply_carrier

    // Active range carrier override; 0 = inactive (sends use the per-protocol carrier).
    // Written by transmitRange() (under _tx_lock) and clearTransmitRange() (lock-free, from the
    // program-stop path), read by the senders under _tx_lock. An aligned 32-bit word, so a read is
    // never torn; volatile so a sender sees a clear on its next send.
    volatile uint32_t _range_carrier_hz = 0;

    uint32_t _carrierFor(uint32_t protocol_hz); // range override if active, else the protocol's carrier
    uint32_t rangeToCarrierHz(uint16_t cm); // clamp + linear interp over IR_RANGE_CALIBRATION

    // Scratch buffer for the outgoing data frame (built then pushed via _copy_encoder).
    // Member (not stack) to avoid ~584 bytes of stack pressure on the Logo VM task. Guarded by
    // _tx_lock: the copy encoder reads it from the ISR until the frame is fully sent.
    rmt_symbol_word_t _data_frame[IR_DATA_MAX_FRAME_SYMBOLS];
};
