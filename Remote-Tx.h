#pragma once

#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "Ir-NEC.h"
#include "Ir-Sony.h"
#include "Ir-Data.h"

#define IR_TX_CARRIER_FREQ_HZ_NEC  38000
#define IR_TX_CARRIER_FREQ_HZ_SONY 40000
#define IR_TX_CARRIER_FREQ_HZ_DATA 38000 // data mode shares NEC's 38 kHz (RX band-pass centre)
#define IR_TX_CARRIER_FREQ_HZ_MAX  40000 // raised from 38000 so Sony's 40kHz is not clamped
#define IR_TX_CARRIER_FREQ_HZ_MIN  30000
#define IR_TX_CARRIER_DUTY         0.33
// Data frames wait a random 0..IR_DATA_TX_BACKOFF_MAX_MS before they go out, so two boards that publish
// on the same period drift apart instead of overlapping every time. This is the most latency a
// sendData() call adds; about half of it on average.
#define IR_DATA_TX_BACKOFF_MAX_MS 100
#define IR_TX_QUEUE_DEPTH         4 // transactions allowed to be pending in the driver
// rmt_transmit() keeps only a pointer to the payload and the encoder reads it later (from the
// ISR once the frame is queued behind another), so every queued frame needs its own slot.
// One more than the queue depth: a slot is rewritten before rmt_transmit() blocks for a
// free transaction, at which point the oldest frame's slot may still be in flight.
#define IR_TX_PAYLOAD_SLOTS       (IR_TX_QUEUE_DEPTH + 1)

// IRM-H638J7 RX is tuned to 38 kHz: TX range peaks when the carrier matches it and
// shrinks as the carrier detunes down toward IR_TX_CARRIER_FREQ_HZ_MIN (30 kHz).
#define IR_TX_RANGE_CARRIER_HZ_MAX 38000 // == RX band-pass centre = maximum range

// PROVISIONAL range->carrier calibration — REFINE FROM BENCH LOGS.
// Workflow: call `transmit_range <cm>`, read the logged carrier Hz on serial,
// measure the actual max distance, then edit these points so each cm maps to the
// measured distance. Points MUST be sorted STRICTLY ascending by cm (no duplicate cm — rangeToCarrierHz divides by the cm span). Linearly interpolated.
struct ir_range_cal_point_t
{
    uint16_t cm;
    uint32_t hz;
};
static constexpr ir_range_cal_point_t IR_RANGE_CALIBRATION[] = {
    {10,  IR_TX_CARRIER_FREQ_HZ_MIN},   // PROVISIONAL near end (most detuned)
    {200, IR_TX_RANGE_CARRIER_HZ_MAX},  // PROVISIONAL far end  (on-tune, max range)
};
static constexpr size_t IR_RANGE_CAL_COUNT =
    sizeof(IR_RANGE_CALIBRATION) / sizeof(IR_RANGE_CALIBRATION[0]);
static_assert(IR_RANGE_CAL_COUNT >= 2, "IR_RANGE_CALIBRATION needs at least 2 points to interpolate");

class IrRemoteTx
{
public:
    IrRemoteTx();
    ~IrRemoteTx();

    void begin(gpio_num_t tx_gpio_num, uint32_t resolution_hz);

    bool sendNEC(uint16_t address, uint16_t command);
    bool sendSony(uint16_t address, uint16_t command);

    // Send one IR data frame (pulse-distance, CRC-8). len <= IR_DATA_MAX_PAYLOAD.
    // Synchronous: blocks the caller until the frame is fully transmitted
    // (~frame duration, hundreds of ms at the data bit rate). Returns false if
    // len is too large or the RMT transmit/wait fails.
    bool sendData(const uint8_t *bytes, uint8_t len);

    // Set IR TX usable range: maps cm -> carrier Hz via IR_RANGE_CALIBRATION (clamped
    // to the table's cm endpoints), activates a carrier override that sendNEC/sendSony
    // honor, applies it, and logs cm -> Hz. clearTransmitRange() reverts to the
    // per-protocol carrier (maximum range).
    void transmitRange(uint16_t cm);
    void clearTransmitRange(void);

    // Thread safety: sendNEC, sendSony, sendData and transmitRange take _tx_lock, so any task may
    // call them (Logo VM tasks on core 1, IR Share on the display task on core 0). A second sender
    // waits for the first frame to finish instead of rewriting the buffers it is still reading.
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
    SemaphoreHandle_t _tx_lock = NULL;               // created in begin()

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
