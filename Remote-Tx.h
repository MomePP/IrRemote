#pragma once

#include "driver/rmt_tx.h"

#include "Ir-NEC.h"
#include "Ir-Sony.h"

#define IR_TX_CARRIER_FREQ_HZ_NEC  38000
#define IR_TX_CARRIER_FREQ_HZ_SONY 40000
#define IR_TX_CARRIER_FREQ_HZ_MAX  40000 // raised from 38000 so Sony's 40kHz is not clamped
#define IR_TX_CARRIER_FREQ_HZ_MIN  30000
#define IR_TX_CARRIER_DUTY         0.33

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
    void updateCarrierFrequency(uint32_t frequency);

    bool sendNEC(uint16_t address, uint16_t command);
    bool sendSony(uint16_t address, uint16_t command);

    // Set IR TX usable range: maps cm -> carrier Hz via IR_RANGE_CALIBRATION (clamped
    // to the table's cm endpoints), activates a carrier override that sendNEC/sendSony
    // honor, applies it, and logs cm -> Hz. clearTransmitRange() reverts to the
    // per-protocol carrier (maximum range).
    void transmitRange(uint16_t cm);
    void clearTransmitRange(void);

private:
    rmt_channel_handle_t _channel;
    rmt_carrier_config_t _carrier_cfg;
    rmt_transmit_config_t _transmit_config;
    rmt_encoder_handle_t _encoder;       // NEC custom encoder
    rmt_encoder_handle_t _copy_encoder;  // used to push pre-built Sony frames
    uint32_t _resolution;                // RMT tick resolution, for Sony frame building
    uint32_t _current_carrier_hz;        // cache to avoid redundant rmt_apply_carrier

    // Active range carrier override; 0 = inactive (sends use the per-protocol carrier).
    // Written by transmitRange() and read by sendNEC/sendSony, reached only from the
    // Logo VM tasks (logoVMTask AND bgLogoVMTask) — both on Core 1, so this aligned
    // 32-bit word needs no volatile/barrier (no torn reads under same-core preemption).
    // NOTE: concurrent IR TX from both VMs simultaneously is unguarded (no mutex),
    // consistent with the existing IR opcode handling; that is pathological usage.
    uint32_t _range_carrier_hz = 0;

    uint32_t rangeToCarrierHz(uint16_t cm); // clamp + linear interp over IR_RANGE_CALIBRATION
};
