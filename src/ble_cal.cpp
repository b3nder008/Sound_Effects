// ─────────────────────────────────────────────────────────────────────────────
// ble_cal.cpp  —  BLE calibration extension implementation
// ─────────────────────────────────────────────────────────────────────────────
//
// Measurement method: accumulates ALL non-zero bandRaw[] samples over the
// 3-second window, then computes per-band average.  This matches the iOS-side
// behaviour (also averages non-zero samples) so both paths give the same result
// and the device result is used when available (it samples at ~31 fps vs ~15 fps).
//
// BLE_CALIBRATION_ENABLED 0  →  all four functions compile to empty stubs.
// ─────────────────────────────────────────────────────────────────────────────

#include "ble_cal.h"
#include "fft.h"

#if BLE_CALIBRATION_ENABLED

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

extern float runtimeFloor[NUM_BANDS];
extern float runtimeSensitivity[NUM_BANDS];
extern float bandRaw[NUM_BANDS];

// ── GATT characteristics ──────────────────────────────────────────────────────
static BLECharacteristic* _charTelem = nullptr;
static BLECharacteristic* _charCmd   = nullptr;
static BLECharacteristic* _charCal   = nullptr;
static bool               _bleCalConnected = false;

// ── Measurement state ─────────────────────────────────────────────────────────
static bool     _measuring   = false;
static uint32_t _measStart   = 0;

// Per-band peak accumulation — maximum non-zero bandRaw[] seen during window
static float    _measPeak[NUM_BANDS];

// ── Telemetry throttle ─────────────────────────────────────────────────────────
static uint8_t  _telemCount  = 0;

// ── Pending write flags (BLE callbacks → loop context) ───────────────────────
static volatile bool    _pendingCmd    = false;
static volatile uint8_t _pendingCmdVal = 0;
static volatile bool    _pendingCal    = false;
static uint8_t          _pendingCalBuf[32];

// ── Pack current runtime values into 32-byte CalParams buffer ─────────────────
static void _pack(uint8_t buf[32]) {
    for (int b = 0; b < NUM_BANDS; b++) {
        uint16_t fv = (uint16_t)constrain((int)runtimeFloor[b], 0, 65535);
        buf[b*2]     = fv >> 8;
        buf[b*2 + 1] = fv & 0xFF;
        uint16_t sv  = (uint16_t)constrain(
                            (int)(runtimeSensitivity[b] * CAL_SENS_SCALE), 0, 65535);
        buf[16 + b*2]     = sv >> 8;
        buf[16 + b*2 + 1] = sv & 0xFF;
    }
}

// ── Send live telemetry (bandRaw[]) ───────────────────────────────────────────
static void _sendTelemetry() {
    if (!_charTelem || !_bleCalConnected) return;
    uint8_t buf[16];
    for (int b = 0; b < NUM_BANDS; b++) {
        uint16_t v = (uint16_t)constrain((int)bandRaw[b], 0, 0xFEFF);
        buf[b*2]     = v >> 8;
        buf[b*2 + 1] = v & 0xFF;
    }
    _charTelem->setValue(buf, 16);
    _charTelem->notify();
}

// ── Send measurement result as sentinel notification ──────────────────────────
// buf[0] == 0xFF signals "this is a floor result, not live telemetry".
// B0 floor packed as buf[1] = floor[0]/8 (max 2040; iOS decodes ×8).
// B1–B7 floors packed as normal uint16 BE.
static void _sendMeasResult() {
    if (!_charTelem || !_bleCalConnected) return;
    uint8_t buf[16];
    buf[0] = 0xFF;  // sentinel
    buf[1] = (uint8_t)constrain((int)(runtimeFloor[0] / 8.0f), 0, 255);
    for (int b = 1; b < NUM_BANDS; b++) {
        uint16_t v = (uint16_t)constrain((int)runtimeFloor[b], 0, 0xFEFF);
        buf[b*2]     = v >> 8;
        buf[b*2 + 1] = v & 0xFF;
    }
    _charTelem->setValue(buf, 16);
    _charTelem->notify();
}

// ── BLE callbacks ─────────────────────────────────────────────────────────────
class CalCmdCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        if (c->getLength() >= 1) {
            _pendingCmdVal = c->getData()[0];
            _pendingCmd    = true;
        }
    }
};

class CalParamsCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        if (c->getLength() >= 32) {
            memcpy(_pendingCalBuf, c->getData(), 32);
            _pendingCal = true;
        }
    }
    void onRead(BLECharacteristic* c) override {
        uint8_t buf[32];
        _pack(buf);
        c->setValue(buf, 32);
    }
};

// ── bleCalInit ────────────────────────────────────────────────────────────────
void bleCalInit(BLEService* svc) {
    _charTelem = svc->createCharacteristic(
        BLE_CHAR_TELEM_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    _charTelem->addDescriptor(new BLE2902());
    uint8_t z[16] = {};
    _charTelem->setValue(z, 16);

    _charCmd = svc->createCharacteristic(
        BLE_CHAR_CMD_UUID, BLECharacteristic::PROPERTY_WRITE_NR);
    _charCmd->setCallbacks(new CalCmdCallback());

    _charCal = svc->createCharacteristic(
        BLE_CHAR_CAL_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR);
    _charCal->setCallbacks(new CalParamsCallback());
    uint8_t init[32]; _pack(init);
    _charCal->setValue(init, 32);
}

// ── bleCalUpdate ──────────────────────────────────────────────────────────────
void bleCalUpdate() {

    // Apply pending command
    if (_pendingCmd) {
        _pendingCmd = false;
        switch (_pendingCmdVal) {

            case CAL_CMD_START_MEAS:
                if (!_measuring) {
                    _measuring  = true;
                    _measStart  = millis();
                    for (int b = 0; b < NUM_BANDS; b++) {
                        _measPeak[b] = 0.0f;
                    }
                }
                break;

            case CAL_CMD_CANCEL_MEAS:
                _measuring = false;
                break;

            case CAL_CMD_RESET_FLOORS:
                _measuring = false;
                fftResetFloors();
                fftSaveToNVS();
                if (_charCal) { uint8_t b[32]; _pack(b); _charCal->setValue(b, 32); }
                break;

            case CAL_CMD_RESET_SENS:
                fftResetSens();
                fftSaveToNVS();
                if (_charCal) { uint8_t b[32]; _pack(b); _charCal->setValue(b, 32); }
                break;

            case CAL_CMD_RESET_ALL:
                _measuring = false;
                fftResetFloors();
                fftResetSens();
                fftSaveToNVS();
                if (_charCal) { uint8_t b[32]; _pack(b); _charCal->setValue(b, 32); }
                break;

            case CAL_CMD_RESTORE_FACTORY:
                _measuring = false;
                fftResetFloors();
                fftResetSens();
                fftSaveToNVS();
                if (_charCal) { uint8_t b[32]; _pack(b); _charCal->setValue(b, 32); }
                break;
        }
    }

    // Apply pending CalParams write from iOS
    if (_pendingCal) {
        _pendingCal = false;
        for (int b = 0; b < NUM_BANDS; b++) {
            uint16_t fv = ((uint16_t)_pendingCalBuf[b*2] << 8)
                         |  _pendingCalBuf[b*2 + 1];
            runtimeFloor[b] = (float)fv;

            uint16_t sv = ((uint16_t)_pendingCalBuf[16 + b*2] << 8)
                         |  _pendingCalBuf[16 + b*2 + 1];
            runtimeSensitivity[b] = (float)sv / (float)CAL_SENS_SCALE;
        }
        fftSaveToNVS();
    }

    // Check measurement completion
    if (_measuring && (millis() - _measStart) >= (uint32_t)CAL_MEAS_MS) {
        _measuring = false;

        // Apply per-band peak as new runtimeFloor[].
        // Guard: if no non-zero samples were seen (audio not running),
        // fall back to half the factory floor rather than writing zero.
        for (int b = 0; b < NUM_BANDS; b++) {
            float peak = _measPeak[b];
            if (peak < 1.0f) {
                peak = fftFactoryFloor(b) * 0.5f;  // fallback
            }
            runtimeFloor[b] = fmaxf(1.0f, roundf(peak));
        }

        fftSaveToNVS();
        _sendMeasResult();

        if (_charCal) {
            uint8_t buf[32]; _pack(buf);
            _charCal->setValue(buf, 32);
        }
    }
}

// ── bleCalOnFftFrame ──────────────────────────────────────────────────────────
// Called from fftProcess() after bandRaw[] is populated (~31 fps).
void bleCalOnFftFrame() {
    // During measurement: track per-band peak of non-zero samples
    if (_measuring) {
        for (int b = 0; b < NUM_BANDS; b++) {
            if (bandRaw[b] > _measPeak[b]) {
                _measPeak[b] = bandRaw[b];
            }
        }
    }

    // Throttle telemetry to ~15 fps
    if (!_bleCalConnected) return;
    if (++_telemCount < CAL_TELEM_EVERY_N) return;
    _telemCount = 0;
    _sendTelemetry();
}

// ── bleCalSetConnected ────────────────────────────────────────────────────────
void bleCalSetConnected(bool connected) {
    _bleCalConnected = connected;
    if (!connected) {
        _measuring  = false;
        _telemCount = 0;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
#else  // stubs
// ─────────────────────────────────────────────────────────────────────────────
#include <BLEServer.h>
void bleCalInit(BLEService*)     {}
void bleCalUpdate()              {}
void bleCalOnFftFrame()          {}
void bleCalSetConnected(bool)    {}
#endif