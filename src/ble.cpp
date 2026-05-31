/*
 * ble.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * BLE GATT server implementation.
 * Uses the ESP32 Arduino BLE library (BLEDevice / NimBLE backend).
 *
 * Library:  "ESP32 BLE Arduino" — already included in the Espressif ESP32
 *           Arduino core.  No extra Library Manager install needed.
 *           PlatformIO: add  lib_deps = ESP32 BLE Arduino  to platformio.ini
 *           Arduino IDE: automatically available when esp32 board package
 *           is installed.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 * iPhone testing with nRF Connect (free, App Store)
 * ─────────────────────────────────────────────────────────────────────────────
 *
 * STEP 1 — Install nRF Connect for Mobile
 *   App Store → search "nRF Connect for Mobile" by Nordic Semiconductor → Get
 *
 * STEP 2 — Power the ESP32-S3 and open nRF Connect
 *   • Tap the "Scanner" tab at the bottom
 *   • Pull to refresh — you should see "LedMatrix" appear in the list
 *   • Tap CONNECT on the LedMatrix row
 *
 * STEP 3 — Find the service
 *   • After connecting you'll see "Client" tab with a list of services
 *   • Tap the service UUID starting with "4fafc201..."
 *   • You'll see 4 characteristics (0001–0004) or 7 (0001–0007) when
 *     BLE_CALIBRATION_ENABLED 1 in ble_cal.h
 *
 * STEP 4 — Subscribe to Status notifications (characteristic 0004)
 *   • Tap the downward-arrow (subscribe/notify) icon on characteristic 0004
 *   • The current [power, mode, brightness] state will appear as 3 hex bytes
 *     e.g.  01 00 96  →  power=on  mode=0 (SPECTRUM)  brightness=150
 *   • Any physical button press or BLE write will push a new notification
 *
 * STEP 5 — Write to characteristics
 *   • Tap the upward-arrow (write) icon on any writable characteristic
 *   • nRF Connect will show a hex keyboard — enter your value:
 *
 *     Power   (0001):  00 = off    01 = on
 *     Mode    (0002):  00 = SPECTRUM  01 = FIRE  02 = TORCH  ... (see visualizer.h)
 *     Bright  (0003):  1E = 30 (dim)  50 = 80   96 = 150   FF = 255 (full)
 *
 *   • Tap SEND — the matrix responds immediately
 *   • The Status characteristic (0004) will fire a notification showing
 *     the new state
 *
 * STEP 6 — Disconnect
 *   • Tap DISCONNECT at the top of the screen
 *   • The ESP32-C3 will begin advertising again automatically
 *   • Physical buttons continue to work while connected and disconnected
 *
 * TIP — LightBlue (by Punch Through) is an alternative free iOS app that
 *   shows characteristic values in decimal as well as hex, which is more
 *   readable for brightness values.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "ble.h"
#include "buttons.h"
#include "visualizer.h"
#include "ble_cal.h"

#if BLE_ENABLED

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>   // CCCD descriptor — required for Notify to work on iOS

// ── Forward declarations ──────────────────────────────────────────────────────
static void _bleSendStatus();

// ── Server + characteristics ──────────────────────────────────────────────────
static BLEServer*         _bleServer    = nullptr;
static BLECharacteristic* _charPower    = nullptr;
static BLECharacteristic* _charMode     = nullptr;
static BLECharacteristic* _charBrite    = nullptr;
static BLECharacteristic* _charStatus   = nullptr;
static bool               _bleConnected = false;

// ── Pending-write flags (set in BLE callback, consumed in loop context) ───────
// Volatile because they're written from a BLE stack task and read from loop().
static volatile bool    _pendingPower  = false;
static volatile bool    _pendingMode   = false;
static volatile bool    _pendingBrite  = false;
static volatile uint8_t _pendingPowerVal = 1;
static volatile uint8_t _pendingModeVal  = 0;
static volatile uint8_t _pendingBriteVal = 150;

// Dirty flag: set whenever state changes (button or BLE) so we send a notify
static volatile bool _statusDirty = false;

// Track previous state to detect button-driven changes
static uint8_t _prevPower = 1;
static uint8_t _prevMode  = 0;
static uint8_t _prevBrite = 150;

// ── Connection callbacks ──────────────────────────────────────────────────────
class BleServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* srv) override {
        _bleConnected = true;
        _statusDirty  = true;
        bleCalSetConnected(true);
}

    void onDisconnect(BLEServer* srv) override {
        _bleConnected = false;
        bleCalSetConnected(false);
        BLEDevice::startAdvertising();
    }
};

// ── Write callbacks ───────────────────────────────────────────────────────────
// These run in the BLE task — only set flags, never call visualizerSetMode()
// or matrix functions directly (not safe from this context).

class PowerWriteCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        uint8_t* data = c->getData();
        if (c->getLength() >= 1) {
            _pendingPowerVal = data[0];
            _pendingPower    = true;
        }
    }
};

class ModeWriteCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        uint8_t* data = c->getData();
        if (c->getLength() >= 1) {
            _pendingModeVal = data[0];
            _pendingMode    = true;
        }
    }
};

class BriteWriteCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        uint8_t* data = c->getData();
        if (c->getLength() >= 1) {
            _pendingBriteVal = data[0];
            _pendingBrite    = true;
        }
    }
};

// ── Status notify helper ──────────────────────────────────────────────────────
static void _bleSendStatus() {
    if (!_bleConnected || !_charStatus) return;
    uint8_t buf[3] = {
        buttonsPowerOn()      ? (uint8_t)1 : (uint8_t)0,
        buttonsCurrentMode(),
        buttonsBrightness()
    };
    _charStatus->setValue(buf, 3);
    _charStatus->notify();
}

// ── Init ──────────────────────────────────────────────────────────────────────
void bleInit() {
    BLEDevice::init(BLE_DEVICE_NAME);
    _bleServer = BLEDevice::createServer();
    _bleServer->setCallbacks(new BleServerCallbacks());

    BLEService* svc = _bleServer->createService(BLEUUID(std::string(BLE_SERVICE_UUID)), 24);

    // Power characteristic — R/W/Notify
    _charPower = svc->createCharacteristic(
        BLE_CHAR_POWER_UUID,
        BLECharacteristic::PROPERTY_READ  |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY);
    _charPower->addDescriptor(new BLE2902());
    _charPower->setCallbacks(new PowerWriteCallback());
    uint8_t pv = 1;
    _charPower->setValue(&pv, 1);

    // Mode characteristic — R/W/Notify
    _charMode = svc->createCharacteristic(
        BLE_CHAR_MODE_UUID,
        BLECharacteristic::PROPERTY_READ  |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY);
    _charMode->addDescriptor(new BLE2902());
    _charMode->setCallbacks(new ModeWriteCallback());
    uint8_t mv = (uint8_t)buttonsCurrentMode();
    _charMode->setValue(&mv, 1);

    // Brightness characteristic — R/W/Notify
    _charBrite = svc->createCharacteristic(
        BLE_CHAR_BRITE_UUID,
        BLECharacteristic::PROPERTY_READ  |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY);
    _charBrite->addDescriptor(new BLE2902());
    _charBrite->setCallbacks(new BriteWriteCallback());
    uint8_t bv = buttonsBrightness();
    _charBrite->setValue(&bv, 1);

    // Status characteristic — R/Notify only (no write)
    _charStatus = svc->createCharacteristic(
        BLE_CHAR_STATUS_UUID,
        BLECharacteristic::PROPERTY_READ  |
        BLECharacteristic::PROPERTY_NOTIFY);
    _charStatus->addDescriptor(new BLE2902());
    uint8_t sv[3] = { 1, (uint8_t)buttonsCurrentMode(), buttonsBrightness() };
    _charStatus->setValue(sv, 3);

    bleCalInit(svc);   // register calibration characteristics (no-op when disabled)
 svc->start();

    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SERVICE_UUID);
    adv->setScanResponse(true);
    // Connectable undirected advertising, fast interval for quick discovery
    adv->setMinPreferred(0x06);
    adv->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
}

// ── Update — call every loop() ────────────────────────────────────────────────
void bleUpdate() {
    // ── Apply any pending writes from BLE callbacks ───────────────────────────
    if (_pendingPower) {
        _pendingPower = false;
        buttonsSetPower(_pendingPowerVal != 0);
        _statusDirty = true;
    }
    if (_pendingMode) {
        _pendingMode = false;
        buttonsSetMode(_pendingModeVal);
        _statusDirty = true;
    }
    if (_pendingBrite) {
        _pendingBrite = false;
        buttonsSetBrightness(_pendingBriteVal);
        _statusDirty = true;
    }

    // ── Detect physical button changes and mark dirty ─────────────────────────
    uint8_t curPower = buttonsPowerOn()    ? 1 : 0;
    uint8_t curMode  = buttonsCurrentMode();
    uint8_t curBrite = buttonsBrightness();

    if (curPower != _prevPower || curMode != _prevMode || curBrite != _prevBrite) {
        _prevPower = curPower;
        _prevMode  = curMode;
        _prevBrite = curBrite;
        _statusDirty = true;

        // Also update the individual characteristic values so a Read
        // from the phone returns the current value
        if (_charPower) _charPower->setValue(&curPower, 1);
        if (_charMode)  _charMode->setValue(&curMode, 1);
        if (_charBrite) _charBrite->setValue(&curBrite, 1);
    }

    // ── Send status notification if anything changed ──────────────────────────
    if (_statusDirty && _bleConnected) {
        _statusDirty = false;
        _bleSendStatus();
}
    bleCalUpdate();   // process pending cal commands (no-op when disabled)

}

// ── Stubs when BLE disabled ───────────────────────────────────────────────────
#else

void bleInit()   {}
void bleUpdate() {}

#endif // BLE_ENABLED

// ─────────────────────────────────────────────────────────────────────────────

