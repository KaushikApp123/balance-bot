/*
 * BLE <-> UART bridge (ESP32-C3-MINI-1 on the carrier board).
 *
 * Exposes the Nordic UART Service so any NUS client works (the Python
 * dashboard, nRF Connect, Bluefruit Connect):
 *   6e400002-...  RX  write / write-without-response  -> UART to STM32
 *   6e400003-...  TX  notify                           <- UART from STM32
 *
 * The bridge does not parse frames. It only moves bytes, so the protocol
 * lives in two places only: firmware/core/src/protocol.c and
 * host/balancebot/protocol.py. Frames carry their own sync bytes and CRC, so
 * BLE packet boundaries do not matter.
 *
 * Wiring: GPIO4 = RX (from STM32 PA9), GPIO5 = TX (to STM32 PA10), 460800 8N1.
 * GPIO20/21 are avoided because the ROM bootloader prints on them at reset.
 */
#include <Arduino.h>
#include <NimBLEDevice.h>

static constexpr int  PIN_UART_RX = 4;
static constexpr int  PIN_UART_TX = 5;
static constexpr int  PIN_LED     = 8;       /* on-board LED, active low */
static constexpr long UART_BAUD   = 460800;
static constexpr uint32_t FLUSH_US = 5000;   /* send a partial chunk after 5 ms */

static const char *DEVICE_NAME = "BalanceBot";
static const char *NUS_SERVICE = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char *NUS_RX      = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
static const char *NUS_TX      = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

static NimBLECharacteristic *g_tx = nullptr;
static volatile bool     g_connected = false;
static volatile uint16_t g_chunk = 20;       /* ATT payload: MTU - 3, starts at 23 - 3 */

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *server, ble_gap_conn_desc *desc) override {
        g_connected = true;
        /* Ask for a 7.5-15 ms interval so 50 Hz telemetry flows smoothly. */
        server->updateConnParams(desc->conn_handle, 6, 12, 0, 200);
        digitalWrite(PIN_LED, LOW);
    }
    void onDisconnect(NimBLEServer *server, ble_gap_conn_desc *desc) override {
        (void)server;
        (void)desc;
        g_connected = false;
        g_chunk = 20;
        digitalWrite(PIN_LED, HIGH);
        /* NimBLE restarts advertising on its own (advertiseOnDisconnect). */
    }
    void onMTUChange(uint16_t mtu, ble_gap_conn_desc *desc) override {
        (void)desc;
        uint16_t c = mtu > 3 ? mtu - 3 : 20;
        g_chunk = c > 244 ? 244 : c;
    }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *chr) override {
        NimBLEAttValue v = chr->getValue();
        Serial1.write(v.data(), v.size());
    }
};

void setup()
{
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);
    Serial.begin(115200);                                   /* USB, for debug */
    Serial1.setRxBufferSize(2048);
    Serial1.begin(UART_BAUD, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setMTU(247);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());
    server->advertiseOnDisconnect(true);

    NimBLEService *svc = server->createService(NUS_SERVICE);
    g_tx = svc->createCharacteristic(NUS_TX, NIMBLE_PROPERTY::NOTIFY);
    NimBLECharacteristic *rx = svc->createCharacteristic(
        NUS_RX, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rx->setCallbacks(new RxCallbacks());
    svc->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(NUS_SERVICE);
    adv->setScanResponse(true);
    adv->start();
    Serial.println("bridge: advertising as BalanceBot");
}

void loop()
{
    static uint8_t buf[256];
    static size_t len = 0;
    static uint32_t first_byte_us = 0;

    while (Serial1.available() && len < sizeof buf) {
        if (len == 0) first_byte_us = micros();
        buf[len++] = (uint8_t)Serial1.read();
    }

    if (!g_connected || g_tx->getSubscribedCount() == 0) {
        len = 0;                 /* nobody listening: drop, never back up the UART */
        delay(1);
        return;
    }

    const size_t chunk = g_chunk;
    const bool timed_out = len > 0 && (micros() - first_byte_us) > FLUSH_US;
    if (len >= chunk || timed_out) {
        size_t sent = 0;
        while (sent < len) {
            size_t n = len - sent;
            if (n > chunk) n = chunk;
            g_tx->notify(&buf[sent], n);
            sent += n;
        }
        len = 0;
    } else {
        delay(1);
    }
}
