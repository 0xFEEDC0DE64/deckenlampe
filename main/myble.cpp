#include "myble.h"

// esp-idf includes
#include <esp_log.h>

// 3rdparty lib includes
#include <NimBLEDevice.h>

// local includes
#include "myconfig.h"

namespace deckenlampe {
namespace {
constexpr const char * const TAG = "BLE";

BLEServer *pServer{};
BLEService *pService{};
BLECharacteristic *pCaracteristic{};

class MyBleCallbacks : public NimBLECharacteristicCallbacks
{
public:
    void onWrite(NimBLECharacteristic* pCharacteristic) override;
};

MyBleCallbacks bleCallbacks;

} // namespace

void init_ble()
{
    if (!config::enable_ble.value())
        return;

    BLEDevice::init(config::ble_name.value());

    const auto serviceUuid{"08cb06e2-9f8b-444c-b167-fc9f217db06d"};

    pServer = BLEDevice::createServer();

    pService = pServer->createService(serviceUuid);

    pCaracteristic = pService->createCharacteristic("ca4bdcac-e317-44b6-881f-4567c6b0273a", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    pCaracteristic->setValue("no value");
    pCaracteristic->setCallbacks(&bleCallbacks);

    if (!pService->start())
        ESP_LOGE(TAG, "failed starting BLE service!");

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(serviceUuid);
    pAdvertising->setScanResponse(true);
    if (!pAdvertising->start())
        ESP_LOGE(TAG, "failed starting BLE advertising!");
    //BLEDevice::startAdvertising();
}

void update_ble()
{
    if (!config::enable_ble.value())
        return;

    // TODO
}

namespace {
void MyBleCallbacks::onWrite(NimBLECharacteristic* pCharacteristic)
{
    const auto &val = pCharacteristic->getValue();
}
} // namespace
} // namespace deckenlampe
