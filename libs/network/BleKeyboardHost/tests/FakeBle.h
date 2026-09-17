#pragma once

// Test driver for the fake NimBLE stack in stubs/NimBLEDevice.h plus the FreeRTOS
// / Arduino / Preferences fakes. It drives the REAL BleKeyboardHost central-role
// code compiled with FREEINK_CAP_BLE_HID_HOST=1:
//
//   resetWorld();                       // clean stack, GATT table, clock, NVS
//   addCharacteristic(0x2A4B, true);    // serve the Report Map
//   setCharacteristicValue(0, map, n);
//   beginHost();                        // BleKeyboardHost::begin()
//   connectTo("AA:BB:CC:DD:EE:FF");     // connect + GATT discovery on the conn task
//   notify(0, report, len);             // deliver an input report
//   host().popKey(ev);                  // observe the translated key event
//
// The tests live in this directory; the library itself is not modified.

#include <NimBLEDevice.h>
#include <stddef.h>
#include <stdint.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "BleKeyboardHost.h"

namespace fakeble {

enum class LinkResult {
  Connected,
  Failed,   // the host reported a failure; lastConnectFailure() has its text
  Timeout,  // neither happened within the wait budget
};

// Optional blocking points for lifecycle tests. The production host must cancel
// each of these through the real NimBLE disconnect path before deleting anything.
enum class BlockingStage : uint8_t {
  None,
  Connect,
  Security,
  Discovery,
};

// State of the fake stack and the fake peripheral's GATT table. One instance per
// process (see state()); resetWorld() restores it between tests.
struct FakeState {
  // --- Fake stack -----------------------------------------------------------
  bool initialized = false;
  // The fake peripheral advertises the HID service unless a test says otherwise.
  bool hidServicePresent = true;
  bool connectSucceeds = true;
  int lastError = 0;
  uint32_t nowMs = 1000;
  size_t allocations = 0;
  size_t initCalls = 0;
  size_t deinitCalls = 0;
  size_t deleteClientCalls = 0;
  size_t cancelConnectCalls = 0;
  size_t disconnectCalls = 0;
  size_t disconnectCallbackCalls = 0;
  size_t connectCalls = 0;
  // NimBLE invokes onDisconnect while the client can still report DISCONNECTING.
  // The test driver can hold that state until completeDisconnect() is called.
  bool holdDisconnectAtDisconnecting = false;
  std::map<std::string, std::vector<uint8_t>> nvs;
  bool nvsPutBytesSucceeds = true;

  // --- Fake peripheral GATT table -------------------------------------------
  // Handle of characteristic `i` is 0x20 + i; tests address characteristics by
  // index, the library addresses them by handle.
  std::vector<std::unique_ptr<NimBLERemoteCharacteristic>> ownedChars;
  std::vector<NimBLERemoteCharacteristic*> chars;

  NimBLEScan scan;
  NimBLERemoteService service;
  NimBLEClient* client = nullptr;

  // --- Driver actions -------------------------------------------------------
  void reset();
  int addCharacteristic(uint16_t uuid, bool canRead, bool canWrite, bool canNotify);
  void setCharacteristicValue(int index, const uint8_t* data, size_t len);
  // Attach a Report Reference descriptor (0x2908) carrying exactly `len` bytes.
  void setReportReference(int index, const uint8_t* bytes, size_t len);
  void completeDisconnect();
  void advertise(const char* addr, const char* name, int rssi);
  size_t retainedScanResults() const;
  bool isSubscribed(int index) const;
  // True when the library wrote exactly this payload to characteristic `index`.
  bool writeWasSent(int index, const uint8_t* data, size_t len) const;
  // Deliver a notification on characteristic `index` (no allocation: reports are
  // passed straight through). False when the characteristic is not subscribed.
  bool notify(int index, const uint8_t* data, size_t len);

 private:
  NimBLERemoteCharacteristic* charAt(int index) const;
};

FakeState& state();

// --- Lifecycle ---------------------------------------------------------------
void resetWorld();
bool beginHost(const char* name = "FreeInk");
void endHost();
LinkResult connectTo(const char* addr, uint32_t timeoutMs = 2000);
std::string lastConnectFailure();

// Test-only lifecycle fault injection. The production host should unwind a
// successful NimBLE init when its connection worker cannot be created.
void setTaskCreateSucceeds(bool ok);
void holdWorkerNotifications(bool hold);

// Lifecycle fault injection. When ignoreCancellation is true, end() must time out
// without deleting the worker/client; releaseBlockingCall() then permits a later
// end() retry to complete.
void setBlockingStage(BlockingStage stage, bool ignoreCancellation = false);
bool waitForBlockingStage(BlockingStage stage, uint32_t timeoutMs = 1000);
void releaseBlockingCall();
BlockingStage blockingStage();

// NimBLE's disconnect callback acknowledges GAP teardown before the client
// necessarily reaches its fully-disconnected state. Hold that state to prove
// end() waits for getDisconnectedClient() before deleting the client.
void setDisconnectAtDisconnecting(bool hold);
void completeDisconnect();

// --- Fake peripheral setup ---------------------------------------------------
void setHidServicePresent(bool present);
void setConnectSucceeds(bool ok);
inline int addCharacteristic(uint16_t uuid, bool canRead = false, bool canWrite = false, bool canNotify = false) {
  return state().addCharacteristic(uuid, canRead, canWrite, canNotify);
}
inline void setCharacteristicValue(int index, const uint8_t* data, size_t len) {
  state().setCharacteristicValue(index, data, len);
}
inline void setReportReference(int index, const uint8_t* bytes, size_t len) {
  state().setReportReference(index, bytes, len);
}
inline bool isSubscribed(int index) { return state().isSubscribed(index); }
inline bool writeWasSent(int index, const uint8_t* data, size_t len) {
  return state().writeWasSent(index, data, len);
}
inline bool notify(int index, const uint8_t* data, size_t len) { return state().notify(index, data, len); }

// --- Clock / heap accounting -------------------------------------------------
uint32_t clockMs();
void advanceMillis(uint32_t ms);
size_t allocationCount();

// The host singleton under test.
freeink::BleKeyboardHost& host();

}  // namespace fakeble
