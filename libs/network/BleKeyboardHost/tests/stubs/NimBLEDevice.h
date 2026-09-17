#pragma once

// Host-test fake for <NimBLE-Arduino>'s <NimBLEDevice.h>.
//
// src/BleKeyboardHost.cpp compiles unmodified with FREEINK_CAP_BLE_HID_HOST=1
// against this header (plus the FreeRTOS/Preferences/Arduino stubs next to it),
// so the tests drive the REAL central-role code: GATT discovery of the HID
// service (Report Map 0x2A4B, Report Reference 0x2908, Protocol Mode 0x2A4E,
// Boot Keyboard Input 0x2A22), report decode, edge detection and the key ring.
//
// Only the API surface that file calls is modelled, with the same names and call
// shapes (NimBLE-Arduino 2.x). The radio and controller are absent, while the
// lifecycle tests can deliberately block connect, security, or discovery and
// release those waiters through cancel/disconnect. A link otherwise comes up
// instantly and security succeeds, so this remains a host contract test rather
// than on-device proof.
//
// The fake peripheral is the GATT table in fakeble::FakeState (FakeBle.h): tests
// add characteristics, connect, then fire notifications through it.

#include <stddef.h>
#include <stdint.h>

#include <Arduino.h>

#include <memory>
#include <string>
#include <vector>

// NimBLE host / security constants the library compares against.
#define BLE_HS_ETIMEOUT 519            // host error reported for a connect timeout
#define BLE_HS_IO_DISPLAY_ONLY 3       // security IO capability
#define BLE_SM_PAIR_KEY_DIST_ENC 0x01  // key distribution flags

struct ble_gap_upd_params;

namespace fakeble {
struct FakeState;
}  // namespace fakeble

typedef void (*NimBLENotifyCallback)(class NimBLERemoteCharacteristic*, uint8_t*, size_t, bool);

class NimBLEUUID {
 public:
  NimBLEUUID() = default;
  explicit NimBLEUUID(uint16_t value16) : value16_(value16) {}
  uint16_t value16() const { return value16_; }
  bool operator==(const NimBLEUUID& other) const { return value16_ == other.value16_; }
  bool operator!=(const NimBLEUUID& other) const { return !(*this == other); }

 private:
  uint16_t value16_ = 0;
};

class NimBLEAddress {
 public:
  NimBLEAddress() = default;
  NimBLEAddress(const std::string& addr, uint8_t type) : addr_(addr), type_(type) {}
  const std::string& toString() const { return addr_; }
  uint8_t getType() const { return type_; }
  bool operator==(const NimBLEAddress& other) const { return addr_ == other.addr_ && type_ == other.type_; }

 private:
  std::string addr_;
  uint8_t type_ = 0;
};

// A view onto the fake server's bytes. The real NimBLEAttValue owns its buffer;
// the library copies every value out immediately, so a view is equivalent here.
class NimBLEAttValue {
 public:
  NimBLEAttValue() = default;
  NimBLEAttValue(const uint8_t* data, size_t size) : data_(data), size_(size) {}
  const uint8_t* data() const { return data_; }
  size_t size() const { return size_; }
  uint8_t operator[](size_t index) const { return data_[index]; }

 private:
  const uint8_t* data_ = nullptr;
  size_t size_ = 0;
};

class NimBLERemoteDescriptor {
 public:
  NimBLEUUID getUUID() const { return uuid_; }
  NimBLEAttValue readValue() const { return NimBLEAttValue(bytes_, len_); }

 private:
  NimBLEUUID uuid_;
  uint8_t bytes_[8] = {0};
  size_t len_ = 0;
  friend struct fakeble::FakeState;
};

class NimBLERemoteCharacteristic {
 public:
  NimBLEUUID getUUID() const { return uuid_; }
  uint16_t getHandle() const { return handle_; }
  bool canRead() const { return canRead_; }
  bool canWrite() const { return canWrite_; }
  bool canNotify() const { return canNotify_; }
  bool writeValue(const uint8_t* data, size_t length, bool response = false);
  NimBLEAttValue readValue() const { return NimBLEAttValue(value_.data(), value_.size()); }
  NimBLERemoteDescriptor* getDescriptor(const NimBLEUUID& uuid);
  bool subscribe(bool notifications, NimBLENotifyCallback callback, bool response = false);

 private:
  NimBLEUUID uuid_;
  uint16_t handle_ = 0;
  bool canRead_ = false;
  bool canWrite_ = false;
  bool canNotify_ = false;
  std::vector<uint8_t> value_;
  NimBLERemoteDescriptor reportReference_;
  bool hasReportReference_ = false;
  std::vector<uint8_t> writes_;
  bool subscribed_ = false;
  NimBLENotifyCallback notifyCallback_ = nullptr;
  friend struct fakeble::FakeState;
};

class NimBLERemoteService {
 public:
  NimBLERemoteCharacteristic* getCharacteristic(const NimBLEUUID& uuid) const;
  const std::vector<NimBLERemoteCharacteristic*>& getCharacteristics(bool refresh = false) const;
};

struct NimBLEConnInfo {};

class NimBLEClientCallbacks;

class NimBLEClient {
 public:
  bool connect(const NimBLEAddress& address);
  bool disconnect();
  bool isConnected() const;
  bool secureConnection();
  bool cancelConnect();
  int getLastError() const;
  NimBLERemoteService* getService(const NimBLEUUID& uuid);
  void setConnectTimeout(uint32_t timeoutMs);
  void setConnectionParams(uint16_t minInterval, uint16_t maxInterval, uint16_t latency, uint16_t timeout);
  void setClientCallbacks(NimBLEClientCallbacks* callbacks, bool deleteOnDisconnect = true);

 private:
  uint32_t connectTimeoutMs_ = 0;
  bool connected_ = false;
  bool disconnecting_ = false;
  NimBLEClientCallbacks* callbacks_ = nullptr;
  friend struct fakeble::FakeState;
  // getDisconnectedClient() models NimBLE's own status check, so the fake device
  // reads the fake client's DISCONNECTING flag directly.
  friend class NimBLEDevice;
};

class NimBLEClientCallbacks {
 public:
  virtual ~NimBLEClientCallbacks() = default;
  virtual void onDisconnect(NimBLEClient*, int) {}
  virtual void onPassKeyEntry(NimBLEConnInfo&) {}
  virtual uint32_t onPassKeyDisplay(NimBLEConnInfo&) { return 0; }
  virtual void onConfirmPasskey(NimBLEConnInfo&, uint32_t) {}
  virtual bool onConnParamsUpdateRequest(NimBLEClient*, const ble_gap_upd_params*) { return true; }
};

class NimBLEAdvertisedDevice {
 public:
  NimBLEAddress getAddress() const { return address_; }
  bool haveName() const { return haveName_; }
  std::string getName() const { return name_; }
  int getRSSI() const { return rssi_; }
  bool haveAppearance() const { return haveAppearance_; }
  uint16_t getAppearance() const { return appearance_; }
  bool isAdvertisingService(const NimBLEUUID&) const { return false; }
  bool isConnectable() const { return true; }

 private:
  NimBLEAddress address_;
  std::string name_;
  int rssi_ = -60;
  bool haveName_ = false;
  bool haveAppearance_ = false;
  uint16_t appearance_ = 0;
  friend struct fakeble::FakeState;
};

class NimBLEScanCallbacks {
 public:
  virtual ~NimBLEScanCallbacks() = default;
  virtual void onResult(const NimBLEAdvertisedDevice*) {}
};

class NimBLEScan {
 public:
  void setScanCallbacks(NimBLEScanCallbacks* callbacks, bool wantDuplicates = false);
  void setActiveScan(bool active);
  void setInterval(uint16_t intervalMs);
  void setWindow(uint16_t windowMs);
  void setMaxResults(uint8_t maxResults);
  bool start(uint32_t durationMs, bool isContinue = false, bool restart = false);
  bool isScanning() const;
  void stop();
  void clearResults();

 private:
  bool scanning_ = false;
  NimBLEScanCallbacks* callbacks_ = nullptr;
  uint8_t maxResults_ = 0xFF;
  std::vector<std::string> retainedResults_;
  friend struct fakeble::FakeState;
};

class NimBLEDevice {
 public:
  static bool init(const std::string& deviceName);
  static bool deinit(bool releaseMem = false);
  static bool isInitialized();
  static void setMTU(uint16_t mtu);
  static void setSecurityAuth(bool bonding, bool mitm, bool sc);
  static void setSecurityIOCap(uint8_t iocap);
  static void setSecurityPasskey(uint32_t passkey);
  static void setSecurityInitKey(uint8_t key);
  static void setSecurityRespKey(uint8_t key);
  static uint32_t getSecurityPasskey();
  static void injectPassKey(NimBLEConnInfo& connInfo, uint32_t passkey);
  static void injectConfirmPasskey(NimBLEConnInfo& connInfo, bool accept);
  static NimBLEScan* getScan();
  static NimBLEClient* createClient();
  static bool deleteClient(NimBLEClient* client);
  static NimBLEClient* getDisconnectedClient();
  static void deleteBond(const NimBLEAddress& address);
};
