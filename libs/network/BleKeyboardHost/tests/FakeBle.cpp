// Implementation of the host-test fakes: NimBLE (stubs/NimBLEDevice.h), the
// FreeRTOS task/notify/critical-section primitives, Arduino's millis()/Serial,
// Preferences (in-memory NVS) and the test driver in FakeBle.h.
//
// Only the behaviour the BLE HID host depends on is faked. The radio and
// controller do not exist here; lifecycle tests can still block and release the
// connect, security, and discovery wait points. Everything else - the GATT walk,
// Report Reference parsing, report decode, edge detection and the key ring - is
// the library's real code.

#include "FakeBle.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <new>
#include <thread>

#include "Preferences.h"

// ---------------------------------------------------------------------------
// Heap accounting: every allocation in this binary goes through these, so a test
// can assert the notification path allocates nothing.
// ---------------------------------------------------------------------------
namespace fakeble {
void countAllocation();
}

void* operator new(size_t size) {
  fakeble::countAllocation();
  void* p = std::malloc(size ? size : 1);
  if (p == nullptr) throw std::bad_alloc();
  return p;
}

void* operator new[](size_t size) {
  fakeble::countAllocation();
  void* p = std::malloc(size ? size : 1);
  if (p == nullptr) throw std::bad_alloc();
  return p;
}

void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, size_t) noexcept { std::free(p); }
void operator delete[](void* p, size_t) noexcept { std::free(p); }

// ---------------------------------------------------------------------------
// Arduino stubs
// ---------------------------------------------------------------------------
SerialStub Serial;
static uint32_t g_clockMs = 1000;

unsigned long millis() { return g_clockMs; }

// ---------------------------------------------------------------------------
// FreeRTOS stubs
// ---------------------------------------------------------------------------
namespace {

std::recursive_mutex& mux() {
  static std::recursive_mutex instance;
  return instance;
}

// Raised inside ulTaskNotifyTake() when BleKeyboardHost::end() deletes the
// connection task, so the task function unwinds instead of looping (the host
// asserts it is idle before deleting it).
struct TaskStop {};

// One fake FreeRTOS task. Allocated with new and intentionally never freed by a
// destructor at process exit, so an unjoined thread can never call terminate().
struct TaskThread {
  std::thread thread;
  std::mutex mutex;
  std::condition_variable cv;
  bool notified = false;
  bool notificationsHeld = false;
  bool stopRequested = false;
};

TaskThread* g_task = nullptr;
bool g_taskCreateSucceeds = true;
thread_local TaskThread* t_currentTask = nullptr;

struct BlockingControl {
  std::mutex mutex;
  std::condition_variable cv;
  fakeble::BlockingStage configured = fakeble::BlockingStage::None;
  fakeble::BlockingStage active = fakeble::BlockingStage::None;
  bool ignoreCancellation = false;
  bool cancelRequested = false;
  bool releaseRequested = false;
};

BlockingControl& blockingControl() {
  static BlockingControl control;
  return control;
}

void joinTask(TaskThread* task) {
  if (task == nullptr) return;
  {
    std::lock_guard<std::mutex> guard(task->mutex);
    task->stopRequested = true;
  }
  task->cv.notify_all();
  if (task->thread.joinable()) task->thread.join();
  if (g_task == task) g_task = nullptr;
  delete task;
}

}  // namespace

void portEnterCritical(portMUX_TYPE*) { mux().lock(); }
void portExitCritical(portMUX_TYPE*) { mux().unlock(); }

BaseType_t xTaskCreate(void (*fn)(void*), const char* name, uint32_t stackDepth, void* arg, UBaseType_t priority,
                       TaskHandle_t* outHandle) {
  (void)name;
  (void)stackDepth;
  (void)priority;
  if (!g_taskCreateSucceeds || fn == nullptr || outHandle == nullptr) return pdFALSE;
  if (g_task != nullptr) joinTask(g_task);
  TaskThread* task = new TaskThread();
  g_task = task;
  task->thread = std::thread([task, fn, arg] {
    t_currentTask = task;
    try {
      fn(arg);
    } catch (const TaskStop&) {
    }
  });
  *outHandle = task;
  return pdTRUE;
}

void vTaskDelete(TaskHandle_t task) { joinTask(static_cast<TaskThread*>(task)); }

void vTaskDelay(uint32_t ticks) {
  g_clockMs += ticks;
  std::this_thread::sleep_for(std::chrono::milliseconds(ticks));
}

void xTaskNotifyGive(TaskHandle_t task) {
  TaskThread* target = static_cast<TaskThread*>(task);
  if (target == nullptr) return;
  {
    std::lock_guard<std::mutex> guard(target->mutex);
    target->notified = true;
  }
  target->cv.notify_all();
}

uint32_t ulTaskNotifyTake(BaseType_t clearOnExit, uint32_t ticksToWait) {
  (void)ticksToWait;
  TaskThread* task = t_currentTask;
  if (task == nullptr) return 0;
  std::unique_lock<std::mutex> lock(task->mutex);
  task->cv.wait(lock, [task] { return (task->notified && !task->notificationsHeld) || task->stopRequested; });
  if (task->stopRequested) throw TaskStop{};
  if (clearOnExit == pdTRUE) task->notified = false;
  return 1;
}

// ---------------------------------------------------------------------------
// Fake stack state
// ---------------------------------------------------------------------------
namespace fakeble {

FakeState& state() {
  static FakeState instance;
  return instance;
}

void countAllocation() { ++state().allocations; }

void setTaskCreateSucceeds(bool ok) {
  g_taskCreateSucceeds = ok;
}

void holdWorkerNotifications(bool hold) {
  if (!g_task) return;
  {
    std::lock_guard<std::mutex> lock(g_task->mutex);
    g_task->notificationsHeld = hold;
  }
  g_task->cv.notify_all();
}

void setBlockingStage(BlockingStage stage, bool ignoreCancellation) {
  BlockingControl& control = blockingControl();
  {
    std::lock_guard<std::mutex> guard(control.mutex);
    control.configured = stage;
    control.active = BlockingStage::None;
    control.ignoreCancellation = ignoreCancellation;
    control.cancelRequested = false;
    control.releaseRequested = stage == BlockingStage::None;
  }
  control.cv.notify_all();
}

bool waitForBlockingStage(BlockingStage stage, uint32_t timeoutMs) {
  BlockingControl& control = blockingControl();
  std::unique_lock<std::mutex> lock(control.mutex);
  return control.cv.wait_for(lock, std::chrono::milliseconds(timeoutMs), [&control, stage] {
    return control.active == stage;
  });
}

void releaseBlockingCall() {
  BlockingControl& control = blockingControl();
  {
    std::lock_guard<std::mutex> guard(control.mutex);
    control.releaseRequested = true;
  }
  control.cv.notify_all();
}

BlockingStage blockingStage() {
  BlockingControl& control = blockingControl();
  std::lock_guard<std::mutex> guard(control.mutex);
  return control.active;
}

void setDisconnectAtDisconnecting(bool hold) { state().holdDisconnectAtDisconnecting = hold; }

void completeDisconnect() { state().completeDisconnect(); }

uint32_t clockMs() { return g_clockMs; }
void advanceMillis(uint32_t ms) { g_clockMs += ms; }
size_t allocationCount() { return state().allocations; }

freeink::BleKeyboardHost& host() { return freeink::BleKeyboardHost::getInstance(); }

void fakeFence() {
  std::lock_guard<std::recursive_mutex> guard(mux());
}

NimBLERemoteCharacteristic* FakeState::charAt(int index) const {
  if (index < 0 || static_cast<size_t>(index) >= chars.size()) return nullptr;
  return chars[static_cast<size_t>(index)];
}

void FakeState::reset() {
  ownedChars.clear();
  chars.clear();
  initialized = false;
  hidServicePresent = true;
  connectSucceeds = true;
  g_taskCreateSucceeds = true;
  lastError = 0;
  nowMs = 1000;
  initCalls = 0;
  deinitCalls = 0;
  deleteClientCalls = 0;
  cancelConnectCalls = 0;
  disconnectCalls = 0;
  disconnectCallbackCalls = 0;
  connectCalls = 0;
  holdDisconnectAtDisconnecting = false;
  nvs.clear();
  nvsPutBytesSucceeds = true;
  scan.scanning_ = false;
  scan.callbacks_ = nullptr;
  scan.maxResults_ = 0xFF;
  scan.retainedResults_.clear();
  if (client != nullptr) {
    delete client;
    client = nullptr;
  }
  g_clockMs = nowMs;
}

int FakeState::addCharacteristic(uint16_t uuid, bool canRead, bool canWrite, bool canNotify) {
  auto chr = std::make_unique<NimBLERemoteCharacteristic>();
  chr->uuid_ = NimBLEUUID(uuid);
  chr->handle_ = static_cast<uint16_t>(0x20 + chars.size());
  chr->canRead_ = canRead;
  chr->canWrite_ = canWrite;
  chr->canNotify_ = canNotify;
  chars.push_back(chr.get());
  ownedChars.push_back(std::move(chr));
  return static_cast<int>(chars.size()) - 1;
}

void FakeState::setCharacteristicValue(int index, const uint8_t* data, size_t len) {
  NimBLERemoteCharacteristic* chr = charAt(index);
  if (chr == nullptr) return;
  chr->value_.assign(data, data + len);
}

void FakeState::setReportReference(int index, const uint8_t* bytes, size_t len) {
  NimBLERemoteCharacteristic* chr = charAt(index);
  if (chr == nullptr || len > sizeof(chr->reportReference_.bytes_)) return;
  chr->reportReference_.uuid_ = NimBLEUUID(0x2908);
  std::memcpy(chr->reportReference_.bytes_, bytes, len);
  chr->reportReference_.len_ = len;
  chr->hasReportReference_ = true;
}

void FakeState::completeDisconnect() {
  if (client != nullptr) client->disconnecting_ = false;
}

bool FakeState::isSubscribed(int index) const {
  const NimBLERemoteCharacteristic* chr = charAt(index);
  return chr != nullptr && chr->subscribed_;
}

bool FakeState::writeWasSent(int index, const uint8_t* data, size_t len) const {
  const NimBLERemoteCharacteristic* chr = charAt(index);
  if (chr == nullptr) return false;
  if (chr->writes_.size() != len) return false;
  return len == 0 || std::memcmp(chr->writes_.data(), data, len) == 0;
}

bool FakeState::notify(int index, const uint8_t* data, size_t len) {
  NimBLERemoteCharacteristic* chr = charAt(index);
  if (chr == nullptr || !chr->subscribed_ || chr->notifyCallback_ == nullptr) return false;
  chr->notifyCallback_(chr, const_cast<uint8_t*>(data), len, false);
  return true;
}

void FakeState::advertise(const char* addr, const char* name, int rssi) {
  if (!scan.scanning_ || !scan.callbacks_) return;
  const auto it = std::find(scan.retainedResults_.begin(), scan.retainedResults_.end(), addr);
  if (it == scan.retainedResults_.end() && scan.maxResults_ != 0) {
    if (scan.maxResults_ != 0xFF && scan.retainedResults_.size() >= scan.maxResults_) return;
    scan.retainedResults_.emplace_back(addr);
  }
  NimBLEAdvertisedDevice dev;
  dev.address_ = NimBLEAddress(std::string(addr), 0);
  dev.name_ = name;
  dev.haveName_ = name[0] != '\0';
  dev.rssi_ = rssi;
  // Fakes stand in for page turners and keyboards: advertise the keyboard
  // appearance, since only HID peers are listed (18/09/2026).
  dev.haveAppearance_ = true;
  dev.appearance_ = 0x03C1;
  scan.callbacks_->onResult(&dev);
}

size_t FakeState::retainedScanResults() const { return scan.retainedResults_.size(); }

void resetWorld() {
  fakeFence();
  endHost();
  setBlockingStage(BlockingStage::None);
  state().reset();
}

bool beginHost(const char* name) { return host().begin(name != nullptr ? name : "FreeInk"); }

void endHost() {
  holdWorkerNotifications(false);
  if (host().isRunning() || host().isStopping()) {
    // Test-only escape hatch for an intentionally uncooperative fake waiter so
    // fixture teardown can release it before resetting the fake client.
    releaseBlockingCall();
    host().end();
  }
  fakeFence();
}

// Text of the last reported connection failure; consumed by connectTo().
static std::string g_lastFailure;

LinkResult connectTo(const char* addr, uint32_t timeoutMs) {
  g_lastFailure.clear();
  freeink::BleKeyboardHost& h = host();
  if (!h.connect(addr)) return LinkResult::Failed;
  char failure[64];
  for (uint32_t waited = 0; waited < timeoutMs; ++waited) {
    fakeFence();  // hand the connection task's writes to this thread
    if (h.isConnected()) return LinkResult::Connected;
    if (h.takeConnectFailure(failure, sizeof failure)) {
      g_lastFailure = failure;
      return LinkResult::Failed;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return LinkResult::Timeout;
}

std::string lastConnectFailure() { return g_lastFailure; }

void setHidServicePresent(bool present) { state().hidServicePresent = present; }
void setConnectSucceeds(bool ok) { state().connectSucceeds = ok; }

}  // namespace fakeble

// ---------------------------------------------------------------------------
// Fake NimBLE stack
// ---------------------------------------------------------------------------
bool waitAtBlockingStage(fakeble::BlockingStage stage, bool& cancelled) {
  BlockingControl& control = blockingControl();
  std::unique_lock<std::mutex> lock(control.mutex);
  if (control.configured != stage) return false;
  control.active = stage;
  control.cv.notify_all();
  control.cv.wait(lock, [&control, stage] {
    return control.active != stage || control.releaseRequested ||
           (control.cancelRequested && !control.ignoreCancellation);
  });
  cancelled = control.cancelRequested && !control.ignoreCancellation;
  if (control.active == stage) control.active = fakeble::BlockingStage::None;
  if (control.configured == stage) control.configured = fakeble::BlockingStage::None;
  control.cancelRequested = false;
  control.releaseRequested = false;
  lock.unlock();
  control.cv.notify_all();
  return true;
}

void requestBlockingCancellation() {
  BlockingControl& control = blockingControl();
  {
    std::lock_guard<std::mutex> guard(control.mutex);
    if (!control.ignoreCancellation) {
      control.cancelRequested = true;
      control.releaseRequested = true;
    } else {
      control.cancelRequested = true;
    }
  }
  control.cv.notify_all();
}

bool NimBLERemoteCharacteristic::writeValue(const uint8_t* data, size_t length, bool response) {
  (void)response;
  writes_.assign(data, data + length);
  value_.assign(data, data + length);
  return true;
}

NimBLERemoteDescriptor* NimBLERemoteCharacteristic::getDescriptor(const NimBLEUUID& uuid) {
  if (!hasReportReference_ || uuid != reportReference_.getUUID()) return nullptr;
  return &reportReference_;
}

bool NimBLERemoteCharacteristic::subscribe(bool notifications, NimBLENotifyCallback callback, bool response) {
  (void)response;
  subscribed_ = notifications;
  notifyCallback_ = notifications ? callback : nullptr;
  return true;
}

NimBLERemoteCharacteristic* NimBLERemoteService::getCharacteristic(const NimBLEUUID& uuid) const {
  for (NimBLERemoteCharacteristic* chr : fakeble::state().chars) {
    if (chr != nullptr && chr->getUUID() == uuid) return chr;
  }
  return nullptr;
}

const std::vector<NimBLERemoteCharacteristic*>& NimBLERemoteService::getCharacteristics(bool refresh) const {
  (void)refresh;
  return fakeble::state().chars;
}

bool NimBLEClient::connect(const NimBLEAddress& address) {
  (void)address;
  fakeble::FakeState& s = fakeble::state();
  s.connectCalls++;
  bool cancelled = false;
  waitAtBlockingStage(fakeble::BlockingStage::Connect, cancelled);
  if (cancelled) {
    s.lastError = BLE_HS_ETIMEOUT;
    connected_ = false;
    return false;
  }
  if (!s.connectSucceeds) {
    s.lastError = BLE_HS_ETIMEOUT;
    connected_ = false;
    return false;
  }
  s.lastError = 0;
  connected_ = true;
  disconnecting_ = false;
  return true;
}

bool NimBLEClient::disconnect() {
  fakeble::FakeState& s = fakeble::state();
  s.disconnectCalls++;
  requestBlockingCancellation();
  const bool wasConnected = connected_;
  connected_ = false;
  if (wasConnected) {
    disconnecting_ = s.holdDisconnectAtDisconnecting;
    if (callbacks_ != nullptr) {
      s.disconnectCallbackCalls++;
      callbacks_->onDisconnect(this, 0);
    }
  }
  return true;
}
bool NimBLEClient::isConnected() const { return connected_; }
bool NimBLEClient::secureConnection() {
  bool cancelled = false;
  waitAtBlockingStage(fakeble::BlockingStage::Security, cancelled);
  if (cancelled) {
    fakeble::state().lastError = BLE_HS_ETIMEOUT;
    connected_ = false;
    return false;
  }
  return true;
}
bool NimBLEClient::cancelConnect() {
  fakeble::state().cancelConnectCalls++;
  requestBlockingCancellation();
  return true;
}

int NimBLEClient::getLastError() const { return fakeble::state().lastError; }

NimBLERemoteService* NimBLEClient::getService(const NimBLEUUID& uuid) {
  bool cancelled = false;
  waitAtBlockingStage(fakeble::BlockingStage::Discovery, cancelled);
  if (cancelled) {
    connected_ = false;
    return nullptr;
  }
  if (uuid.value16() != 0x1812 || !fakeble::state().hidServicePresent) return nullptr;
  return &fakeble::state().service;
}

void NimBLEClient::setConnectTimeout(uint32_t timeoutMs) { connectTimeoutMs_ = timeoutMs; }

void NimBLEClient::setConnectionParams(uint16_t minInterval, uint16_t maxInterval, uint16_t latency, uint16_t timeout) {
  (void)minInterval;
  (void)maxInterval;
  (void)latency;
  (void)timeout;
}

void NimBLEClient::setClientCallbacks(NimBLEClientCallbacks* callbacks, bool deleteOnDisconnect) {
  (void)callbacks;
  (void)deleteOnDisconnect;
  callbacks_ = callbacks;
}

void NimBLEScan::setScanCallbacks(NimBLEScanCallbacks* callbacks, bool wantDuplicates) {
  (void)wantDuplicates;
  callbacks_ = callbacks;
}

void NimBLEScan::setActiveScan(bool active) { (void)active; }
void NimBLEScan::setInterval(uint16_t intervalMs) { (void)intervalMs; }
void NimBLEScan::setWindow(uint16_t windowMs) { (void)windowMs; }
void NimBLEScan::setMaxResults(uint8_t maxResults) { maxResults_ = maxResults; }

bool NimBLEScan::start(uint32_t durationMs, bool isContinue, bool restart) {
  (void)durationMs;
  (void)isContinue;
  (void)restart;
  scanning_ = true;
  return true;
}

bool NimBLEScan::isScanning() const { return scanning_; }
void NimBLEScan::stop() { scanning_ = false; }
void NimBLEScan::clearResults() { retainedResults_.clear(); }

bool NimBLEDevice::init(const std::string& deviceName) {
  (void)deviceName;
  fakeble::state().initCalls++;
  fakeble::state().initialized = true;
  return true;
}

bool NimBLEDevice::deinit(bool releaseMem) {
  (void)releaseMem;
  fakeble::state().deinitCalls++;
  fakeble::state().initialized = false;
  return true;
}

bool NimBLEDevice::isInitialized() { return fakeble::state().initialized; }

void NimBLEDevice::setMTU(uint16_t mtu) { (void)mtu; }
void NimBLEDevice::setSecurityAuth(bool bonding, bool mitm, bool sc) {
  (void)bonding;
  (void)mitm;
  (void)sc;
}
void NimBLEDevice::setSecurityIOCap(uint8_t iocap) { (void)iocap; }
void NimBLEDevice::setSecurityPasskey(uint32_t passkey) { (void)passkey; }
void NimBLEDevice::setSecurityInitKey(uint8_t key) { (void)key; }
void NimBLEDevice::setSecurityRespKey(uint8_t key) { (void)key; }
uint32_t NimBLEDevice::getSecurityPasskey() { return 123456; }
void NimBLEDevice::injectPassKey(NimBLEConnInfo& connInfo, uint32_t passkey) {
  (void)connInfo;
  (void)passkey;
}
void NimBLEDevice::injectConfirmPasskey(NimBLEConnInfo& connInfo, bool accept) {
  (void)connInfo;
  (void)accept;
}

NimBLEScan* NimBLEDevice::getScan() { return &fakeble::state().scan; }

NimBLEClient* NimBLEDevice::createClient() {
  fakeble::FakeState& s = fakeble::state();
  if (s.client == nullptr) s.client = new NimBLEClient();
  return s.client;
}

bool NimBLEDevice::deleteClient(NimBLEClient* client) {
  fakeble::FakeState& s = fakeble::state();
  s.deleteClientCalls++;
  if (client != nullptr && client == s.client) {
    delete client;
    s.client = nullptr;
    return true;
  }
  return false;
}

NimBLEClient* NimBLEDevice::getDisconnectedClient() {
  fakeble::FakeState& s = fakeble::state();
  return s.client != nullptr && !s.client->isConnected() && !s.client->disconnecting_ ? s.client : nullptr;
}

void NimBLEDevice::deleteBond(const NimBLEAddress& address) { (void)address; }

// ---------------------------------------------------------------------------
// Fake Preferences (in-memory NVS)
// ---------------------------------------------------------------------------
namespace {

std::string nvsKey(const std::string& ns, const char* key) { return ns + "/" + (key != nullptr ? key : ""); }

}  // namespace

bool Preferences::begin(const char* name, bool readOnly) {
  (void)readOnly;
  ns_ = name != nullptr ? name : "";
  return true;
}

uint8_t Preferences::getUChar(const char* key, uint8_t defaultValue) {
  const auto it = fakeble::state().nvs.find(nvsKey(ns_, key));
  if (it == fakeble::state().nvs.end() || it->second.empty()) return defaultValue;
  return it->second[0];
}

size_t Preferences::getBytes(const char* key, void* buf, size_t maxLen) {
  const auto it = fakeble::state().nvs.find(nvsKey(ns_, key));
  if (it == fakeble::state().nvs.end() || buf == nullptr) return 0;
  const size_t len = std::min(maxLen, it->second.size());
  std::memcpy(buf, it->second.data(), len);
  return len;
}

size_t Preferences::putUChar(const char* key, uint8_t value) {
  std::vector<uint8_t>& slot = fakeble::state().nvs[nvsKey(ns_, key)];
  slot.assign(1, value);
  return 1;
}

size_t Preferences::putBytes(const char* key, const void* buf, size_t len) {
  if (!fakeble::state().nvsPutBytesSucceeds) return 0;
  if (buf == nullptr) return 0;
  std::vector<uint8_t>& slot = fakeble::state().nvs[nvsKey(ns_, key)];
  const uint8_t* bytes = static_cast<const uint8_t*>(buf);
  slot.assign(bytes, bytes + len);
  return len;
}
