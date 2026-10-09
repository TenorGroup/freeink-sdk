#include <gtest/gtest.h>

#include <cstring>
#include <limits>

#include "FakeBle.h"
#include "HidDescriptors.h"
#include "RadioPort.h"
#include "Runtime.h"

namespace bleturner::port {
bool compiledIn() { return true; }
bool begin() { return fakeble::beginHost(); }
bool end(uint32_t timeoutMs) { return fakeble::host().end(timeoutMs); }
bool running() { return fakeble::host().isRunning(); }
bool stopping() { return fakeble::host().isStopping(); }
bool connected() { return fakeble::host().isConnected(); }
bool connecting() { return fakeble::host().isConnecting(); }
ConnectionActivity connectionActivity() {
  const auto activity = fakeble::host().reconnectActivity();
  return {activity.busy, activity.advertisements};
}
bool scanning() { return fakeble::host().isScanning(); }
void poll() { fakeble::host().poll(); }
bool popRaw(RawEdge&) { return false; }
bool popKey(KeyPress&) { return false; }
bool armBondedReconnect(uint8_t policy, const char* priority) {
  return fakeble::host().armBondedReconnect(policy == 1 ? freeink::PickPolicy::First : freeink::PickPolicy::Priority,
                                          priority);
}
Peer linked() { return {fakeble::host().connectedAddr(), fakeble::host().connectedName()}; }
void scan(uint32_t durationMs) {
  if (durationMs) fakeble::host().startScan(durationMs);
  else fakeble::host().stopScan();
}
bool connect(const char* address) { return fakeble::host().connect(address); }
void disconnect() { fakeble::host().disconnect(); }
void forget(const char* address) { fakeble::host().forget(address); }
bool takeConnectFailure(char* output, size_t size) { return fakeble::host().takeConnectFailure(output, size); }
uint8_t bondCount() { return fakeble::host().pairedCount(); }
Peer bond(uint8_t index) { return {fakeble::host().paired(index).addr, fakeble::host().paired(index).name}; }
uint8_t foundCount() { return fakeble::host().deviceCount(); }
Peer found(uint8_t index) { return {fakeble::host().device(index).addr, fakeble::host().device(index).name}; }
bool spawnStart() { detail::startTask(); return true; }
uint32_t nowMs() { return fakeble::clockMs(); }
void sleepMs(uint32_t durationMs) { fakeble::advanceMillis(durationMs); }
Memo& restartMemo() { static Memo memo{}; return memo; }
}

namespace {
constexpr char kPrimary[] = "7d:de:5c:bd:ae:ca";
constexpr char kSecondary[] = "AA:BB:CC:DD:EE:02";
bleturner::Heap heap() { return {200000, 100000}; }
bool yes() { return true; }
bool no() { return false; }
bool deliver(bleturner::Action) { return false; }
void noop() {}
void hold(bool) {}
void log(bool, const char*, va_list) {}
const bleturner::Host kHost{heap, yes, deliver, yes, no, noop, hold, log, nullptr};

class BleIdleRuntimeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    fakeble::resetWorld();
    freeink::PairedHidDevice bonds[2]{};
    std::strcpy(bonds[0].addr, kPrimary);
    std::strcpy(bonds[1].addr, kSecondary);
    fakeble::state().nvs["freeink-hid/n"] = {2};
    const auto* bytes = reinterpret_cast<const uint8_t*>(bonds);
    fakeble::state().nvs["freeink-hid/b"] = {bytes, bytes + sizeof bonds};
    const int map = fakeble::addCharacteristic(0x2A4B, true);
    fakeble::setCharacteristicValue(map, hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
    fakeble::addCharacteristic(0x2A4D, false, false, true);
    config.enabled = 1;
    std::strcpy(config.peerAddr, kPrimary);
    bleturner::begin(kHost, config);
    scene.where = bleturner::Where::Reader;
    scene.pageShown = true;
    scene.visit = 1;
    tick();
    tick();
    ASSERT_TRUE(fakeble::host().isRunning());
    ASSERT_TRUE(fakeble::host().isScanning());
  }
  void TearDown() override {
    fakeble::releaseBlockingCall();
    fakeble::waitForWorkerIdle();
    bleturner::beforeScreenChange();
    fakeble::endHost();
  }
  void tick() { bleturner::tick(scene); }
  void stopIdleRadio() {
    fakeble::advanceMillis(30000);
    tick();
    ASSERT_TRUE(bleturner::status().idleStopped);
    ASSERT_TRUE(fakeble::waitForWorkerIdle());
    tick();
    ASSERT_FALSE(fakeble::host().isRunning());
    ASSERT_FALSE(fakeble::host().isStopping());
  }
  void advertise(const char* address, bool connectable = true) {
    fakeble::host().onScanResultIngest(address, nullptr, -40, 1, false, connectable);
  }
  void connectingAcrossDeadline(fakeble::BlockingStage stage) {
    fakeble::advanceMillis(25500);
    advertise(kPrimary);
    fakeble::setBlockingStage(stage);
    tick();
    ASSERT_TRUE(fakeble::waitForBlockingStage(stage));
    fakeble::advanceMillis(4500);
    tick();
    EXPECT_TRUE(fakeble::host().isRunning());
    EXPECT_TRUE(fakeble::host().isConnecting());
    EXPECT_FALSE(fakeble::host().isStopping());
    EXPECT_FALSE(bleturner::status().idleStopped);
    fakeble::releaseBlockingCall();
    ASSERT_TRUE(fakeble::waitForWorkerIdle());
    EXPECT_TRUE(fakeble::host().isConnected());
  }
  bleturner::Config config{};
  bleturner::Scene scene{};
};

TEST_F(BleIdleRuntimeTest, BondedPickAndHidSetupCrossThirtySecondsWithoutRadioStop) {
  fakeble::advanceMillis(25500);
  advertise(kPrimary);
  fakeble::setBlockingStage(fakeble::BlockingStage::Discovery);
  tick();
  ASSERT_TRUE(fakeble::waitForBlockingStage(fakeble::BlockingStage::Discovery));
  EXPECT_EQ(fakeble::state().connectAddresses, std::vector<std::string>{kPrimary});
  EXPECT_EQ(fakeble::host().deviceCount(), 0);
  EXPECT_FALSE(fakeble::host().isScanning());
  fakeble::advanceMillis(4500);
  tick();
  EXPECT_TRUE(fakeble::host().isRunning());
  EXPECT_TRUE(fakeble::host().isConnecting());
  EXPECT_FALSE(fakeble::host().isStopping());
  EXPECT_FALSE(bleturner::status().idleStopped);
  EXPECT_EQ(fakeble::state().deinitCalls, 0u);
  fakeble::releaseBlockingCall();
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_TRUE(fakeble::host().isConnected());
  EXPECT_TRUE(fakeble::isSubscribed(1));
}

TEST_F(BleIdleRuntimeTest, ThirtySecondsWithoutAdvertisingStopsRadio) {
  fakeble::advanceMillis(29999);
  tick();
  EXPECT_TRUE(fakeble::host().isRunning());
  fakeble::advanceMillis(1);
  tick();
  EXPECT_FALSE(fakeble::host().isRunning());
  EXPECT_TRUE(bleturner::status().idleStopped);
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  tick();
  EXPECT_EQ(fakeble::state().deinitCalls, 1u);
}

TEST_F(BleIdleRuntimeTest, IdleStopIgnoresPageKeyAndAfterPaint) {
  stopIdleRadio();
  scene.localKey = true;
  bleturner::afterPaint();
  tick();
  EXPECT_TRUE(bleturner::status().idleStopped);
  EXPECT_FALSE(fakeble::host().isRunning());
  EXPECT_EQ(fakeble::state().deinitCalls, 1u);
  scene.localKey = false;
  for (int paint = 0; paint < 3; ++paint) {
    bleturner::afterPaint();
    fakeble::advanceMillis(1000);
    tick();
    EXPECT_TRUE(bleturner::status().idleStopped);
    EXPECT_FALSE(fakeble::host().isRunning());
  }
}

TEST_F(BleIdleRuntimeTest, IdleStopIgnoresPageKeyWithoutPaint) {
  stopIdleRadio();
  scene.localKey = true;
  tick();
  EXPECT_TRUE(bleturner::status().idleStopped);
  EXPECT_FALSE(fakeble::host().isRunning());
}

TEST_F(BleIdleRuntimeTest, IdleStopIgnoresPaintWithoutPageKey) {
  stopIdleRadio();
  bleturner::afterPaint();
  tick();
  EXPECT_TRUE(bleturner::status().idleStopped);
  EXPECT_FALSE(fakeble::host().isRunning());
}

TEST_F(BleIdleRuntimeTest, ExplicitConnectRestartsIdleRadioAndConnectsRemote) {
  stopIdleRadio();
  bleturner::requestConnect();
  tick();
  tick();
  EXPECT_FALSE(bleturner::status().idleStopped);
  ASSERT_TRUE(fakeble::host().isRunning());
  ASSERT_TRUE(fakeble::host().isScanning());
  advertise(kPrimary);
  tick();
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_TRUE(fakeble::host().isConnected());
  EXPECT_TRUE(fakeble::isSubscribed(1));
}

TEST_F(BleIdleRuntimeTest, ReopenedBookRestartsIdleRadio) {
  stopIdleRadio();
  scene.where = bleturner::Where::Elsewhere;
  tick();
  EXPECT_FALSE(fakeble::host().isRunning());
  scene.where = bleturner::Where::Reader;
  ++scene.visit;
  tick();
  tick();
  EXPECT_FALSE(bleturner::status().idleStopped);
  EXPECT_TRUE(fakeble::host().isRunning());
  EXPECT_TRUE(fakeble::host().isScanning());
}

TEST_F(BleIdleRuntimeTest, NewBookVisitRestartsIdleRadioWithoutPageKey) {
  stopIdleRadio();
  ++scene.visit;
  tick();
  tick();
  EXPECT_FALSE(bleturner::status().idleStopped);
  EXPECT_TRUE(fakeble::host().isRunning());
}

TEST_F(BleIdleRuntimeTest, BuildSuspensionResumesOnlyAfterPaint) {
  ASSERT_NE(bleturner::beforeChapterBuild(), bleturner::BuildRelease::NotHeld);
  EXPECT_FALSE(bleturner::status().idleStopped);
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  tick();
  EXPECT_FALSE(fakeble::host().isRunning());
  scene.localKey = true;
  tick();
  EXPECT_FALSE(fakeble::host().isRunning());
  scene.localKey = false;
  bleturner::afterPaint();
  tick();
  tick();
  EXPECT_FALSE(bleturner::status().idleStopped);
  EXPECT_TRUE(fakeble::host().isRunning());
  EXPECT_TRUE(fakeble::host().isScanning());
}

TEST_F(BleIdleRuntimeTest, PriorityWindowCrossesThirtySecondsAndConnectsSecondary) {
  fakeble::advanceMillis(29000);
  advertise(kSecondary);
  tick();
  fakeble::advanceMillis(1000);
  tick();
  EXPECT_TRUE(fakeble::host().isRunning());
  EXPECT_FALSE(bleturner::status().idleStopped);
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
  fakeble::advanceMillis(2000);
  tick();
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_TRUE(fakeble::host().isConnected());
  EXPECT_EQ(fakeble::state().connectAddresses, std::vector<std::string>{kSecondary});
}

TEST_F(BleIdleRuntimeTest, GapConnectCrossesThirtySecondsWithoutRadioStop) {
  connectingAcrossDeadline(fakeble::BlockingStage::Connect);
}

TEST_F(BleIdleRuntimeTest, PairingCrossesThirtySecondsWithoutRadioStop) {
  connectingAcrossDeadline(fakeble::BlockingStage::Security);
}

TEST_F(BleIdleRuntimeTest, FailedHidAttemptLeavesThirtySecondsForNextAdvertisement) {
  fakeble::setHidServicePresent(false);
  fakeble::advanceMillis(25500);
  advertise(kPrimary);
  tick();
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_FALSE(fakeble::host().isConnected());
  EXPECT_FALSE(fakeble::host().isConnecting());
  tick();
  fakeble::advanceMillis(29999);
  tick();
  EXPECT_TRUE(fakeble::host().isRunning());
  fakeble::advanceMillis(1);
  tick();
  EXPECT_TRUE(bleturner::status().idleStopped);
}

TEST_F(BleIdleRuntimeTest, AdvertisingAndFailureBetweenTicksRestartIdleClock) {
  fakeble::setConnectSucceeds(false);
  fakeble::advanceMillis(29999);
  advertise(kPrimary);
  fakeble::host().poll();
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_FALSE(fakeble::host().reconnectActivity().busy);
  tick();
  fakeble::advanceMillis(29999);
  tick();
  EXPECT_TRUE(fakeble::host().isRunning());
  fakeble::advanceMillis(1);
  tick();
  EXPECT_TRUE(bleturner::status().idleStopped);
}

TEST_F(BleIdleRuntimeTest, UnbondedAndNonconnectableAdvertisementsDoNotExtendIdle) {
  const auto before = fakeble::host().reconnectActivity().advertisements;
  fakeble::advanceMillis(29999);
  advertise("AA:BB:CC:DD:EE:99");
  advertise(kPrimary, false);
  EXPECT_EQ(fakeble::host().reconnectActivity().advertisements, before);
  tick();
  fakeble::advanceMillis(1);
  tick();
  EXPECT_TRUE(bleturner::status().idleStopped);
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
}

TEST_F(BleIdleRuntimeTest, PrimaryAdvertisementAtIdleDeadlineStillConnects) {
  fakeble::advanceMillis(30000);
  advertise(kPrimary);
  tick();
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_TRUE(fakeble::host().isConnected());
  EXPECT_FALSE(bleturner::status().idleStopped);
}

TEST_F(BleIdleRuntimeTest, QuietDeadlineSurvivesMillisWraparound) {
  ASSERT_TRUE(bleturner::beforeScreenChange(1000));
  scene.where = bleturner::Where::Elsewhere;
  tick();
  fakeble::advanceMillis(std::numeric_limits<uint32_t>::max() - fakeble::clockMs() - 10000);
  scene.where = bleturner::Where::Reader;
  ++scene.visit;
  tick();
  tick();
  fakeble::advanceMillis(29999);
  tick();
  EXPECT_TRUE(fakeble::host().isRunning());
  fakeble::advanceMillis(1);
  tick();
  EXPECT_TRUE(bleturner::status().idleStopped);
}
}
