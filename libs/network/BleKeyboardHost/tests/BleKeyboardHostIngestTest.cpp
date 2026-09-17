// Host tests for the BLE HID host central-role path: GATT discovery of the HID
// service (Report Map, Report Reference 0x2908, Protocol Mode, Boot Keyboard
// Input), report decode, press/release edge detection and the key ring.
//
// The suite compiles src/BleKeyboardHost.cpp with FREEINK_CAP_BLE_HID_HOST=1 and
// drives it through the fake NimBLE stack in FakeBle.{h,cpp} - so the code under
// test is the real implementation, on a plain host compiler, with no hardware,
// no ESP-IDF and no NimBLE download. What that does NOT prove (radio, bonding,
// MTU, on-device lifecycle) is listed in the handoff report.
//
// Run from the repository root:
//   cmake -S test -B build/test && cmake --build build/test
//   ctest --test-dir build/test --output-on-failure -R BleKeyboardHostIngestTest

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <chrono>
#include <initializer_list>
#include <thread>
#include <vector>

#include "BleKeyboardHost.h"
#include "FakeBle.h"
#include "HidDescriptors.h"
#include "HidKeymap.h"

using namespace freeink;  // test-only: the SDK's namespace

namespace {

constexpr char kAddr[] = "AA:BB:CC:DD:EE:FF";

// HID service characteristics the host discovers (HID 1.11 section 3.4).
enum : uint16_t {
  kUuidReportMap = 0x2A4B,
  kUuidReport = 0x2A4D,
  kUuidProtocolMode = 0x2A4E,
  kUuidBootKbdInput = 0x2A22,
};

// One 8-byte boot-shaped report: modifier byte, reserved byte, six key slots.
std::vector<uint8_t> bootReport(uint8_t mods, std::initializer_list<uint8_t> keys) {
  std::vector<uint8_t> report(8, 0);
  report[0] = mods;
  uint8_t slot = 0;
  for (uint8_t key : keys) {
    if (slot >= 6) break;
    report[2 + slot++] = key;
  }
  return report;
}

class IngestTest : public ::testing::Test {
 protected:
  void SetUp() override { fakeble::resetWorld(); }
  void TearDown() override { fakeble::endHost(); }

  // Serve `descriptor` as the Report Map; returns the characteristic's index.
  int serveReportMap(const uint8_t* descriptor, size_t len) {
    const int index = fakeble::addCharacteristic(kUuidReportMap, /*canRead=*/true);
    fakeble::setCharacteristicValue(index, descriptor, len);
    return index;
  }

  // Add the Protocol Mode characteristic the host writes Report Protocol to.
  int serveProtocolMode() { return fakeble::addCharacteristic(kUuidProtocolMode, false, /*canWrite=*/true); }

  // Add an Input Report characteristic; `reference` (when given) is its Report
  // Reference descriptor (0x2908) payload as {report id, report type}.
  int serveInputReport(const uint8_t* reference, size_t referenceLen) {
    const int index = fakeble::addCharacteristic(kUuidReport, false, false, /*canNotify=*/true);
    if (reference != nullptr) fakeble::setReportReference(index, reference, referenceLen);
    return index;
  }

  bool beginAndConnect() {
    if (!fakeble::beginHost()) return false;
    return fakeble::connectTo(kAddr) == fakeble::LinkResult::Connected;
  }

  // Deliver an input report; the decode + edge-detect path must not allocate per
  // notification (the library documents a heap-free hot path).
  void deliver(int index, const uint8_t* data, size_t len) {
    const size_t before = fakeble::allocationCount();
    const bool delivered = fakeble::notify(index, data, len);
    const size_t after = fakeble::allocationCount();
    EXPECT_TRUE(delivered);
    EXPECT_EQ(after, before) << "notification path allocated";
  }
  void deliver(int index, const std::vector<uint8_t>& bytes) { deliver(index, bytes.data(), bytes.size()); }

  KeyEvent popKey() {
    KeyEvent ev;
    EXPECT_TRUE(fakeble::host().popKey(ev));
    return ev;
  }
  // popKey() on an empty ring returns false.
  void expectEmptyRing() {
    KeyEvent ev;
    EXPECT_FALSE(fakeble::host().popKey(ev));
  }
};

// --- Keyboard map: press / hold / release ------------------------------------

TEST_F(IngestTest, KeyboardMapEmitsOnePressPerEdgeAndNoneWhileHeld) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  const int protocol = serveProtocolMode();
  const int input = serveInputReport(nullptr, 0);

  ASSERT_TRUE(fakeble::beginHost());
  ASSERT_EQ(fakeble::connectTo(kAddr), fakeble::LinkResult::Connected);
  EXPECT_TRUE(fakeble::host().isConnected());
  EXPECT_STREQ(fakeble::host().connectedName(), kAddr);

  // Report Protocol (1) is requested before subscribing, like a desktop host.
  const uint8_t reportProtocol[1] = {1};
  EXPECT_TRUE(fakeble::writeWasSent(protocol, reportProtocol, 1));
  EXPECT_TRUE(fakeble::isSubscribed(input));

  // A link-up also records the bond for auto-reconnect.
  ASSERT_EQ(fakeble::host().pairedCount(), 1);
  EXPECT_STREQ(fakeble::host().paired(0).addr, kAddr);

  // Shift + 'a': modifiers come from byte 0, keys from byte 2.
  const std::vector<uint8_t> shiftA = bootReport(HID_LSHIFT, {0x04});
  deliver(input, shiftA);
  const KeyEvent pressed = popKey();
  EXPECT_EQ(pressed.ch, 'A');
  EXPECT_EQ(pressed.keycode, 0x04);
  EXPECT_EQ(pressed.mods, HID_LSHIFT);
  EXPECT_EQ(pressed.special, SpecialKey::None);
  EXPECT_TRUE(pressed.pressed);
  expectEmptyRing();

  // The same report again means the key is still down: no extra turn.
  deliver(input, shiftA);
  expectEmptyRing();

  // poll() must not synthesize repeats either - the page-turner anti-double-turn
  // rule: one physical press is exactly one event.
  for (int i = 0; i < 5; ++i) fakeble::host().poll();
  expectEmptyRing();

  // Release, then the same key again: a fresh press re-triggers.
  deliver(input, bootReport(0, {}));
  expectEmptyRing();
  deliver(input, shiftA);
  EXPECT_EQ(popKey().ch, 'A');
  expectEmptyRing();

  // ErrorRollOver (0x01) and a usage repeated inside one report: no phantom press.
  deliver(input, bootReport(0, {0x01}));
  expectEmptyRing();
  deliver(input, bootReport(0, {0x04, 0x04}));
  EXPECT_EQ(popKey().keycode, 0x04);
  expectEmptyRing();

  // Special keys keep their identity and leave `ch` empty.
  deliver(input, bootReport(0, {0x50}));
  const KeyEvent special = popKey();
  EXPECT_EQ(special.special, SpecialKey::Left);
  EXPECT_EQ(special.ch, 0);
  EXPECT_EQ(special.keycode, 0x50);

  // Remotes that stream a held key and never send a release frame: once the
  // stale-release window has passed, the next notification is a new press.
  fakeble::advanceMillis(200);
  fakeble::host().poll();
  expectEmptyRing();  // the timeout itself emits nothing
  deliver(input, bootReport(0, {0x50}));
  EXPECT_EQ(popKey().keycode, 0x50);
  expectEmptyRing();
}

// --- Report ids, multiple characteristics, Report Reference ------------------

TEST_F(IngestTest, MultiReportDeviceResolvesLayoutsByReportReference) {
  serveReportMap(hidtest::kTwoReports, sizeof hidtest::kTwoReports);
  const int protocol = serveProtocolMode();

  const uint8_t refKeyboard[2] = {1, 1};   // report id 1, type 1 = Input
  const uint8_t refConsumer[2] = {2, 1};   // report id 2, type 1 = Input
  const uint8_t refOutput[2] = {1, 2};     // type 2 = Output: not an input report
  const int keyboard = serveInputReport(refKeyboard, sizeof refKeyboard);
  const int consumer = serveInputReport(refConsumer, sizeof refConsumer);
  const int output = serveInputReport(refOutput, sizeof refOutput);
  const int secondConsumer = serveInputReport(refConsumer, sizeof refConsumer);

  ASSERT_TRUE(fakeble::beginHost());
  ASSERT_EQ(fakeble::connectTo(kAddr), fakeble::LinkResult::Connected);
  const uint8_t reportProtocol[1] = {1};
  EXPECT_TRUE(fakeble::writeWasSent(protocol, reportProtocol, 1));

  EXPECT_TRUE(fakeble::isSubscribed(keyboard));
  EXPECT_TRUE(fakeble::isSubscribed(consumer));
  EXPECT_TRUE(fakeble::isSubscribed(secondConsumer));
  EXPECT_FALSE(fakeble::isSubscribed(output));  // an Output report is never an input source
  EXPECT_FALSE(fakeble::notify(output, refOutput, sizeof refOutput));

  // Report id 1 selects the keyboard layout: modifiers at payload byte 0, six key
  // bytes from payload byte 2 (report byte 3, behind the id byte).
  const uint8_t kbPress[9] = {1, HID_LCTRL, 0x00, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00};
  deliver(keyboard, kbPress, sizeof kbPress);
  const KeyEvent kbEvent = popKey();
  EXPECT_EQ(kbEvent.keycode, 0x26);
  EXPECT_EQ(kbEvent.mods, HID_LCTRL);
  EXPECT_EQ(kbEvent.ch, '9');  // usage 0x26 = '9' (HID usage table); Ctrl only sets `mods`
  const uint8_t kbRelease[9] = {1, 0, 0, 0, 0, 0, 0, 0, 0};
  deliver(keyboard, kbRelease, sizeof kbRelease);
  expectEmptyRing();

  // Report id 2 is the 16-bit Consumer array at payload offset 0 - two bytes, not
  // a six-key array.
  const uint8_t consumerPress[3] = {2, 0xCD, 0x00};
  deliver(consumer, consumerPress, sizeof consumerPress);
  const KeyEvent consumerEvent = popKey();
  EXPECT_EQ(consumerEvent.keycode, 0xCD);
  EXPECT_EQ(consumerEvent.ch, 0);  // consumer codes have no ASCII translation
  EXPECT_EQ(consumerEvent.special, SpecialKey::None);
  const uint8_t consumerRelease[3] = {2, 0x00, 0x00};
  deliver(consumer, consumerRelease, sizeof consumerRelease);
  expectEmptyRing();

  // A remote that omits the id byte its descriptor declares: the Report Reference
  // of the notifying characteristic names the layout instead.
  const uint8_t consumerNoId[2] = {0xCD, 0x00};
  deliver(consumer, consumerNoId, sizeof consumerNoId);
  EXPECT_EQ(popKey().keycode, 0xCD);
  const uint8_t consumerNoIdRelease[2] = {0x00, 0x00};
  deliver(consumer, consumerNoIdRelease, sizeof consumerNoIdRelease);
  expectEmptyRing();

  // A second characteristic carrying the same report id decodes identically: the
  // identity comes from each characteristic's own Report Reference.
  const uint8_t secondPress[2] = {0xDA, 0x00};
  deliver(secondConsumer, secondPress, sizeof secondPress);
  EXPECT_EQ(popKey().keycode, 0xDA);
}

TEST_F(IngestTest, BootKeyboardInputCharacteristicIsTheFallback) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  const int boot = fakeble::addCharacteristic(kUuidBootKbdInput, false, false, /*canNotify=*/true);

  ASSERT_TRUE(fakeble::beginHost());
  ASSERT_EQ(fakeble::connectTo(kAddr), fakeble::LinkResult::Connected);
  EXPECT_TRUE(fakeble::isSubscribed(boot));

  const std::vector<uint8_t> press = bootReport(HID_LSHIFT, {0x04});
  deliver(boot, press);
  const KeyEvent event = popKey();
  EXPECT_EQ(event.ch, 'A');
  EXPECT_EQ(event.keycode, 0x04);
  EXPECT_EQ(event.mods, HID_LSHIFT);
}

// --- Reports the parsed map cannot place -------------------------------------

TEST_F(IngestTest, UnmappedReportFallsBackToKeyboardShapeOnce) {
  // The descriptor declares report ids 1 and 2, but this characteristic carries no
  // Report Reference and the report carries no id byte the map knows.
  serveReportMap(hidtest::kTwoReports, sizeof hidtest::kTwoReports);
  const int unmapped = serveInputReport(nullptr, 0);
  const int boot = fakeble::addCharacteristic(kUuidBootKbdInput, false, false, /*canNotify=*/true);

  ASSERT_TRUE(fakeble::beginHost());
  ASSERT_EQ(fakeble::connectTo(kAddr), fakeble::LinkResult::Connected);
  EXPECT_TRUE(fakeble::isSubscribed(unmapped));
  EXPECT_FALSE(fakeble::isSubscribed(boot));  // 0x2A4D won, so no boot fallback

  // [mod][reserved][k0..k5]: the last-resort shape still yields one press per edge.
  const std::vector<uint8_t> press = bootReport(0, {0x04});
  deliver(unmapped, press);
  EXPECT_EQ(popKey().keycode, 0x04);
  deliver(unmapped, press);
  expectEmptyRing();  // held: one event per press, not per notification
  deliver(unmapped, std::vector<uint8_t>(8, 0));
  expectEmptyRing();
}

// --- Descriptors and reports past the ceilings -------------------------------

TEST_F(IngestTest, OversizedReportMapFallsBackWithoutCrashing) {
  // Past the 1 KiB descriptor ceiling: refused outright, so the host connects and
  // decodes from the last-resort path instead of trusting a huge map.
  std::vector<uint8_t> huge(hidtest::kBootKeyboard, hidtest::kBootKeyboard + sizeof hidtest::kBootKeyboard);
  huge.resize(1100, 0x15);
  serveReportMap(huge.data(), huge.size());
  const int input = serveInputReport(nullptr, 0);

  ASSERT_TRUE(fakeble::beginHost());
  ASSERT_EQ(fakeble::connectTo(kAddr), fakeble::LinkResult::Connected);  // an unusable map is not a connect failure
  EXPECT_TRUE(fakeble::isSubscribed(input));

  const std::vector<uint8_t> press = bootReport(0, {0x04});
  deliver(input, press);
  EXPECT_EQ(popKey().keycode, 0x04);
  deliver(input, press);
  expectEmptyRing();

  // A 256-byte notification (far past the 64-byte payload the parser models) is
  // read only where the layout says: no overrun, no allocation.
  std::vector<uint8_t> longReport(256, 0);
  longReport[2] = 0x05;
  deliver(input, longReport);
  EXPECT_EQ(popKey().keycode, 0x05);
  expectEmptyRing();
}

TEST_F(IngestTest, TruncatedReportMapKeepsTheFieldsItParsed) {
  // The last Input item asks for 255 x 8 bits and is dropped; the keyboard fields
  // before it stay usable, and reports are decoded through them.
  serveReportMap(hidtest::kBootThenOverflow, sizeof hidtest::kBootThenOverflow);
  const int input = serveInputReport(nullptr, 0);

  ASSERT_TRUE(fakeble::beginHost());
  ASSERT_EQ(fakeble::connectTo(kAddr), fakeble::LinkResult::Connected);

  const std::vector<uint8_t> press = bootReport(HID_LCTRL, {0x05});
  deliver(input, press);
  const KeyEvent event = popKey();
  EXPECT_EQ(event.keycode, 0x05);
  EXPECT_EQ(event.mods, HID_LCTRL);
  expectEmptyRing();

  // One byte shorter than the layout needs is refused, not guessed: the host falls
  // back to the keyboard shape rather than shifting fields.
  const uint8_t shortReport[7] = {HID_LCTRL, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00};
  deliver(input, shortReport, sizeof shortReport);
  EXPECT_EQ(popKey().keycode, 0x06);
}

// --- Key ring ----------------------------------------------------------------

TEST_F(IngestTest, KeyRingBoundaryKeepsFifoAndDropsOnOverflow) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  const int input = serveInputReport(nullptr, 0);
  ASSERT_TRUE(fakeble::beginHost());
  ASSERT_EQ(fakeble::connectTo(kAddr), fakeble::LinkResult::Connected);

  // 18 distinct usages, six per report - three more than the ring can hold. The
  // overflow must be dropped, not block the notification path or wrap onto the
  // unread entries.
  const std::vector<uint8_t> first = bootReport(0, {0x04, 0x05, 0x06, 0x07, 0x08, 0x09});
  const std::vector<uint8_t> second = bootReport(0, {0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F});
  const std::vector<uint8_t> third = bootReport(0, {0x10, 0x11, 0x12, 0x13, 0x14, 0x15});
  deliver(input, first);
  deliver(input, second);
  deliver(input, third);

  // kKeyQueueLen is 16 with one slot left free to tell full from empty: 15 events
  // survive, in the order they arrived.
  EXPECT_EQ(BleKeyboardHost::kKeyQueueLen, 16);
  for (uint8_t usage = 0x04; usage <= 0x12; ++usage) {
    EXPECT_EQ(popKey().keycode, usage) << "usage " << static_cast<int>(usage);
  }
  expectEmptyRing();

  // After a full drain the ring indices wrap; ordering still holds.
  const std::vector<uint8_t> fourth = bootReport(0, {0x20, 0x21});
  deliver(input, fourth);
  EXPECT_EQ(popKey().keycode, 0x20);
  EXPECT_EQ(popKey().keycode, 0x21);
  expectEmptyRing();
}

TEST_F(IngestTest, PopKeyOnEmptyRingReturnsFalse) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  const int input = serveInputReport(nullptr, 0);
  ASSERT_TRUE(fakeble::beginHost());
  ASSERT_EQ(fakeble::connectTo(kAddr), fakeble::LinkResult::Connected);

  expectEmptyRing();

  // A release frame carries no key and must not enqueue anything either.
  deliver(input, std::vector<uint8_t>(8, 0));
  expectEmptyRing();
}

// --- Cooperative teardown ----------------------------------------------------

TEST_F(IngestTest, EndTimeoutRetainsLiveWorkerAndBeginRejectsUntilRetry) {
  ASSERT_TRUE(fakeble::beginHost());
  fakeble::setBlockingStage(fakeble::BlockingStage::Connect, /*ignoreCancellation=*/true);
  ASSERT_TRUE(fakeble::host().connect(kAddr));
  ASSERT_TRUE(fakeble::waitForBlockingStage(fakeble::BlockingStage::Connect));

  NimBLEClient* clientBefore = fakeble::state().client;
  const size_t deinitBefore = fakeble::state().deinitCalls;
  const size_t deleteBefore = fakeble::state().deleteClientCalls;
  EXPECT_FALSE(fakeble::host().end(0));
  EXPECT_FALSE(fakeble::host().isRunning());
  EXPECT_TRUE(fakeble::host().isStopping());
  EXPECT_EQ(fakeble::state().client, clientBefore);
  EXPECT_EQ(fakeble::state().deinitCalls, deinitBefore);
  EXPECT_EQ(fakeble::state().deleteClientCalls, deleteBefore);
  EXPECT_EQ(fakeble::state().connectCalls, 1);

  // A non-zero budget is bounded too. The deliberately stuck fake keeps all
  // resources alive until an explicit release, so this exercises the old-red /
  // new-green timeout contract without deleting a live worker.
  EXPECT_FALSE(fakeble::host().end(100));
  EXPECT_TRUE(fakeble::host().isStopping());
  EXPECT_EQ(fakeble::state().client, clientBefore);
  EXPECT_EQ(fakeble::state().deinitCalls, deinitBefore);
  EXPECT_EQ(fakeble::state().deleteClientCalls, deleteBefore);

  // begin() must not invoke the old NimBLE self-heal while the worker/client are
  // still live. The same client remains available for the later end() retry.
  EXPECT_FALSE(fakeble::beginHost());
  EXPECT_EQ(fakeble::state().client, clientBefore);
  EXPECT_EQ(fakeble::state().deinitCalls, deinitBefore);

  fakeble::releaseBlockingCall();
  ASSERT_TRUE(fakeble::host().end(1000));
  char lateFailure[64];
  EXPECT_FALSE(fakeble::host().takeConnectFailure(lateFailure, sizeof lateFailure));
  EXPECT_FALSE(fakeble::host().isStopping());
  EXPECT_FALSE(NimBLEDevice::isInitialized());
  EXPECT_EQ(fakeble::state().client, nullptr);

  // A fully completed teardown remains reinitializable.
  ASSERT_TRUE(fakeble::beginHost());
  EXPECT_TRUE(fakeble::host().end(1000));
}

TEST_F(IngestTest, EndCancelsSecurityWaitBeforeDeletingClient) {
  ASSERT_TRUE(fakeble::beginHost());
  fakeble::setBlockingStage(fakeble::BlockingStage::Security);
  ASSERT_TRUE(fakeble::host().connect(kAddr));
  ASSERT_TRUE(fakeble::waitForBlockingStage(fakeble::BlockingStage::Security));

  EXPECT_TRUE(fakeble::host().end(1000));
  EXPECT_GE(fakeble::state().cancelConnectCalls, 1u);
  EXPECT_GE(fakeble::state().disconnectCalls, 1u);
  EXPECT_EQ(fakeble::state().deleteClientCalls, 1u);
  EXPECT_FALSE(fakeble::host().isStopping());
  EXPECT_FALSE(NimBLEDevice::isInitialized());
}

TEST_F(IngestTest, EndCancelsDiscoveryWaitBeforeDeletingClient) {
  ASSERT_TRUE(fakeble::beginHost());
  fakeble::setBlockingStage(fakeble::BlockingStage::Discovery);
  ASSERT_TRUE(fakeble::host().connect(kAddr));
  ASSERT_TRUE(fakeble::waitForBlockingStage(fakeble::BlockingStage::Discovery));

  EXPECT_TRUE(fakeble::host().end(1000));
  EXPECT_GE(fakeble::state().cancelConnectCalls, 1u);
  EXPECT_GE(fakeble::state().disconnectCalls, 1u);
  EXPECT_EQ(fakeble::state().deleteClientCalls, 1u);
  EXPECT_FALSE(fakeble::host().isStopping());
  EXPECT_FALSE(NimBLEDevice::isInitialized());
}

TEST_F(IngestTest, EndWaitsForFullyDisconnectedClientAfterDisconnectCallback) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  serveInputReport(nullptr, 0);
  ASSERT_TRUE(beginAndConnect());

  NimBLEClient* clientBefore = fakeble::state().client;
  ASSERT_NE(clientBefore, nullptr);
  const size_t deinitBefore = fakeble::state().deinitCalls;
  const size_t deleteBefore = fakeble::state().deleteClientCalls;

  // The callback is delivered, while NimBLE keeps the client DISCONNECTING.
  // The first bounded end lets the real worker park before it reports the
  // pending client teardown.
  fakeble::setDisconnectAtDisconnecting(true);
  EXPECT_FALSE(fakeble::host().end(1000));
  EXPECT_EQ(fakeble::state().disconnectCallbackCalls, 1u);
  EXPECT_TRUE(fakeble::host().isStopping());
  EXPECT_EQ(fakeble::state().client, clientBefore);
  EXPECT_EQ(NimBLEDevice::getDisconnectedClient(), nullptr);
  EXPECT_EQ(fakeble::state().deinitCalls, deinitBefore);
  EXPECT_EQ(fakeble::state().deleteClientCalls, deleteBefore);

  // With the worker already parked, end(0) is nonblocking and must preserve the
  // client and initialized stack while the underlying state is DISCONNECTING.
  EXPECT_FALSE(fakeble::host().end(0));
  EXPECT_EQ(fakeble::state().client, clientBefore);
  EXPECT_EQ(fakeble::state().deinitCalls, deinitBefore);
  EXPECT_EQ(fakeble::state().deleteClientCalls, deleteBefore);

  // Once the fake GAP state reaches DISCONNECTED, the same client can be safely
  // reclaimed and a zero-budget retry completes the pending teardown.
  fakeble::completeDisconnect();
  EXPECT_EQ(NimBLEDevice::getDisconnectedClient(), clientBefore);
  EXPECT_TRUE(fakeble::host().end(0));
  EXPECT_EQ(fakeble::state().deleteClientCalls, deleteBefore + 1);
  EXPECT_EQ(fakeble::state().client, nullptr);
  EXPECT_FALSE(NimBLEDevice::isInitialized());
  EXPECT_FALSE(fakeble::host().isStopping());
}

// --- Connection failure paths -------------------------------------------------

TEST_F(IngestTest, CrowdedScanRetainsOnlyTheHostBoundedDeviceList) {
  ASSERT_TRUE(fakeble::beginHost());
  fakeble::host().startScan();
  for (unsigned i = 0; i < 200; ++i) {
    char addr[18];
    snprintf(addr, sizeof addr, "AA:BB:CC:DD:00:%02X", i);
    fakeble::state().advertise(addr, "Remote", -100 + static_cast<int>(i));
  }
  EXPECT_EQ(fakeble::state().retainedScanResults(), 0u);
  EXPECT_EQ(fakeble::host().deviceCount(), BleKeyboardHost::kMaxDiscovered);
  bool nearestPresent = false;
  for (uint8_t i = 0; i < fakeble::host().deviceCount(); ++i) {
    if (strcmp(fakeble::host().device(i).addr, "AA:BB:CC:DD:00:C7") == 0) nearestPresent = true;
  }
  EXPECT_TRUE(nearestPresent);
}

TEST_F(IngestTest, ScanCancellationBeforeWorkerWakeAllowsTheNextConnection) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  serveInputReport(nullptr, 0);
  ASSERT_TRUE(fakeble::beginHost());
  fakeble::holdWorkerNotifications(true);
  ASSERT_TRUE(fakeble::host().connect(kAddr));
  // The scan cancels a queued reconnect before the worker can enter NimBLE.
  // Its bounded wait expires, then the worker receives that cancellation.
  fakeble::host().startScan();
  fakeble::holdWorkerNotifications(false);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (fakeble::host().isConnecting() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_FALSE(fakeble::host().isConnecting());
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
  EXPECT_EQ(fakeble::connectTo(kAddr), fakeble::LinkResult::Connected);
}

TEST_F(IngestTest, MissingBondBlobCannotResurrectPreviousSessionRecords) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  serveInputReport(nullptr, 0);
  ASSERT_TRUE(beginAndConnect());
  ASSERT_TRUE(fakeble::host().end());
  fakeble::state().nvs.erase("freeink-hid/b");
  ASSERT_TRUE(fakeble::beginHost());
  EXPECT_EQ(fakeble::host().pairedCount(), 0);
}

TEST_F(IngestTest, TruncatedBondBlobIsRejected) {
  fakeble::state().nvs["freeink-hid/n"] = {1};
  fakeble::state().nvs["freeink-hid/b"] = {'A', 'A'};
  ASSERT_TRUE(fakeble::beginHost());
  EXPECT_EQ(fakeble::host().pairedCount(), 0);
}

TEST_F(IngestTest, StoredBondStringsAreValidatedBeforeUse) {
  PairedHidDevice records[2];
  memcpy(records[0].addr, kAddr, sizeof kAddr);
  memset(records[0].name, 'N', sizeof records[0].name);
  memset(records[1].addr, 'A', sizeof records[1].addr);
  memcpy(records[1].name, "Bad address", sizeof "Bad address");
  const auto* bytes = reinterpret_cast<const uint8_t*>(records);
  fakeble::state().nvs["freeink-hid/n"] = {2};
  fakeble::state().nvs["freeink-hid/b"] = std::vector<uint8_t>(bytes, bytes + sizeof records);
  ASSERT_TRUE(fakeble::beginHost());
  EXPECT_EQ(fakeble::host().pairedCount(), 1);
  EXPECT_EQ(fakeble::host().paired(0).name[31], '\0');
}

TEST_F(IngestTest, FailedBondBlobWriteKeepsPreviousStoredCount) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  serveInputReport(nullptr, 0);
  fakeble::state().nvs["freeink-hid/n"] = {0};
  fakeble::state().nvsPutBytesSucceeds = false;
  ASSERT_TRUE(beginAndConnect());
  EXPECT_EQ(fakeble::state().nvs["freeink-hid/n"], std::vector<uint8_t>({0}));
}

TEST_F(IngestTest, BeginFailsCleanlyWhenConnectionTaskCannotStart) {
  fakeble::setTaskCreateSucceeds(false);

  EXPECT_FALSE(fakeble::beginHost());
  EXPECT_FALSE(fakeble::host().isRunning());
  EXPECT_FALSE(NimBLEDevice::isInitialized());
  EXPECT_EQ(fakeble::state().client, nullptr);

  // A failed begin must leave the singleton retryable after the fault clears.
  fakeble::setTaskCreateSucceeds(true);
  EXPECT_TRUE(fakeble::beginHost());
}

TEST_F(IngestTest, ConnectFailureIsReportedAsTimeout) {
  fakeble::setConnectSucceeds(false);
  ASSERT_TRUE(fakeble::beginHost());

  EXPECT_EQ(fakeble::connectTo(kAddr), fakeble::LinkResult::Failed);
  EXPECT_EQ(fakeble::lastConnectFailure(), "Connect timeout");
  EXPECT_FALSE(fakeble::host().isConnected());
  EXPECT_EQ(fakeble::host().pairedCount(), 0);
}

TEST_F(IngestTest, PeripheralWithoutHidServiceIsRejected) {
  fakeble::setHidServicePresent(false);
  ASSERT_TRUE(fakeble::beginHost());

  EXPECT_EQ(fakeble::connectTo(kAddr), fakeble::LinkResult::Failed);
  EXPECT_EQ(fakeble::lastConnectFailure(), "Not a HID device");
  EXPECT_FALSE(fakeble::host().isConnected());
  EXPECT_EQ(fakeble::host().pairedCount(), 0);
}

TEST_F(IngestTest, HidServiceWithoutUsableInputReportIsRejected) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  ASSERT_TRUE(fakeble::beginHost());

  EXPECT_EQ(fakeble::connectTo(kAddr), fakeble::LinkResult::Failed);
  EXPECT_EQ(fakeble::lastConnectFailure(), "No HID input report");
  EXPECT_FALSE(fakeble::host().isConnected());
  EXPECT_EQ(fakeble::host().pairedCount(), 0);
}

}  // namespace
