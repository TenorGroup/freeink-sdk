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

#include <algorithm>

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

  // Release, then the same key again: the release is one event of its own (the app
  // times the hold from the two edges), then a fresh press re-triggers.
  deliver(input, bootReport(0, {}));
  const KeyEvent released = popKey();
  EXPECT_EQ(released.keycode, 0x04);
  EXPECT_FALSE(released.pressed);
  expectEmptyRing();
  deliver(input, shiftA);
  EXPECT_EQ(popKey().ch, 'A');
  expectEmptyRing();

  // ErrorRollOver (0x01) and a usage repeated inside one report: no phantom press.
  // The rollover report no longer lists 0x04, so it does end that key's press.
  deliver(input, bootReport(0, {0x01}));
  EXPECT_FALSE(popKey().pressed);
  expectEmptyRing();
  deliver(input, bootReport(0, {0x04, 0x04}));
  EXPECT_EQ(popKey().keycode, 0x04);
  expectEmptyRing();

  // Special keys keep their identity and leave `ch` empty.
  deliver(input, bootReport(0, {0x50}));
  EXPECT_FALSE(popKey().pressed);  // 0x04 left the report first
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

// --- Key releases: the second half of a press/release pair -------------------
//
// A remote that maps a button to "next page" can also mean "next chapter" when the
// button is HELD. The host cannot decide that, but it can hand the app both edges
// of the press so the app measures the hold itself. One usage is tracked at a time:
// a page-turner sends one button, and the app only ever waits on one.

TEST_F(IngestTest, HeldUsageDisappearingEmitsOneReleaseEvent) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  serveProtocolMode();
  const int input = serveInputReport(nullptr, 0);
  ASSERT_TRUE(beginAndConnect());

  // Press: unchanged - one event, pressed = true.
  deliver(input, bootReport(0, {0x4E}));
  const KeyEvent down = popKey();
  EXPECT_EQ(down.keycode, 0x4E);
  EXPECT_TRUE(down.pressed);
  expectEmptyRing();

  // Still down: the repeat report adds nothing, in either direction.
  deliver(input, bootReport(0, {0x4E}));
  expectEmptyRing();

  // Gone from the report: exactly one release, same usage, same modifiers.
  deliver(input, bootReport(0, {}));
  const KeyEvent up = popKey();
  EXPECT_EQ(up.keycode, 0x4E);
  EXPECT_EQ(up.mods, 0);
  EXPECT_FALSE(up.pressed);
  expectEmptyRing();

  // A second release frame owes nothing.
  deliver(input, bootReport(0, {}));
  expectEmptyRing();

  // The modifiers of the PRESS travel with the release, so an app that ignores
  // modified keys treats both edges the same way.
  deliver(input, bootReport(HID_LSHIFT, {0x04}));
  EXPECT_EQ(popKey().mods, HID_LSHIFT);
  deliver(input, bootReport(0, {}));
  const KeyEvent shiftUp = popKey();
  EXPECT_EQ(shiftUp.keycode, 0x04);
  EXPECT_EQ(shiftUp.mods, HID_LSHIFT);
  EXPECT_FALSE(shiftUp.pressed);
}

TEST_F(IngestTest, BootKeyboardFallbackEmitsTheSameReleasePair) {
  // No usable map for this characteristic: the last-resort keyboard shape decodes
  // it, and it owes the same release the mapped path owes.
  serveReportMap(hidtest::kTwoReports, sizeof hidtest::kTwoReports);
  const int unmapped = serveInputReport(nullptr, 0);
  ASSERT_TRUE(beginAndConnect());
  ASSERT_TRUE(fakeble::isSubscribed(unmapped));

  deliver(unmapped, bootReport(0, {0x04}));
  EXPECT_TRUE(popKey().pressed);
  deliver(unmapped, bootReport(0, {0x04}));
  expectEmptyRing();
  deliver(unmapped, std::vector<uint8_t>(8, 0));
  const KeyEvent up = popKey();
  EXPECT_EQ(up.keycode, 0x04);
  EXPECT_FALSE(up.pressed);
  expectEmptyRing();
}

TEST_F(IngestTest, ValueCodedRemoteEmitsReleaseOnItsZeroFrame) {
  // The shape a page-turner remote actually sends: three bytes, the code in byte 0,
  // an all-zero frame on release. No map can place it, and it is too short for the
  // keyboard shape, so it goes through the generic code path.
  serveReportMap(hidtest::kNoInput, sizeof hidtest::kNoInput);
  const int input = serveInputReport(nullptr, 0);
  ASSERT_TRUE(beginAndConnect());

  const uint8_t press[3] = {0x02, 0x00, 0x00};
  const uint8_t release[3] = {0x00, 0x00, 0x00};
  deliver(input, press, sizeof press);
  const KeyEvent down = popKey();
  EXPECT_EQ(down.keycode, 0x02);
  EXPECT_TRUE(down.pressed);
  expectEmptyRing();

  deliver(input, release, sizeof release);
  const KeyEvent up = popKey();
  EXPECT_EQ(up.keycode, 0x02);
  EXPECT_FALSE(up.pressed);
  expectEmptyRing();

  // Two taps in a row are two complete pairs, not one long one.
  deliver(input, press, sizeof press);
  EXPECT_TRUE(popKey().pressed);
  deliver(input, release, sizeof release);
  EXPECT_FALSE(popKey().pressed);
  expectEmptyRing();
}

TEST_F(IngestTest, StaleReleaseTimeoutKeepsTheOwedReleaseForASilentRemote) {
  // A remote that sends one frame per edge and nothing in between: the stale-release
  // timeout still ages the press-edge state out (so the next press re-triggers), and
  // it must NOT invent a release - the button can still be down. Inventing one here
  // would cap every hold at the 150 ms timeout.
  serveReportMap(hidtest::kNoInput, sizeof hidtest::kNoInput);
  const int input = serveInputReport(nullptr, 0);
  ASSERT_TRUE(beginAndConnect());

  const uint8_t press[3] = {0x02, 0x00, 0x00};
  const uint8_t release[3] = {0x00, 0x00, 0x00};
  deliver(input, press, sizeof press);
  EXPECT_TRUE(popKey().pressed);

  fakeble::advanceMillis(200);
  for (int i = 0; i < 5; ++i) fakeble::host().poll();
  expectEmptyRing();  // the timeout itself still emits nothing

  // The real release arrives late; the host still owes it and pays it once.
  fakeble::advanceMillis(600);
  deliver(input, release, sizeof release);
  const KeyEvent up = popKey();
  EXPECT_EQ(up.keycode, 0x02);
  EXPECT_FALSE(up.pressed);
  expectEmptyRing();

  // And the aged-out press edge means the same button presses again cleanly.
  deliver(input, press, sizeof press);
  EXPECT_TRUE(popKey().pressed);
}

TEST_F(IngestTest, StaleReleaseTimeoutPaysTheOwedReleaseForAStreamingRemote) {
  // The other family: the remote streams the held key and never sends a release
  // frame. Silence IS the release here, so the timeout pays the owed release once
  // - otherwise the app would wait forever for an edge that never comes.
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  const int input = serveInputReport(nullptr, 0);
  ASSERT_TRUE(beginAndConnect());

  const std::vector<uint8_t> held = bootReport(0, {0x4E});
  deliver(input, held);
  EXPECT_TRUE(popKey().pressed);
  for (int i = 0; i < 4; ++i) {
    fakeble::advanceMillis(40);
    deliver(input, held);
    fakeble::host().poll();
    expectEmptyRing();  // still streaming: nothing is owed yet
  }

  fakeble::advanceMillis(200);
  fakeble::host().poll();
  const KeyEvent up = popKey();
  EXPECT_EQ(up.keycode, 0x4E);
  EXPECT_FALSE(up.pressed);

  // Once paid, further polls owe nothing.
  for (int i = 0; i < 5; ++i) {
    fakeble::advanceMillis(200);
    fakeble::host().poll();
  }
  expectEmptyRing();
}

TEST_F(IngestTest, ReportStreamFreshFollowsTheRemoteNotifications) {
  // What the app needs to tell "the button is still down and the remote keeps
  // saying so" from "the remote went quiet".
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  const int input = serveInputReport(nullptr, 0);
  EXPECT_FALSE(fakeble::host().reportStreamFresh());  // not even running
  ASSERT_TRUE(beginAndConnect());
  EXPECT_FALSE(fakeble::host().reportStreamFresh());  // connected, nothing said yet

  deliver(input, bootReport(0, {0x4E}));
  EXPECT_TRUE(fakeble::host().reportStreamFresh());
  fakeble::advanceMillis(100);
  EXPECT_TRUE(fakeble::host().reportStreamFresh());
  fakeble::advanceMillis(100);
  EXPECT_FALSE(fakeble::host().reportStreamFresh());
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
  EXPECT_FALSE(popKey().pressed);
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
  EXPECT_FALSE(popKey().pressed);
  expectEmptyRing();

  // A remote that omits the id byte its descriptor declares: the Report Reference
  // of the notifying characteristic names the layout instead.
  const uint8_t consumerNoId[2] = {0xCD, 0x00};
  deliver(consumer, consumerNoId, sizeof consumerNoId);
  EXPECT_EQ(popKey().keycode, 0xCD);
  const uint8_t consumerNoIdRelease[2] = {0x00, 0x00};
  deliver(consumer, consumerNoIdRelease, sizeof consumerNoIdRelease);
  EXPECT_FALSE(popKey().pressed);
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
  EXPECT_FALSE(popKey().pressed);
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
  EXPECT_FALSE(popKey().pressed);  // 0x04 is no longer in the report
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
  EXPECT_FALSE(popKey().pressed);  // 0x05 came up when the report stopped listing it
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
  // survive, in the order they arrived. Each report also ends the press the report
  // before it left owing, so the order is six presses, that batch's release, and so
  // on - a released usage is written {usage, pressed=false}.
  EXPECT_EQ(BleKeyboardHost::kKeyQueueLen, 16);
  const std::pair<uint8_t, bool> expected[15] = {
      {0x04, true},  {0x05, true}, {0x06, true}, {0x07, true}, {0x08, true},
      {0x09, true},  {0x09, false},
      {0x0A, true},  {0x0B, true}, {0x0C, true}, {0x0D, true}, {0x0E, true},
      {0x0F, true},  {0x0F, false},
      {0x10, true}};
  for (size_t i = 0; i < 15; ++i) {
    const KeyEvent event = popKey();
    EXPECT_EQ(event.keycode, expected[i].first) << "slot " << i;
    EXPECT_EQ(event.pressed, expected[i].second) << "slot " << i;
  }
  expectEmptyRing();

  // After a full drain the ring indices wrap; ordering still holds. 0x15 was the last
  // press recorded before the overflow, so its release leads the next report.
  const std::vector<uint8_t> fourth = bootReport(0, {0x20, 0x21});
  deliver(input, fourth);
  const KeyEvent stale = popKey();
  EXPECT_EQ(stale.keycode, 0x15);
  EXPECT_FALSE(stale.pressed);
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

TEST_F(IngestTest, AutoReconnectMakesOnePassAndExplicitConnectRearmsIt) {
  PairedHidDevice bonds[2] = {};
  strcpy(bonds[0].addr, kAddr);
  strcpy(bonds[1].addr, "AA:BB:CC:DD:EE:FE");
  fakeble::state().nvs["freeink-hid/n"] = {2};
  const auto* bytes = reinterpret_cast<const uint8_t*>(bonds);
  fakeble::state().nvs["freeink-hid/b"] = {bytes, bytes + sizeof bonds};
  fakeble::setConnectSucceeds(false);
  ASSERT_TRUE(fakeble::beginHost());
  for (int i = 0; i < 5; ++i) {
    fakeble::advanceMillis(4001);
    fakeble::host().poll();
    ASSERT_TRUE(fakeble::waitForWorkerIdle());
  }
  EXPECT_EQ(fakeble::state().connectCalls, 2u);

  ASSERT_TRUE(fakeble::host().connect(kAddr));
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  for (int i = 0; i < 4; ++i) {
    fakeble::advanceMillis(4001);
    fakeble::host().poll();
    ASSERT_TRUE(fakeble::waitForWorkerIdle());
  }
  // The explicit request already tried bond zero; only bond one remains.
  EXPECT_EQ(fakeble::state().connectCalls, 4u);
}

TEST_F(IngestTest, LinkLossRearmsOnceAndExplicitDisconnectRemainsDisconnected) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  serveInputReport(nullptr, 0);
  ASSERT_TRUE(beginAndConnect());
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  fakeble::state().client->disconnect();  // peer loss, not the public user action
  fakeble::advanceMillis(4001);
  fakeble::host().poll();
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_TRUE(fakeble::host().isConnected());
  EXPECT_EQ(fakeble::state().connectCalls, 2u);

  fakeble::host().disconnect();
  fakeble::advanceMillis(4001);
  fakeble::host().poll();
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_FALSE(fakeble::host().isConnected());
  EXPECT_EQ(fakeble::state().connectCalls, 2u);
}

class ConnectionDeadlineTest : public IngestTest, public ::testing::WithParamInterface<fakeble::BlockingStage> {};

TEST_P(ConnectionDeadlineTest, CancelsWholeAttemptWithoutDeletingTheLiveStack) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  serveInputReport(nullptr, 0);
  ASSERT_TRUE(fakeble::beginHost());
  fakeble::setBlockingStage(GetParam());
  ASSERT_TRUE(fakeble::host().connect(kAddr));
  ASSERT_TRUE(fakeble::waitForBlockingStage(GetParam()));
  auto* client = fakeble::state().client;
  fakeble::advanceMillis(14999);
  fakeble::host().poll();
  EXPECT_EQ(fakeble::state().cancelConnectCalls, 0u);
  fakeble::advanceMillis(1);
  fakeble::host().poll();
  EXPECT_GT(fakeble::state().cancelConnectCalls, 0u);
  EXPECT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_FALSE(fakeble::host().isConnecting());
  EXPECT_FALSE(fakeble::host().isConnected());
  EXPECT_EQ(fakeble::state().client, client);
  EXPECT_EQ(fakeble::state().deleteClientCalls, 0u);
  EXPECT_TRUE(fakeble::host().isRunning());
  char failure[48] = {};
  EXPECT_TRUE(fakeble::host().takeConnectFailure(failure, sizeof failure));
  EXPECT_STREQ(failure, "Connect timeout");
  // A second poll must not resurrect the cancelled target from a stale wake.
  fakeble::host().poll();
  EXPECT_EQ(fakeble::state().connectCalls, 1u);
}

INSTANTIATE_TEST_SUITE_P(AllStages, ConnectionDeadlineTest,
                        ::testing::Values(fakeble::BlockingStage::Connect, fakeble::BlockingStage::Security,
                                          fakeble::BlockingStage::Discovery));

TEST_F(IngestTest, QueuedAttemptDeadlineSurvivesClockWrapAndSkipsLateWorkerWake) {
  ASSERT_TRUE(fakeble::beginHost());
  fakeble::holdWorkerNotifications(true);
  fakeble::advanceMillis(UINT32_MAX - fakeble::clockMs() - 7000);
  ASSERT_TRUE(fakeble::host().connect(kAddr));
  fakeble::advanceMillis(15000);
  fakeble::host().poll();
  fakeble::holdWorkerNotifications(false);
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
  EXPECT_FALSE(fakeble::host().isConnecting());
  char failure[48] = {};
  EXPECT_TRUE(fakeble::host().takeConnectFailure(failure, sizeof failure));
  EXPECT_STREQ(failure, "Connect timeout");
}

TEST_F(IngestTest, DeadlineRetainsUncooperativeWorkerAndSuppressesLateSuccess) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  serveInputReport(nullptr, 0);
  ASSERT_TRUE(fakeble::beginHost());
  fakeble::setBlockingStage(fakeble::BlockingStage::Security, /*ignoreCancellation=*/true);
  ASSERT_TRUE(fakeble::host().connect(kAddr));
  ASSERT_TRUE(fakeble::waitForBlockingStage(fakeble::BlockingStage::Security));
  auto* client = fakeble::state().client;
  fakeble::advanceMillis(15000);
  fakeble::host().poll();
  EXPECT_EQ(fakeble::state().client, client);
  EXPECT_EQ(fakeble::state().deleteClientCalls, 0u);
  EXPECT_FALSE(fakeble::host().connect(kAddr));
  fakeble::host().onLinkUp(kAddr, "Late peer", 0);
  EXPECT_FALSE(fakeble::host().isConnected());
  fakeble::releaseBlockingCall();
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_FALSE(fakeble::host().isConnected());
  EXPECT_EQ(fakeble::state().connectCalls, 1u);
}

TEST_F(IngestTest, DeadlineRetryWaitsForTheClientToFinishDisconnecting) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  serveInputReport(nullptr, 0);
  ASSERT_TRUE(fakeble::beginHost());
  fakeble::setBlockingStage(fakeble::BlockingStage::Security);
  fakeble::setDisconnectAtDisconnecting(true);
  ASSERT_TRUE(fakeble::host().connect(kAddr));
  ASSERT_TRUE(fakeble::waitForBlockingStage(fakeble::BlockingStage::Security));
  fakeble::advanceMillis(15000);
  fakeble::host().poll();
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_EQ(NimBLEDevice::getDisconnectedClient(), nullptr);
  EXPECT_FALSE(fakeble::host().connect(kAddr));
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_EQ(fakeble::state().connectCalls, 1u);

  fakeble::completeDisconnect();
  ASSERT_TRUE(fakeble::host().connect(kAddr));
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_TRUE(fakeble::host().isConnected());
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


// A reader can select one saved remote without spending its reconnect budget on
// other bonded keyboards. The default one-pass policy above remains unchanged.
class SelectedReconnectTest : public IngestTest {
 protected:
  static constexpr const char* selected = "AA:BB:CC:DD:EE:FE";
  void SetUp() override {
    IngestTest::SetUp();
    PairedHidDevice bonds[2] = {};
    strcpy(bonds[0].addr, kAddr);
    strcpy(bonds[1].addr, selected);
    fakeble::state().nvs["freeink-hid/n"] = {2};
    const auto* bytes = reinterpret_cast<const uint8_t*>(bonds);
    fakeble::state().nvs["freeink-hid/b"] = {bytes, bytes + sizeof bonds};
    serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
    serveInputReport(nullptr, 0);
    fakeble::setConnectSucceeds(false);
    ASSERT_TRUE(fakeble::beginHost());
  }
  void pollAndWait() {
    fakeble::host().poll();
    ASSERT_TRUE(fakeble::waitForWorkerIdle());
  }
};

TEST_F(SelectedReconnectTest, SkipsUnrelatedFirstBondAndRetriesSelectedWhenItWakes) {
  const auto originalNvs = fakeble::state().nvs;
  ASSERT_TRUE(fakeble::host().armSelectedPeerReconnect(selected));
  pollAndWait();
  ASSERT_EQ(fakeble::state().connectAddresses, std::vector<std::string>{selected});
  fakeble::advanceMillis(3999);
  pollAndWait();
  EXPECT_EQ(fakeble::state().connectCalls, 1u);
  fakeble::setConnectSucceeds(true);
  fakeble::advanceMillis(1);
  pollAndWait();
  EXPECT_TRUE(fakeble::host().isConnected());
  EXPECT_EQ(fakeble::state().connectAddresses, (std::vector<std::string>{selected, selected}));
  EXPECT_EQ(fakeble::state().nvs, originalNvs);
}

TEST_F(SelectedReconnectTest, StopsAfterSixAttemptsWithoutFallingBackToOtherBonds) {
  ASSERT_TRUE(fakeble::host().armSelectedPeerReconnect(selected));
  for (int i = 0; i < 20; ++i) {
    pollAndWait();
    fakeble::advanceMillis(4000);
  }
  EXPECT_EQ(fakeble::state().connectAddresses, std::vector<std::string>(6, selected));
}

TEST_F(SelectedReconnectTest, WindowExpiresBeforeADeferredPollCanStartAnotherAttempt) {
  ASSERT_TRUE(fakeble::host().armSelectedPeerReconnect(selected));
  pollAndWait();
  fakeble::advanceMillis(120000);
  pollAndWait();
  EXPECT_EQ(fakeble::state().connectCalls, 1u);
}

TEST_F(SelectedReconnectTest, InvalidTargetDoesNotReplaceDefaultSchedule) {
  EXPECT_FALSE(fakeble::host().armSelectedPeerReconnect(nullptr));
  EXPECT_FALSE(fakeble::host().armSelectedPeerReconnect(""));
  const char unterminated[18] = {'A','A',':','B','B',':','C','C',':','D','D',':','E','E',':','F','E','X'};
  EXPECT_FALSE(fakeble::host().armSelectedPeerReconnect(unterminated));
  EXPECT_FALSE(fakeble::host().armSelectedPeerReconnect("AA:BB:CC:DD:EE:00"));
  fakeble::advanceMillis(4000);
  pollAndWait();
  EXPECT_EQ(fakeble::state().connectAddresses, std::vector<std::string>{kAddr});
}

TEST_F(SelectedReconnectTest, RepeatedArmAndPollCannotDuplicateQueuedAttempt) {
  ASSERT_TRUE(fakeble::host().armSelectedPeerReconnect(selected));
  EXPECT_FALSE(fakeble::host().armSelectedPeerReconnect(selected));
  fakeble::holdWorkerNotifications(true);
  fakeble::host().poll();
  EXPECT_TRUE(fakeble::host().isConnecting());
  EXPECT_FALSE(fakeble::host().armSelectedPeerReconnect(kAddr));
  for (int i = 0; i < 10; ++i) fakeble::host().poll();
  fakeble::holdWorkerNotifications(false);
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_EQ(fakeble::state().connectAddresses, std::vector<std::string>{selected});
}

TEST_F(SelectedReconnectTest, DisconnectCancelsQueuedAttemptAndSuppressesRetries) {
  ASSERT_TRUE(fakeble::host().armSelectedPeerReconnect(selected));
  fakeble::holdWorkerNotifications(true);
  fakeble::host().poll();
  fakeble::host().disconnect();
  fakeble::holdWorkerNotifications(false);
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  fakeble::advanceMillis(4000);
  pollAndWait();
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
}

TEST_F(SelectedReconnectTest, DisconnectCancelsInFlightAttemptAndSuppressesRetries) {
  ASSERT_TRUE(fakeble::host().armSelectedPeerReconnect(selected));
  fakeble::setBlockingStage(fakeble::BlockingStage::Connect);
  fakeble::host().poll();
  ASSERT_TRUE(fakeble::waitForBlockingStage(fakeble::BlockingStage::Connect));
  EXPECT_FALSE(fakeble::host().armSelectedPeerReconnect(kAddr));
  fakeble::host().disconnect();
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  fakeble::advanceMillis(4000);
  pollAndWait();
  EXPECT_EQ(fakeble::state().connectCalls, 1u);
  EXPECT_FALSE(fakeble::host().isConnected());
}

TEST_F(SelectedReconnectTest, EndCancelsPlanAndNextBeginRestoresDefaultPolicy) {
  ASSERT_TRUE(fakeble::host().armSelectedPeerReconnect(selected));
  pollAndWait();
  ASSERT_TRUE(fakeble::host().end());
  EXPECT_FALSE(fakeble::host().armSelectedPeerReconnect(selected));
  ASSERT_TRUE(fakeble::beginHost());
  fakeble::advanceMillis(4000);
  pollAndWait();
  EXPECT_EQ(fakeble::state().connectAddresses, (std::vector<std::string>{selected, kAddr}));
}

TEST_F(SelectedReconnectTest, ExplicitConnectReplacesSelectedPlan) {
  ASSERT_TRUE(fakeble::host().armSelectedPeerReconnect(selected));
  ASSERT_TRUE(fakeble::host().connect(kAddr));
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  fakeble::advanceMillis(4000);
  pollAndWait();
  fakeble::advanceMillis(4000);
  pollAndWait();
  EXPECT_EQ(fakeble::state().connectAddresses, (std::vector<std::string>{kAddr, selected}));
}

TEST_F(SelectedReconnectTest, BackoffAndWindowSurviveMillisWrap) {
  fakeble::advanceMillis(UINT32_MAX - fakeble::clockMs() - 1000);
  ASSERT_TRUE(fakeble::host().armSelectedPeerReconnect(selected));
  pollAndWait();
  fakeble::advanceMillis(3999);
  pollAndWait();
  EXPECT_EQ(fakeble::state().connectCalls, 1u);
  fakeble::advanceMillis(1);
  pollAndWait();
  EXPECT_EQ(fakeble::state().connectCalls, 2u);
  fakeble::advanceMillis(116000);
  pollAndWait();
  EXPECT_EQ(fakeble::state().connectCalls, 2u);
}

TEST_F(SelectedReconnectTest, UnexpectedLinkLossRetriesSamePeerAndUserDisconnectStaysOff) {
  fakeble::setConnectSucceeds(true);
  ASSERT_TRUE(fakeble::host().armSelectedPeerReconnect(selected));
  pollAndWait();
  ASSERT_TRUE(fakeble::host().isConnected());
  fakeble::advanceMillis(120001);
  fakeble::state().client->disconnect();
  fakeble::advanceMillis(4000);
  pollAndWait();
  EXPECT_TRUE(fakeble::host().isConnected());
  EXPECT_EQ(fakeble::state().connectAddresses, (std::vector<std::string>{selected, selected}));
  fakeble::host().disconnect();
  fakeble::advanceMillis(4000);
  pollAndWait();
  EXPECT_FALSE(fakeble::host().isConnected());
  EXPECT_EQ(fakeble::state().connectCalls, 2u);
}

TEST_F(SelectedReconnectTest, EndCancelsQueuedAttemptBeforeWorkerCanConnect) {
  ASSERT_TRUE(fakeble::host().armSelectedPeerReconnect(selected));
  fakeble::holdWorkerNotifications(true);
  fakeble::host().poll();
  EXPECT_FALSE(fakeble::host().end(0));
  EXPECT_FALSE(fakeble::host().armSelectedPeerReconnect(selected));
  fakeble::holdWorkerNotifications(false);
  ASSERT_TRUE(fakeble::host().end());
  EXPECT_EQ(fakeble::state().connectCalls, 0u);
}

TEST_F(SelectedReconnectTest, ExistingScanRejectsArmWithoutStoppingScan) {
  fakeble::host().startScan();
  EXPECT_FALSE(fakeble::host().armSelectedPeerReconnect(selected));
  EXPECT_TRUE(fakeble::host().isScanning());
}

TEST_F(SelectedReconnectTest, ExistingConnectedPeerRejectsArmWithoutDroppingLink) {
  fakeble::setConnectSucceeds(true);
  ASSERT_EQ(fakeble::connectTo(kAddr), fakeble::LinkResult::Connected);
  EXPECT_FALSE(fakeble::host().armSelectedPeerReconnect(selected));
  EXPECT_TRUE(fakeble::host().isConnected());
}

// --- Raw button edges (read from the report bytes, before the map decode) ------

namespace {

// Raw edges the host queued so far, in order.
std::vector<RawButtonEvent> drainRaw() {
  std::vector<RawButtonEvent> edges;
  RawButtonEvent ev;
  while (fakeble::host().popRawButton(ev)) edges.push_back(ev);
  return edges;
}

void drainKeys() {
  KeyEvent ev;
  while (fakeble::host().popKey(ev)) {
  }
}

}  // namespace

class RawButtonTest : public IngestTest {
 protected:
  // The three-button remote: its map, and one Input characteristic per report id,
  // each declaring its id through a Report Reference.
  void connectThreeButtonRemote() {
    serveReportMap(hidtest::kThreeButtonRemote, sizeof hidtest::kThreeButtonRemote);
    serveProtocolMode();
    const uint8_t ref1[2] = {1, 1};
    const uint8_t ref3[2] = {3, 1};
    const uint8_t ref2[2] = {2, 1};
    const uint8_t ref6[2] = {6, 1};
    keyboard_ = serveInputReport(ref1, sizeof ref1);
    media_ = serveInputReport(ref3, sizeof ref3);
    vendor_ = serveInputReport(ref2, sizeof ref2);
    small_ = serveInputReport(ref6, sizeof ref6);
    ASSERT_TRUE(beginAndConnect());
  }

  void frame(int characteristic, std::initializer_list<uint8_t> bytes) {
    const std::vector<uint8_t> data(bytes);
    deliver(characteristic, data);
  }

  int keyboard_ = -1;
  int media_ = -1;
  int vendor_ = -1;
  int small_ = -1;
};

TEST_F(RawButtonTest, RebuiltMapParsesLikeTheDeviceLogged) {
  // The device logged: reports=4 kbd=1 consumer=1 usable=1 truncated=1, and per id
  // 1: 64 bits 1 key field 1 mod field, 3: 24 bits 8 key fields, 2: 56 bits no
  // field, 6: 8 bits 8 key fields. The fixture is only a stand-in if it matches.
  HidReportMap map;
  ASSERT_TRUE(parseHidReportMap(hidtest::kThreeButtonRemote, sizeof hidtest::kThreeButtonRemote, map));
  EXPECT_TRUE(map.hasKeyboardPage);
  EXPECT_TRUE(map.hasConsumerPage);
  EXPECT_TRUE(map.truncated);
  ASSERT_EQ(map.reportCount, 4);
  const uint8_t ids[4] = {1, 3, 2, 6};
  const uint16_t bits[4] = {64, 24, 56, 8};
  const uint8_t keys[4] = {1, 8, 0, 8};
  const uint8_t mods[4] = {1, 0, 0, 0};
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(map.reports[i].id, ids[i]) << i;
    EXPECT_EQ(map.reports[i].bits, bits[i]) << i;
    EXPECT_EQ(map.reports[i].keyFieldCount, keys[i]) << i;
    EXPECT_EQ(map.reports[i].modFieldCount, mods[i]) << i;
  }
}

TEST_F(RawButtonTest, ThirdButtonTapReachesTheRawRingThoughTheDecoderSeesNothing) {
  connectThreeButtonRemote();
  frame(media_, {0x00, 0x02, 0x00});
  frame(media_, {0x00, 0x00, 0x00});

  // The decoder has no field on byte 1: no key event at all, as on the device.
  expectEmptyRing();

  const std::vector<RawButtonEvent> edges = drainRaw();
  ASSERT_EQ(edges.size(), 2u);
  EXPECT_TRUE(edges[0].pressed);
  EXPECT_EQ(edges[0].reportId, 3);
  EXPECT_EQ(edges[0].byteIndex, 1);
  EXPECT_EQ(edges[0].value, 0x02);
  EXPECT_EQ(edges[0].keycode, 0);
  EXPECT_FALSE(edges[1].pressed);
  EXPECT_EQ(edges[1].code(), edges[0].code()) << "the release names the button that came up";
}

TEST_F(RawButtonTest, HeldThirdButtonFrameKeepsItsOwnIdentity) {
  connectThreeButtonRemote();
  frame(media_, {0x08, 0x00, 0x00});
  const std::vector<RawButtonEvent> edges = drainRaw();
  ASSERT_EQ(edges.size(), 1u);
  EXPECT_TRUE(edges[0].pressed);
  EXPECT_EQ(edges[0].code(), 0x030008u);
  // The decoder cut the 16-bit media usage to its low byte: 0x30, which binds to
  // nothing. The raw identity keeps the byte the remote actually sent.
  EXPECT_EQ(edges[0].keycode, 0x30);
}

TEST_F(RawButtonTest, PageButtonKeepsItsKeyEventsAndGainsRawEdges) {
  connectThreeButtonRemote();
  frame(media_, {0x02, 0x00, 0x00});
  const KeyEvent down = popKey();
  EXPECT_EQ(down.keycode, 0x02);
  EXPECT_TRUE(down.pressed);
  expectEmptyRing();
  frame(media_, {0x00, 0x00, 0x00});
  const KeyEvent up = popKey();
  EXPECT_EQ(up.keycode, 0x02);
  EXPECT_FALSE(up.pressed);
  expectEmptyRing();

  const std::vector<RawButtonEvent> edges = drainRaw();
  ASSERT_EQ(edges.size(), 2u);
  EXPECT_TRUE(edges[0].pressed);
  EXPECT_EQ(edges[0].code(), 0x030002u);
  EXPECT_EQ(edges[0].keycode, 0x02) << "the press carries the key the decoder read from the same frame";
  EXPECT_FALSE(edges[1].pressed);
  EXPECT_EQ(edges[1].code(), 0x030002u);
}

TEST_F(RawButtonTest, PressOnlyFramesAfterSilenceAreNewPresses) {
  // One-byte frames with no release in between, 400 ms apart: every frame is a
  // new press of the same button.
  connectThreeButtonRemote();
  for (int i = 0; i < 3; ++i) {
    frame(small_, {0x01});
    fakeble::advanceMillis(400);
    fakeble::host().poll();
  }
  int presses = 0;
  for (const RawButtonEvent& e : drainRaw()) {
    EXPECT_EQ(e.code(), 0x060001u);
    if (e.pressed) ++presses;
  }
  EXPECT_EQ(presses, 3);
}

TEST_F(RawButtonTest, StreamedHoldIsOnePressAndOneReleaseAfterSilence) {
  connectThreeButtonRemote();
  for (int i = 0; i < 6; ++i) {
    frame(media_, {0x00, 0x02, 0x00});
    fakeble::advanceMillis(40);
    fakeble::host().poll();
  }
  std::vector<RawButtonEvent> edges = drainRaw();
  ASSERT_EQ(edges.size(), 1u);
  EXPECT_TRUE(edges[0].pressed);
  const uint32_t pressedAt = edges[0].atMs;

  fakeble::advanceMillis(200);
  fakeble::host().poll();
  edges = drainRaw();
  ASSERT_EQ(edges.size(), 1u);
  EXPECT_FALSE(edges[0].pressed);
  EXPECT_EQ(edges[0].code(), 0x030102u);
  EXPECT_EQ(edges[0].atMs - pressedAt, 200u) << "the release is dated by the last frame, not by the poll";

  for (int i = 0; i < 3; ++i) {
    fakeble::advanceMillis(200);
    fakeble::host().poll();
  }
  EXPECT_TRUE(drainRaw().empty());
}

TEST_F(RawButtonTest, SilentHoldKeepsItsReleaseUntilTheReleaseFrame) {
  // One frame per edge and nothing between: silence is a held button, so the
  // hold time is the gap between the two frames.
  connectThreeButtonRemote();
  frame(media_, {0x00, 0x02, 0x00});
  fakeble::advanceMillis(900);
  for (int i = 0; i < 3; ++i) fakeble::host().poll();
  frame(media_, {0x00, 0x00, 0x00});
  const std::vector<RawButtonEvent> edges = drainRaw();
  ASSERT_EQ(edges.size(), 2u);
  EXPECT_TRUE(edges[0].pressed);
  EXPECT_FALSE(edges[1].pressed);
  EXPECT_EQ(edges[1].atMs - edges[0].atMs, 900u);
}

TEST_F(RawButtonTest, KeyboardModifierByteIsNotTheButton) {
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  const int input = serveInputReport(nullptr, 0);
  ASSERT_TRUE(beginAndConnect());
  deliver(input, bootReport(HID_LSHIFT, {0x04}));
  const std::vector<RawButtonEvent> edges = drainRaw();
  ASSERT_EQ(edges.size(), 1u);
  EXPECT_EQ(edges[0].byteIndex, 2);
  EXPECT_EQ(edges[0].value, 0x04);
  EXPECT_EQ(edges[0].keycode, 0x04);
  EXPECT_EQ(edges[0].mods, HID_LSHIFT);
  drainKeys();
}

TEST_F(RawButtonTest, IdByteAndReportReferenceNameTheSameButton) {
  // A remote that prefixes the id byte its descriptor declares, and one that
  // leaves it out: the same button reads as the same identity.
  serveReportMap(hidtest::kTwoReports, sizeof hidtest::kTwoReports);
  const uint8_t refConsumer[2] = {2, 1};
  const int consumer = serveInputReport(refConsumer, sizeof refConsumer);
  ASSERT_TRUE(beginAndConnect());
  const uint8_t withId[3] = {2, 0xCD, 0x00};
  const uint8_t withIdUp[3] = {2, 0x00, 0x00};
  const uint8_t noId[2] = {0xCD, 0x00};
  const uint8_t noIdUp[2] = {0x00, 0x00};
  deliver(consumer, withId, sizeof withId);
  deliver(consumer, withIdUp, sizeof withIdUp);
  deliver(consumer, noId, sizeof noId);
  deliver(consumer, noIdUp, sizeof noIdUp);
  const std::vector<RawButtonEvent> edges = drainRaw();
  ASSERT_EQ(edges.size(), 4u);
  EXPECT_EQ(edges[0].code(), 0x0200CDu);
  EXPECT_EQ(edges[2].code(), 0x0200CDu);
  EXPECT_TRUE(edges[2].pressed);
  drainKeys();
}

TEST_F(RawButtonTest, ConnectedAddressFollowsTheLink) {
  EXPECT_STREQ(fakeble::host().connectedAddr(), "");
  connectThreeButtonRemote();
  EXPECT_STREQ(fakeble::host().connectedAddr(), kAddr);
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  fakeble::host().disconnect();
  fakeble::advanceMillis(10);
  fakeble::host().poll();
  ASSERT_TRUE(fakeble::waitForWorkerIdle());
  EXPECT_FALSE(fakeble::host().isConnected());
  EXPECT_STREQ(fakeble::host().connectedAddr(), "");
}

TEST_F(RawButtonTest, RawRingOverflowDropsWholePressesAndNeverARelease) {
  connectThreeButtonRemote();
  const uint16_t overflowsBefore = fakeble::host().rawOverflows();  // a count since boot
  // Eight taps (sixteen edges) into a ring with seven usable slots, nobody draining:
  // a burst while the reader lays out a page. What is dropped must be whole presses;
  // a press that got in must keep its release, or the button stays down for the app.
  for (uint8_t i = 1; i <= 8; ++i) {
    frame(media_, {i, 0x00, 0x00});
    frame(media_, {0x00, 0x00, 0x00});
  }
  std::vector<RawButtonEvent> edges = drainRaw();
  int presses = 0;
  int releases = 0;
  for (const RawButtonEvent& e : edges) (e.pressed ? presses : releases)++;
  EXPECT_EQ(presses, releases) << "a press lost its release";
  EXPECT_EQ(presses, 3);
  EXPECT_EQ(edges[0].value, 1);
  EXPECT_EQ(fakeble::host().rawOverflows() - overflowsBefore, 5) << "every dropped press is counted";

  // A press accepted with the ring nearly full still gets its release when the
  // next button replaces it in a single frame.
  frame(media_, {0x01, 0x00, 0x00});
  frame(media_, {0x02, 0x00, 0x00});
  frame(media_, {0x03, 0x00, 0x00});
  frame(media_, {0x00, 0x00, 0x00});
  frame(media_, {0x00, 0x00, 0x00});
  presses = releases = 0;
  edges = drainRaw();
  for (const RawButtonEvent& e : edges) (e.pressed ? presses : releases)++;
  EXPECT_EQ(presses, releases);
  EXPECT_EQ(presses, 3);

  // Once drained, taps flow again.
  frame(media_, {0x04, 0x00, 0x00});
  frame(media_, {0x00, 0x00, 0x00});
  EXPECT_EQ(drainRaw().size(), 2u);
  drainKeys();
}

TEST_F(RawButtonTest, ConstantStatusByteRemoteKeepsOneIdentityPerButton) {
  // A remote whose frames carry a status byte that is never 0 (0x12 at rest), with
  // its buttons as bits in later bytes. The first non-zero byte is the status byte
  // in every frame: read that way, every button, and the rest frame, is one code
  // and nothing ever comes up. Buttons must be read against the rest frame.
  serveReportMap(hidtest::kNoInput, sizeof hidtest::kNoInput);
  const int input = serveInputReport(nullptr, 0);
  ASSERT_TRUE(beginAndConnect());
  const uint8_t rest[3] = {0x12, 0x00, 0x00};
  const uint8_t a[3] = {0x12, 0x04, 0x00};
  const uint8_t b[3] = {0x12, 0x00, 0x08};
  deliver(input, rest, sizeof rest);  // the state report many remotes send first
  std::vector<uint32_t> pressed;
  int open = 0;
  for (int round = 0; round < 2; ++round) {
    deliver(input, a, sizeof a);
    deliver(input, rest, sizeof rest);
    deliver(input, b, sizeof b);
    deliver(input, rest, sizeof rest);
    for (const RawButtonEvent& e : drainRaw()) {  // the app drains every pass
      open += e.pressed ? 1 : -1;
      if (e.pressed) pressed.push_back(e.code());
    }
  }
  EXPECT_EQ(open, 0) << "a press was left without its release";
  const uint32_t codeA = 0x000104;
  const uint32_t codeB = 0x000208;
  EXPECT_EQ(std::count(pressed.begin(), pressed.end(), codeA), 2);
  EXPECT_EQ(std::count(pressed.begin(), pressed.end(), codeB), 2);
  drainKeys();
}

TEST_F(RawButtonTest, GamepadAxisButtonsAreKnownByTheZoneTheDecoderReads) {
  // Axis-pair gamepad mode: byte 0 is a status byte (0x12 rest, 0x13 pressed) that
  // every button raises, bytes 1-4 two axes that ramp while held. No byte of such a
  // frame names a button; the decoder names it on the release frame from the axis
  // zones. Two directions must give two identities, and no byte identity at all.
  serveReportMap(hidtest::kNoInput, sizeof hidtest::kNoInput);
  const int input = serveInputReport(nullptr, 0);
  ASSERT_TRUE(beginAndConnect());
  const uint8_t rest[5] = {0x12, 0xD0, 0x07, 0xD0, 0x07};
  const uint8_t upDown[5] = {0x13, 0xD0, 0x07, 0x84, 0x03};
  const uint8_t upUp[5] = {0x12, 0xD0, 0x07, 0x84, 0x03};
  const uint8_t downDown[5] = {0x13, 0xD0, 0x07, 0x10, 0x0E};
  const uint8_t downUp[5] = {0x12, 0xD0, 0x07, 0x10, 0x0E};
  deliver(input, rest, sizeof rest);
  deliver(input, upDown, sizeof upDown);
  deliver(input, upUp, sizeof upUp);
  deliver(input, rest, sizeof rest);
  deliver(input, downDown, sizeof downDown);
  deliver(input, downUp, sizeof downUp);
  deliver(input, rest, sizeof rest);
  std::vector<uint32_t> pressed;
  int open = 0;
  for (const RawButtonEvent& e : drainRaw()) {
    open += e.pressed ? 1 : -1;
    if (e.pressed) pressed.push_back(e.code());
  }
  EXPECT_EQ(open, 0);
  ASSERT_EQ(pressed.size(), 2u) << "byte identities leaked from an axis frame";
  EXPECT_NE(pressed[0], pressed[1]);
  EXPECT_EQ(pressed[0] & 0xFF, 0x43u) << "up = axis 1 centered, axis 2 low";
  EXPECT_EQ(pressed[1] & 0xFF, 0x45u) << "down = axis 1 centered, axis 2 high";
  drainKeys();
}

TEST_F(RawButtonTest, KeyAddedWhileAnotherIsHeldStillReachesTheApp) {
  // A held, then B: the first differing byte is still A's, so no byte edge names B.
  // The key the decoder read must still reach the app, or a remote with a table
  // loses B (its fallback to the usage mapping runs on raw press edges).
  serveReportMap(hidtest::kBootKeyboard, sizeof hidtest::kBootKeyboard);
  const int input = serveInputReport(nullptr, 0);
  ASSERT_TRUE(beginAndConnect());
  deliver(input, bootReport(0, {0x04}));  // a first tap, so the rest frame is known
  deliver(input, bootReport(0, {}));
  drainRaw();
  deliver(input, bootReport(0, {0x04}));
  deliver(input, bootReport(0, {0x04, 0x05}));
  bool sawB = false;
  for (const RawButtonEvent& e : drainRaw()) sawB = sawB || (e.pressed && e.keycode == 0x05);
  EXPECT_TRUE(sawB);
  drainKeys();
}
