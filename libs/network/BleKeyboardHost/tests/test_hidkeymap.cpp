// Host tests for the HID Report Map parser in src/HidKeymap.{h,cpp}.
//
// Pure g++/clang++: no NimBLE, no Arduino runtime. HidKeymap.h pulls in
// include/BleKeyboardHost.h for SpecialKey/KeyEvent, and that header only needs
// <Arduino.h> to exist - which the one-line stub in the build commands supplies.
//
// Build and run from freeink-sdk/libs/network/BleKeyboardHost:
//
//   mkdir -p /tmp/freeink-host-stub
//   printf '#pragma once\n#include <stdint.h>\n#include <stddef.h>\ntypedef uint8_t byte;\n' \
//       > /tmp/freeink-host-stub/Arduino.h
//   g++ -std=c++17 -Wall -Wextra -Werror -I include -I /tmp/freeink-host-stub \
//       src/HidKeymap.cpp tests/test_hidkeymap.cpp -o /tmp/hidtest
//   /tmp/hidtest
//
// Same under the sanitizers (proves the fixed arrays stay in bounds):
//
//   g++ -std=c++17 -fsanitize=address,undefined -fno-sanitize-recover=all \
//       -I include -I /tmp/freeink-host-stub src/HidKeymap.cpp \
//       tests/test_hidkeymap.cpp -o /tmp/hidtest_asan && /tmp/hidtest_asan
//
// Exit status: 0 when every check passes, 1 otherwise.

#include <cstdint>
#include <cstdio>

#include "../src/HidKeymap.h"

using namespace freeink;  // test-only: the parser lives in the SDK namespace

namespace {

// --- Tiny check harness ------------------------------------------------------

int gChecks = 0;
int gFailures = 0;

void checkTrue(bool ok, const char* what, int line) {
  ++gChecks;
  if (!ok) {
    ++gFailures;
    std::printf("FAIL line %d: %s\n", line, what);
  }
}

void checkEq(long long got, long long want, const char* what, int line) {
  ++gChecks;
  if (got != want) {
    ++gFailures;
    std::printf("FAIL line %d: %s (got %lld, want %lld)\n", line, what, got, want);
  }
}

#define CHECK(cond) checkTrue((cond), #cond, __LINE__)
#define CHECK_EQ(got, want) \
  checkEq(static_cast<long long>(got), static_cast<long long>(want), #got " == " #want, __LINE__)

// --- Descriptors -------------------------------------------------------------

// HID 1.11 Appendix B.1 keyboard, report id 0: modifier byte, reserved byte, six
// key bytes (8-byte report).
const uint8_t kBootKeyboard[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x05, 0x07,        //   Usage Page (Keyboard/Keypad)
    0x19, 0xE0,        //   Usage Minimum (0xE0)
    0x29, 0xE7,        //   Usage Maximum (0xE7)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data, Variable, Absolute) -> modifiers
    0x95, 0x01,        //   Report Count (1)
    0x75, 0x08,        //   Report Size (8)
    0x81, 0x03,        //   Input (Constant)                  -> reserved byte
    0x05, 0x07,        //   Usage Page (Keyboard/Keypad)
    0x19, 0x00,        //   Usage Minimum (0)
    0x29, 0x65,        //   Usage Maximum (0x65)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x65,        //   Logical Maximum (0x65)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x06,        //   Report Count (6)
    0x81, 0x00,        //   Input (Data, Array, Absolute)     -> six key bytes
    0xC0,              // End Collection
};

// Two report ids behind one collection: id 1 is the keyboard above, id 2 is a
// 16-bit Consumer Control array - a different page, size and byte offset.
const uint8_t kTwoReports[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01,                    // Generic Desktop / Keyboard / Collection
    0x85, 0x01,                                            // Report ID (1)
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,                    //   Keyboard page, usages 0xE0..0xE7
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08,        //   logical 0..1, size 1, count 8
    0x81, 0x02,                                            //   Input (Data, Variable) -> modifiers
    0x95, 0x01, 0x75, 0x08, 0x81, 0x03,                    //   reserved byte
    0x19, 0x00, 0x29, 0x65, 0x75, 0x08, 0x95, 0x06,        //   usages 0..0x65, size 8, count 6
    0x81, 0x00,                                            //   Input (Data, Array)    -> six key bytes
    0x85, 0x02,                                            // Report ID (2)
    0x05, 0x0C, 0x09, 0x01,                                //   Consumer page, Usage (Consumer Control)
    0x15, 0x00, 0x26, 0xFF, 0x03,                          //   Logical Minimum 0, Maximum 0x03FF
    0x19, 0x00, 0x2A, 0xFF, 0x03,                          //   Usage Minimum 0, Maximum 0x03FF
    0x75, 0x10, 0x95, 0x01,                                //   Report Size 16, Report Count 1
    0x81, 0x00,                                            //   Input (Data, Array, Absolute)
    0xC0,                                                  // End Collection
};

// Same keyboard layout, but the eight modifier usages are an explicit Usage list
// instead of Usage Minimum/Maximum (both spellings are common in the wild).
const uint8_t kListedModifiers[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01,
    0x05, 0x07,
    0x09, 0xE0, 0x09, 0xE1, 0x09, 0xE2, 0x09, 0xE3,  // Usage (LeftCtrl ..)
    0x09, 0xE4, 0x09, 0xE5, 0x09, 0xE6, 0x09, 0xE7,  // Usage (.. RightGUI)
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08,
    0x81, 0x02,                                      // Input (Data, Variable) -> modifiers
    0x95, 0x01, 0x75, 0x08, 0x81, 0x03,              // reserved byte
    0x19, 0x00, 0x29, 0x65, 0x75, 0x08, 0x95, 0x06,
    0x81, 0x00,                                      // Input (Data, Array)    -> six key bytes
    0xC0,
};

// A consumer byte in front of the modifier byte: the modifier mask is at byte 1,
// not at byte 0, so a byte-0 assumption would decode it as a key.
const uint8_t kMixedPages[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01,              // Generic Desktop / Keyboard / Collection
    0x05, 0x0C, 0x09, 0x01,                          //   Consumer page, Usage (Consumer Control)
    0x75, 0x08, 0x95, 0x01, 0x81, 0x02,              //   Input (Data, Variable) -> byte 0
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7,              //   Keyboard page, usages 0xE0..0xE7
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02,              //   Input (Data, Variable) -> byte 1
    0x19, 0x00, 0x29, 0x65, 0x75, 0x08, 0x95, 0x06,
    0x81, 0x00,                                      //   Input (Data, Array)    -> bytes 2..7
    0xC0,
};

// A keyboard whose last Input asks for 255 x 8 bits - past the modelled payload.
// The earlier fields stay parsed; the oversized item is dropped, not written.
const uint8_t kBootThenOverflow[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x75, 0x01, 0x95,
    0x08, 0x81, 0x02, 0x95, 0x01, 0x75, 0x08, 0x81, 0x03, 0x19, 0x00, 0x29, 0x65, 0x75, 0x08,
    0x95, 0x06, 0x81, 0x00, 0x19, 0x00, 0x29, 0x65, 0x75, 0x08, 0x95, 0xFF, 0x81, 0x00, 0xC0,
};

// --- Malformed descriptors ---------------------------------------------------

const uint8_t kNoInput[] = {0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0xC0};

const uint8_t kReportSizeZero[] = {0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x75, 0x00, 0x95, 0x06, 0x81, 0x00};

const uint8_t kReportCountZero[] = {0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x75, 0x08, 0x95, 0x00, 0x81, 0x00};

// 255 elements x 8 bits = 2040 bits, past kHidMaxReportBits.
const uint8_t kReportCountHuge[] = {0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x75, 0x08, 0x95, 0xFF, 0x81, 0x00};

// A two-byte Report Count of 256, past kHidMaxFieldBits.
const uint8_t kReportCount256[] = {0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x75, 0x08, 0x96, 0x00, 0x01, 0x81, 0x00};

// Cut in the middle of an item: "Report Size" with its data byte missing.
const uint8_t kCutItem[] = {0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x75};

// Cut in the middle of the Input item itself.
const uint8_t kCutInput[] = {0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x75, 0x01, 0x95, 0x08, 0x81};

// Long item whose data field runs past the end of the descriptor.
const uint8_t kCutLongItem[] = {0xFE, 0x04};

// Nothing but padding: bits are consumed, no field to decode.
const uint8_t kConstantOnly[] = {0x05, 0x07, 0x75, 0x08, 0x95, 0x01, 0x81, 0x03};

// --- Cases -------------------------------------------------------------------

void testBootKeyboard() {
  std::printf("[boot keyboard, no report id]\n");
  HidReportMap map;
  CHECK(parseHidReportMap(kBootKeyboard, sizeof kBootKeyboard, map));
  CHECK(map.usable);
  CHECK(!map.truncated);
  CHECK(!map.hasId);
  CHECK(map.hasKeyboardPage);
  CHECK_EQ(map.reportCount, 1);
  CHECK_EQ(map.reports[0].id, 0);
  CHECK_EQ(map.reports[0].bits, 64);
  CHECK_EQ(map.reports[0].keyFieldCount, 1);
  CHECK_EQ(map.reports[0].modFieldCount, 1);

  // Modifier mask in byte 0, six-key array from byte 2 - the mapping the caller
  // needs, byte for byte.
  const HidByteLayout bytes = hidByteLayout(map.reports[0]);
  CHECK_EQ(bytes.modByte, 0);
  CHECK_EQ(bytes.modBytes, 1);
  CHECK_EQ(bytes.keyByte, 2);
  CHECK_EQ(bytes.keyBytes, 6);
  CHECK_EQ(bytes.keyBits, 8);
  CHECK_EQ(map.preferredByteIndex, 2);

  // [mods][reserved][k0..k5] with Shift down: 'a' becomes 'A'.
  const uint8_t report[8] = {HID_LSHIFT, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00};
  HidReportView view;
  CHECK(decodeHidReport(map, report, sizeof report, view));
  CHECK_EQ(view.id, 0);
  CHECK_EQ(view.idFromReference, 0);
  CHECK_EQ(view.mods, HID_LSHIFT);
  CHECK_EQ(view.keyCount, 1);
  CHECK_EQ(view.keys[0], 0x04);

  char ch = 0;
  SpecialKey special = SpecialKey::None;
  CHECK(hidTranslate(view.keys[0], view.mods, ch, special));
  CHECK_EQ(ch, 'A');
  CHECK(special == SpecialKey::None);

  // Six keys at once keep report order; the reserved byte is not a key.
  const uint8_t six[8] = {0x00, 0x00, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
  CHECK(decodeHidReport(map, six, sizeof six, view));
  CHECK_EQ(view.mods, 0);
  CHECK_EQ(view.keyCount, 6);
  CHECK_EQ(view.keys[0], 0x04);
  CHECK_EQ(view.keys[5], 0x09);

  // Release frame.
  const uint8_t release[8] = {0};
  CHECK(decodeHidReport(map, release, sizeof release, view));
  CHECK_EQ(view.keyCount, 0);
  CHECK_EQ(view.mods, 0);

  // ErrorRollOver (0x01) is not a press, and a usage listed twice is one key.
  const uint8_t bad[8] = {0x00, 0x00, 0x01, 0x04, 0x04, 0x00, 0x00, 0x00};
  CHECK(decodeHidReport(map, bad, sizeof bad, view));
  CHECK_EQ(view.keyCount, 1);
  CHECK_EQ(view.keys[0], 0x04);

  // Remotes that prefix a 0x00 id byte the descriptor never declared.
  const uint8_t legacy[9] = {0x00, HID_LSHIFT, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK(decodeHidReport(map, legacy, sizeof legacy, view));
  CHECK_EQ(view.mods, HID_LSHIFT);
  CHECK_EQ(view.keys[0], 0x04);

  // Payload shorter than the layout it claims: refuse rather than re-interpret.
  const uint8_t shortReport[4] = {0x02, 0x00, 0x04, 0x00};
  CHECK(!decodeHidReport(map, shortReport, sizeof shortReport, view));
  CHECK(!decodeHidReport(map, nullptr, 0, view));
}

void testReportIdSelection() {
  std::printf("[report id selection]\n");
  HidReportMap map;
  CHECK(parseHidReportMap(kTwoReports, sizeof kTwoReports, map));
  CHECK(map.usable);
  CHECK(!map.truncated);
  CHECK(map.hasId);
  CHECK_EQ(map.reportCount, 2);
  CHECK_EQ(map.reports[0].id, 1);
  CHECK_EQ(map.reports[1].id, 2);
  CHECK_EQ(map.reports[0].bits, 64);
  CHECK_EQ(map.reports[1].bits, 16);

  const HidByteLayout kb = hidByteLayout(map.reports[0]);
  CHECK_EQ(kb.modByte, 0);
  CHECK_EQ(kb.keyByte, 2);
  CHECK_EQ(kb.keyBytes, 6);
  const HidByteLayout consumer = hidByteLayout(map.reports[1]);
  CHECK_EQ(consumer.modByte, 0xFF);  // this report has no modifier field
  CHECK_EQ(consumer.modBytes, 0);
  CHECK_EQ(consumer.keyByte, 0);
  CHECK_EQ(consumer.keyBits, 16);
  CHECK_EQ(consumer.keyBytes, 2);

  HidReportView view;
  // Id byte 1 selects the keyboard layout.
  const uint8_t report1[9] = {1, HID_LCTRL, 0x00, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK(decodeHidReport(map, report1, sizeof report1, view));
  CHECK_EQ(view.id, 1);
  CHECK_EQ(view.idFromReference, 0);
  CHECK_EQ(view.mods, HID_LCTRL);
  CHECK_EQ(view.keyCount, 1);
  CHECK_EQ(view.keys[0], 0x26);

  // Id byte 2 selects the 16-bit consumer array (offset 0, not the key array).
  const uint8_t report2[3] = {2, 0xCD, 0x00};
  CHECK(decodeHidReport(map, report2, sizeof report2, view));
  CHECK_EQ(view.id, 2);
  CHECK_EQ(view.mods, 0);
  CHECK_EQ(view.keyCount, 1);
  CHECK_EQ(view.keys[0], 0xCD);

  // Unknown id with no Report Reference to fall back on.
  const uint8_t report3[3] = {3, 0x26, 0x00};
  CHECK(!decodeHidReport(map, report3, sizeof report3, view));

  // No id byte in the payload, but the characteristic's Report Reference names
  // the layout: id 2 is still resolved.
  const uint8_t noIdByte[2] = {0xCD, 0x00};
  CHECK(decodeHidReport(map, noIdByte, sizeof noIdByte, view, 2));
  CHECK_EQ(view.id, 2);
  CHECK_EQ(view.idFromReference, 1);
  CHECK_EQ(view.keys[0], 0xCD);

  // Report 2 needs 16 payload bits; a one-byte payload is refused.
  const uint8_t oneByte[1] = {0xCD};
  CHECK(!decodeHidReport(map, oneByte, sizeof oneByte, view, 2));

  // A zero-initialized map decodes nothing at all.
  HidReportMap empty{};
  CHECK(!decodeHidReport(empty, report1, sizeof report1, view));
}

void testModifierBits() {
  std::printf("[modifier bit mapping]\n");
  HidReportMap map;
  CHECK(parseHidReportMap(kBootKeyboard, sizeof kBootKeyboard, map));
  HidReportView view;

  // Bit i of the mask is usage 0xE0 + i: bit 1 -> 0xE1 LeftShift, bit 7 -> 0xE7
  // RightGUI.
  const uint8_t lshift[8] = {HID_LSHIFT, 0, 0, 0, 0, 0, 0, 0};
  CHECK(decodeHidReport(map, lshift, sizeof lshift, view));
  CHECK_EQ(view.mods, HID_LSHIFT);
  CHECK_EQ(view.keyCount, 0);  // a modifier is reported through mods, not keys

  const uint8_t rgui[8] = {HID_RGUI, 0, 0, 0, 0, 0, 0, 0};
  CHECK(decodeHidReport(map, rgui, sizeof rgui, view));
  CHECK_EQ(view.mods, HID_RGUI);
  CHECK_EQ(view.keyCount, 0);

  // Two modifiers at once survive the round trip unchanged.
  const uint8_t both[8] = {HID_LCTRL | HID_RALT, 0, 0, 0, 0, 0, 0, 0};
  CHECK(decodeHidReport(map, both, sizeof both, view));
  CHECK_EQ(view.mods, HID_LCTRL | HID_RALT);

  // The same eight usages declared as an explicit Usage list map to the same bits.
  HidReportMap listed;
  CHECK(parseHidReportMap(kListedModifiers, sizeof kListedModifiers, listed));
  CHECK(listed.usable);
  CHECK(!listed.truncated);
  const HidByteLayout listedBytes = hidByteLayout(listed.reports[0]);
  CHECK_EQ(listedBytes.modByte, 0);
  CHECK_EQ(listedBytes.keyByte, 2);

  const uint8_t ralt[8] = {HID_RALT, 0, 0x04, 0, 0, 0, 0, 0};
  CHECK(decodeHidReport(listed, ralt, sizeof ralt, view));
  CHECK_EQ(view.mods, HID_RALT);  // bit 6 -> 0xE6
  CHECK_EQ(view.keyCount, 1);
  CHECK_EQ(view.keys[0], 0x04);

  // Modifiers behind another field: the mask sits at byte 1, and byte 0 (the
  // consumer field) is not mistaken for a key.
  HidReportMap mixed;
  CHECK(parseHidReportMap(kMixedPages, sizeof kMixedPages, mixed));
  CHECK(mixed.usable);
  CHECK_EQ(mixed.reportCount, 1);
  const HidByteLayout mixedBytes = hidByteLayout(mixed.reports[0]);
  CHECK_EQ(mixedBytes.modByte, 1);
  CHECK_EQ(mixedBytes.keyByte, 0);  // first key field is the consumer byte
  CHECK_EQ(mixed.reports[0].keyFieldCount, 2);

  const uint8_t shifted[8] = {0x00, HID_RSHIFT, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00};
  CHECK(decodeHidReport(mixed, shifted, sizeof shifted, view));
  CHECK_EQ(view.mods, HID_RSHIFT);
  CHECK_EQ(view.keyCount, 1);
  CHECK_EQ(view.keys[0], 0x26);

  char ch = 0;
  SpecialKey special = SpecialKey::None;
  CHECK(hidTranslate(view.keys[0], view.mods, ch, special));
  CHECK_EQ(ch, '(');  // Shift + '9'
}

void testMalformedDescriptors() {
  std::printf("[malformed / truncated descriptors]\n");
  HidReportMap map;
  uint8_t tooLong[kHidMaxDescriptorBytes + 1] = {0x05, 0x01};
  HidReportView view;
  const uint8_t report[8] = {0};

  // Null, empty and over-long descriptors.
  CHECK(!parseHidReportMap(nullptr, 12, map));
  CHECK(!parseHidReportMap(kBootKeyboard, 0, map));
  CHECK(!parseHidReportMap(tooLong, sizeof tooLong, map));
  CHECK(!map.usable);

  // No Input item at all.
  CHECK(!parseHidReportMap(kNoInput, sizeof kNoInput, map));
  CHECK(!map.usable);
  CHECK_EQ(map.reportCount, 0);

  // Report Size 0 / Report Count 0: nothing can be placed.
  CHECK(!parseHidReportMap(kReportSizeZero, sizeof kReportSizeZero, map));
  CHECK(!map.usable);
  CHECK(map.truncated);

  CHECK(!parseHidReportMap(kReportCountZero, sizeof kReportCountZero, map));
  CHECK(!map.usable);

  // Counts past the ceilings: one byte (255) and two bytes (256).
  CHECK(!parseHidReportMap(kReportCountHuge, sizeof kReportCountHuge, map));
  CHECK(!map.usable);
  CHECK(map.truncated);

  CHECK(!parseHidReportMap(kReportCount256, sizeof kReportCount256, map));
  CHECK(!map.usable);

  // Cut mid-item: the parser must stop, not read past the buffer.
  CHECK(!parseHidReportMap(kCutItem, sizeof kCutItem, map));
  CHECK(!map.usable);
  CHECK(!parseHidReportMap(kCutInput, sizeof kCutInput, map));
  CHECK(!map.usable);
  CHECK(!parseHidReportMap(kCutLongItem, sizeof kCutLongItem, map));
  CHECK(!map.usable);

  // An Input that is pure padding carries no usages to decode.
  CHECK(!parseHidReportMap(kConstantOnly, sizeof kConstantOnly, map));
  CHECK(!map.usable);

  // Whatever the rejection, no layout survives that could decode a report.
  CHECK(!decodeHidReport(map, report, sizeof report, view));

  // A layout with no fields has no byte view either.
  HidReportLayout none{};
  const HidByteLayout noneBytes = hidByteLayout(none);
  CHECK_EQ(noneBytes.modByte, 0xFF);
  CHECK_EQ(noneBytes.keyByte, 0xFF);
  CHECK_EQ(noneBytes.keyBits, 0);
}

void testCeilingStopsWithoutCorruption() {
  std::printf("[ceiling hit keeps earlier fields usable]\n");
  HidReportMap map;
  CHECK(parseHidReportMap(kBootThenOverflow, sizeof kBootThenOverflow, map));
  CHECK(map.usable);
  CHECK(map.truncated);  // the oversized Input was dropped
  CHECK_EQ(map.reportCount, 1);
  CHECK_EQ(map.reports[0].keyFieldCount, 1);
  CHECK_EQ(map.reports[0].modFieldCount, 1);
  CHECK_EQ(map.reports[0].bits, kHidMaxReportBits);  // cursor clamped, no wrap

  // The fields parsed before the oversized item still decode.
  const uint8_t report[8] = {HID_LCTRL, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
  HidReportView view;
  CHECK(decodeHidReport(map, report, sizeof report, view));
  CHECK_EQ(view.mods, HID_LCTRL);
  CHECK_EQ(view.keyCount, 1);
  CHECK_EQ(view.keys[0], 0x05);
}

}  // namespace

int main() {
  testBootKeyboard();
  testReportIdSelection();
  testModifierBits();
  testMalformedDescriptors();
  testCeilingStopsWithoutCorruption();

  std::printf("hidkeymap: %d checks, %d passed, %d failed\n", gChecks, gChecks - gFailures, gFailures);
  return gFailures == 0 ? 0 : 1;
}
