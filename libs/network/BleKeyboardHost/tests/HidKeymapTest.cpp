// Host tests for the HID Report Map parser and the usage -> KeyEvent translation
// in src/HidKeymap.{h,cpp}.
//
// Pure logic: no NimBLE, no Arduino runtime beyond the one-line stub the public
// header needs for `byte`, no device. The central-role path (GATT discovery,
// notification decode, key ring) is covered by BleKeyboardHostIngestTest.cpp.
//
// Run from the repository root:
//   cmake -S test -B build/test && cmake --build build/test
//   ctest --test-dir build/test --output-on-failure -R HidKeymapTest
// or directly: build/test/.../HidKeymapTest

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "HidDescriptors.h"
#include "HidKeymap.h"

using namespace freeink;  // test-only: the parser lives in the SDK namespace
using hidtest::kBootKeyboard;
using hidtest::kBootThenOverflow;
using hidtest::kConstantOnly;
using hidtest::kCutInput;
using hidtest::kCutItem;
using hidtest::kCutLongItem;
using hidtest::kListedModifiers;
using hidtest::kLongItemOverrun;
using hidtest::kMixedPages;
using hidtest::kNoInput;
using hidtest::kReportCount256;
using hidtest::kReportCountHuge;
using hidtest::kReportCountZero;
using hidtest::kReportSizeZero;
using hidtest::kTwoReports;

namespace {

// --- Report map: a plain boot keyboard ---------------------------------------

TEST(HidKeymap, ParsesBootKeyboardAndDecodesReports) {
  HidReportMap map;
  ASSERT_TRUE(parseHidReportMap(kBootKeyboard, sizeof kBootKeyboard, map));
  EXPECT_TRUE(map.usable);
  EXPECT_FALSE(map.truncated);
  EXPECT_FALSE(map.hasId);
  EXPECT_TRUE(map.hasKeyboardPage);
  EXPECT_FALSE(map.hasConsumerPage);
  EXPECT_EQ(map.reportCount, 1);
  EXPECT_EQ(map.reports[0].id, 0);
  EXPECT_EQ(map.reports[0].bits, 64);
  EXPECT_EQ(map.reports[0].keyFieldCount, 1);
  EXPECT_EQ(map.reports[0].modFieldCount, 1);

  // Modifier mask in byte 0, six-key array from byte 2 - the mapping the caller
  // needs, byte for byte.
  const HidByteLayout bytes = hidByteLayout(map.reports[0]);
  EXPECT_EQ(bytes.modByte, 0);
  EXPECT_EQ(bytes.modBytes, 1);
  EXPECT_EQ(bytes.keyByte, 2);
  EXPECT_EQ(bytes.keyBytes, 6);
  EXPECT_EQ(bytes.keyBits, 8);
  EXPECT_EQ(map.preferredByteIndex, 2);

  // [mods][reserved][k0..k5] with Shift down: 'a' becomes 'A'.
  HidReportView view;
  const uint8_t report[8] = {HID_LSHIFT, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00};
  ASSERT_TRUE(decodeHidReport(map, report, sizeof report, view));
  EXPECT_EQ(view.id, 0);
  EXPECT_FALSE(view.idFromReference);
  EXPECT_EQ(view.mods, HID_LSHIFT);
  EXPECT_EQ(view.keyCount, 1);
  EXPECT_EQ(view.keys[0], 0x04);

  char ch = 0;
  SpecialKey special = SpecialKey::None;
  ASSERT_TRUE(hidTranslate(view.keys[0], view.mods, ch, special));
  EXPECT_EQ(ch, 'A');
  EXPECT_EQ(special, SpecialKey::None);

  // Six keys at once keep report order; the reserved byte is not a key.
  const uint8_t six[8] = {0x00, 0x00, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
  ASSERT_TRUE(decodeHidReport(map, six, sizeof six, view));
  EXPECT_EQ(view.mods, 0);
  EXPECT_EQ(view.keyCount, 6);
  EXPECT_EQ(view.keys[0], 0x04);
  EXPECT_EQ(view.keys[5], 0x09);

  // Release frame.
  const uint8_t release[8] = {0};
  ASSERT_TRUE(decodeHidReport(map, release, sizeof release, view));
  EXPECT_EQ(view.keyCount, 0);
  EXPECT_EQ(view.mods, 0);

  // ErrorRollOver (0x01) is not a press, and a usage listed twice is one key.
  const uint8_t bad[8] = {0x00, 0x00, 0x01, 0x04, 0x04, 0x00, 0x00, 0x00};
  ASSERT_TRUE(decodeHidReport(map, bad, sizeof bad, view));
  EXPECT_EQ(view.keyCount, 1);
  EXPECT_EQ(view.keys[0], 0x04);

  // Remotes that prefix a 0x00 id byte the descriptor never declared.
  const uint8_t legacy[9] = {0x00, HID_LSHIFT, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00};
  ASSERT_TRUE(decodeHidReport(map, legacy, sizeof legacy, view));
  EXPECT_EQ(view.mods, HID_LSHIFT);
  ASSERT_EQ(view.keyCount, 1);
  EXPECT_EQ(view.keys[0], 0x04);

  // Payload shorter than the layout it claims: refuse rather than re-interpret.
  const uint8_t shortReport[4] = {0x02, 0x00, 0x04, 0x00};
  EXPECT_FALSE(decodeHidReport(map, shortReport, sizeof shortReport, view));
  EXPECT_FALSE(decodeHidReport(map, nullptr, 0, view));
}

// --- Report ids and Report Reference selection -------------------------------

TEST(HidKeymap, SelectsLayoutByReportIdAndReference) {
  HidReportMap map;
  ASSERT_TRUE(parseHidReportMap(kTwoReports, sizeof kTwoReports, map));
  EXPECT_TRUE(map.usable);
  EXPECT_FALSE(map.truncated);
  EXPECT_TRUE(map.hasId);
  EXPECT_TRUE(map.hasKeyboardPage);
  EXPECT_TRUE(map.hasConsumerPage);
  ASSERT_EQ(map.reportCount, 2);
  EXPECT_EQ(map.reports[0].id, 1);
  EXPECT_EQ(map.reports[1].id, 2);
  EXPECT_EQ(map.reports[0].bits, 64);
  EXPECT_EQ(map.reports[1].bits, 16);

  const HidByteLayout kb = hidByteLayout(map.reports[0]);
  EXPECT_EQ(kb.modByte, 0);
  EXPECT_EQ(kb.keyByte, 2);
  EXPECT_EQ(kb.keyBytes, 6);
  EXPECT_EQ(kb.keyBits, 8);

  // The consumer layout: 16-bit elements at byte 0 and no modifier field.
  const HidByteLayout consumer = hidByteLayout(map.reports[1]);
  EXPECT_EQ(consumer.modByte, 0xFF);
  EXPECT_EQ(consumer.modBytes, 0);
  EXPECT_EQ(consumer.keyByte, 0);
  EXPECT_EQ(consumer.keyBits, 16);
  EXPECT_EQ(consumer.keyBytes, 2);

  HidReportView view;
  // Id byte 1 selects the keyboard layout.
  const uint8_t report1[9] = {1, HID_LCTRL, 0x00, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00};
  ASSERT_TRUE(decodeHidReport(map, report1, sizeof report1, view));
  EXPECT_EQ(view.id, 1);
  EXPECT_FALSE(view.idFromReference);
  EXPECT_EQ(view.mods, HID_LCTRL);
  EXPECT_EQ(view.keyCount, 1);
  EXPECT_EQ(view.keys[0], 0x26);

  // Id byte 2 selects the 16-bit consumer array (offset 0, not the key array).
  const uint8_t report2[3] = {2, 0xCD, 0x00};
  ASSERT_TRUE(decodeHidReport(map, report2, sizeof report2, view));
  EXPECT_EQ(view.id, 2);
  EXPECT_EQ(view.mods, 0);
  EXPECT_EQ(view.keyCount, 1);
  EXPECT_EQ(view.keys[0], 0xCD);

  // Unknown id with no Report Reference to fall back on.
  const uint8_t report3[3] = {3, 0x26, 0x00};
  EXPECT_FALSE(decodeHidReport(map, report3, sizeof report3, view));

  // No id byte in the payload, but the characteristic's Report Reference (0x2908)
  // names the layout: id 2 is still resolved.
  const uint8_t noIdByte[2] = {0xCD, 0x00};
  ASSERT_TRUE(decodeHidReport(map, noIdByte, sizeof noIdByte, view, 2));
  EXPECT_EQ(view.id, 2);
  EXPECT_TRUE(view.idFromReference);
  EXPECT_EQ(view.keys[0], 0xCD);

  // Report 2 needs 16 payload bits; a one-byte payload is refused.
  const uint8_t oneByte[1] = {0xCD};
  EXPECT_FALSE(decodeHidReport(map, oneByte, sizeof oneByte, view, 2));

  // A zero-initialized map decodes nothing at all.
  HidReportMap empty{};
  EXPECT_FALSE(decodeHidReport(empty, report1, sizeof report1, view));
}

// --- Modifier bit mapping ----------------------------------------------------

TEST(HidKeymap, MapsModifierBitsWhereverTheDescriptorPutsThem) {
  HidReportMap map;
  ASSERT_TRUE(parseHidReportMap(kBootKeyboard, sizeof kBootKeyboard, map));
  HidReportView view;

  // Bit i of the mask is usage 0xE0 + i: bit 1 -> 0xE1 LeftShift, bit 7 ->
  // 0xE7 RightGUI.
  const uint8_t lshift[8] = {HID_LSHIFT, 0, 0, 0, 0, 0, 0, 0};
  ASSERT_TRUE(decodeHidReport(map, lshift, sizeof lshift, view));
  EXPECT_EQ(view.mods, HID_LSHIFT);
  EXPECT_EQ(view.keyCount, 0);  // a modifier is reported through mods, not keys

  const uint8_t rgui[8] = {HID_RGUI, 0, 0, 0, 0, 0, 0, 0};
  ASSERT_TRUE(decodeHidReport(map, rgui, sizeof rgui, view));
  EXPECT_EQ(view.mods, HID_RGUI);
  EXPECT_EQ(view.keyCount, 0);

  // Two modifiers at once survive the round trip unchanged.
  const uint8_t both[8] = {HID_LCTRL | HID_RALT, 0, 0, 0, 0, 0, 0, 0};
  ASSERT_TRUE(decodeHidReport(map, both, sizeof both, view));
  EXPECT_EQ(view.mods, HID_LCTRL | HID_RALT);

  // The same eight usages declared as an explicit Usage list map to the same bits.
  HidReportMap listed;
  ASSERT_TRUE(parseHidReportMap(kListedModifiers, sizeof kListedModifiers, listed));
  EXPECT_TRUE(listed.usable);
  EXPECT_FALSE(listed.truncated);
  const HidByteLayout listedBytes = hidByteLayout(listed.reports[0]);
  EXPECT_EQ(listedBytes.modByte, 0);
  EXPECT_EQ(listedBytes.keyByte, 2);

  const uint8_t ralt[8] = {HID_RALT, 0, 0x04, 0, 0, 0, 0, 0};
  ASSERT_TRUE(decodeHidReport(listed, ralt, sizeof ralt, view));
  EXPECT_EQ(view.mods, HID_RALT);  // bit 6 -> 0xE6
  ASSERT_EQ(view.keyCount, 1);
  EXPECT_EQ(view.keys[0], 0x04);

  // Modifiers behind another field: the mask sits at byte 1, and byte 0 (the
  // consumer field) is not mistaken for a key.
  HidReportMap mixed;
  ASSERT_TRUE(parseHidReportMap(kMixedPages, sizeof kMixedPages, mixed));
  EXPECT_TRUE(mixed.usable);
  ASSERT_EQ(mixed.reportCount, 1);
  const HidByteLayout mixedBytes = hidByteLayout(mixed.reports[0]);
  EXPECT_EQ(mixedBytes.modByte, 1);
  EXPECT_EQ(mixedBytes.keyByte, 0);  // first key field is the consumer byte
  EXPECT_EQ(mixed.reports[0].keyFieldCount, 2);

  const uint8_t shifted[8] = {0x00, HID_RSHIFT, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00};
  ASSERT_TRUE(decodeHidReport(mixed, shifted, sizeof shifted, view));
  EXPECT_EQ(view.mods, HID_RSHIFT);
  ASSERT_EQ(view.keyCount, 1);
  EXPECT_EQ(view.keys[0], 0x26);

  char ch = 0;
  SpecialKey special = SpecialKey::None;
  ASSERT_TRUE(hidTranslate(view.keys[0], view.mods, ch, special));
  EXPECT_EQ(ch, '(');  // Shift + '9'
}

// --- Usage -> character / special key translation ----------------------------

TEST(HidKeymap, TranslatesUsagesToCharactersAndSpecialKeys) {
  char ch = 0;
  SpecialKey special = SpecialKey::None;

  ASSERT_TRUE(hidTranslate(0x04, 0, ch, special));
  EXPECT_EQ(ch, 'a');
  EXPECT_EQ(special, SpecialKey::None);

  ASSERT_TRUE(hidTranslate(0x04, HID_LSHIFT, ch, special));
  EXPECT_EQ(ch, 'A');

  ASSERT_TRUE(hidTranslate(0x1E, HID_RSHIFT, ch, special));  // '1' + Shift
  EXPECT_EQ(ch, '!');

  ASSERT_TRUE(hidTranslate(0x2C, 0, ch, special));  // Space
  EXPECT_EQ(ch, ' ');
  EXPECT_EQ(special, SpecialKey::None);

  struct SpecialCase {
    uint8_t usage;
    SpecialKey expected;
  };
  const SpecialCase specials[] = {
      {0x28, SpecialKey::Enter},     {0x58, SpecialKey::Enter},  // Keypad Enter
      {0x29, SpecialKey::Escape},    {0x2A, SpecialKey::Backspace},
      {0x2B, SpecialKey::Tab},       {0x4C, SpecialKey::Delete},
      {0x4A, SpecialKey::Home},      {0x4B, SpecialKey::PageUp},
      {0x4D, SpecialKey::End},       {0x4E, SpecialKey::PageDown},
      {0x4F, SpecialKey::Right},     {0x50, SpecialKey::Left},
      {0x51, SpecialKey::Down},      {0x52, SpecialKey::Up},
  };
  for (const SpecialCase& c : specials) {
    ASSERT_TRUE(hidTranslate(c.usage, 0, ch, special)) << "usage " << static_cast<int>(c.usage);
    EXPECT_EQ(ch, 0) << "usage " << static_cast<int>(c.usage);
    EXPECT_EQ(special, c.expected) << "usage " << static_cast<int>(c.usage);
  }

  // Usages that map to nothing meaningful: a modifier, an F-key, a page-turner
  // consumer code. They still come back with ch == 0 / special == None so the
  // caller can surface the raw keycode.
  EXPECT_FALSE(hidTranslate(0xE1, 0, ch, special));
  EXPECT_EQ(ch, 0);
  EXPECT_EQ(special, SpecialKey::None);
  EXPECT_FALSE(hidTranslate(0x3A, 0, ch, special));  // F1
  EXPECT_FALSE(hidTranslate(0xCD, 0, ch, special));  // consumer page code
  EXPECT_EQ(ch, 0);
  EXPECT_FALSE(hidTranslate(0x00, 0, ch, special));
}

// --- Malformed / truncated descriptors ---------------------------------------

TEST(HidKeymap, RejectsMalformedDescriptorsSafely) {
  HidReportMap map;
  HidReportView view;
  const uint8_t report[8] = {0};

  // Null and empty descriptors.
  EXPECT_FALSE(parseHidReportMap(nullptr, 12, map));
  EXPECT_FALSE(parseHidReportMap(kBootKeyboard, 0, map));
  EXPECT_FALSE(map.usable);

  // No Input item at all.
  EXPECT_FALSE(parseHidReportMap(kNoInput, sizeof kNoInput, map));
  EXPECT_FALSE(map.usable);
  EXPECT_EQ(map.reportCount, 0);

  // Report Size 0 / Report Count 0: nothing can be placed.
  EXPECT_FALSE(parseHidReportMap(kReportSizeZero, sizeof kReportSizeZero, map));
  EXPECT_FALSE(map.usable);
  EXPECT_TRUE(map.truncated);

  EXPECT_FALSE(parseHidReportMap(kReportCountZero, sizeof kReportCountZero, map));
  EXPECT_FALSE(map.usable);

  // Counts past the ceilings: one byte (255) and two bytes (256).
  EXPECT_FALSE(parseHidReportMap(kReportCountHuge, sizeof kReportCountHuge, map));
  EXPECT_FALSE(map.usable);
  EXPECT_TRUE(map.truncated);

  EXPECT_FALSE(parseHidReportMap(kReportCount256, sizeof kReportCount256, map));
  EXPECT_FALSE(map.usable);

  // Cut mid-item: the parser must stop, not read past the buffer.
  EXPECT_FALSE(parseHidReportMap(kCutItem, sizeof kCutItem, map));
  EXPECT_FALSE(map.usable);
  EXPECT_FALSE(parseHidReportMap(kCutInput, sizeof kCutInput, map));
  EXPECT_FALSE(map.usable);
  EXPECT_FALSE(parseHidReportMap(kCutLongItem, sizeof kCutLongItem, map));
  EXPECT_FALSE(map.usable);

  // A long item whose declared payload runs past the end of the descriptor.
  EXPECT_FALSE(parseHidReportMap(kLongItemOverrun, sizeof kLongItemOverrun, map));
  EXPECT_FALSE(map.usable);

  // An Input that is pure padding carries no usages to decode.
  EXPECT_FALSE(parseHidReportMap(kConstantOnly, sizeof kConstantOnly, map));
  EXPECT_FALSE(map.usable);

  // Whatever the rejection, no layout survives that could decode a report.
  EXPECT_FALSE(decodeHidReport(map, report, sizeof report, view));

  // A layout with no fields has no byte view either.
  HidReportLayout none{};
  const HidByteLayout noneBytes = hidByteLayout(none);
  EXPECT_EQ(noneBytes.modByte, 0xFF);
  EXPECT_EQ(noneBytes.keyByte, 0xFF);
  EXPECT_EQ(noneBytes.keyBits, 0);
}

TEST(HidKeymap, RejectsDescriptorsPastTheByteCeiling) {
  HidReportMap map;
  HidReportView view;

  // Exactly at the ceiling: accepted, and the fields parsed before the padding
  // keep their offsets. The filler is a run of Logical Minimum items, which the
  // parser ignores but must still walk item by item.
  std::vector<uint8_t> atCap(kBootKeyboard, kBootKeyboard + sizeof kBootKeyboard);
  while (atCap.size() + 2 <= kHidMaxDescriptorBytes) {
    atCap.push_back(0x15);
    atCap.push_back(0x00);
  }
  // A one-byte item (Logical Minimum with a zero-length data field) closes an odd
  // leftover byte, so the descriptor lands exactly on the ceiling.
  while (atCap.size() < static_cast<size_t>(kHidMaxDescriptorBytes)) atCap.push_back(0x14);
  ASSERT_EQ(atCap.size(), static_cast<size_t>(kHidMaxDescriptorBytes));
  ASSERT_TRUE(parseHidReportMap(atCap.data(), atCap.size(), map));
  EXPECT_TRUE(map.usable);
  EXPECT_EQ(map.reports[0].bits, 64);
  const HidByteLayout atCapBytes = hidByteLayout(map.reports[0]);
  EXPECT_EQ(atCapBytes.modByte, 0);
  EXPECT_EQ(atCapBytes.keyByte, 2);

  const uint8_t report[8] = {HID_LCTRL, 0x00, 0x05, 0, 0, 0, 0, 0};
  ASSERT_TRUE(decodeHidReport(map, report, sizeof report, view));
  EXPECT_EQ(view.mods, HID_LCTRL);
  ASSERT_EQ(view.keyCount, 1);
  EXPECT_EQ(view.keys[0], 0x05);

  // One byte past the ceiling: rejected outright, so a hostile or corrupt Report
  // Map cannot make the device walk (and trust) a huge descriptor.
  std::vector<uint8_t> overCap = atCap;
  overCap.push_back(0x15);
  overCap.push_back(0x00);
  EXPECT_FALSE(parseHidReportMap(overCap.data(), overCap.size(), map));
  EXPECT_FALSE(map.usable);
}

TEST(HidKeymap, CeilingHitKeepsEarlierFieldsUsable) {
  HidReportMap map;
  ASSERT_TRUE(parseHidReportMap(kBootThenOverflow, sizeof kBootThenOverflow, map));
  EXPECT_TRUE(map.usable);
  EXPECT_TRUE(map.truncated);  // the oversized Input was dropped
  EXPECT_EQ(map.reportCount, 1);
  EXPECT_EQ(map.reports[0].keyFieldCount, 1);
  EXPECT_EQ(map.reports[0].modFieldCount, 1);
  EXPECT_EQ(map.reports[0].bits, kHidMaxReportBits);  // cursor clamped, no wrap

  // The fields parsed before the oversized item still decode.
  const uint8_t report[8] = {HID_LCTRL, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
  HidReportView view;
  ASSERT_TRUE(decodeHidReport(map, report, sizeof report, view));
  EXPECT_EQ(view.mods, HID_LCTRL);
  ASSERT_EQ(view.keyCount, 1);
  EXPECT_EQ(view.keys[0], 0x05);
}

}  // namespace
