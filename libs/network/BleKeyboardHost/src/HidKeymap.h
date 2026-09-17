#pragma once

// HID keyboard usage -> character / SpecialKey translation (US QWERTY) and HID
// Report Map descriptor parsing. Pure logic, no NimBLE - compiled regardless of
// FREEINK_CAP_BLE_HID_HOST so it can be host-tested and reused. See the USB HID
// Usage Tables (page 0x07) and the HID 1.11 specification (section 6.2.2).

#include <stddef.h>
#include <stdint.h>

#include "BleKeyboardHost.h"

namespace freeink {

// HID keyboard modifier bitmask (byte 0 of a boot/report-protocol report).
enum HidMod : uint8_t {
  HID_LCTRL = 0x01,
  HID_LSHIFT = 0x02,
  HID_LALT = 0x04,
  HID_LGUI = 0x08,
  HID_RCTRL = 0x10,
  HID_RSHIFT = 0x20,
  HID_RALT = 0x40,
  HID_RGUI = 0x80,
};

inline bool hidShift(uint8_t mods) { return (mods & (HID_LSHIFT | HID_RSHIFT)) != 0; }
inline bool hidCtrl(uint8_t mods) { return (mods & (HID_LCTRL | HID_RCTRL)) != 0; }

// Translate a HID usage id (+ modifier byte) into a KeyEvent payload.
// Writes `ch` (printable ASCII, else 0) and `special`. Returns false for keys
// that map to nothing meaningful (modifier-only, F-keys, etc.).
bool hidTranslate(uint8_t usage, uint8_t mods, char& ch, SpecialKey& special);

// ---------------------------------------------------------------------------
// HID Report Map parsing (HID 1.11 section 6.2.2).
//
// Where a report keeps its modifier bits and its key usages is decided by the
// Report Map descriptor, not by a byte pattern: a boot keyboard puts a modifier
// byte at offset 0 and its key array at offset 2, a consumer page-turner puts a
// 16-bit usage at offset 0, and a device that uses report ids carries several
// unrelated layouts behind the same characteristic. The descriptor is therefore
// parsed item by item into the fixed-capacity map below, and every incoming
// report is decoded through it.
//
// Pure logic: no NimBLE, no heap, fixed arrays sized from the ceilings below.
// Items that exceed a ceiling are skipped - never written out of bounds - and
// flagged through HidReportMap::truncated so the caller can fall back.
// ---------------------------------------------------------------------------

// Capacity ceilings. All parser storage is sized from these constants.
enum : uint8_t {
  kHidMaxReports = 4,             // report ids kept (id 0 = reports with no Report ID item)
  kHidMaxKeyFieldsPerReport = 8,  // Input fields that carry key/button usages
  kHidMaxModFieldsPerReport = 2,  // Input fields that carry modifier usages
  kHidMaxKeysPerReport = 6,       // usages decoded from one report (a keyboard's 6 key slots)
  kHidMaxLocalUsages = 8,         // Usage items accepted before one main item
  kHidMaxGlobalStack = 4,         // Push/Pop nesting depth
};
enum : uint16_t {
  kHidMaxDescriptorBytes = 1024,  // longer Report Maps are rejected outright
  kHidMaxReportBits = 512,        // report payload modelled (64 bytes)
  kHidMaxFieldBits = 255,         // Report Size / Report Count ceiling
};
// Worst-case RAM of one parsed map; asserted in HidKeymap.cpp.
constexpr size_t kHidReportMapMaxBytes = 384;

// Input item flag bits we care about (HID 1.11 section 6.2.2.5).
enum HidInputFlags : uint8_t {
  kHidInputConstant = 0x01,  // padding: consumes report bits, carries no field
  kHidInputVariable = 0x02,  // per-bit fields (vs. an array of usages)
};

// How to turn an element value back into a HID usage.
enum HidFieldFlags : uint8_t {
  kHidFieldArray = 0x01,          // Input data is an array: usage = usageMin + value
  kHidFieldHasUsageMin = 0x02,    // a Usage Minimum (or a contiguous Usage list) was declared
  kHidFieldExplicitUsages = 0x04, // the field came from an explicit list of Usage items
};

// One Input field of the report map: where it sits in the payload (`bitOffset`
// bits from the start of the report, `count` elements of `bitSize` bits each)
// and how an element maps back to a usage.
struct HidField {
  uint16_t usageMin;   // Usage Minimum, or the first usage of a contiguous list
  uint16_t bitOffset;  // bit offset inside the report payload
  uint8_t bitSize;     // bits per element
  uint8_t count;       // elements
  uint8_t page;        // Usage Page the field was declared under (0x07, 0x0C, ...)
  uint8_t flags;       // HidFieldFlags
};

// One report of the map: the layout that belongs to a single report id (or to
// the un-numbered stream when the descriptor declares no Report ID item).
struct HidReportLayout {
  uint8_t id;      // report id this layout belongs to
  uint16_t bits;   // payload size the descriptor declares, in bits
  uint8_t keyFieldCount;
  uint8_t modFieldCount;
  HidField keyFields[kHidMaxKeyFieldsPerReport];
  HidField modFields[kHidMaxModFieldsPerReport];
};

// A parsed Report Map. Value-initialize before parsing (aggregate value-init).
struct HidReportMap {
  uint8_t reportCount;         // layouts stored in `reports`
  HidReportLayout reports[kHidMaxReports];
  bool usable;                 // at least one Input field was mapped
  bool truncated;              // a ceiling was hit; the offending item was skipped
  bool hasId;                  // the descriptor uses Report ID items (payloads are id-prefixed)
  bool hasKeyboardPage;        // a Usage Page 0x07 (Keyboard/Keypad) item was seen
  bool hasConsumerPage;        // a Usage Page 0x0C (Consumer) item was seen
  // First mapped key byte, or 0xFF when the map has none. Default-initialized to
  // the sentinel - 0 would name the modifier byte of a map that has no keys yet,
  // and callers read this field to pick a byte out of an undecodable report.
  uint8_t preferredByteIndex = 0xFF;
};

// Byte-level view of one layout, for callers that reason in bytes (and for the
// host tests): where the modifier mask and the first key field sit in the report
// payload and how many bytes each spans. 0xFF/0 mark a field the layout does not
// have. The offsets come from the fields' bit offsets, so they are exact for
// byte-aligned fields and name the byte the field starts in otherwise; the exact
// bit position is always HidField::bitOffset.
struct HidByteLayout {
  uint8_t modByte;   // byte offset of the modifier mask
  uint8_t modBytes;  // bytes the modifier mask spans
  uint8_t keyByte;   // byte offset of the key array (or key bitmap)
  uint8_t keyBytes;  // bytes the first key field spans
  uint8_t keyBits;   // bits per key element (8 for a boot keyboard's key array)
};

// Derive the byte view of one report layout. Pure arithmetic over the parsed
// fields - no allocation, no descriptor access.
HidByteLayout hidByteLayout(const HidReportLayout& layout);

// Parse a HID Report Map descriptor into `out`. Returns false when the descriptor
// is NULL/empty, longer than kHidMaxDescriptorBytes, or yields no Input field we
// can decode (no Input item at all, Report Size/Count outside the ceilings, an
// Input on a page whose usages never reach KeyEvent, or nothing but padding). A
// false result always leaves `out.usable` false and no layout able to decode
// anything, so a caller that ignores the return value still cannot decode from a
// rejected map; the diagnostic flags are left set for logging.
//
// Items that exceed a ceiling, and items cut off by the end of the descriptor,
// are skipped without writing out of bounds and flagged through `out.truncated`;
// fields parsed before them keep their offsets and stay usable.
bool parseHidReportMap(const uint8_t* descriptor, size_t len, HidReportMap& out);

// Decoded view of one incoming report (fixed capacity, no heap). `keys` holds
// the resolved usages of the keys held in this report; `mods` is the modifier
// bitmask (HidMod bits).
struct HidReportView {
  uint8_t id;             // report id the layout was resolved for
  bool idFromReference;   // true when the characteristic's report id selected it, not an id byte
  uint8_t mods;
  uint8_t keyCount;
  uint8_t keys[kHidMaxKeysPerReport];
};

// Decode one incoming report through `map`. `referenceId` is the report id the
// notifying characteristic's Report Reference descriptor declared (0xFFFF when
// unknown); it is only consulted when the report itself carries no id the map
// knows. Returns false when the map has no usable layout for this report - the
// unknown report id, a payload shorter than its own layout, or an empty map -
// so the caller can decide on a last-resort fallback.
bool decodeHidReport(const HidReportMap& map, const uint8_t* data, size_t len, HidReportView& out,
                     uint16_t referenceId = 0xFFFF);

}  // namespace freeink
