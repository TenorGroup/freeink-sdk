#include "HidKeymap.h"

namespace freeink {

namespace {

// Base (unshifted) ASCII for HID usages 0x00..0x38; 0 = not a printable key
// here (specials handled separately). 0x04='a' .. 0x1D='z', 0x1E='1'.. 0x27='0'.
constexpr char kBase[0x39] = {
    /*00*/ 0,   0,   0,   0,
    /*04*/ 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
    /*11*/ 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
    /*1E*/ '1', '2', '3', '4', '5', '6', '7', '8', '9', '0',
    /*28*/ 0,   0,   0,   0,   ' ',  // 0x2C = Space
    /*2D*/ '-', '=', '[', ']', '\\', 0 /*0x32 non-US #*/, ';', '\'', '`', ',', '.', '/',
};

// Shifted ASCII for the same range.
constexpr char kShift[0x39] = {
    /*00*/ 0,   0,   0,   0,
    /*04*/ 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M',
    /*11*/ 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
    /*1E*/ '!', '@', '#', '$', '%', '^', '&', '*', '(', ')',
    /*28*/ 0,   0,   0,   0,   ' ',
    /*2D*/ '_', '+', '{', '}', '|', 0, ':', '"', '~', '<', '>', '?',
};

}  // namespace

bool hidTranslate(uint8_t usage, uint8_t mods, char& ch, SpecialKey& special) {
  ch = 0;
  special = SpecialKey::None;

  switch (usage) {
    case 0x28:  // Enter
    case 0x58:  // Keypad Enter
      special = SpecialKey::Enter;
      return true;
    case 0x29:
      special = SpecialKey::Escape;
      return true;
    case 0x2A:
      special = SpecialKey::Backspace;
      return true;
    case 0x2B:
      special = SpecialKey::Tab;
      return true;
    case 0x4C:
      special = SpecialKey::Delete;
      return true;
    case 0x4A:
      special = SpecialKey::Home;
      return true;
    case 0x4B:
      special = SpecialKey::PageUp;
      return true;
    case 0x4D:
      special = SpecialKey::End;
      return true;
    case 0x4E:
      special = SpecialKey::PageDown;
      return true;
    case 0x4F:
      special = SpecialKey::Right;
      return true;
    case 0x50:
      special = SpecialKey::Left;
      return true;
    case 0x51:
      special = SpecialKey::Down;
      return true;
    case 0x52:
      special = SpecialKey::Up;
      return true;
    default:
      break;
  }

  if (usage < sizeof(kBase)) {
    const char c = hidShift(mods) ? kShift[usage] : kBase[usage];
    if (c != 0) {
      ch = c;
      return true;
    }
  }
  return false;
}

// --- HID Report Map parsing -------------------------------------------------

static_assert(sizeof(HidReportMap) <= kHidReportMapMaxBytes, "HidReportMap exceeds its RAM ceiling");

namespace {

// Item prefix: bTag(4) | bType(2) | bSize(2). bSize 0/1/2/3 means 0/1/2/4 data bytes.
constexpr uint8_t kTypeMain = 0x00;
constexpr uint8_t kTypeGlobal = 0x01;
constexpr uint8_t kTypeLocal = 0x02;

// Main item tags.
constexpr uint8_t kMainInput = 0x08;
constexpr uint8_t kMainCollection = 0x0A;
constexpr uint8_t kMainEndCollection = 0x0C;

// Global item tags.
constexpr uint8_t kGlobalUsagePage = 0x00;
constexpr uint8_t kGlobalReportSize = 0x07;
constexpr uint8_t kGlobalReportId = 0x08;
constexpr uint8_t kGlobalReportCount = 0x09;
constexpr uint8_t kGlobalPush = 0x0A;
constexpr uint8_t kGlobalPop = 0x0B;

// Local item tags.
constexpr uint8_t kLocalUsage = 0x00;
constexpr uint8_t kLocalUsageMin = 0x01;
constexpr uint8_t kLocalUsageMax = 0x02;

constexpr uint8_t kLongItemPrefix = 0xFE;
constexpr uint16_t kPageKeyboard = 0x07;
constexpr uint16_t kPageConsumer = 0x0C;
constexpr uint16_t kUsageErrorRollOver = 0x01;  // page 0x07 only
constexpr uint16_t kUsageModifierMin = 0xE0;
constexpr uint16_t kUsageModifierMax = 0xE7;

// Global items persist until changed; Local items are consumed by every main item.
struct GlobalState {
  uint16_t usagePage;
  uint8_t reportId;
  uint32_t reportSize;
  uint32_t reportCount;
};

struct LocalState {
  uint16_t usages[kHidMaxLocalUsages];
  uint8_t usageCount;
  uint16_t usageMin;
  uint16_t usageMax;
  bool hasUsageMin;
  bool hasUsageMax;
};

// The usages the pending local items describe. `declared` is false when the main
// item named no usage at all - then an element value is the usage id itself.
struct UsageRange {
  uint16_t lo;
  uint16_t hi;
  bool declared;
};

void clearLocals(LocalState& l) {
  l.usageCount = 0;
  l.usageMin = 0;
  l.usageMax = 0;
  l.hasUsageMin = false;
  l.hasUsageMax = false;
}

size_t itemDataBytes(uint8_t sizeCode) { return sizeCode == 3 ? 4 : sizeCode; }

uint32_t itemValue(const uint8_t* data, size_t nbytes) {
  uint32_t v = 0;
  for (size_t i = 0; i < nbytes; ++i) v |= static_cast<uint32_t>(data[i]) << (8 * i);
  return v;
}

HidReportLayout* layoutFor(HidReportMap& map, uint8_t id) {
  for (uint8_t i = 0; i < map.reportCount; ++i) {
    if (map.reports[i].id == id) return &map.reports[i];
  }
  return nullptr;
}

const HidReportLayout* layoutFor(const HidReportMap& map, uint8_t id) {
  for (uint8_t i = 0; i < map.reportCount; ++i) {
    if (map.reports[i].id == id) return &map.reports[i];
  }
  return nullptr;
}

UsageRange localUsageRange(const LocalState& l) {
  UsageRange r;
  r.lo = 0;
  r.hi = 0;
  r.declared = false;
  if (l.hasUsageMin) {
    r.lo = l.usageMin;
    r.hi = l.hasUsageMax ? l.usageMax : l.usageMin;
    r.declared = true;
  }
  for (uint8_t i = 0; i < l.usageCount; ++i) {
    if (!r.declared) {
      r.lo = l.usages[i];
      r.hi = l.usages[i];
      r.declared = true;
    } else {
      if (l.usages[i] < r.lo) r.lo = l.usages[i];
      if (l.usages[i] > r.hi) r.hi = l.usages[i];
    }
  }
  return r;
}

// An explicit Usage list only describes a field when the usages are contiguous:
// then bit/element i is `usages[0] + i` and one field covers the whole range.
bool usagesContiguous(const LocalState& l) {
  for (uint8_t i = 1; i < l.usageCount; ++i) {
    if (l.usages[i] != static_cast<uint16_t>(l.usages[0] + i)) return false;
  }
  return l.usageCount > 0;
}

void addKeyField(HidReportMap& map, HidReportLayout& layout, uint8_t page, uint16_t bitOffset, uint8_t bitSize,
                 uint8_t count, uint16_t usageMin, uint8_t flags) {
  if (bitSize == 0 || count == 0) {
    map.truncated = true;
    return;
  }
  if (layout.keyFieldCount >= kHidMaxKeyFieldsPerReport) {
    map.truncated = true;  // ceiling reached: dropped, never stored out of bounds
    return;
  }
  HidField& f = layout.keyFields[layout.keyFieldCount++];
  f.usageMin = usageMin;
  f.bitOffset = bitOffset;
  f.bitSize = bitSize;
  f.count = count;
  f.page = page;
  f.flags = flags;
  if (map.preferredByteIndex == 0xFF && (bitOffset & 0x07) == 0) {
    map.preferredByteIndex = static_cast<uint8_t>(bitOffset >> 3);
  }
  map.usable = true;
}

// Apply one Input main item: consume its bits in the current report and, when it
// carries Data, record where its usages live.
void recordInput(HidReportMap& map, const GlobalState& g, const LocalState& l, uint32_t itemData) {
  HidReportLayout* layout = layoutFor(map, g.reportId);
  if (layout == nullptr) return;  // no slot for this report id: parser stopped earlier

  // Malformed Report Size/Count: no bits to consume and nothing to record.
  if (g.reportSize == 0 || g.reportCount == 0 || g.reportSize > kHidMaxFieldBits ||
      g.reportCount > kHidMaxFieldBits) {
    map.truncated = true;
    return;
  }
  const uint32_t bits = g.reportSize * g.reportCount;
  const uint16_t start = layout->bits;
  if (static_cast<uint32_t>(start) + bits > kHidMaxReportBits) {
    // Field runs past the payload we model: skip it and clamp the cursor so every
    // later offset stays inside the bound instead of wrapping.
    map.truncated = true;
    layout->bits = kHidMaxReportBits;
    return;
  }
  layout->bits = static_cast<uint16_t>(start + bits);
  if ((itemData & kHidInputConstant) != 0) return;  // padding: bits consumed, no field

  const bool variable = (itemData & kHidInputVariable) != 0;
  const UsageRange range = localUsageRange(l);

  // Modifier bitfield: keyboard page, usages inside 0xE0..0xE7, one bit per usage
  // (the boot keyboard's `19 E0 29 E7 75 01 95 08 81 02`). Those bits line up with
  // HidMod, so the byte at this offset IS the modifier mask.
  if (g.usagePage == kPageKeyboard && range.declared && range.lo >= kUsageModifierMin &&
      range.hi <= kUsageModifierMax) {
    if (!variable) return;  // an array of modifier usages is not a bitmask: fallback
    if (layout->modFieldCount >= kHidMaxModFieldsPerReport) {
      map.truncated = true;
      return;
    }
    HidField& f = layout->modFields[layout->modFieldCount++];
    f.usageMin = range.lo;
    f.bitOffset = start;
    f.bitSize = static_cast<uint8_t>(bits > 8 ? 8 : bits);  // one byte holds every modifier
    f.count = 1;
    f.page = kPageKeyboard;
    f.flags = kHidFieldHasUsageMin;
    map.usable = true;
    return;
  }

  // Only the pages whose usages reach KeyEvent are worth modelling; a vendor page
  // or a generic-desktop axis is left to the caller's fallback.
  if (g.usagePage != kPageKeyboard && g.usagePage != kPageConsumer) return;

  uint16_t usageMin = 0;
  uint8_t count = static_cast<uint8_t>(g.reportCount);
  uint8_t flags = variable ? 0 : kHidFieldArray;
  const bool explicitList = !l.hasUsageMin && !l.hasUsageMax && l.usageCount > 0;

  if (l.hasUsageMin) {
    usageMin = l.usageMin;
    flags |= kHidFieldHasUsageMin;
  } else if (explicitList && usagesContiguous(l)) {
    usageMin = l.usages[0];
    flags |= kHidFieldHasUsageMin | kHidFieldExplicitUsages;
    if (count > l.usageCount) count = l.usageCount;  // the list names no more than this
  } else if (explicitList) {
    if (variable && g.reportSize == 1) {
      // Per-bit field with a non-contiguous usage list (media-key bitmaps): no base
      // value describes it, so one single-bit field per usage.
      for (uint8_t i = 0; i < l.usageCount; ++i) {
        addKeyField(map, *layout, static_cast<uint8_t>(g.usagePage), static_cast<uint16_t>(start + i), 1, 1,
                    l.usages[i], kHidFieldHasUsageMin | kHidFieldExplicitUsages);
      }
      return;
    }
    return;  // array indexed into a list, or a multi-bit variable: fallback
  }

  addKeyField(map, *layout, static_cast<uint8_t>(g.usagePage), start, static_cast<uint8_t>(g.reportSize), count,
              usageMin, flags);
}

uint32_t readReportBits(const uint8_t* payload, size_t payloadBytes, uint16_t bitOffset, uint8_t bitCount) {
  uint32_t v = 0;
  for (uint8_t i = 0; i < bitCount; ++i) {
    const uint16_t bit = static_cast<uint16_t>(bitOffset + i);
    if (static_cast<size_t>(bit) >= payloadBytes * 8) return v;  // coverage is checked first; stay bounded
    if ((payload[bit >> 3] & (1u << (bit & 7))) != 0) v |= (1u << i);
  }
  return v;
}

bool fieldFits(const HidField& f, uint32_t payloadBits) {
  if (f.bitSize == 0 || f.count == 0) return false;
  return static_cast<uint32_t>(f.bitOffset) + static_cast<uint32_t>(f.bitSize) * f.count <= payloadBits;
}

void addDecodedKey(HidReportView& view, const HidField& f, uint32_t usage) {
  if (usage == 0) return;
  // Page 0x07 reserves 0x01 for ErrorRollOver and 0xE0..0xE7 for the modifiers,
  // which are already reported through `mods`; neither is a key press.
  if (f.page == kPageKeyboard &&
      (usage == kUsageErrorRollOver || (usage >= kUsageModifierMin && usage <= kUsageModifierMax))) {
    return;
  }
  if (view.keyCount >= kHidMaxKeysPerReport) return;  // ceiling: extra keys dropped
  for (uint8_t i = 0; i < view.keyCount; ++i) {
    if (view.keys[i] == static_cast<uint8_t>(usage)) return;  // same usage twice in one report
  }
  // KeyEvent::keycode is 8-bit, so usages above 0xFF keep their low byte (the app
  // binds codes, and a consumer usage such as 0x0223 has no 16-bit home in the
  // event). Nothing else is truncated.
  view.keys[view.keyCount++] = static_cast<uint8_t>(usage);
}

}  // namespace

bool parseHidReportMap(const uint8_t* descriptor, size_t len, HidReportMap& out) {
  out = HidReportMap();
  if (descriptor == nullptr || len == 0 || len > static_cast<size_t>(kHidMaxDescriptorBytes)) return false;

  GlobalState g;
  g.usagePage = 0;
  g.reportId = 0;
  g.reportSize = 0;
  g.reportCount = 0;
  GlobalState stack[kHidMaxGlobalStack];
  uint8_t stackDepth = 0;
  LocalState l;
  clearLocals(l);

  size_t i = 0;
  while (i < len) {
    const uint8_t prefix = descriptor[i++];
    if (prefix == kLongItemPrefix) {
      // Long item: bDataSize, bLongItemTag, data - none of it is Input/Usage state.
      if (i + 2 > len) {
        out.truncated = true;
        break;
      }
      const size_t dataSize = descriptor[i];
      i += 2 + dataSize;
      if (i > len) {
        out.truncated = true;
        break;
      }
      continue;
    }

    const size_t nbytes = itemDataBytes(static_cast<uint8_t>(prefix & 0x03));
    if (i + nbytes > len) {
      out.truncated = true;  // item cut off by the end of the descriptor
      break;
    }
    const uint32_t value = itemValue(descriptor + i, nbytes);
    i += nbytes;

    const uint8_t type = static_cast<uint8_t>((prefix >> 2) & 0x03);
    const uint8_t tag = static_cast<uint8_t>((prefix >> 4) & 0x0F);

    if (type == kTypeGlobal) {
      switch (tag) {
        case kGlobalUsagePage:
          g.usagePage = static_cast<uint16_t>(value);
          if (g.usagePage == kPageKeyboard) out.hasKeyboardPage = true;
          else if (g.usagePage == kPageConsumer) out.hasConsumerPage = true;
          break;
        case kGlobalReportSize:
          g.reportSize = value;
          break;
        case kGlobalReportCount:
          g.reportCount = value;
          break;
        case kGlobalReportId:
          g.reportId = static_cast<uint8_t>(value & 0xFF);
          if (g.reportId != 0) out.hasId = true;  // id 0 means "no report id"
          break;
        case kGlobalPush:
          if (stackDepth < kHidMaxGlobalStack) stack[stackDepth++] = g;
          else out.truncated = true;
          break;
        case kGlobalPop:
          if (stackDepth > 0) g = stack[--stackDepth];
          else out.truncated = true;
          break;
        default:
          break;  // logical/physical limits and unit items do not place fields
      }
      continue;
    }

    if (type == kTypeLocal) {
      switch (tag) {
        case kLocalUsage:
          if (l.usageCount < kHidMaxLocalUsages) l.usages[l.usageCount++] = static_cast<uint16_t>(value);
          else out.truncated = true;
          break;
        case kLocalUsageMin:
          l.usageMin = static_cast<uint16_t>(value);
          l.hasUsageMin = true;
          break;
        case kLocalUsageMax:
          l.usageMax = static_cast<uint16_t>(value);
          l.hasUsageMax = true;
          break;
        default:
          break;  // designator / string / delimiter items do not place fields
      }
      continue;
    }

    if (type != kTypeMain) continue;  // reserved item type

    if (tag == kMainInput) {
      HidReportLayout* layout = layoutFor(out, g.reportId);
      if (layout == nullptr) {
        if (out.reportCount >= kHidMaxReports) {
          // Report id ceiling reached: stop rather than place later fields against
          // the wrong layout (a partial map is worse than none).
          out.truncated = true;
          break;
        }
        layout = &out.reports[out.reportCount++];
        layout->id = g.reportId;
      }
      recordInput(out, g, l, value);
    } else if (tag == kMainCollection || tag == kMainEndCollection) {
      clearLocals(l);  // locals belong to a single main item, collections included
      continue;
    }
    // Output (0x09) and Feature (0x0B) reports have their own payload; they do not
    // consume Input bits, but they do consume the pending locals.
    clearLocals(l);
  }

  return out.usable;
}

bool decodeHidReport(const HidReportMap& map, const uint8_t* data, size_t len, HidReportView& out,
                     uint16_t referenceId) {
  out = HidReportView();
  if (data == nullptr || len == 0 || map.reportCount == 0) return false;

  const HidReportLayout* layout = nullptr;
  size_t payloadOffset = 0;
  bool fromReference = false;

  if (map.hasId) {
    const HidReportLayout* byByte = layoutFor(map, data[0]);
    if (byByte != nullptr) {
      layout = byByte;
      payloadOffset = 1;
    } else if (referenceId != 0xFFFF) {
      // The descriptor declares report ids but the peripheral sent no id byte.
      // The characteristic that notified us names the layout through its Report
      // Reference descriptor (0x2908), so the report keeps its identity anyway.
      const HidReportLayout* byReference = layoutFor(map, static_cast<uint8_t>(referenceId));
      if (byReference != nullptr) {
        layout = byReference;
        fromReference = true;
      }
    }
  } else {
    layout = layoutFor(map, 0);
    if (layout != nullptr && len == static_cast<size_t>((layout->bits + 7) / 8) + 1 && data[0] == 0) {
      // Un-declared report id byte: a payload one byte longer than the layout that
      // starts with 0x00 is the legacy shape seen from remotes that prefix an id
      // even though their descriptor declares none.
      payloadOffset = 1;
    }
  }
  if (layout == nullptr) return false;
  if (layout->keyFieldCount == 0 && layout->modFieldCount == 0) return false;

  const size_t payloadBytes = len - payloadOffset;
  const uint8_t* payload = data + payloadOffset;
  const uint32_t payloadBits = static_cast<uint32_t>(payloadBytes * 8);

  // Every field must fit the payload. A report shorter than its own layout is
  // malformed, and guessing (shifting fields, clamping counts) would decode the
  // wrong keys - so the caller falls back instead.
  for (uint8_t i = 0; i < layout->keyFieldCount; ++i) {
    if (!fieldFits(layout->keyFields[i], payloadBits)) return false;
  }
  for (uint8_t i = 0; i < layout->modFieldCount; ++i) {
    if (!fieldFits(layout->modFields[i], payloadBits)) return false;
  }

  out.id = layout->id;
  out.idFromReference = fromReference;

  for (uint8_t i = 0; i < layout->modFieldCount; ++i) {
    const HidField& f = layout->modFields[i];
    out.mods = static_cast<uint8_t>(out.mods | readReportBits(payload, payloadBytes, f.bitOffset, f.bitSize));
  }

  for (uint8_t i = 0; i < layout->keyFieldCount; ++i) {
    const HidField& f = layout->keyFields[i];
    if ((f.flags & kHidFieldArray) != 0 || f.bitSize != 1) {
      // Array element (or a multi-bit variable element): the value is an offset from
      // Usage Minimum - or the usage id itself when no usage was declared.
      for (uint8_t e = 0; e < f.count; ++e) {
        const uint32_t value =
            readReportBits(payload, payloadBytes, static_cast<uint16_t>(f.bitOffset + e * f.bitSize), f.bitSize);
        if (value == 0) continue;
        addDecodedKey(out, f, (f.flags & kHidFieldHasUsageMin) != 0 ? f.usageMin + value : value);
      }
    } else {
      // Per-bit variable field: bit i is the i-th usage of the declared range.
      for (uint8_t b = 0; b < f.count; ++b) {
        if (readReportBits(payload, payloadBytes, static_cast<uint16_t>(f.bitOffset + b), 1) == 0) continue;
        addDecodedKey(out, f, f.usageMin + b);
      }
    }
  }

  return true;
}

// --- Byte-level view of a layout ---------------------------------------------
//
// How many bytes a field touches: its bit offset rounded down to the containing
// byte, plus the bits it spans rounded up (a field need not be byte-aligned - the
// bit-packed media-key bitmaps are not - and an element need not fill a byte).
namespace {

uint8_t fieldSpanBytes(const HidField& f) {
  const uint32_t bits = static_cast<uint32_t>(f.bitOffset & 0x07) + static_cast<uint32_t>(f.bitSize) * f.count;
  const uint32_t bytes = (bits + 7) / 8;
  return static_cast<uint8_t>(bytes > 0xFF ? 0xFF : bytes);
}

}  // namespace

HidByteLayout hidByteLayout(const HidReportLayout& layout) {
  HidByteLayout b;
  b.modByte = 0xFF;
  b.modBytes = 0;
  b.keyByte = 0xFF;
  b.keyBytes = 0;
  b.keyBits = 0;

  if (layout.modFieldCount > 0) {
    const HidField& f = layout.modFields[0];
    b.modByte = static_cast<uint8_t>(f.bitOffset >> 3);
    b.modBytes = fieldSpanBytes(f);
  }
  if (layout.keyFieldCount > 0) {
    const HidField& f = layout.keyFields[0];
    b.keyByte = static_cast<uint8_t>(f.bitOffset >> 3);
    b.keyBytes = fieldSpanBytes(f);
    b.keyBits = f.bitSize;
  }
  return b;
}

}  // namespace freeink
