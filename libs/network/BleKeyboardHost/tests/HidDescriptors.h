#pragma once

// HID Report Map descriptors shared by the two host test suites: the parser
// suite (HidKeymapTest.cpp) and the central-role ingest suite
// (BleKeyboardHostIngestTest.cpp), which serves them over the fake GATT server.

#include <stdint.h>

namespace hidtest {

// HID 1.11 Appendix B.1 keyboard, report id 0: modifier byte, reserved byte, six
// key bytes (8-byte report).
inline constexpr uint8_t kBootKeyboard[] = {
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
inline constexpr uint8_t kTwoReports[] = {
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
inline constexpr uint8_t kListedModifiers[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07,
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
inline constexpr uint8_t kMixedPages[] = {
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
inline constexpr uint8_t kBootThenOverflow[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x75, 0x01, 0x95,
    0x08, 0x81, 0x02, 0x95, 0x01, 0x75, 0x08, 0x81, 0x03, 0x19, 0x00, 0x29, 0x65, 0x75, 0x08,
    0x95, 0x06, 0x81, 0x00, 0x19, 0x00, 0x29, 0x65, 0x75, 0x08, 0x95, 0xFF, 0x81, 0x00, 0xC0,
};

// --- Malformed / rejected descriptors ----------------------------------------

inline constexpr uint8_t kNoInput[] = {0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0xC0};

inline constexpr uint8_t kReportSizeZero[] = {0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x75, 0x00, 0x95, 0x06, 0x81, 0x00};

inline constexpr uint8_t kReportCountZero[] = {0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x75, 0x08, 0x95, 0x00, 0x81, 0x00};

// 255 elements x 8 bits = 2040 bits, past kHidMaxReportBits.
inline constexpr uint8_t kReportCountHuge[] = {0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x75, 0x08, 0x95, 0xFF, 0x81, 0x00};

// A two-byte Report Count of 256, past kHidMaxFieldBits (255).
inline constexpr uint8_t kReportCount256[] = {0x05, 0x07, 0x19, 0x00, 0x29, 0x65,
                                              0x75, 0x08, 0x96, 0x00, 0x01, 0x81, 0x00};

// Cut in the middle of an item: "Report Size" with its data byte missing.
inline constexpr uint8_t kCutItem[] = {0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x75};

// Cut in the middle of the Input item itself.
inline constexpr uint8_t kCutInput[] = {0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x75, 0x01, 0x95, 0x08, 0x81};

// Long item whose data field runs past the end of the descriptor.
inline constexpr uint8_t kCutLongItem[] = {0xFE, 0x04};

// Nothing but padding: bits are consumed, no field to decode.
inline constexpr uint8_t kConstantOnly[] = {0x05, 0x07, 0x75, 0x08, 0x95, 0x01, 0x81, 0x03};

// A long item claiming more data than the descriptor holds; the parser must stop
// instead of reading past the buffer or spinning on it.
inline constexpr uint8_t kLongItemOverrun[] = {0xFE, 0xFF, 0x01};

}  // namespace hidtest
