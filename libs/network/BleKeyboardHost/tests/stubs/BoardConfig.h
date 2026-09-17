#pragma once

// Host-test stub for <BoardConfig.h>.
//
// The BLE HID host only reads the capability macro that gates its NimBLE code;
// the board profiles (GPIO, display, power) are irrelevant to a host build and
// pull in esp_rom/GPIO headers. The values below mirror the default policy in
// freeink-sdk/libs/hardware/BoardConfig/include/BoardConfig.h, with the BLE HID
// host capability switched ON so the tests exercise the real central-role path
// instead of the stub bodies. This is a host-test compile definition only: no
// firmware build flag, env or platformio.ini is touched.

#define FREEINK_CAP_BLE_HID_HOST 1
#define FREEINK_CAP_BLE_KEYBOARD 1
#define FREEINK_BLE_HID_SHOW_UNNAMED_DEVICES 0
#define FREEINK_BLE_HID_REQUIRE_MITM 0

// Extended advertising is off in this fake: the scan-debug/EXT_ADV branches stay
// out of the compiled path, so the fake needs no PHY model.
#define CONFIG_BT_NIMBLE_EXT_ADV 0
