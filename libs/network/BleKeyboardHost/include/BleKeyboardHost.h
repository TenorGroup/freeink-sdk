#pragma once

// FreeInk SDK - BLE HID host (singleton).
//
// Pairs with and connects to a Bluetooth Low Energy HID peripheral (central
// role) and exposes translated key events (printable chars + a SpecialKey enum)
// plus scan/pair/connect controls for a settings UI. One peripheral at a time.
//
// Capability-gated: the real NimBLE implementation compiles only when
// FREEINK_CAP_BLE_HID_HOST is set (and the firmware adds NimBLE-Arduino to its
// lib_deps); otherwise every method links a stub so callers need no #ifdefs and
// no BLE code is pulled in. This header is deliberately NimBLE-free so consumers
// (and host builds) never include the BLE stack just to see the API.
//
// Memory: all storage is fixed-capacity (no std::vector / heap in the hot path).
// BLE callbacks run on the NimBLE host task and hand data to the app through a
// small spinlock-guarded ring; drain it from the main loop with popKey().
//
// BLE-only - the ESP32-C3/S3 has no Bluetooth Classic radio, so Classic-only HID
// peripherals cannot connect.

// C++-only. The guard keeps a `-include` of this header (see the x3-ble env in
// platformio.ini) harmless in the firmware's C translation units (wolfSSL,
// expat, FreeType, the Arduino core's .c files): there it expands to nothing.
#ifdef __cplusplus

#include <Arduino.h>
#include <atomic>
#include <stddef.h>
#include <stdint.h>

namespace freeink {

enum class PickPolicy : uint8_t { Priority = 0, First = 1 };

// Non-character keys an editor/UI cares about. Printable keys arrive as `ch`.
enum class SpecialKey : uint8_t {
  None = 0,
  Enter,
  Backspace,
  Tab,
  Escape,
  Delete,
  Left,
  Right,
  Up,
  Down,
  Home,
  End,
  PageUp,
  PageDown,
};

// One decoded key edge. `pressed` is true on the press edge and false on the
// matching release, so an app can time how long a button was held (a page-turner
// remote that means "next chapter" when its next-page button is held needs both
// edges). No auto-repeat is synthesized: one physical press is one press event.
// One usage is tracked at a time, which is what a page-turner sends.
struct KeyEvent {
  char ch = 0;                          // printable ASCII, or 0 for a special key
  uint8_t keycode = 0;                  // raw HID usage id
  uint8_t mods = 0;                     // HID modifier bitmask (ctrl/shift/alt/gui)
  SpecialKey special = SpecialKey::None;
  bool pressed = true;
};

// One button edge read from the report BYTES, before the Report Map decode. The
// identity is where the first non-zero byte sits: the report id (the id byte when
// the frame carries one, else the Report Reference of the characteristic, else 0),
// the payload byte index and its value. It survives what the decoder cannot see: a
// bitmap past the parser's field ceiling, a 16-bit usage cut to 8 bits, two pages
// that share a low byte. `keycode`/`mods` are what the decoder read from the same
// frame (0 when it read no new key), so an app that routes raw edges can still fall
// back to its keycode bindings for a button it has not learned.
struct RawButtonEvent {
  uint8_t reportId = 0;
  uint8_t byteIndex = 0;
  uint8_t value = 0;
  bool pressed = false;
  uint8_t keycode = 0;
  uint8_t mods = 0;
  // Release only: the press it ends was read before the report's rest frame was known,
  // and the rest frame learned since is that press's own byte, so the "press" was the
  // remote idling on a non-zero status byte, not a button. A learning screen drops it.
  bool wasRest = false;
  uint32_t atMs = 0;  // millis() when the frame arrived, for hold timing
  // value | byteIndex << 8 | reportId << 16; never 0 for an edge.
  uint32_t code() const {
    return static_cast<uint32_t>(value) | static_cast<uint32_t>(byteIndex) << 8 |
           static_cast<uint32_t>(reportId) << 16;
  }
};

// A BLE device seen during a scan.
struct DiscoveredDevice {
  char addr[18] = {0};  // "AA:BB:CC:DD:EE:FF"
  char name[32] = {0};  // falls back to the address when no name was received
  int rssi = 0;
  uint8_t addrType = 0;  // BLE address type, needed to reconnect
  bool hasName = false;  // true when the advertised name was actually received
  bool hid = false;      // advertises the HID service (0x1812)
  bool connectable = false;
};

// A BLE HID peripheral the host has bonded with (persisted in NVS for auto-reconnect).
struct PairedHidDevice {
  char addr[18] = {0};
  char name[32] = {0};
  uint8_t addrType = 0;
};
using PairedKeyboard = PairedHidDevice;  // Backward-compatible SDK name.

class BleKeyboardHost {
 public:
  static constexpr uint8_t kMaxDiscovered = 24;
  static constexpr uint8_t kMaxBonds = 4;
  static constexpr uint8_t kKeyQueueLen = 16;
  // Eight presses with their releases, plus the slot that tells a full ring from an
  // empty one: a burst of taps while the reader lays out a page loses none.
  static constexpr uint8_t kRawQueueLen = 17;

  static BleKeyboardHost& getInstance();

  // Init the NimBLE central, security (Just Works bonding), and load the saved
  // pairing list. Safe to call once. Returns false if BLE init failed or the
  // capability is compiled out.
  bool begin(const char* hostName = "FreeInk");

  // Request a complete BLE teardown and return true only after the worker, client,
  // and NimBLE stack are fully stopped. A timeout leaves the worker/client/stack
  // intact and marks teardown pending, so a later end() can retry safely. The
  // default budget is 1 second and is capped at 2 seconds. end(0) only services
  // an already-safe teardown and never waits.
  bool end(uint32_t timeoutMs = 1000);

  // True after end() requested teardown but before every worker/client/stack
  // resource has been released. begin() rejects while this is true.
  bool isStopping() const;

  // Pump per main-loop iteration: drives bounded auto-reconnect and held-key expiry.
  // Cheap; never blocks.
  void poll();

  // True while the NimBLE stack is initialized (between a successful begin() and
  // end()). Lets the app gate CPU-frequency and lifecycle decisions on whether BLE
  // is actually resident, independent of the user's on/off preference.
  bool isRunning() const { return begun_.load(std::memory_order_acquire); }

  // --- Discovery -------------------------------------------------------------
  void startScan(uint32_t ms = 5000);
  void stopScan();
  bool isScanning() const { return scanning_; }
  uint8_t deviceCount() const { return deviceCount_; }
  const DiscoveredDevice& device(uint8_t i) const;
  // Free scan bookkeeping after connecting, to reclaim RAM while writing.
  void releaseScanResults();

  // --- Connection ------------------------------------------------------------
  // Begin an async connect to a scanned/bonded address. isConnected() flips once
  // the link is encrypted and the HID input report is subscribed. Each complete
  // attempt has a 15-second deadline serviced by poll(). Explicit connect rearms
  // one pass through saved bonds; disconnect suppresses automatic reconnection.
  bool connect(const char* addr);
  bool armBondedReconnect(PickPolicy policy, const char* priorityAddr);
  void disconnect();
  bool isConnected() const { return connected_; }
  bool isConnecting() const { return connecting_; }
  const char* connectedName() const { return connName_; }
  // Address of the peer on the live link ("" when none). Settings keyed by remote
  // must use this, not the saved choice: auto-reconnect can bring up another bond.
  const char* connectedAddr() const { return connAddr_; }
  bool takeConnectFailure(char* out, size_t outLen);
  bool takePairingPasskey(uint32_t& out);

  // --- Pairings (persisted) --------------------------------------------------
  uint8_t pairedCount() const { return bondCount_; }
  const PairedHidDevice& paired(uint8_t i) const;
  void forget(const char* addr);

  // --- Translated input ------------------------------------------------------
  // Pop the next key event. Returns false when the queue is empty.
  bool popKey(KeyEvent& out);
  // Pop the next raw button edge (see RawButtonEvent). Filled from the same frames
  // as popKey(), in its own ring, so an app that ignores it sees no change.
  bool popRawButton(RawButtonEvent& out);
  // Raw presses dropped because the ring was full (a burst nobody drained). A press
  // is dropped whole: one that got in always keeps a slot for its release.
  uint16_t rawOverflows() const { return rawOverflow_; }

  // True while the peer is still streaming input reports - one arrived inside the
  // host's own stale-release window. Lets an app tell "the button is still down and
  // the remote keeps saying so" from "the remote went quiet", which is the only way
  // to time a button HOLD on a remote that streams instead of sending a release frame.
  bool reportStreamFresh() const;

  // --- Internal: called by the NimBLE backend (not for app use). These keep the
  // public header free of NimBLE types - the .cpp translates BLE objects into
  // these plain calls. -------------------------------------------------------
  void onScanResultIngest(const char* addr, const char* name, int rssi, uint8_t type, bool hid, bool connectable);
  void onReportIngest(const uint8_t* data, size_t len);
  void onLinkUp(const char* addr, const char* name, uint8_t type);
  void onLinkDown();
  void onConnectFailed(const char* reason);
  void onPairingPasskey(uint32_t passkey);

 private:
  bool connectInternal(const char* addr, bool explicitRequest);
  void enqueue(const KeyEvent& ev);    // ring push (spinlock-guarded)
  void emitUsage(uint8_t usage, uint8_t mods, bool pressed);  // translate + enqueue
  bool payOwedRelease();               // emit the release a recorded press still owes
  void decodeReport(const uint8_t* data, size_t len);  // map decode -> key events
  // Raw edge ring push; the caller holds the ring lock. Refused (false) unless `keep`
  // slots stay free after it: a press keeps one for its own release, so no release
  // of a press that got in is ever dropped. `wasRest`: see RawButtonEvent.
  bool pushRawLocked(uint32_t code, bool pressed, uint32_t atMs, uint8_t keycode, uint8_t mods, uint8_t keep,
                     bool wasRest = false);
  void persistBonds();
  void loadBonds();
  BleKeyboardHost() = default;
  BleKeyboardHost(const BleKeyboardHost&) = delete;
  BleKeyboardHost& operator=(const BleKeyboardHost&) = delete;

  // Plain, NimBLE-free state shared by both the real and stub builds. The NimBLE
  // objects, spinlock, and connection task live file-static in the .cpp so this
  // header pulls in nothing.
  DiscoveredDevice devices_[kMaxDiscovered];
  uint8_t deviceCount_ = 0;
  PairedHidDevice bonds_[kMaxBonds];
  uint8_t bondCount_ = 0;
  KeyEvent ring_[kKeyQueueLen];
  volatile uint8_t ringHead_ = 0;  // next write
  volatile uint8_t ringTail_ = 0;  // next read
  char connName_[32] = {0};
  char connAddr_[18] = {0};
  char connectFailure_[48] = {0};
  volatile uint32_t pairingPasskey_ = 0;
  volatile bool connected_ = false;
  volatile bool connecting_ = false;
  volatile bool connectFailed_ = false;
  volatile bool pairingPasskeyReady_ = false;
  volatile bool scanning_ = false;
  std::atomic<bool> begun_{false};

  // Key auto-repeat: HID delivers one report per state change, so holding a key
  // (backspace, arrows) only sends a single press. The backend records the held
  // usage here and poll() synthesizes repeats after an initial delay.
  volatile uint8_t heldUsage_ = 0;
  volatile uint8_t heldMods_ = 0;
  volatile uint32_t heldSince_ = 0;
  volatile uint32_t lastRepeat_ = 0;
  uint8_t prevKeys_[6] = {0};  // backend-task only

  // The release a press still owes. emitUsage() records it on the press edge and
  // payOwedRelease() settles it - when the usage leaves the report, or (only for a
  // remote that was streaming) when the reports stop. It deliberately outlives the
  // stale-release timeout below: a remote that sends one frame per edge can hold a
  // button far longer than that window, and capping the release there would cap
  // every hold at 150 ms.
  volatile uint8_t owedUsage_ = 0;
  volatile uint8_t owedMods_ = 0;
  volatile uint8_t reportsSincePress_ = 0;  // 1 = press only, >1 = the remote streams

  // Raw button edges (RawButtonEvent), their own ring. rawCode_ is the button the
  // last frame held (0 = none); rawReports_ counts frames that repeated it, which
  // is what tells a streaming remote (silence = release) from one that sends one
  // frame per edge (silence = still held). Both are guarded by the ring lock.
  RawButtonEvent rawRing_[kRawQueueLen];
  volatile uint8_t rawHead_ = 0;
  volatile uint8_t rawTail_ = 0;
  uint32_t rawCode_ = 0;
  uint8_t rawReports_ = 0;
  uint16_t rawOverflow_ = 0;
  bool rawDropped_ = false;  // rawCode_'s press was dropped: its release is not sent either
  bool rawGuessed_ = false;  // rawCode_ was read against the all-zero guess, its rest not yet known
  // The rest frame (nothing pressed) and the last frame of each report id, so a
  // button is read as what changed against rest. Backend task only.
  struct RawRest {
    uint8_t id;
    uint8_t frames;  // frames seen, capped at 2: the first one may itself be the rest state
    bool known;      // rest is a frame the remote sent, not the all-zero guess
    uint8_t rest[8];
    uint8_t prev[8];
  };
  RawRest rawRest_[4];
  uint8_t rawRestCount_ = 0;
  bool frameAxisPad_ = false;  // the decoder read this frame as an axis-pair gamepad
  // First key the decoder pressed in the frame being ingested (backend task only).
  uint8_t framePressUsage_ = 0;
  uint8_t framePressMods_ = 0;
};

}  // namespace freeink

// App-friendly accessors. BleKbd is kept for source compatibility.
#define BleHid ::freeink::BleKeyboardHost::getInstance()
#define BleKbd ::freeink::BleKeyboardHost::getInstance()

#endif  // __cplusplus
