#include <cassert>
#include <iostream>
#include <vector>

#include "driver/Uc8279Driver.h"
#include "lut/Uc8279X3Luts.h"

int main() {
  using namespace freeink;
  constexpr size_t n = 792 / 8 * 528;
  EpdBus bus;
  Uc8279Driver driver;
  std::vector<uint8_t> bw(n, 0x88), lsb(n, 0xAA), msb(n, 0xCC);
  assert(driver.grayscaleCapabilities(GrayscaleMode::Absolute).supported());
  assert(driver.grayscaleCapabilities().stripUploads && !driver.grayscaleCapabilities().asyncBase);
  driver.begin(bus);
  driver.displayGrayscaleBase(bus, bw.data(), RefreshMode::Half, false);
  driver.copyGrayscaleLsb(bus, lsb.data());
  driver.copyGrayscaleMsb(bus, msb.data());
  // Inputs are passed unchanged regardless of gray-pixel coverage.
  assert(bus.oldPlane == lsb && bus.newPlane == msb);
  driver.displayGray(bus, bw.data(), false, nullptr, false);
  assert(bus.rawRegisters == std::vector<uint8_t>({0x20, 0x23, 0x22, 0x21, 0x24}));
  assert(bus.lastBank == std::vector<uint8_t>(kUc8279X3_XtfAa[0], kUc8279X3_XtfAa[0] + 49));

  for (bool turnOff : {false, true}) {
    const auto before = bus.refreshes;
    driver.beginGrayscale(bus, bw.data(), GrayscaleMode::Absolute, RefreshMode::Half, false);
    assert(bus.refreshes == before);  // No visible B/W preview before the complete gray image.
    driver.copyGrayscaleLsb(bus, lsb.data());
    driver.copyGrayscaleMsb(bus, msb.data());
    driver.displayGray(bus, bw.data(), turnOff, nullptr, true);
    assert(bus.refreshes == before + 1);
    assert(bus.rawRegisters == std::vector<uint8_t>({0x20, 0x24, 0x22, 0x23, 0x21}));
    assert(bus.lastBank == std::vector<uint8_t>(kUc8279X3_Xth4[0], kUc8279X3_Xth4[0] + 49));
    // Cleanup must not clear the required physical rebase after an absolute pass.
    driver.cleanupGrayscaleBuffers(bus, bw.data());
    driver.display(bus, bw.data(), nullptr, RefreshMode::Fast, true);
    assert(bus.oldPlane == bw && bus.newPlane == bw);
    assert(bus.lastBwBank == std::vector<uint8_t>(kUc8279X3_BwGc[0] + 1, kUc8279X3_BwGc[0] + 43));
  }
  driver.deepSleep(bus);
  driver.begin(bus);
  driver.display(bus, bw.data(), nullptr, RefreshMode::Fast, false);
  assert(bus.oldPlane == bw && bus.newPlane == bw);

  // Drive rails across a standing screen. Page turns keep them up; the idle
  // hook drops them with one POF; the next turn powers up and stays on DU.
  const auto at = [](const std::vector<uint8_t>& c, uint8_t v) {
    for (size_t i = 0; i < c.size(); ++i)
      if (c[i] == v) return static_cast<long>(i);
    return -1L;
  };
  const std::vector<uint8_t> du(kUc8279X3_BwDu[0] + 1, kUc8279X3_BwDu[0] + 43);
  {
    EpdBus b;
    Uc8279Driver d;
    d.begin(b);
    for (int i = 0; i < 4; ++i) d.display(b, bw.data(), nullptr, RefreshMode::Fast, false);
    assert(at(b.cmds, 0x02) < 0);  // adjacent turns: no POF between them
    b.cmds.clear();
    d.controllerIdle(b);
    assert(b.cmds == std::vector<uint8_t>({0x02}));  // idle: rails off
    d.controllerIdle(b);
    assert(b.cmds == std::vector<uint8_t>({0x02}));  // already off: silent
    b.cmds.clear();
    d.display(b, bw.data(), nullptr, RefreshMode::Fast, false);
    assert(at(b.cmds, 0x04) >= 0 && at(b.cmds, 0x04) < at(b.cmds, 0x12));
    assert(at(b.cmds, 0x02) < 0 && b.lastBwBank == du);
    // Sleep ends with POF then DSLP, whether the rails were up or idled.
    b.cmds.clear();
    d.deepSleep(b);
    assert(b.cmds == std::vector<uint8_t>({0x02, 0x07}));
    d.begin(b);
    d.display(b, bw.data(), nullptr, RefreshMode::Fast, false);
    d.controllerIdle(b);
    b.cmds.clear();
    d.deepSleep(b);
    assert(b.cmds == std::vector<uint8_t>({0x07}));
    // Gray sleep image: the gray waveform leaves the rails up, sleep still POFs.
    d.begin(b);
    d.displayGrayscaleBase(b, bw.data(), RefreshMode::Half, false);
    d.copyGrayscaleLsb(b, lsb.data());
    d.copyGrayscaleMsb(b, msb.data());
    d.displayGray(b, bw.data(), false, nullptr, true);
    b.cmds.clear();
    d.deepSleep(b);
    assert(b.cmds == std::vector<uint8_t>({0x02, 0x07}));
  }
  std::cout << "PASS: UC8279 overlay/absolute LUT mapping, unchanged planes, cleanup and wake\n";
  std::cout << "PASS: UC8279 idle rails off, adjacent turns stay powered, sleep ends POF+DSLP\n";
}
