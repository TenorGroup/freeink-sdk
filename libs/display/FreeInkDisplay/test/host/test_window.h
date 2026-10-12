#pragma once

static size_t windowCommands(const EpdBus& bus, uint8_t cmd) {
  return std::count_if(bus.writes.begin(), bus.writes.end(), [cmd](const auto& w) { return w.command == cmd; });
}

static size_t windowBytes(const EpdBus& bus, uint8_t cmd) {
  size_t size = 0;
  for (const auto& w : bus.writes) if (w.command == cmd) size += w.bytes.size();
  return size;
}

// Interpret only RAM addressing. Electrical scan, BUSY timing and charge need a panel.
static void checkWindowTrace(const EpdBus& bus, const Bytes& target, unsigned x, unsigned y, unsigned w,
                             unsigned h, unsigned scan = 1) {
  const unsigned xs = x & ~7u, xe = (x + w - 1) | 7u, ys = y + 120, ye = ys + h - 1;
  const Bytes expectedPtl = {uint8_t(xs >> 8), uint8_t(xs), uint8_t(xe >> 8), uint8_t(xe),
                            uint8_t(ys >> 8), uint8_t(ys), uint8_t(ye >> 8), uint8_t(ye), uint8_t(scan)};
  bool partial = false, addressed = false, refreshed = false;
  unsigned planes = 0;
  Bytes oldRam(60000, 0xD3), newRam = oldRam;
  for (const auto& entry : bus.writes) {
    if (entry.command == 0x91) partial = true;
    if (entry.command == 0x90) {
      assert(partial && entry.bytes == expectedPtl);
      addressed = true;
    }
    if (entry.command == 0x12) {
      assert(partial && addressed && planes == 1);
      refreshed = true;
    }
    if (entry.command == 0x10 || entry.command == 0x13) {
      assert(partial && addressed);
      assert(entry.command == (refreshed ? 0x10 : 0x13));
      if (refreshed) {
        const size_t position = &entry - bus.writes.data();
        assert(std::find(bus.waitPositions.begin(), bus.waitPositions.end(), position) != bus.waitPositions.end());
      }
      assert(entry.bytes.size() == (xe - xs + 1) / 8 * h && entry.transactions == 1);
      auto& ram = refreshed ? oldRam : newRam;
      size_t pos = 0;
      for (unsigned row = y; row < y + h; ++row)
        for (unsigned col = xs / 8; col <= xe / 8; ++col) {
          assert(entry.bytes[pos] == target[row * 100 + col]);
          ram[(row + 120) * 100 + col] = entry.bytes[pos++];
        }
      ++planes;
    }
    if (entry.command == 0x92) {
      assert(planes == 2 && refreshed);
      partial = false;
    }
  }
  assert(!partial && planes == 2 && windowCommands(bus, 0x12) == 1 && windowCommands(bus, 0x90) == 1);
  for (unsigned row = 0; row < 600; ++row)
    for (unsigned col = 0; col < 100; ++col) {
      const bool inside = row >= ys && row <= ye && col >= xs / 8 && col <= xe / 8;
      const auto expected = inside ? target[(row - 120) * 100 + col] : 0xD3;
      assert(oldRam[row * 100 + col] == expected && newRam[row * 100 + col] == expected);
    }
}

static void testWindowBw() {
  BoardConfig::ACTIVE.board = BoardConfig::Board::XteinkX4Pro;
  Uc8279X4Driver driver;
  EpdBus bus;
  const auto initial = frame(8), target = frame(92);
  driver.begin(bus);
  driver.display(bus, initial.data(), nullptr, RefreshMode::Full, false);
  const auto run = [&](unsigned x, unsigned y, unsigned w, unsigned h) {
    bus.clear();
    driver.displayWindow(bus, target.data(), nullptr, x, y, w, h, false);
    checkWindowTrace(bus, target, x, y, w, h);
  };
  run(80, 120, 600, 8);
  std::printf("BW 600x8 DTM1=%zu DTM2=%zu DRF=%zu\n", windowBytes(bus, 0x10), windowBytes(bus, 0x13),
              windowCommands(bus, 0x12));
  for (unsigned x : {0u, 1u}) for (unsigned w : {1u, 7u, 8u, 9u})
    for (unsigned y : {0u, 479u}) run(x, y, w, 1);
  run(799, 479, 1, 1);
  run(72, 30, 254, 383);
  BoardConfig::ACTIVE.board = BoardConfig::Board::XteinkX4Classic;
  bus.clear();
  driver.displayWindow(bus, target.data(), nullptr, 80, 120, 600, 8, false);
  assert(lastRegister(bus, 0xe5) == 0x5a && windowBytes(bus, 0x13) == 60000);
  BoardConfig::ACTIVE.board = BoardConfig::Board::XteinkX4Pro;
  run(72, 30, 254, 383);
  std::printf("BW cover DTM1=%zu DTM2=%zu\n", windowBytes(bus, 0x10), windowBytes(bus, 0x13));
  for (unsigned failure = 0; failure < 6; ++failure) {
    driver.display(bus, initial.data(), nullptr, RefreshMode::Full, false);
    if (failure == 0) driver._oldPlaneValid = false;
    if (failure == 1) driver.requestResync(1);
    if (failure == 2) driver.beginGrayscale(bus, initial.data(), GrayscaleMode::Absolute, RefreshMode::Fast, false);
    bus.clear();
    driver.displayWindow(bus, target.data(), nullptr, failure == 3 ? 800 : 0, 0,
                         failure == 4 ? 0 : (failure == 5 ? 801 : 8), 8, false);
    assert(windowBytes(bus, 0x13) == 60000 && windowCommands(bus, 0x12) == 1);
  }
  free(driver._grayBase);
}

static void testWindowGray() {
  BoardConfig::ACTIVE.board = BoardConfig::Board::XteinkX4Pro;
  BoardConfig::ACTIVE.displayControllerVariant = 0x02;
  Uc8279X4Driver driver;
  FreeInkDisplay display(12, 11, 13, 18, 14, 6);
  display._driver = &driver;
  display.begin();
  const auto base = frame(31), lsb = frame(74), msb = frame(109);
  std::memcpy(display.getFrameBuffer(), base.data(), base.size());
  display.displayBuffer(FreeInkDisplay::FULL_REFRESH);
  display.syncWriteBufferFromActive();
  auto& bus = display._bus;
  bus.clear();
  assert(display.displayGrayscaleWindow(lsb.data(), msb.data(), 73, 30, 254, 383));
  const Bytes ptl = {0, 72, 1, 71, 0, 150, 2, 20, 1};
  bool partial = false, addressed = false;
  unsigned planes = 0, drf = 0;
  Bytes oldRam(60000, 0xD3), newRam = oldRam;
  for (const auto& entry : bus.writes) {
    if (entry.command == 0x91) partial = true;
    if (entry.command == 0x90) { assert(partial && entry.bytes == ptl); addressed = true; }
    if (entry.command == 0x12) { assert(planes == 2 && partial); ++drf; }
    if (entry.command == 0x10 || entry.command == 0x13) {
      assert(partial && addressed && planes < 4);
      assert(entry.command == (planes % 2 == 0 ? 0x10 : 0x13));
      assert(entry.bytes.size() == 32 * 383 && entry.transactions == 1);
      assert(drf == (planes < 2 ? 0 : 1));
      auto& ram = planes % 2 == 0 ? oldRam : newRam;
      size_t offset = 0;
      for (unsigned y = 30; y < 413; ++y) for (unsigned x = 9; x < 41; ++x) {
        const size_t i = y * 100 + x;
        const uint8_t expected = planes < 2 ? uint8_t(~(planes == 0 ? lsb[i] : msb[i])) : base[i];
        assert(entry.bytes[offset] == expected);
        ram[(y + 120) * 100 + x] = entry.bytes[offset++];
      }
      ++planes;
    }
    if (entry.command == 0x92) { assert(planes == 4 && drf == 1); partial = false; }
  }
  assert(!partial && planes == 4 && drf == 1);
  for (unsigned y = 0; y < 600; ++y) for (unsigned x = 0; x < 100; ++x) {
    const uint8_t expected = y >= 150 && y < 533 && x >= 9 && x < 41 ? base[(y - 120) * 100 + x] : 0xD3;
    assert(oldRam[y * 100 + x] == expected && newRam[y * 100 + x] == expected);
  }
  std::printf("AA cover planes=4 bytes=%zu DRF=%u\n", windowBytes(bus, 0x10) + windowBytes(bus, 0x13), drf);
  bus.clear();
  display.cleanupGrayscaleBuffers(base.data());
  assert(windowBytes(bus, 0x10) == 12256 && windowBytes(bus, 0x13) == 0 && windowCommands(bus, 0x12) == 0);
  bus.clear();
  display.displayWindow(80, 120, 600, 8);
  checkWindowTrace(bus, base, 80, 120, 600, 8);
  for (bool missingLsb : {true, false}) {
    bus.clear();
    assert(!display.displayGrayscaleWindow(missingLsb ? nullptr : lsb.data(), missingLsb ? msb.data() : nullptr,
                                          73, 30, 254, 383));
    assert(windowCommands(bus, 0x12) == 0);
    display.displayWindow(80, 120, 600, 8);
    assert(windowBytes(bus, 0x13) == 60000 && windowCommands(bus, 0x12) == 1);
  }
  free(driver._grayBase);
  driver._grayBase = nullptr;
  driver._grayBaseValid = false;
  bus.clear();
  assert(!display.displayGrayscaleWindow(lsb.data(), msb.data(), 73, 30, 254, 383));
  assert(windowCommands(bus, 0x12) == 1 && windowBytes(bus, 0x13) == 60000);
  assert((lastRegister(bus, 0x00) & 0x20) == 0);
  display.releaseBuffers();
}

static void testWindowRecovery() {
  BoardConfig::ACTIVE.board = BoardConfig::Board::XteinkX4Pro;
  Uc8279X4Driver driver;
  FreeInkDisplay display(12, 11, 13, 18, 14, 6);
  display._driver = &driver;
  display.begin();
  const auto base = frame(31), lsb = frame(74), msb = frame(109);
  auto& bus = display._bus;
  const auto reset = [&] {
    std::memcpy(display.getFrameBuffer(), base.data(), base.size());
    display.displayBuffer(FreeInkDisplay::FULL_REFRESH);
    display.syncWriteBufferFromActive();
  };
  const auto local = [&] {
    bus.clear();
    assert(display.displayGrayscaleWindow(lsb.data(), msb.data(), 73, 30, 254, 383));
  };
  reset();
  for (unsigned pass = 1; pass <= 11; ++pass) {
    local();
    const size_t bytes = windowBytes(bus, 0x10) + windowBytes(bus, 0x13);
    const size_t drf = windowCommands(bus, 0x12);
    if (pass >= 9) std::printf("AA recovery pass=%u bytes=%zu DRF=%zu\n", pass, bytes, drf);
    assert(bytes == (pass <= 10 ? 49024u : 420000u));
    assert(drf == (pass <= 10 ? 1u : 2u));
    display.cleanupGrayscaleBuffers(base.data());
  }
  for (unsigned count : {1u, 5u, 10u}) {
    reset();
    for (unsigned pass = 0; pass < count; ++pass) local();
    display.endGrayscaleWindowSession();
    bus.clear();
    display.displayWindow(80, 120, 600, 8);
    assert(windowBytes(bus, 0x13) == 60000 && windowCommands(bus, 0x12) == 1);
    assert(lastRegister(bus, 0xe5) == 0x1e);
  }
  reset();
  local();
  display.endGrayscaleWindowSession();
  local();
  assert(windowCommands(bus, 0x12) == 2 && windowBytes(bus, 0x13) == 180000);
  reset();
  local();
  bus.clear();
  assert(display.displayGrayscaleWindow(lsb.data(), msb.data(), 73, 30, 248, 382));
  assert(windowCommands(bus, 0x12) == 2 && windowBytes(bus, 0x13) == 180000);
  display.releaseBuffers();
  free(driver._grayBase);
}

#ifdef TENOR_PRESS_PROBE
static void testWindowScan() {
  Uc8279X4Driver driver;
  EpdBus bus;
  const auto base = frame(5);
  driver.begin(bus);
  driver.display(bus, base.data(), nullptr, RefreshMode::Full, false);
  for (uint8_t scan : {0, 1}) {
    uc8279X4WindowScan = scan;
    bus.clear();
    driver.displayWindow(bus, base.data(), nullptr, 80, 120, 600, 8, false);
    checkWindowTrace(bus, base, 80, 120, 600, 8, scan);
  }
  uc8279X4WindowScan = 1;
  free(driver._grayBase);
}
#endif
