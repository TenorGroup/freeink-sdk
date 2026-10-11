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
