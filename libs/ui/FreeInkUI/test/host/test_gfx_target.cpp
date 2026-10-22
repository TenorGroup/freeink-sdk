// Host test for the GfxRenderer adapter's bitmap sink: a skin that lays a screen out with painting off still learns
// where each picture (a list's label image) would stand. Run with test/host/run.sh.

#include <FreeInkUIGfxRenderer.h>

#include <cstdio>
#include <vector>

namespace fui = freeink::ui;

namespace {
int failed = 0;
#define CHECK(cond)                                                \
  do {                                                             \
    if (!(cond)) {                                                 \
      ++failed;                                                    \
      std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);  \
    }                                                              \
  } while (0)

struct Seen {
  fui::Rect rect;
  const uint8_t* data;
  fui::BitmapMode mode;
  fui::Color color;
};
void collect(void* ctx, const fui::Rect rect, const fui::BitmapRef bitmap, const fui::BitmapMode mode,
             const fui::Paint& foreground, const fui::Rotation) {
  static_cast<std::vector<Seen>*>(ctx)->push_back({rect, bitmap.data, mode, foreground.color});
}
const uint8_t kBits[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
}  // namespace

int main() {
  GfxRenderer renderer;
  fui::GfxRendererTarget target(renderer);
  const fui::BitmapRef bits{kBits, 8, 8, fui::BitmapFormat::BW1, false};
  std::vector<Seen> seen;

  // Painting on, no sink: ink lands as before.
  target.bitmap({10, 10, 8, 8}, bits, fui::BitmapMode::Center);
  CHECK(renderer.pixels == 64);

  // Painting off, no sink: nothing.
  renderer.pixels = 0;
  target.setPaintingEnabled(false);
  target.bitmap({10, 10, 8, 8}, bits, fui::BitmapMode::Center);
  CHECK(renderer.pixels == 0);

  // Painting off, with the sink: nothing drawn, the picture reported with its rect, mode and ink.
  target.setBitmapSink(collect, &seen);
  target.bitmap({20, 30, 8, 8}, bits, fui::BitmapMode::TileX, fui::Paint::solid(fui::Color::White));
  CHECK(renderer.pixels == 0);
  CHECK(seen.size() == 1);
  CHECK(seen.size() == 1 && seen[0].rect.x == 20 && seen[0].rect.y == 30 && seen[0].data == kBits);
  CHECK(seen.size() == 1 && seen[0].mode == fui::BitmapMode::TileX && seen[0].color == fui::Color::White);

  // Empty rect or empty bitmap: not reported.
  target.bitmap({0, 0, 0, 8}, bits, fui::BitmapMode::Center);
  target.bitmap({0, 0, 8, 8}, fui::BitmapRef{}, fui::BitmapMode::Center);
  CHECK(seen.size() == 1);

  // Painting on with the sink: the picture is reported AND drawn (the sink watches; it does not swallow).
  target.setPaintingEnabled(true);
  target.bitmap({10, 10, 8, 8}, bits, fui::BitmapMode::Center);
  CHECK(seen.size() == 2 && renderer.pixels == 64);

  // Removed: back to silence.
  target.setBitmapSink(nullptr, nullptr);
  target.bitmap({10, 10, 8, 8}, bits, fui::BitmapMode::Center);
  CHECK(seen.size() == 2);

  // A real list with a label image, laid out with painting off: the sink gets the picture where the label stands.
  const fui::LabelImage name{{kBits, 8, 8, fui::BitmapFormat::BW1, false}, 6};
  fui::ListItem item;
  item.label = "Named";
  item.labelImage = &name;
  fui::ListProps props;
  props.items = &item;
  props.count = 1;
  props.rowHeight = 56;
  props.scrollIndicator = false;
  props.labelAscent = 20;
  seen.clear();
  renderer.pixels = renderer.texts = 0;
  target.setPaintingEnabled(false);
  target.setBitmapSink(collect, &seen);
  fui::DeviceContext device = target.deviceContext();
  fui::InputSnapshot input;
  fui::InteractionBuffer<4> hits;
  fui::Frame<4> frame(target, device, input, hits);
  fui::list(frame, fui::Rect{0, 0, 240, 100}, props);
  CHECK(renderer.pixels == 0 && renderer.texts == 0);
  CHECK(seen.size() == 1 && seen[0].data == kBits && seen[0].rect.x == 8 && seen[0].rect.width == 8);
  CHECK(seen.size() == 1 && seen[0].rect.y == (56 - 24) / 2 + 20 - 6);
  CHECK(seen.size() == 1 && seen[0].mode == fui::BitmapMode::TileX);

  if (failed) return 1;
  std::puts("test_gfx_target: ok");
  return 0;
}
