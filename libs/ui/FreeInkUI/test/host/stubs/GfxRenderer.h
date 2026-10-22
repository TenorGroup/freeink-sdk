#pragma once
// A GfxRenderer with just what GfxRendererTarget calls, counting ink pixels, for test_gfx_target.cpp.
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>
enum class Color { White, Black, LightGray, DarkGray, Clear };
struct EpdFontFamily {
  enum Style { REGULAR = 0, BOLD = 1, SUP = 2, SUB = 4 };
  struct Glyph { int width = 8, height = 20, left = 0, top = 20; };
  const Glyph* getGlyph(uint32_t, Style) const { return nullptr; }
};
class GfxRenderer {
 public:
  mutable int pixels = 0, texts = 0;
  mutable std::array<int, 4> clip{0, 0, 480, 800};
  int getOrientation() const { return 0; }
  void getOrientedViewableTRBL(int* t, int* r, int* b, int* l) const { *t = *r = *b = *l = 0; }
  const std::map<int, EpdFontFamily>& getFontMap() const { static std::map<int, EpdFontFamily> m; return m; }
  int getScreenWidth() const { return 480; }
  int getScreenHeight() const { return 800; }
  auto getClipRect() const { return clip; }
  void setClipRect(int x, int y, int w, int h) const { clip = {x, y, w, h}; }
  int getLineHeight(int) const { return 24; }
  int getFontAscenderSize(int) const { return 20; }
  int getTextWidth(int, const char* s, EpdFontFamily::Style = EpdFontFamily::REGULAR) const { return strlen(s) * 8; }
  std::string truncatedText(int, const char* s, int, EpdFontFamily::Style) const { return s; }
  std::vector<std::string> wrappedText(int, const char* s, int, int, EpdFontFamily::Style) const { return {s}; }
  template <class... A> void fillRect(A...) const {}
  template <class... A> void fillRectDither(A...) const {}
  template <class... A> void fillRoundedRect(A...) const {}
  template <class... A> void drawRoundedRect(A...) const {}
  template <class... A> void drawRect(A...) const {}
  template <class... A> void drawLine(A...) const {}
  template <class... A> void fillPolygon(A...) const {}
  template <class... A> void drawText(A...) const { ++texts; }
  template <class... A> void drawTextRotated90CW(A...) const { ++texts; }
  void drawPixel(int, int, bool = true) const { ++pixels; }
};
