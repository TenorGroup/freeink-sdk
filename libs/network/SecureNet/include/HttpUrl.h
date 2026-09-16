#pragma once

#include <cstdint>
#include <string_view>

namespace freeink::http_url {
struct Parts {
  std::string_view scheme;
  std::string_view host;
  std::string_view target;
  uint16_t port = 0;
  bool tls = false;
};
constexpr bool equalFolded(std::string_view a, std::string_view b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i) {
    const char x = a[i] >= 'A' && a[i] <= 'Z' ? a[i] + 32 : a[i];
    const char y = b[i] >= 'A' && b[i] <= 'Z' ? b[i] + 32 : b[i];
    if (x != y) return false;
  }
  return true;
}
inline bool parse(std::string_view url, Parts& out) {
  out = {};
  for (unsigned char c : url)
    if (c <= 32 || c == 127 || c == '\\') return false;
  const auto sep = url.find("://");
  if (sep == std::string_view::npos) return false;
  out.scheme = url.substr(0, sep);
  out.tls = equalFolded(out.scheme, "https");
  if (!out.tls && !equalFolded(out.scheme, "http")) return false;
  const auto begin = sep + 3;
  auto end = url.find_first_of("/?#", begin);
  if (end == std::string_view::npos) end = url.size();
  const auto authority = url.substr(begin, end - begin);
  const auto colon = authority.find(':');
  out.host = authority.substr(0, colon);
  if (out.host.empty()) return false;
  for (unsigned char c : out.host) {
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-' || c == '.'))
      return false;
  }
  out.port = out.tls ? 443 : 80;
  if (colon != std::string_view::npos) {
    const auto port = authority.substr(colon + 1);
    if (port.empty() || port.size() > 5) return false;
    uint32_t value = 0;
    for (char c : port) {
      if (c < '0' || c > '9') return false;
      value = value * 10 + (c - '0');
    }
    if (!value || value > 65535) return false;
    out.port = static_cast<uint16_t>(value);
  }
  const auto fragment = url.find('#', end);
  out.target = url.substr(end, fragment == std::string_view::npos ? url.size() - end : fragment - end);
  return true;
}
inline bool sameOrigin(std::string_view a, std::string_view b) {
  Parts x, y;
  return parse(a, x) && parse(b, y) && x.tls == y.tls && x.port == y.port && equalFolded(x.host, y.host);
}
inline bool redirectAllowed(std::string_view from, std::string_view to) {
  Parts x, y;
  return parse(from, x) && parse(to, y) && (!x.tls || y.tls);
}
}  // namespace freeink::http_url
