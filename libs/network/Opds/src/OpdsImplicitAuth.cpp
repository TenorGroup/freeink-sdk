#include "OpdsImplicitAuth.h"

#include <OpdsLog.h>

#if defined(FREEINK_NET_WOLFSSL)

#include <Arduino.h>
#include <HeapFloor.h>
#include <OpdsLoginForm.h>
#include <SecureHttpClient.h>

#include <algorithm>

namespace {

// A login chain is short: implicit endpoint -> SSO login page -> credential
// POST -> SSO handoff -> member sign-in -> opds:// callback.
constexpr int MAX_STEPS = 10;
// Login pages are small (a few KB); the cap bounds heap during auth.
constexpr size_t MAX_PAGE_BYTES = 16 * 1024;
constexpr int AUTH_TIMEOUT_MS = 30000;

// Add the OAuth parameters an implicit grant needs to the authorize URL.
// Authentication for OPDS 1.0 (3.4/3.5) defines a shared client identifier
// (http://opds-spec.org/auth/client) and callback (opds://authorize/) so a
// browserless client can complete the flow; standard OAuth servers (e.g.
// ebooks.com's IdentityServer) return 400 on the authorize endpoint without
// them. Parameters the server already put in the link are preserved.
void ensureImplicitAuthParams(std::string& url) {
  const auto hasParam = [&url](const char* name) {
    const std::string needle = std::string(name) + "=";
    for (size_t pos = url.find(needle); pos != std::string::npos; pos = url.find(needle, pos + 1)) {
      const char before = pos == 0 ? '?' : url[pos - 1];
      if (before == '?' || before == '&') return true;
    }
    return false;
  };
  const auto add = [&url](const char* nameValue) {
    url += url.find('?') == std::string::npos ? '?' : '&';
    url += nameValue;
  };
  if (!hasParam("response_type")) add("response_type=token");
  if (!hasParam("client_id")) add("client_id=http%3A%2F%2Fopds-spec.org%2Fauth%2Fclient");
  if (!hasParam("redirect_uri")) add("redirect_uri=opds%3A%2F%2Fauthorize%2F");
}

// Bare host of a URL (no scheme, no port) for cookie domain matching.
std::string hostOf(const std::string& url) {
  size_t start = url.find("://");
  start = start == std::string::npos ? 0 : start + 3;
  size_t end = url.find('/', start);
  if (end == std::string::npos) end = url.size();
  const size_t colon = url.find(':', start);
  if (colon != std::string::npos && colon < end) end = colon;
  return url.substr(start, end - start);
}

}  // namespace

bool opdsImplicitAuthenticate(const std::string& authenticateUrl, const std::string& username,
                              const std::string& password, std::string& outToken, const OpdsRootsFor& rootsFor) {
  OpdsCookieJar jar;
  std::string url = authenticateUrl;
  ensureImplicitAuthParams(url);
  std::string postBody;
  bool isPost = false;
  bool passwordSubmitted = false;
  bool wasTls = false;

  for (int step = 0; step < MAX_STEPS; ++step) {
    freeink::SecureHttpClient http;
    http.setTimeout(AUTH_TIMEOUT_MS);
    freeink::http_url::Parts parts;
    if (!freeink::http_url::parse(url, parts)) {
      LOG_ERR("OPDS", "Implicit auth: bad URL");
      return false;
    }
    if (parts.tls) {
      const char* roots = rootsFor ? rootsFor(url) : nullptr;
      if (!roots) {
        LOG_ERR("OPDS", "Implicit auth: no roots to verify the server");
        return false;
      }
      http.setCACert(roots);
    } else if (wasTls) {
      LOG_ERR("OPDS", "Implicit auth: refusing to step down from https to http");
      return false;
    }
    wasTls = parts.tls;
    if (!http.begin(url)) {
      LOG_ERR("OPDS", "Implicit auth: bad URL");
      return false;
    }
    http.setUserAgent("CrossPoint-ESP32-" CROSSPOINT_VERSION);
    const std::string host = hostOf(url);
    const std::string cookies = jar.headerFor(host);
    if (!cookies.empty()) http.addHeader("Cookie", cookies);
    if (isPost) http.addHeader("Content-Type", "application/x-www-form-urlencoded");

    // The page grows to MAX_PAGE_BYTES with the TLS session live. A block from 4 KB up is asked of the heap floor
    // (founder rule 20) before it is taken; a realloc holds the old block and the new one. Refused, the login stops
    // with nothing sent, where the string would take the heap under the floor or abort on a failed allocation.
    std::string page;
    bool noRoom = false;
    const auto onData = [&page, &noRoom](const uint8_t* data, size_t len) {
      const size_t take = std::min(len, MAX_PAGE_BYTES - page.size());
      if (page.size() + take > page.capacity()) {
        const size_t want = std::min(MAX_PAGE_BYTES, std::max(page.size() + take, 2 * page.capacity()));
        if (want >= heapfloor::kBigBytes &&
            !heapfloor::leaves(page.capacity() + want, want, ESP.getFreeHeap(), ESP.getMaxAllocHeap())) {
          noRoom = true;
          return false;
        }
        page.reserve(want);
      }
      page.append(reinterpret_cast<const char*>(data), take);
      return true;
    };
    const int status =
        isPost ? http.sendRequest("POST", reinterpret_cast<const uint8_t*>(postBody.data()), postBody.size(), onData)
               : http.GET(onData);
    if (noRoom) {
      LOG_ERR("OPDS", "Implicit auth: no room on the heap for the login page at step %d (free %u, largest %u)", step,
              static_cast<unsigned>(ESP.getFreeHeap()), static_cast<unsigned>(ESP.getMaxAllocHeap()));
      return false;
    }
    if (status < 0) {
      LOG_ERR("OPDS", "Implicit auth: request failed at step %d", step);
      return false;
    }
    for (const auto& header : http.getHeaders()) {
      if (header.first == "set-cookie") jar.store(host, header.second);
    }
    isPost = false;
    postBody.clear();

    if (status >= 300 && status < 400) {
      const std::string location = http.getHeader("location");
      if (location.empty()) return false;
      if (location.rfind("opds://", 0) == 0) {
        outToken = extractOpdsAccessToken(location);
        if (outToken.empty()) LOG_ERR("OPDS", "Implicit auth: callback without access_token");
        return !outToken.empty();
      }
      if (!freeink::SecureHttpClient::resolveUrl(url, location, url)) {
        LOG_ERR("OPDS", "Implicit auth: bad redirect");
        return false;
      }
      continue;
    }

    if (status == 200) {
      const OpdsLoginForm form = buildOpdsLoginForm(page.data(), page.size(), username, password);
      if (!form.found) {
        // Diagnostic: distinguish an empty/short body (redirect or cookie
        // issue) from a real page whose form we failed to parse.
        const bool hasForm = page.find("<form") != std::string::npos;
        const bool hasHtml = page.find("<html") != std::string::npos || page.find("<!DOCTYPE") != std::string::npos;
        LOG_ERR("OPDS", "Implicit auth: no login form found (%u bytes, <form>=%d <html>=%d)",
                static_cast<unsigned>(page.size()), hasForm, hasHtml);
        return false;
      }
      if (form.hasPassword && passwordSubmitted) {
        // The password page reappears after we submitted it: bad credentials.
        LOG_ERR("OPDS", "Implicit auth: credentials rejected");
        return false;
      }
      if (!form.action.empty() && !freeink::SecureHttpClient::resolveUrl(url, form.action, url)) {
        LOG_ERR("OPDS", "Implicit auth: bad form action");
        return false;
      }
      // Identifier-first logins (ebooks.com) show an email page, then a
      // password page; submit each in turn until the opds:// callback.
      if (form.hasPassword) passwordSubmitted = true;
      postBody = form.body;
      isPost = true;
      continue;
    }

    LOG_ERR("OPDS", "Implicit auth: unexpected status %d", status);
    return false;
  }
  LOG_ERR("OPDS", "Implicit auth: too many steps");
  return false;
}

#else

bool opdsImplicitAuthenticate(const std::string&, const std::string&, const std::string&, std::string&,
                              const OpdsRootsFor&) {
  LOG_ERR("OPDS", "Implicit OAuth requires the wolfSSL HTTP client");
  return false;
}

#endif
