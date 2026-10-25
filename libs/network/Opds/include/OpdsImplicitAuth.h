#pragma once
#include <functional>
#include <string>

/**
 * Drive an OPDS OAuth implicit-grant login without a browser. Follows the
 * authentication document's "authenticate" URL, fills the first HTML login
 * form encountered with the stored credentials (carrying session cookies
 * across redirect hops, including federated SSO on another host), and
 * captures the access token from the opds://authorize callback redirect.
 *
 * Verified against Cantook/De Marque catalogs (Lirtuel + Samarcande SSO).
 * The password goes out in a form POST, so every https step is verified against the roots `rootsFor` gives
 * for that URL (nullptr: it cannot be verified, the login stops), and a step that would leave https for
 * http stops it too. Only available on wolfSSL builds; returns false elsewhere.
 */
using OpdsRootsFor = std::function<const char*(const std::string& url)>;
bool opdsImplicitAuthenticate(const std::string& authenticateUrl, const std::string& username,
                              const std::string& password, std::string& outToken, const OpdsRootsFor& rootsFor);
