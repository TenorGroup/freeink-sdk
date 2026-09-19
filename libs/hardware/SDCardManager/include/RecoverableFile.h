#pragma once
#include <cstring>
#include <string>

namespace freeink {
// Recover the previous committed file if interruption left only its backup.
// Staging is deliberately ignored: completion and validation belong to callers.
// FAT/card power failure may damage metadata; these renames are not a transaction.
template <typename Store>
bool recoverFile(Store& storage, const char* destination) {
  if (storage.exists(destination)) return true;
  const std::string backup = std::string(destination) + ".davbak";
  return !storage.exists(backup.c_str()) || storage.rename(backup.c_str(), destination);
}

// Call only after staging has passed write, sync, close and consumer validation.
// An existing backup blocks replacement until the consumer validates/reconciles it.
// A failed cleanup still means committed, and is reported separately to the caller.
template <typename Store>
bool replaceFile(Store& storage, const char* source, const char* destination,
                 bool* backupCleanupPending = nullptr) {
  if (backupCleanupPending) *backupCleanupPending = false;
  if (std::strcmp(source, destination) == 0 || !storage.exists(source)) return false;
  const std::string backup = std::string(destination) + ".davbak";
  if (storage.exists(backup.c_str())) return false;
  const bool existed = storage.exists(destination);
  if (existed && !storage.rename(destination, backup.c_str())) return false;
  if (!storage.rename(source, destination)) {
    if (existed) storage.rename(backup.c_str(), destination);
    return false;
  }
  if (existed && !storage.remove(backup.c_str()) && backupCleanupPending) *backupCleanupPending = true;
  return true;
}
}  // namespace freeink
