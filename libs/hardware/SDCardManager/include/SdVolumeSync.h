#pragma once
// Writes out the sectors SdFat holds in its caches: the data/directory sector and, with
// USE_SEPARATE_FAT_CACHE, the FAT sector. Ending a volume does not (FsVolume::end() only forgets
// it, FatPartition::end() leaves the FAT cache), so a card that loses power after end() can
// lose FAT and directory updates. A file's sync() writes both caches of its volume
// (FatFile::sync, ExFatFile::sync), and the root directory is always there to call it on.
// A template so host tests run the same code over SdFat's FatVolume.
template <typename Volume>
bool syncSdVolume(Volume& volume) {
  auto root = volume.open("/");
  const bool ok = root && root.sync();
  root.close();
  return ok;
}
