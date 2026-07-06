/*
 * data_path.hpp
 *
 * Interim replacement for the retired FolderPath utility: resolves the
 * data root from XMNAV_DATA_DIR, falling back to ./data.
 * TODO(revive): make data locations explicit configuration instead of
 * a process-global path convention.
 */

#ifndef XMNAV_STATE_LATTICE_DATA_PATH_HPP
#define XMNAV_STATE_LATTICE_DATA_PATH_HPP

#include <cstdlib>
#include <string>

namespace xmotion {
inline std::string GetDataFolderPath() {
  const char *env = std::getenv("XMNAV_DATA_DIR");
  return env != nullptr ? std::string(env) : std::string("./data");
}
}  // namespace xmotion

#endif  // XMNAV_STATE_LATTICE_DATA_PATH_HPP
