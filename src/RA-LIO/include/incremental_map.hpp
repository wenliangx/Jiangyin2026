#pragma once

#include <ikd-Tree/ikd_Tree.h>

#include "common_lib.hpp"

namespace ra_lio {

// Boundary around the upstream ikd-Tree implementation. Keeping the native type
// behind this adapter prevents its global macros and ownership model from spreading
// into the rest of the package while preserving its algorithm unchanged.
class IncrementalMap {
 public:
  using NativeMap = KD_TREE<Point>;

  [[nodiscard]] NativeMap& native() noexcept { return map_; }
  [[nodiscard]] const NativeMap& native() const noexcept { return map_; }
  [[nodiscard]] bool empty() const noexcept { return map_.Root_Node == nullptr; }

 private:
  NativeMap map_;
};

}  // namespace ra_lio
