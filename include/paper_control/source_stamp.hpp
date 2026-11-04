#pragma once
#include <cstdint>

namespace paper_control {
// Transport-neutral metadata. CapabilityMapper only copies this value;
// its velocity/acceleration computations do not read the timestamp.
struct SourceStamp {
  std::uint32_t sec{0};
  std::uint32_t nsec{0};
};
}  // namespace paper_control

