#pragma once

#include <set>
#include <vector>


namespace pmpc {
namespace controls {

/// Utility function used in ZOH and LERP parameterizations.
std::vector<int> processFreeSamples(const std::vector<int>& frees) {
  // order and remove the duplicates
  std::set<int> s( frees.begin(), frees.end() );
  // make sure that '0' is in the set
  s.insert(0);
  // return back the ordered result, as a vector
  return std::vector<int>(s.begin(), s.end());
}

} // namespace controls
} // namespace pmpc
