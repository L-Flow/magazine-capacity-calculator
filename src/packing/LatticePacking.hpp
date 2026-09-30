#pragma once

#include "packing/PackingRegion.hpp"

namespace magazine::packing {

struct LatticeOptions {
    int phaseDivisions{6};
};

PackingResult packFcc(const AxisAlignedBox& box, double radius,
                      const LatticeOptions& options = {});
PackingResult packFcc(const PackingRegion& region, double radius,
                      const LatticeOptions& options = {});
PackingResult packHcp(const AxisAlignedBox& box, double radius,
                      const LatticeOptions& options = {});
PackingResult packHcp(const PackingRegion& region, double radius,
                      const LatticeOptions& options = {});
PackingResult packBestFccOrHcp(const AxisAlignedBox& box, double radius,
                               const LatticeOptions& options = {});
PackingResult packBestFccOrHcp(const PackingRegion& region, double radius,
                               const LatticeOptions& options = {});

} // namespace magazine::packing

