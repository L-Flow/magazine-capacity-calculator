#pragma once

#include "packing/PackingRegion.hpp"

#include <string>

namespace magazine::packing {

struct ValidationReport {
    bool allInside{true};
    bool noOverlap{true};
    double minimumCenterDistanceMm{0.0};
    double maximumPenetrationMm{0.0};
    std::string message;
};

ValidationReport validatePacking(const AxisAlignedBox& box,
                                 const PackingResult& result,
                                 double allowedPenetrationMm = 0.05);
ValidationReport validatePacking(const PackingRegion& region,
                                 const PackingResult& result,
                                 double allowedPenetrationMm = 0.05);

} // namespace magazine::packing

