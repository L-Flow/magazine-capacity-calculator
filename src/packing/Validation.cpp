#include "packing/Validation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace magazine::packing {

ValidationReport validatePacking(const AxisAlignedBox& box,
                                 const PackingResult& result,
                                 double allowedPenetrationMm) {
    return validatePacking(boxPackingRegion(box), result,
                           allowedPenetrationMm);
}

ValidationReport validatePacking(const PackingRegion& region,
                                 const PackingResult& result,
                                 double allowedPenetrationMm) {
    ValidationReport report;
    report.minimumCenterDistanceMm = result.centers.size() < 2
        ? 0.0
        : std::numeric_limits<double>::infinity();

    for (const Vec3& center : result.centers) {
        // Keep the wall classification strict. allowedPenetrationMm is an
        // overlap-reporting tolerance, not a license to place a sphere
        // outside the CAD solid.
        if (!region.containsSphere(center, result.sphereRadiusMm, 1.0e-5)) {
            report.allInside = false;
        }
    }

    const double diameter = 2.0 * result.sphereRadiusMm;
    for (std::size_t i = 0; i < result.centers.size(); ++i) {
        for (std::size_t j = i + 1; j < result.centers.size(); ++j) {
            const double distance =
                std::sqrt(squaredLength(result.centers[j] - result.centers[i]));
            report.minimumCenterDistanceMm =
                std::min(report.minimumCenterDistanceMm, distance);
            report.maximumPenetrationMm =
                std::max(report.maximumPenetrationMm, diameter - distance);
        }
    }
    report.maximumPenetrationMm = std::max(0.0, report.maximumPenetrationMm);
    report.noOverlap = report.maximumPenetrationMm <= allowedPenetrationMm;

    std::ostringstream message;
    message << "inside=" << (report.allInside ? "yes" : "no")
            << ", overlap=" << (report.noOverlap ? "no" : "yes")
            << ", max penetration=" << report.maximumPenetrationMm << " mm";
    report.message = message.str();
    return report;
}

} // namespace magazine::packing
