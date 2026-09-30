#include "packing/LatticePacking.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace magazine::packing {
namespace {

int lowerIndex(double shift, double period) {
    return static_cast<int>(std::floor((-shift) / period)) - 1;
}

int upperIndex(double extent, double shift, double period) {
    return static_cast<int>(std::ceil((extent - shift) / period)) + 1;
}

void validate(const PackingRegion& region, double radius,
              const LatticeOptions& options) {
    if (!region.validFor(radius)) {
        throw std::invalid_argument("packing region is invalid for the selected sphere");
    }
    if (options.phaseDivisions < 1) {
        throw std::invalid_argument("phaseDivisions must be positive");
    }
}

} // namespace

PackingResult packFcc(const AxisAlignedBox& box, double radius,
                      const LatticeOptions& options) {
    return packFcc(boxPackingRegion(box), radius, options);
}

PackingResult packFcc(const PackingRegion& region, double radius,
                      const LatticeOptions& options) {
    validate(region, radius, options);
    const AxisAlignedBox& box = region.bounds;
    const double cell = 2.0 * std::sqrt(2.0) * radius;
    const std::array<Vec3, 4> basis{{
        {0.0, 0.0, 0.0},
        {0.0, cell / 2.0, cell / 2.0},
        {cell / 2.0, 0.0, cell / 2.0},
        {cell / 2.0, cell / 2.0, 0.0},
    }};

    PackingResult best{"FCC ideal reference", radius, {}, 0, 0};
    const int phases = options.phaseDivisions;
    for (int px = 0; px < phases; ++px) {
        for (int py = 0; py < phases; ++py) {
            for (int pz = 0; pz < phases; ++pz) {
                const Vec3 shift{
                    radius + cell * static_cast<double>(px) / phases,
                    radius + cell * static_cast<double>(py) / phases,
                    radius + cell * static_cast<double>(pz) / phases,
                };
                std::vector<Vec3> centers;
                for (int ix = lowerIndex(shift.x, cell);
                     ix <= upperIndex(box.widthMm, shift.x, cell); ++ix) {
                    for (int iy = lowerIndex(shift.y, cell);
                         iy <= upperIndex(box.depthMm, shift.y, cell); ++iy) {
                        for (int iz = lowerIndex(shift.z, cell);
                             iz <= upperIndex(box.heightMm, shift.z, cell); ++iz) {
                            const Vec3 origin{shift.x + ix * cell,
                                              shift.y + iy * cell,
                                              shift.z + iz * cell};
                            for (const Vec3& offset : basis) {
                                const Vec3 center = origin + offset;
                                if (region.containsSphere(center, radius, 1.0e-7)) {
                                    centers.push_back(center);
                                }
                            }
                        }
                    }
                }
                if (centers.size() > best.centers.size()) {
                    best.centers = std::move(centers);
                }
            }
        }
    }
    return best;
}

PackingResult packHcp(const AxisAlignedBox& box, double radius,
                      const LatticeOptions& options) {
    return packHcp(boxPackingRegion(box), radius, options);
}

PackingResult packHcp(const PackingRegion& region, double radius,
                      const LatticeOptions& options) {
    validate(region, radius, options);
    const AxisAlignedBox& box = region.bounds;
    const double xPeriod = 2.0 * radius;
    const double rowSpacing = std::sqrt(3.0) * radius;
    const double layerSpacing = 2.0 * radius * std::sqrt(2.0 / 3.0);
    const double zPeriod = 2.0 * layerSpacing;

    PackingResult best{"HCP ideal reference", radius, {}, 0, 0};
    const int phases = options.phaseDivisions;
    for (int px = 0; px < phases; ++px) {
        for (int py = 0; py < phases; ++py) {
            for (int pz = 0; pz < phases; ++pz) {
                const double shiftX = radius + xPeriod * px / phases;
                const double shiftY = radius + rowSpacing * py / phases;
                const double shiftZ = radius + zPeriod * pz / phases;
                std::vector<Vec3> centers;

                for (int layer = lowerIndex(shiftZ, layerSpacing);
                     layer <= upperIndex(box.heightMm, shiftZ, layerSpacing);
                     ++layer) {
                    const bool layerB = (layer & 1) != 0;
                    const double z = shiftZ + layer * layerSpacing;
                    const double layerX = layerB ? radius : 0.0;
                    const double layerY = layerB ? rowSpacing / 3.0 : 0.0;
                    for (int row = lowerIndex(shiftY + layerY, rowSpacing);
                         row <= upperIndex(box.depthMm, shiftY + layerY,
                                           rowSpacing);
                         ++row) {
                        const double y = shiftY + layerY + row * rowSpacing;
                        const double rowX = (row & 1) != 0 ? radius : 0.0;
                        for (int column = lowerIndex(shiftX + layerX + rowX,
                                                     xPeriod);
                             column <= upperIndex(box.widthMm,
                                                  shiftX + layerX + rowX,
                                                  xPeriod);
                             ++column) {
                            const Vec3 center{
                                shiftX + layerX + rowX + column * xPeriod,
                                y,
                                z,
                            };
                            if (region.containsSphere(center, radius, 1.0e-7)) {
                                centers.push_back(center);
                            }
                        }
                    }
                }
                if (centers.size() > best.centers.size()) {
                    best.centers = std::move(centers);
                }
            }
        }
    }
    return best;
}

PackingResult packBestFccOrHcp(const AxisAlignedBox& box, double radius,
                               const LatticeOptions& options) {
    return packBestFccOrHcp(boxPackingRegion(box), radius, options);
}

PackingResult packBestFccOrHcp(const PackingRegion& region, double radius,
                               const LatticeOptions& options) {
    PackingResult fcc = packFcc(region, radius, options);
    PackingResult hcp = packHcp(region, radius, options);
    return hcp.centers.size() > fcc.centers.size() ? std::move(hcp)
                                                   : std::move(fcc);
}

} // namespace magazine::packing

