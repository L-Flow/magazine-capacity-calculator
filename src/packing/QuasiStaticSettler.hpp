#pragma once

#include "packing/PackingRegion.hpp"

namespace magazine::packing {

struct SettlingOptions {
    std::uint64_t seed{20260929};
    std::size_t failedInsertionsBeforeStop{180};
    std::size_t candidateTrialsPerSphere{24};
    int relaxationDirections{16};
    int maximumRelaxationIterations{28};
    double initialLateralStepRadiusFactor{0.65};
    double minimumLateralStepMm{0.03};
    double improvementToleranceMm{1.0e-5};
};

PackingResult settleWithoutFriction(const AxisAlignedBox& box, double radius,
                                    const SettlingOptions& options = {});
PackingResult settleWithoutFriction(const PackingRegion& region, double radius,
                                    const SettlingOptions& options = {});

} // namespace magazine::packing

