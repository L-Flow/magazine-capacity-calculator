#pragma once

#include "packing/PackingRegion.hpp"

#include <functional>

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
    // A bounded deterministic pass repairs holes that pure random insertion
    // can leave behind in narrow CAD pockets after it reaches a local jam.
    std::size_t systematicSweepPasses{3};
    std::size_t systematicSweepMaximumCandidates{1600};
    double systematicSweepSpacingDiameterFactor{0.75};
    // Zero keeps the library API unlimited. Interactive CAD callers set a
    // finite budget so a difficult STEP assembly cannot run indefinitely.
    std::uint64_t maximumRuntimeMilliseconds{0};
    // Optional cooperative cancellation hook for interactive callers.
    std::function<bool()> cancellationRequested;
};

PackingResult settleWithoutFriction(const AxisAlignedBox& box, double radius,
                                    const SettlingOptions& options = {});
PackingResult settleWithoutFriction(const PackingRegion& region, double radius,
                                    const SettlingOptions& options = {});

} // namespace magazine::packing
