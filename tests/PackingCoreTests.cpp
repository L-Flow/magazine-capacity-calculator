#include "packing/LatticePacking.hpp"
#include "packing/QuasiStaticSettler.hpp"
#include "packing/Validation.hpp"

#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

} // namespace

int main() {
    using namespace magazine::packing;

    const AxisAlignedBox box{102.0, 85.0, 68.0};
    constexpr double radius = 8.5;

    const PackingResult fcc = packFcc(box, radius, {5});
    const PackingResult hcp = packHcp(box, radius, {5});
    require(!fcc.centers.empty(), "FCC result must contain spheres");
    require(!hcp.centers.empty(), "HCP result must contain spheres");
    require(validatePacking(box, fcc, 1.0e-4).allInside,
            "FCC spheres must stay inside");
    require(validatePacking(box, fcc, 1.0e-4).noOverlap,
            "FCC spheres must not overlap");
    require(validatePacking(box, hcp, 1.0e-4).allInside,
            "HCP spheres must stay inside");
    require(validatePacking(box, hcp, 1.0e-4).noOverlap,
            "HCP spheres must not overlap");

    SettlingOptions options;
    options.seed = 42;
    options.failedInsertionsBeforeStop = 60;
    options.candidateTrialsPerSphere = 16;
    options.maximumRelaxationIterations = 20;
    const PackingResult settled = settleWithoutFriction(box, radius, options);
    const ValidationReport validation = validatePacking(box, settled, 0.15);
    require(!settled.centers.empty(), "settling must accept spheres");
    require(validation.allInside, "settled spheres must stay inside");
    require(validation.noOverlap, "settled spheres must not overlap materially");
    require(settled.centers.size() <=
                std::max(fcc.centers.size(), hcp.centers.size()) + 8,
            "settling should remain near the lattice reference for a box");

    std::cout << "FCC=" << fcc.centers.size()
              << " HCP=" << hcp.centers.size()
              << " settled=" << settled.centers.size() << '\n';
    return 0;
}

