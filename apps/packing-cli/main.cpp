#include "packing/LatticePacking.hpp"
#include "packing/QuasiStaticSettler.hpp"
#include "packing/Validation.hpp"

#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>

using magazine::packing::AxisAlignedBox;
using magazine::packing::PackingResult;

namespace {

void printResult(const AxisAlignedBox& box, const PackingResult& result) {
    const auto validation = magazine::packing::validatePacking(box, result);
    std::cout << result.method << ": " << result.centers.size() << " spheres\n"
              << "  " << validation.message << "\n";
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 5 && argc != 6) {
        std::cerr << "usage: packing-cli <width-mm> <depth-mm> <height-mm> "
                     "<diameter-mm> [seed]\n";
        return 2;
    }

    try {
        const AxisAlignedBox box{std::stod(argv[1]), std::stod(argv[2]),
                                 std::stod(argv[3])};
        const double radius = std::stod(argv[4]) / 2.0;
        magazine::packing::SettlingOptions settling;
        if (argc == 6) {
            settling.seed = static_cast<std::uint64_t>(std::stoull(argv[5]));
        }

        std::cout << std::fixed << std::setprecision(3)
                  << "container: " << box.widthMm << " x " << box.depthMm
                  << " x " << box.heightMm << " mm\n"
                  << "sphere diameter: " << 2.0 * radius << " mm\n";

        const PackingResult fcc = magazine::packing::packFcc(box, radius);
        const PackingResult hcp = magazine::packing::packHcp(box, radius);
        const PackingResult settled =
            magazine::packing::settleWithoutFriction(box, radius, settling);
        printResult(box, fcc);
        printResult(box, hcp);
        printResult(box, settled);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}

