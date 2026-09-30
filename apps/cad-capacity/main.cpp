#include "cad/CadImporter.hpp"
#include "cad/CadPackingRegion.hpp"
#include "packing/LatticePacking.hpp"
#include "packing/QuasiStaticSettler.hpp"
#include "packing/Validation.hpp"

#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>

int wmain(int argc, wchar_t** argv) {
    if (argc < 2 || argc > 3) {
        std::wcerr << L"usage: cad-capacity <model.step> [diameter-mm]\n";
        return EXIT_FAILURE;
    }
    try {
        const double diameter = argc == 3 ? std::stod(argv[2]) : 17.0;
        const auto model = magazine::cad::readCad(std::filesystem::path(argv[1]));
        const auto cad = magazine::cad::makeVerticalPackingRegion(model);
        const double radius = diameter / 2.0;
        magazine::packing::LatticeOptions latticeOptions;
        latticeOptions.phaseDivisions = 2;
        const auto lattice = magazine::packing::packBestFccOrHcp(
            cad.region, radius, latticeOptions);
        const auto latticeCheck = magazine::packing::validatePacking(
            cad.region, lattice);
        std::cout << std::fixed << std::setprecision(3)
                  << "diameter-mm: " << diameter << '\n'
                  << "solver-bounds-mm: " << cad.region.bounds.widthMm << " x "
                  << cad.region.bounds.depthMm << " x "
                  << cad.region.bounds.heightMm << '\n'
                  << "lattice: " << lattice.centers.size() << " ("
                  << lattice.method << "), " << latticeCheck.message << '\n';

        magazine::packing::SettlingOptions options;
        options.failedInsertionsBeforeStop = 80;
        options.candidateTrialsPerSphere = 12;
        const auto settled = magazine::packing::settleWithoutFriction(
            cad.region, radius, options);
        const auto settledCheck = magazine::packing::validatePacking(
            cad.region, settled, 0.15);
        std::cout << "settled: " << settled.centers.size() << ", "
                  << settledCheck.message << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "cad-capacity failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
