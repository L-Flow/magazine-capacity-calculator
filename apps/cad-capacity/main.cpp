#include "cad/CadImporter.hpp"
#include "cad/AssemblyPackingRegion.hpp"
#include "cad/CadPackingRegion.hpp"
#include "packing/LatticePacking.hpp"
#include "packing/QuasiStaticSettler.hpp"
#include "packing/Validation.hpp"

#include <cstdlib>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

namespace {

TopoDS_Face faceAt(const TopoDS_Shape& shape, std::size_t requestedIndex) {
    std::size_t index = 0;
    for (TopExp_Explorer it(shape, TopAbs_FACE); it.More(); it.Next()) {
        if (index++ == requestedIndex) return TopoDS::Face(it.Current());
    }
    return {};
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2 || argc > 10) {
        std::wcerr << L"usage: cad-capacity <model.step> [diameter-mm]\n"
                   << L"   or: cad-capacity --assembly <model.step> "
                      L"<face-index> <gx> <gy> <gz> [diameter-mm] [cell-mm] "
                      L"[--extract-only|--settle]\n";
        return EXIT_FAILURE;
    }
    try {
        const bool assemblyMode = std::wstring(argv[1]) == L"--assembly";
        const int pathIndex = assemblyMode ? 2 : 1;
        const auto model = magazine::cad::readCad(
            std::filesystem::path(argv[pathIndex]));
        if (assemblyMode) {
            if (argc < 7) {
                throw std::invalid_argument("assembly mode requires face and gravity");
            }
            const std::size_t faceIndex = std::stoull(argv[3]);
            const TopoDS_Face entry = faceAt(model.shape, faceIndex);
            if (entry.IsNull()) throw std::invalid_argument("face index is out of range");
            const gp_Dir gravity(std::stod(argv[4]), std::stod(argv[5]),
                                   std::stod(argv[6]));
            const double diameter = argc >= 8 ? std::stod(argv[7]) : 17.0;
            magazine::cad::AssemblyExtractionOptions extraction;
            if (argc >= 9) extraction.cellSizeMm = std::stod(argv[8]);
            const bool extractOnly = argc == 10 &&
                                     std::wstring(argv[9]) == L"--extract-only";
            const bool settleMode = argc == 10 &&
                                    std::wstring(argv[9]) == L"--settle";
            if (argc == 10 && !extractOnly && !settleMode) {
                throw std::invalid_argument("unknown assembly option");
            }
            const auto assembly = magazine::cad::makeAssemblyPackingRegion(
                model, gravity, entry, extraction);
            std::cout << std::fixed << std::setprecision(3)
                      << "source-solids: " << assembly.sourceSolidCount << '\n'
                      << "local-obstacles: " << assembly.obstacleCount << '\n'
                      << "classified-solids: " << assembly.classifiedObstacleCount << '\n'
                      << "reachable-cells: " << assembly.reachableCellCount << '\n'
                      << "cell-mm: " << assembly.cellSizeMm << '\n'
                      << "solver-bounds-mm: " << assembly.region.bounds.widthMm
                      << " x " << assembly.region.bounds.depthMm << " x "
                      << assembly.region.bounds.heightMm << '\n'
                      << std::flush;
            if (extractOnly) return EXIT_SUCCESS;
            const double radius = diameter / 2.0;
            if (settleMode) {
                magazine::packing::SettlingOptions options;
                options.failedInsertionsBeforeStop = 72;
                options.candidateTrialsPerSphere = 12;
                options.relaxationDirections = 10;
                options.maximumRelaxationIterations = 18;
                const auto start = std::chrono::steady_clock::now();
                const auto settled = magazine::packing::settleWithoutFriction(
                    assembly.region, radius, options);
                const auto elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count();
                const auto settledCheck = magazine::packing::validatePacking(
                    assembly.region, settled, 0.15);
                std::cout << "settled: " << settled.centers.size() << ", "
                          << settledCheck.message << ", elapsed-s: "
                          << elapsed << '\n';
            } else {
                magazine::packing::LatticeOptions latticeOptions;
                latticeOptions.phaseDivisions = 2;
                const auto lattice = magazine::packing::packBestFccOrHcp(
                    assembly.region, radius, latticeOptions);
                const auto check = magazine::packing::validatePacking(
                    assembly.region, lattice);
                std::cout << "lattice: " << lattice.centers.size() << " ("
                          << lattice.method << "), " << check.message << '\n';
            }
            return EXIT_SUCCESS;
        }
        const double diameter = argc == 3 ? std::stod(argv[2]) : 17.0;
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
