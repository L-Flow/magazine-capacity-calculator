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
#include <sstream>
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

std::vector<TopoDS_Face> facesAt(const TopoDS_Shape& shape,
                                 const std::wstring& csv) {
    std::vector<TopoDS_Face> faces;
    std::wstringstream stream(csv);
    std::wstring token;
    while (std::getline(stream, token, L',')) {
        if (token.empty()) continue;
        const std::size_t displayedIndex = std::stoull(token);
        if (displayedIndex == 0) {
            throw std::invalid_argument("displayed STEP face indices start at 1");
        }
        const TopoDS_Face face = faceAt(shape, displayedIndex - 1);
        if (face.IsNull()) throw std::invalid_argument("face index is out of range");
        faces.push_back(face);
    }
    if (faces.empty()) throw std::invalid_argument("no face indices supplied");
    return faces;
}

} // namespace

int wmain(int argc, wchar_t** argv) {
    if (argc < 2 || argc > 16) {
        std::wcerr << L"usage: cad-capacity <model.step> [diameter-mm]\n"
                   << L"   or: cad-capacity --assembly <model.step> "
                      L"<face-index> <gx> <gy> <gz> [diameter-mm] [cell-mm] "
                      L"[--extract-only|--settle]\n"
                   << L"   or: cad-capacity --assembly-faces <model.step> "
                      L"<face-index,face-index,...> <gx> <gy> <gz> "
                      L"<seed-x> <seed-y> <seed-z> "
                      L"<entry-x> <entry-y> <entry-z> [diameter-mm] [cell-mm] "
                      L"[--extract-only]\n";
        return EXIT_FAILURE;
    }
    try {
        const bool assemblyMode = std::wstring(argv[1]) == L"--assembly";
        const bool assemblyFacesMode = std::wstring(argv[1]) == L"--assembly-faces";
        const int pathIndex = assemblyMode ? 2 : 1;
        if (assemblyFacesMode) {
            if (argc < 13 || argc > 16) throw std::invalid_argument(
                "assembly-faces mode requires faces, gravity, seed, and entry point");
            const auto model = magazine::cad::readCad(
                std::filesystem::path(argv[2]));
            const auto faces = facesAt(model.shape, argv[3]);
            const gp_Dir gravity(std::stod(argv[4]), std::stod(argv[5]),
                                 std::stod(argv[6]));
            magazine::cad::AssemblyExtractionOptions extraction;
            extraction.seedPointSource = gp_Pnt(std::stod(argv[7]),
                                                std::stod(argv[8]),
                                                std::stod(argv[9]));
            extraction.entryPointSource = gp_Pnt(std::stod(argv[10]),
                                                 std::stod(argv[11]),
                                                 std::stod(argv[12]));
            const double diameter = argc >= 14 ? std::stod(argv[13]) : 17.0;
            if (argc >= 15) extraction.cellSizeMm = std::stod(argv[14]);
            const bool extractOnly = argc == 16 &&
                                     std::wstring(argv[15]) == L"--extract-only";
            if (argc == 16 && !extractOnly) {
                throw std::invalid_argument("unknown assembly-faces option");
            }
            const auto assembly = magazine::cad::makeAssemblyPackingRegion(
                model, gravity, faces, extraction);
            std::cout << std::fixed << std::setprecision(3)
                      << "source-solids: " << assembly.sourceSolidCount << '\n'
                      << "local-obstacles: " << assembly.obstacleCount << '\n'
                      << "reachable-cells: " << assembly.reachableCellCount << '\n'
                      << "selected-faces: " << assembly.selectedBoundaryFaceCount << '\n'
                      << "solver-bounds-mm: " << assembly.region.bounds.widthMm
                      << " x " << assembly.region.bounds.depthMm << " x "
                      << assembly.region.bounds.heightMm << '\n';
            if (extractOnly) return EXIT_SUCCESS;
            const double radius = diameter / 2.0;
            magazine::packing::LatticeOptions latticeOptions;
            latticeOptions.phaseDivisions = 2;
            const auto lattice = magazine::packing::packBestFccOrHcp(
                assembly.region, radius, latticeOptions);
            const auto check = magazine::packing::validatePacking(
                assembly.region, lattice);
            std::cout << "lattice: " << lattice.centers.size() << " ("
                      << lattice.method << "), " << check.message << '\n';
            return EXIT_SUCCESS;
        }
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
                options.failedInsertionsBeforeStop = 96;
                options.candidateTrialsPerSphere = 16;
                options.relaxationDirections = 12;
                options.maximumRelaxationIterations = 20;
                options.systematicSweepPasses = 2;
                options.systematicSweepMaximumCandidates = 600;
                options.systematicSweepSpacingDiameterFactor = 0.95;
                options.maximumRuntimeMilliseconds = 20000;
                const auto start = std::chrono::steady_clock::now();
                const auto settled = magazine::packing::settleWithoutFriction(
                    assembly.region, radius, options);
                const auto elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count();
                const auto settledCheck = magazine::packing::validatePacking(
                    assembly.region, settled, 0.15);
                std::cout << "settled: " << settled.centers.size() << ", "
                          << settledCheck.message << ", elapsed-s: "
                          << elapsed
                          << (settled.stoppedByTimeLimit
                                  ? ", stopped-by-time-limit=yes"
                                  : ", stopped-by-time-limit=no")
                          << '\n';
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
        options.failedInsertionsBeforeStop = 96;
        options.candidateTrialsPerSphere = 16;
        options.systematicSweepPasses = 2;
        options.systematicSweepMaximumCandidates = 600;
        options.systematicSweepSpacingDiameterFactor = 0.95;
        options.maximumRuntimeMilliseconds = 20000;
        const auto settled = magazine::packing::settleWithoutFriction(
            cad.region, radius, options);
        const auto settledCheck = magazine::packing::validatePacking(
            cad.region, settled, 0.15);
        std::cout << "settled: " << settled.centers.size() << ", "
                  << settledCheck.message
                  << (settled.stoppedByTimeLimit
                          ? ", stopped-by-time-limit=yes"
                          : ", stopped-by-time-limit=no")
                  << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "cad-capacity failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
