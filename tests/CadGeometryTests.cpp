#include "cad/CadPackingRegion.hpp"
#include "cad/AssemblyPackingRegion.hpp"
#include "packing/LatticePacking.hpp"
#include "packing/Validation.hpp"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Builder.hxx>
#include <Bnd_Box.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

magazine::cad::CadImportResult makeBoxModel() {
    magazine::cad::CadImportResult model;
    model.path = "synthetic-box";
    model.format = "STEP";
    model.shape = BRepPrimAPI_MakeBox(40.0, 30.0, 20.0).Shape();
    model.minX = 0.0;
    model.minY = 0.0;
    model.minZ = 0.0;
    model.maxX = 40.0;
    model.maxY = 30.0;
    model.maxZ = 20.0;
    model.solidCount = 1;
    model.shellCount = 1;
    model.faceCount = 6;
    return model;
}

TopoDS_Face firstFace(const TopoDS_Shape& shape) {
    for (TopExp_Explorer it(shape, TopAbs_FACE); it.More(); it.Next()) {
        return TopoDS::Face(it.Current());
    }
    return {};
}

TopoDS_Face lowestFaceAtZ(const TopoDS_Shape& shape, double z) {
    for (TopExp_Explorer it(shape, TopAbs_FACE); it.More(); it.Next()) {
        const TopoDS_Face face = TopoDS::Face(it.Current());
        Bnd_Box bounds;
        BRepBndLib::Add(face, bounds);
        double minX = 0.0;
        double minY = 0.0;
        double minZ = 0.0;
        double maxX = 0.0;
        double maxY = 0.0;
        double maxZ = 0.0;
        bounds.Get(minX, minY, minZ, maxX, maxY, maxZ);
        if (std::abs(minZ - z) < 1.0e-6 &&
            std::abs(maxZ - z) < 1.0e-6) {
            return face;
        }
    }
    return {};
}

TopoDS_Face faceAtPlane(const TopoDS_Shape& shape, char axis, double value) {
    for (TopExp_Explorer it(shape, TopAbs_FACE); it.More(); it.Next()) {
        const TopoDS_Face face = TopoDS::Face(it.Current());
        Bnd_Box bounds;
        BRepBndLib::Add(face, bounds);
        double minX = 0.0;
        double minY = 0.0;
        double minZ = 0.0;
        double maxX = 0.0;
        double maxY = 0.0;
        double maxZ = 0.0;
        bounds.Get(minX, minY, minZ, maxX, maxY, maxZ);
        const bool matches = axis == 'x'
            ? std::abs(minX - value) < 1.0e-4 &&
                  std::abs(maxX - value) < 1.0e-4
            : axis == 'y'
                  ? std::abs(minY - value) < 1.0e-4 &&
                        std::abs(maxY - value) < 1.0e-4
                  : std::abs(minZ - value) < 1.0e-4 &&
                        std::abs(maxZ - value) < 1.0e-4;
        if (matches) return face;
    }
    return {};
}

} // namespace

int main() {
    using namespace magazine::packing;
    const auto model = makeBoxModel();
    const TopoDS_Face entry = firstFace(model.shape);
    require(!entry.IsNull(), "synthetic model must have a selectable face");

    const auto zAligned = magazine::cad::makeGravityAlignedPackingRegion(
        model, gp_Dir(0.0, 0.0, -1.0), entry);
    require(zAligned.hasEntryFace, "Z gravity must preserve selected face");
    require(std::abs(zAligned.region.bounds.widthMm - 40.0) < 1.0e-6 &&
                std::abs(zAligned.region.bounds.depthMm - 30.0) < 1.0e-6 &&
                std::abs(zAligned.region.bounds.heightMm - 20.0) < 1.0e-6,
            "Z gravity must preserve box dimensions");

    const auto xAligned = magazine::cad::makeGravityAlignedPackingRegion(
        model, gp_Dir(1.0, 0.0, 0.0), entry);
    require(xAligned.hasEntryFace, "X gravity must preserve selected face");
    require(xAligned.region.bounds.validFor(4.0),
            "X gravity region must remain valid");
    const auto packed = packBestFccOrHcp(xAligned.region, 4.0, {2});
    const auto validation = validatePacking(xAligned.region, packed);
    require(!packed.centers.empty(), "X gravity packing must contain spheres");
    require(validation.allInside && validation.noOverlap,
            "X gravity packing must validate");

    BRep_Builder builder;
    TopoDS_Compound assembly;
    builder.MakeCompound(assembly);
    builder.Add(assembly, BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, 0.0),
                                               40.0, 40.0, 4.0).Shape());
    builder.Add(assembly, BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, 0.0),
                                               4.0, 40.0, 40.0).Shape());
    builder.Add(assembly, BRepPrimAPI_MakeBox(gp_Pnt(36.0, 0.0, 0.0),
                                               4.0, 40.0, 40.0).Shape());
    builder.Add(assembly, BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, 0.0),
                                               40.0, 4.0, 40.0).Shape());
    builder.Add(assembly, BRepPrimAPI_MakeBox(gp_Pnt(0.0, 36.0, 0.0),
                                               40.0, 4.0, 40.0).Shape());
    builder.Add(assembly, BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, 36.0),
                                               40.0, 40.0, 4.0).Shape());
    magazine::cad::CadImportResult assemblyModel;
    assemblyModel.shape = assembly;
    assemblyModel.minX = 0.0;
    assemblyModel.minY = 0.0;
    assemblyModel.minZ = 0.0;
    assemblyModel.maxX = 40.0;
    assemblyModel.maxY = 40.0;
    assemblyModel.maxZ = 40.0;
    assemblyModel.solidCount = 6;
    assemblyModel.shellCount = 6;
    assemblyModel.faceCount = 36;
    const auto assemblyEntry = lowestFaceAtZ(assembly, 36.0);
    require(!assemblyEntry.IsNull(), "assembly test must have a ceiling entry face");
    magazine::cad::AssemblyExtractionOptions extraction;
    extraction.cellSizeMm = 4.0;
    extraction.maximumCells = 100000;
    const auto extracted = magazine::cad::makeAssemblyPackingRegion(
        assemblyModel, gp_Dir(0.0, 0.0, -1.0), assemblyEntry, extraction);
    require(extracted.reachableCellCount > 0,
            "assembly extraction must find the cavity component");
    require(extracted.region.containsSphere({20.0, 20.0, 20.0}, 3.0, 1.0e-5),
            "assembly cavity center must accept a sphere");
    // The extracted region is cropped to the reachable component, whose
    // local Z origin is the original cavity floor at 4 mm.
    require(!extracted.region.containsSphere({20.0, 20.0, 1.0}, 3.0, 1.0e-5),
            "assembly floor must reject a penetrating sphere");

    magazine::cad::AssemblyExtractionOptions pointExtraction = extraction;
    pointExtraction.entryPointSource = gp_Pnt(20.0, 20.0, 36.0);
    const auto pointSelected = magazine::cad::makeAssemblyPackingRegion(
        assemblyModel, gp_Dir(0.0, 0.0, -1.0), gp_Pnt(20.0, 20.0, 20.0),
        pointExtraction);
    require(pointSelected.region.containsSphere({20.0, 20.0, 20.0}, 3.0,
                                                 1.0e-5),
            "point-selected assembly must preserve the selected cavity");
    require(!pointSelected.region.containsSphere({20.0, 20.0, 1.0}, 3.0,
                                                  1.0e-5),
            "point-selected assembly must reject the cavity floor");

    std::vector<TopoDS_Face> boundaryFaces{
        faceAtPlane(assembly, 'x', 4.0),
        faceAtPlane(assembly, 'x', 36.0),
        faceAtPlane(assembly, 'y', 4.0),
        faceAtPlane(assembly, 'y', 36.0),
        faceAtPlane(assembly, 'z', 4.0),
        faceAtPlane(assembly, 'z', 36.0),
    };
    for (const auto& face : boundaryFaces) {
        require(!face.IsNull(),
                "boundary-face test must find every selected cavity wall");
    }
    const auto faceSelected = magazine::cad::makeAssemblyPackingRegion(
        assemblyModel, gp_Dir(0.0, 0.0, -1.0), boundaryFaces, extraction);
    require(faceSelected.region.containsSphere({20.0, 20.0, 20.0}, 3.0,
                                                1.0e-5),
            "multi-face assembly selection must preserve the cavity center");
    require(!faceSelected.region.containsSphere({2.0, 20.0, 20.0}, 3.0,
                                                 1.0e-5) &&
                !faceSelected.region.containsSphere({20.0, 2.0, 20.0}, 3.0,
                                                     1.0e-5) &&
                !faceSelected.region.containsSphere({20.0, 20.0, 2.0}, 3.0,
                                                     1.0e-5),
            "multi-face assembly selection must reject space outside the walls");
    require(faceSelected.boundarySolidCount == boundaryFaces.size(),
            "every selected boundary face must map to a retained source solid");
    require(!faceSelected.displayShape.IsNull(),
            "selected boundary solids must remain visible after extraction");

    // The GUI supplies both the selected boundary faces and a free-space
    // point. The explicit point must select the cavity component without
    // weakening the one-sided wall constraints.
    magazine::cad::AssemblyExtractionOptions faceAndPointExtraction = extraction;
    faceAndPointExtraction.seedPointSource = gp_Pnt(20.0, 20.0, 20.0);
    faceAndPointExtraction.entryPointSource = gp_Pnt(20.0, 20.0, 36.0);
    const auto faceAndPointSelected = magazine::cad::makeAssemblyPackingRegion(
        assemblyModel, gp_Dir(0.0, 0.0, -1.0), boundaryFaces,
        faceAndPointExtraction);
    require(faceAndPointSelected.region.containsSphere(
                {20.0, 20.0, 20.0}, 3.0, 1.0e-5),
            "boundary plus explicit point must preserve the selected cavity");
    require(!faceAndPointSelected.region.containsSphere(
                {2.0, 20.0, 20.0}, 3.0, 1.0e-5),
            "boundary plus explicit point must retain wall exclusion");

    // A single finite wall is a valid hint for the same cavity. Its local
    // plane must not become an infinite slab that collapses the component.
    const auto singleWall = magazine::cad::makeAssemblyPackingRegion(
        assemblyModel, gp_Dir(0.0, 0.0, -1.0),
        std::vector<TopoDS_Face>{faceAtPlane(assembly, 'x', 4.0)}, extraction);
    require(singleWall.region.containsSphere({20.0, 20.0, 20.0}, 3.0,
                                              1.0e-5),
            "single finite boundary face must preserve the cavity center");
    require(singleWall.region.bounds.widthMm > 16.0 &&
                singleWall.region.bounds.depthMm > 16.0,
            "single finite boundary face must not collapse the cavity to a strip");

    // Regression: a selected finite boundary must stop an open cavity from
    // flooding into the vehicle exterior. The test enclosure is deliberately
    // open at the top and a distant dummy solid extends the grid above the
    // selected faces; the exterior point must still be rejected.
    BRep_Builder openBuilder;
    TopoDS_Compound openAssembly;
    openBuilder.MakeCompound(openAssembly);
    openBuilder.Add(openAssembly, BRepPrimAPI_MakeBox(
        gp_Pnt(0.0, 0.0, 0.0), 40.0, 40.0, 4.0).Shape());
    openBuilder.Add(openAssembly, BRepPrimAPI_MakeBox(
        gp_Pnt(0.0, 0.0, 0.0), 4.0, 40.0, 40.0).Shape());
    openBuilder.Add(openAssembly, BRepPrimAPI_MakeBox(
        gp_Pnt(36.0, 0.0, 0.0), 4.0, 40.0, 40.0).Shape());
    openBuilder.Add(openAssembly, BRepPrimAPI_MakeBox(
        gp_Pnt(0.0, 0.0, 0.0), 40.0, 4.0, 40.0).Shape());
    openBuilder.Add(openAssembly, BRepPrimAPI_MakeBox(
        gp_Pnt(0.0, 36.0, 0.0), 40.0, 4.0, 40.0).Shape());
    openBuilder.Add(openAssembly, BRepPrimAPI_MakeBox(
        gp_Pnt(100.0, 100.0, 196.0), 2.0, 2.0, 2.0).Shape());
    magazine::cad::CadImportResult openModel;
    openModel.shape = openAssembly;
    openModel.minX = 0.0;
    openModel.minY = 0.0;
    openModel.minZ = 0.0;
    openModel.maxX = 102.0;
    openModel.maxY = 102.0;
    openModel.maxZ = 198.0;
    openModel.solidCount = 6;
    openModel.shellCount = 6;
    openModel.faceCount = 31;
    std::vector<TopoDS_Face> openBoundaryFaces{
        faceAtPlane(openAssembly, 'x', 4.0),
        faceAtPlane(openAssembly, 'x', 36.0),
        faceAtPlane(openAssembly, 'y', 4.0),
        faceAtPlane(openAssembly, 'y', 36.0),
        faceAtPlane(openAssembly, 'z', 4.0),
    };
    const auto openExtracted = magazine::cad::makeAssemblyPackingRegion(
        openModel, gp_Dir(0.0, 0.0, -1.0), openBoundaryFaces, extraction);
    require(openExtracted.region.containsSphere({20.0, 20.0, 20.0}, 3.0,
                                                 1.0e-5),
            "open boundary enclosure must preserve its cavity");
    require(!openExtracted.region.containsSphere({20.0, 20.0, 100.0}, 3.0,
                                                  1.0e-5),
            "finite selected boundary must reject exterior air above cavity");
    require(!openExtracted.region.containsSphere({20.0, 20.0, 39.0}, 3.0,
                                                  1.0e-5),
            "open cavity must reject a sphere protruding above its selected rim");

    std::cout << "z-bounds=" << zAligned.region.bounds.widthMm << 'x'
              << zAligned.region.bounds.depthMm << 'x'
              << zAligned.region.bounds.heightMm << " x-gravity-count="
              << packed.centers.size() << '\n';
    return 0;
}
