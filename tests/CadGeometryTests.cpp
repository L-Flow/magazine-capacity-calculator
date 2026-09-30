#include "cad/CadPackingRegion.hpp"
#include "packing/LatticePacking.hpp"
#include "packing/Validation.hpp"

#include <BRepPrimAPI_MakeBox.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS.hxx>

#include <cstdlib>
#include <cmath>
#include <iostream>

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

    std::cout << "z-bounds=" << zAligned.region.bounds.widthMm << 'x'
              << zAligned.region.bounds.depthMm << 'x'
              << zAligned.region.bounds.heightMm << " x-gravity-count="
              << packed.centers.size() << '\n';
    return 0;
}
