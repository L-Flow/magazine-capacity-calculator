#pragma once

#include "cad/CadImporter.hpp"
#include "packing/PackingRegion.hpp"

#include <TopoDS_Face.hxx>
#include <gp_Dir.hxx>

namespace magazine::cad {

struct CadPackingRegion {
    magazine::packing::PackingRegion region;
    TopoDS_Shape displayShape;
    TopoDS_Face displayEntryFace;
    double displayDepthMm{0.0};
    gp_Dir sourceGravity{0.0, -1.0, 0.0};
    bool hasEntryFace{false};
};

CadPackingRegion makeGravityAlignedPackingRegion(
    const CadImportResult& model, const gp_Dir& gravityDirection,
    const TopoDS_Face& entryFace = TopoDS_Face());

// Compatibility wrapper for the original model convention: source -Y is
// gravity and the source model is already upright.
CadPackingRegion makeVerticalPackingRegion(const CadImportResult& model);

} // namespace magazine::cad
