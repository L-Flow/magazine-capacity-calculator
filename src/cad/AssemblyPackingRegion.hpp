#pragma once

#include "cad/CadImporter.hpp"
#include "packing/PackingRegion.hpp"

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

#include <cstddef>
#include <optional>
#include <vector>

namespace magazine::cad {

struct AssemblyExtractionOptions {
    // This grid is used for connectivity only. Final sphere clearance is
    // checked against the original OCCT geometry (lazily triangulated per
    // nearby solid, with an OCCT fallback for malformed parts).
    double cellSizeMm{8.0};
    double obstacleInflationMm{0.5};
    double seedOffsetMm{24.0};
    double maximumSeedSearchMm{240.0};
    // A selected assembly face is a reference for the physical opening, not
    // necessarily the opening itself.  Its projection plus this margin is
    // used as a finite virtual gate so outside air cannot flood the cavity.
    double entryApertureMarginMm{48.0};
    double minimumEntryApertureWidthMm{120.0};
    double minimumEntryApertureDepthMm{160.0};
    // Full vehicle assemblies commonly contain hundreds of unrelated parts.
    // Only parts within this cube around the selected entry are relevant to
    // the extracted magazine component.
    double obstacleRangeMm{200.0};
    std::size_t maximumCells{600000};
    // Point-picking mode supplies a free-space seed and, when available, the
    // near boundary of the finite free-space segment under the mouse ray.
    // Both points use the imported model coordinates.
    std::optional<gp_Pnt> seedPointSource;
    std::optional<gp_Pnt> entryPointSource;
};

struct AssemblyPackingRegion {
    magazine::packing::PackingRegion region;
    TopoDS_Shape displayShape;
    TopoDS_Face displayEntryFace;
    std::size_t obstacleCount{0};
    std::size_t sourceSolidCount{0};
    std::size_t classifiedObstacleCount{0};
    std::size_t reachableCellCount{0};
    std::size_t selectedBoundaryFaceCount{0};
    std::size_t boundaryConstraintCount{0};
    // Number of distinct solids owning selected boundary faces that were
    // forced to remain in the local obstacle/display set.
    std::size_t boundarySolidCount{0};
    double cellSizeMm{0.0};
    // True when strict tangential-footprint intersection was empty and the
    // extractor had to use a seed-localized selected-face envelope.
    bool usedTangentialEnvelopeFallback{false};
    bool usedLocalizedFallbackWindow{false};
    bool sourceTopologyValid{true};
};

// Treat the imported assembly as material obstacles and extract the free
// component behind the selected entry face. Gravity points from the entry
// towards the magazine, so it also supplies the seed search direction. The
// selected face projection is expanded into a finite virtual gate to keep
// exterior air from being mistaken for the magazine cavity.
AssemblyPackingRegion makeAssemblyPackingRegion(
    const CadImportResult& model, const gp_Dir& gravityDirection,
    const TopoDS_Face& entryFace,
    const AssemblyExtractionOptions& options = {});

// Extract the assembly component containing a user-selected free-space point.
// The optional entryPointSource in options becomes the finite virtual gate
// reference when the view could identify a near boundary.
AssemblyPackingRegion makeAssemblyPackingRegion(
    const CadImportResult& model, const gp_Dir& gravityDirection,
    const gp_Pnt& seedPointSource,
    const AssemblyExtractionOptions& options = {});

// Extract the assembly free-space component constrained by a user-confirmed
// set of boundary faces. The selected faces are treated as geometric walls;
// their common free side supplies the local seed and their projected envelope
// prevents the flood fill from escaping into unrelated vehicle cavities.
AssemblyPackingRegion makeAssemblyPackingRegion(
    const CadImportResult& model, const gp_Dir& gravityDirection,
    const std::vector<TopoDS_Face>& boundaryFaces,
    const AssemblyExtractionOptions& options = {});

} // namespace magazine::cad
