#include "cad/CadPackingRegion.hpp"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_ExtPF.hxx>
#include <BRepGProp.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomLProp_SLProps.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopAbs_State.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

namespace magazine::cad {
namespace {

struct PlaneConstraint {
    gp_Pnt point;
    gp_Dir outward;
};

struct CylinderConstraint {
    gp_Pnt location;
    gp_Dir axis;
    double radius{0.0};
    bool interiorIsInside{true};
    double axisMin{0.0};
    double axisMax{0.0};
};

struct FallbackCell {
    Bnd_Box bounds;
};

struct BoundaryFace {
    TopoDS_Face face;
    Bnd_Box bounds;
    GeomAbs_SurfaceType type{GeomAbs_OtherSurface};
    gp_Pnt planePoint;
    gp_Dir planeNormal;
    gp_Pnt cylinderLocation;
    gp_Dir cylinderAxis;
    double cylinderRadius{0.0};
    double cylinderAxisMin{0.0};
    double cylinderAxisMax{0.0};
};

struct BoundaryConstraints {
    std::shared_ptr<BRepClass3d_SolidClassifier> classifier;
    std::vector<BoundaryFace> faces;
    std::vector<PlaneConstraint> planes;
    std::vector<CylinderConstraint> cylinders;
    std::vector<FallbackCell> fallbackCells;
    bool analytic{true};
};

struct LocalFrame {
    gp_Dir x;
    gp_Dir y;
    gp_Dir up;
    gp_Trsf sourceToLocal;
};

double radialDistance(const gp_Pnt& point, const gp_Pnt& location,
                      const gp_Dir& axis) {
    const gp_Vec delta(location, point);
    const double axial = delta.Dot(gp_Vec(axis));
    const gp_Vec radial = delta - gp_Vec(axis) * axial;
    return radial.Magnitude();
}

gp_Dir faceOutwardNormal(const TopoDS_Face& face, const gp_Dir& fallback) {
    BRepAdaptor_Surface surface(face, true);
    gp_Dir normal = fallback;
    if (surface.GetType() == GeomAbs_Plane) {
        normal = surface.Plane().Axis().Direction();
    } else {
        Standard_Real u1 = 0.0;
        Standard_Real u2 = 0.0;
        Standard_Real v1 = 0.0;
        Standard_Real v2 = 0.0;
        u1 = surface.FirstUParameter();
        u2 = surface.LastUParameter();
        v1 = surface.FirstVParameter();
        v2 = surface.LastVParameter();
        const Handle(Geom_Surface) geometry = BRep_Tool::Surface(face);
        if (!geometry.IsNull() && std::isfinite(u1) && std::isfinite(u2) &&
            std::isfinite(v1) && std::isfinite(v2)) {
            GeomLProp_SLProps properties(geometry, 0.5 * (u1 + u2),
                                         0.5 * (v1 + v2), 1, 1.0e-9);
            if (properties.IsNormalDefined()) normal = properties.Normal();
        }
    }
    if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
    return normal;
}

LocalFrame makeLocalFrame(const CadImportResult& model,
                          const gp_Dir& gravityDirection,
                          const TopoDS_Face& entryFace) {
    gp_Vec up(-gravityDirection.X(), -gravityDirection.Y(),
              -gravityDirection.Z());
    if (up.SquareMagnitude() <= 1.0e-18) {
        throw std::invalid_argument("gravity direction must be non-zero");
    }
    up.Normalize();

    gp_Vec x;
    if (!entryFace.IsNull()) {
        const gp_Dir outward = faceOutwardNormal(entryFace, gp::DZ());
        x = gp_Vec(outward);
        x -= up * x.Dot(up);
    }
    if (x.SquareMagnitude() <= 1.0e-12) {
        x = gp_Vec(1.0, 0.0, 0.0);
        x -= up * x.Dot(up);
    }
    if (x.SquareMagnitude() <= 1.0e-12) {
        x = gp_Vec(0.0, 1.0, 0.0);
        x -= up * x.Dot(up);
    }
    if (x.SquareMagnitude() <= 1.0e-12) {
        x = gp_Vec(0.0, 0.0, 1.0);
        x -= up * x.Dot(up);
    }
    x.Normalize();
    gp_Vec y = up.Crossed(x);
    y.Normalize();

    LocalFrame frame{gp_Dir(x), gp_Dir(y), gp_Dir(up), gp_Trsf()};
    frame.sourceToLocal.SetValues(
        frame.x.X(), frame.x.Y(), frame.x.Z(), 0.0,
        frame.y.X(), frame.y.Y(), frame.y.Z(), 0.0,
        frame.up.X(), frame.up.Y(), frame.up.Z(), 0.0);

    const std::array<double, 2> xs{{model.minX, model.maxX}};
    const std::array<double, 2> ys{{model.minY, model.maxY}};
    const std::array<double, 2> zs{{model.minZ, model.maxZ}};
    double minX = std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double minZ = std::numeric_limits<double>::infinity();
    for (const double sourceX : xs) {
        for (const double sourceY : ys) {
            for (const double sourceZ : zs) {
                const gp_Pnt transformed = gp_Pnt(sourceX, sourceY, sourceZ)
                    .Transformed(frame.sourceToLocal);
                minX = std::min(minX, transformed.X());
                minY = std::min(minY, transformed.Y());
                minZ = std::min(minZ, transformed.Z());
            }
        }
    }
    frame.sourceToLocal.SetTranslationPart(gp_Vec(-minX, -minY, -minZ));
    return frame;
}

bool analyticSphereInside(const CadImportResult& model,
                          const BoundaryConstraints& constraints,
                          const magazine::packing::Vec3& center, double radius,
                          double tolerance) {
    const gp_Pnt sourceCenter(center.x + model.minX,
                              center.y + model.minY,
                              center.z + model.minZ);
    for (const PlaneConstraint& plane : constraints.planes) {
        const double clearance =
            gp_Vec(sourceCenter, plane.point).Dot(gp_Vec(plane.outward));
        if (clearance + tolerance < radius) return false;
    }
    for (const CylinderConstraint& cylinder : constraints.cylinders) {
        const gp_Vec axialDelta(cylinder.location, sourceCenter);
        const double axial = axialDelta.Dot(gp_Vec(cylinder.axis));
        if (axial + radius < cylinder.axisMin ||
            axial - radius > cylinder.axisMax) {
            continue;
        }
        const double radial = radialDistance(sourceCenter, cylinder.location,
                                             cylinder.axis);
        const double clearance = cylinder.interiorIsInside
            ? cylinder.radius - radial
            : radial - cylinder.radius;
        if (clearance + tolerance < radius) return false;
    }
    return true;
}

bool preciseSphereInside(const CadImportResult& model,
                         const BoundaryConstraints& constraints,
                         const magazine::packing::Vec3& center, double radius,
                         double tolerance) {
    const gp_Pnt sourceCenter(center.x + model.minX,
                              center.y + model.minY,
                              center.z + model.minZ);
    // Reject candidates that are visibly too close to analytic faces before
    // invoking the comparatively expensive solid classifier.
    Bnd_Box sphereBounds;
    sphereBounds.Update(sourceCenter.X() - radius - tolerance,
                        sourceCenter.Y() - radius - tolerance,
                        sourceCenter.Z() - radius - tolerance,
                        sourceCenter.X() + radius + tolerance,
                        sourceCenter.Y() + radius + tolerance,
                        sourceCenter.Z() + radius + tolerance);
    for (const BoundaryFace& boundary : constraints.faces) {
        if (boundary.type != GeomAbs_Plane &&
            boundary.type != GeomAbs_Cylinder) {
            continue;
        }
        Bnd_Box nearby = boundary.bounds;
        nearby.Enlarge(radius + tolerance);
        if (nearby.IsOut(sphereBounds)) continue;

        if (boundary.type == GeomAbs_Plane) {
            const double planeDistance = std::abs(
                gp_Vec(sourceCenter, boundary.planePoint)
                    .Dot(gp_Vec(boundary.planeNormal)));
            if (planeDistance < std::max(0.0, radius - tolerance)) {
                return false;
            }
            continue;
        }
        if (boundary.type == GeomAbs_Cylinder) {
            const gp_Vec axialDelta(boundary.cylinderLocation, sourceCenter);
            const double axial =
                axialDelta.Dot(gp_Vec(boundary.cylinderAxis));
            if (axial + radius < boundary.cylinderAxisMin ||
                axial - radius > boundary.cylinderAxisMax) {
                continue;
            }
            const double radial = radialDistance(
                sourceCenter, boundary.cylinderLocation,
                boundary.cylinderAxis);
            if (std::abs(radial - boundary.cylinderRadius) <
                std::max(0.0, radius - tolerance)) {
                return false;
            }
            continue;
        }

    }
    if (!constraints.classifier) return false;
    constraints.classifier->Perform(
        sourceCenter, std::max(1.0e-7, tolerance));
    if (constraints.classifier->State() != TopAbs_IN) return false;

    // A center can be inside a solid while the sphere still crosses a
    // non-analytic wall. Use OCCT extrema only for those uncommon faces.
    const TopoDS_Vertex centerVertex =
        BRepBuilderAPI_MakeVertex(sourceCenter).Vertex();
    const double clearanceSquared =
        std::pow(std::max(0.0, radius - tolerance), 2.0);
    for (const BoundaryFace& boundary : constraints.faces) {
        if (boundary.type == GeomAbs_Plane ||
            boundary.type == GeomAbs_Cylinder) {
            continue;
        }
        Bnd_Box nearby = boundary.bounds;
        nearby.Enlarge(radius + tolerance);
        if (nearby.IsOut(sphereBounds)) continue;
        BRepExtrema_ExtPF extrema(centerVertex, boundary.face,
                                  Extrema_ExtFlag_MIN, Extrema_ExtAlgo_Grad);
        if (!extrema.IsDone()) return false;
        double minimumDistanceSquared = std::numeric_limits<double>::infinity();
        for (int index = 1; index <= extrema.NbExt(); ++index) {
            minimumDistanceSquared =
                std::min(minimumDistanceSquared, extrema.SquareDistance(index));
        }
        if (minimumDistanceSquared < clearanceSquared) return false;
    }
    return true;
}

bool sphereInside(const CadImportResult& model,
                  const BoundaryConstraints& constraints,
                  const magazine::packing::Vec3& center, double radius,
                  double tolerance) {
    const bool analyticInside =
        constraints.analytic &&
        analyticSphereInside(model, constraints, center, radius, tolerance);
    if (analyticInside) {
        return true;
    }

    const gp_Pnt sourceCenter(center.x + model.minX,
                              center.y + model.minY,
                              center.z + model.minZ);
    for (const FallbackCell& cell : constraints.fallbackCells) {
        Bnd_Box candidateCell = cell.bounds;
        candidateCell.Enlarge(radius + tolerance);
        if (!candidateCell.IsOut(sourceCenter) &&
            analyticSphereInside(model, constraints, center, 0.0,
                                 tolerance)) {
            // The coarse cell is only a candidate index. The final decision
            // must use the exact solid and face-clearance checks below.
            return preciseSphereInside(model, constraints, center, radius,
                                       tolerance);
        }
    }
    return constraints.analytic
        ? false
        : preciseSphereInside(model, constraints, center, radius, tolerance);
}

CadPackingRegion makeLocalPackingRegion(const CadImportResult& model,
                                        const TopoDS_Face& entryFace,
                                        const gp_Dir& sourceGravity) {
    if (model.solidCount != 1 || model.shape.IsNull()) {
        throw std::invalid_argument(
            "CAD packing region requires exactly one non-null solid");
    }
    const magazine::packing::AxisAlignedBox bounds{
        model.widthMm(), model.depthMm(), model.heightMm()};
    BoundaryConstraints constraints;
    GProp_GProps volumeProperties;
    BRepGProp::VolumeProperties(model.shape, volumeProperties);
    const gp_Pnt interiorPoint = volumeProperties.CentreOfMass();
    for (TopExp_Explorer it(model.shape, TopAbs_FACE); it.More(); it.Next()) {
        const TopoDS_Face face = TopoDS::Face(it.Current());
        Bnd_Box faceBounds;
        BRepBndLib::Add(face, faceBounds);
        BRepAdaptor_Surface surface(face, true);
        if (surface.GetType() == GeomAbs_Plane) {
            gp_Dir normal = surface.Plane().Axis().Direction();
            if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
            constraints.planes.push_back({surface.Plane().Location(), normal});
            constraints.faces.push_back({face, faceBounds, GeomAbs_Plane,
                                         surface.Plane().Location(), normal});
        } else if (surface.GetType() == GeomAbs_Cylinder) {
            const gp_Cylinder cylinder = surface.Cylinder();
            const double referenceRadial = radialDistance(
                interiorPoint, cylinder.Location(), cylinder.Axis().Direction());
            double minAxis = std::numeric_limits<double>::infinity();
            double maxAxis = -std::numeric_limits<double>::infinity();
            double minX = 0.0;
            double minY = 0.0;
            double minZ = 0.0;
            double maxX = 0.0;
            double maxY = 0.0;
            double maxZ = 0.0;
            faceBounds.Get(minX, minY, minZ, maxX, maxY, maxZ);
            for (const double x : {minX, maxX}) {
                for (const double y : {minY, maxY}) {
                    for (const double z : {minZ, maxZ}) {
                        const double axial = gp_Vec(
                            cylinder.Location(), gp_Pnt(x, y, z))
                                                    .Dot(gp_Vec(cylinder.Axis().Direction()));
                        minAxis = std::min(minAxis, axial);
                        maxAxis = std::max(maxAxis, axial);
                    }
                }
            }
            constraints.cylinders.push_back(
                {cylinder.Location(), cylinder.Axis().Direction(),
                 cylinder.Radius(), referenceRadial < cylinder.Radius(),
                 minAxis, maxAxis});
            constraints.faces.push_back(
                {face, faceBounds, GeomAbs_Cylinder, gp_Pnt(), gp_Dir(),
                 cylinder.Location(), cylinder.Axis().Direction(),
                 cylinder.Radius(), minAxis, maxAxis});
        } else {
            constraints.analytic = false;
            constraints.faces.push_back({face, faceBounds, surface.GetType()});
        }
    }
    // Build the classifier once per imported/aligned solid.  Constructing a
    // new classifier for every lattice candidate repeatedly rebuilds OCCT's
    // solid explorer and makes realistic CAD models impractically slow.
    constraints.classifier =
        std::make_shared<BRepClass3d_SolidClassifier>(model.shape);

    // Probe the non-convex remainder once. The analytic half-space test is
    // deliberately conservative for a concave solid, so samples rejected by
    // it are checked with OCCT and retained as coarse fallback cells when
    // they are truly inside. Queries enlarge these cells by the active radius.
    if (constraints.analytic) {
        constexpr double sampleStep = 64.0;
        constexpr double maximumRadius = 21.0;
        const int countX = static_cast<int>(
            std::ceil(model.widthMm() / sampleStep));
        const int countY = static_cast<int>(
            std::ceil(model.depthMm() / sampleStep));
        const int countZ = static_cast<int>(
            std::ceil(model.heightMm() / sampleStep));
        for (int ix = 0; ix < countX; ++ix) {
            for (int iy = 0; iy < countY; ++iy) {
                for (int iz = 0; iz < countZ; ++iz) {
                    const double x = std::min(
                        model.widthMm(), (ix + 0.5) * sampleStep);
                    const double y = std::min(
                        model.depthMm(), (iy + 0.5) * sampleStep);
                    const double z = std::min(
                        model.heightMm(), (iz + 0.5) * sampleStep);
                    const magazine::packing::Vec3 sample{x, y, z};
                    if (analyticSphereInside(model, constraints, sample,
                                             maximumRadius, 1.0e-5)) {
                        continue;
                    }
                    const gp_Pnt sourceSample(x + model.minX,
                                              y + model.minY,
                                              z + model.minZ);
                    constraints.classifier->Perform(sourceSample, 1.0e-5);
                    if (constraints.classifier->State() != TopAbs_IN) {
                        continue;
                    }
                    Bnd_Box cell;
                    cell.Update(sourceSample.X() - sampleStep * 0.5,
                                sourceSample.Y() - sampleStep * 0.5,
                                sourceSample.Z() - sampleStep * 0.5,
                                sourceSample.X() + sampleStep * 0.5,
                                sourceSample.Y() + sampleStep * 0.5,
                                sourceSample.Z() + sampleStep * 0.5);
                    constraints.fallbackCells.push_back({cell});
                }
            }
        }
    }
    CadPackingRegion result;
    result.displayDepthMm = bounds.depthMm;
    result.region.bounds = bounds;
    result.region.containsSphere = [model, constraints](
        const magazine::packing::Vec3& center, double radius, double tolerance) {
        return sphereInside(model, constraints, center, radius, tolerance);
    };
    result.region.verticalInterval = [model, constraints, bounds](
        double x, double y, double radius)
        -> std::optional<magazine::packing::VerticalInterval> {
        if (x < radius || x > bounds.widthMm - radius ||
            y < radius || y > bounds.depthMm - radius) {
            return std::nullopt;
        }

        // Gravity has been transformed to -local Z.  Find the complete range
        // of sphere-center heights in this vertical column; this also handles
        // sloped, stepped, and cylindrical walls without assuming a box.
        const int scanSteps = 24;
        const double step = bounds.heightMm / scanSteps;
        auto valid = [&](double z) {
            return sphereInside(model, constraints, {x, y, z}, radius,
                                1.0e-5);
        };
        int first = -1;
        int last = -1;
        for (int index = 0; index <= scanSteps; ++index) {
            if (valid(index * step)) {
                if (first < 0) first = index;
                last = index;
            }
        }
        if (first < 0) return std::nullopt;

        auto refine = [&](double outside, double inside) {
            for (int iteration = 0; iteration < 18; ++iteration) {
                const double middle = 0.5 * (outside + inside);
                if (valid(middle)) inside = middle;
                else outside = middle;
            }
            return inside;
        };
        const double lower = first == 0
            ? 0.0
            : refine((first - 1) * step, first * step);
        const double upper = last == scanSteps
            ? bounds.heightMm
            : refine((last + 1) * step, last * step);
        if (upper + 1.0e-5 < lower) return std::nullopt;
        return magazine::packing::VerticalInterval{lower, upper};
    };

    if (!entryFace.IsNull()) {
        GProp_GProps faceProperties;
        BRepGProp::SurfaceProperties(entryFace, faceProperties);
        const gp_Pnt center = faceProperties.CentreOfMass();
        const gp_Dir outward = faceOutwardNormal(entryFace, gp::DZ());
        gp_Dir inward = outward;
        inward.Reverse();
        result.region.entryFace = magazine::packing::EntryFaceInfo{
            {center.X() - model.minX, center.Y() - model.minY,
             center.Z() - model.minZ},
            {inward.X(), inward.Y(), inward.Z()}};
        result.hasEntryFace = true;
        result.displayEntryFace = entryFace;
    }
    result.displayShape = model.shape;
    result.sourceGravity = sourceGravity;
    return result;
}

} // namespace

CadPackingRegion makeGravityAlignedPackingRegion(
    const CadImportResult& model, const gp_Dir& gravityDirection,
    const TopoDS_Face& entryFace) {
    const LocalFrame frame = makeLocalFrame(model, gravityDirection, entryFace);
    const TopoDS_Shape alignedShape =
        BRepBuilderAPI_Transform(model.shape, frame.sourceToLocal, true).Shape();

    CadImportResult aligned = model;
    aligned.shape = alignedShape;
    Bnd_Box alignedBounds;
    BRepBndLib::Add(aligned.shape, alignedBounds);
    if (alignedBounds.IsVoid()) {
        throw std::invalid_argument("CAD model has no measurable geometry");
    }
    alignedBounds.Get(aligned.minX, aligned.minY, aligned.minZ,
                      aligned.maxX, aligned.maxY, aligned.maxZ);

    TopoDS_Face alignedEntryFace;
    if (!entryFace.IsNull()) {
        const TopoDS_Shape transformedEntry =
            BRepBuilderAPI_Transform(entryFace, frame.sourceToLocal, true).Shape();
        if (!transformedEntry.IsNull() &&
            transformedEntry.ShapeType() == TopAbs_FACE) {
            alignedEntryFace = TopoDS::Face(transformedEntry);
        }
    }

    // The aligned shape already lives in the solver frame: +Z is up and -Z
    // is the user-selected gravity direction.  No CAD axis name remains in
    // the solver or in the visualization.
    return makeLocalPackingRegion(aligned, alignedEntryFace,
                                  gravityDirection);
}

CadPackingRegion makeVerticalPackingRegion(const CadImportResult& model) {
    return makeGravityAlignedPackingRegion(model, gp_Dir(0.0, -1.0, 0.0));
}

} // namespace magazine::cad
