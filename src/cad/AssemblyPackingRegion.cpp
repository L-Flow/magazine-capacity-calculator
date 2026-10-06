#include "cad/AssemblyPackingRegion.hpp"

#include <BRepBndLib.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomLProp_SLProps.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_State.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace magazine::cad {
namespace {

struct CellKey {
    int x{0};
    int y{0};
    int z{0};

    bool operator==(const CellKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct CellKeyHash {
    std::size_t operator()(const CellKey& key) const {
        std::size_t hash = static_cast<std::size_t>(key.x) * 73856093U;
        hash ^= static_cast<std::size_t>(key.y) * 19349663U;
        hash ^= static_cast<std::size_t>(key.z) * 83492791U;
        return hash;
    }
};

struct LocalFrame {
    gp_Dir x;
    gp_Dir y;
    gp_Dir up;
    gp_Trsf sourceToLocal;
};

struct Triangle {
    gp_Pnt a;
    gp_Pnt b;
    gp_Pnt c;
};

struct Obstacle {
    TopoDS_Shape shape;
    Bnd_Box bounds;
    double minX{0.0};
    double minY{0.0};
    double minZ{0.0};
    double maxX{0.0};
    double maxY{0.0};
    double maxZ{0.0};
    mutable std::shared_ptr<BRepClass3d_SolidClassifier> classifier;
    mutable std::vector<Triangle> triangles;
    mutable bool meshReady{false};
    // A solid that owns one of the user-confirmed boundary faces must remain
    // available even when the local obstacle crop would otherwise discard it.
    // These parts are both displayed and retained as collision obstacles.
    bool boundaryEssential{false};
};

struct FaceConstraint {
    gp_Pnt point;
    gp_Dir normal;
    Bnd_Box bounds;
    int normalAxis{2};
};

double pointSegmentDistanceSquared(const gp_Pnt& point, const gp_Pnt& start,
                                   const gp_Pnt& end) {
    const gp_Vec segment(start, end);
    const double lengthSquared = segment.SquareMagnitude();
    if (lengthSquared <= 1.0e-18) {
        return point.SquareDistance(start);
    }
    const double parameter =
        std::clamp(gp_Vec(start, point).Dot(segment) / lengthSquared,
                   0.0, 1.0);
    return point.SquareDistance(start.Translated(segment * parameter));
}

double pointTriangleDistanceSquared(const gp_Pnt& point,
                                    const Triangle& triangle) {
    const gp_Vec ab(triangle.a, triangle.b);
    const gp_Vec ac(triangle.a, triangle.c);
    const gp_Vec ap(triangle.a, point);
    const double d1 = ab.Dot(ap);
    const double d2 = ac.Dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) return point.SquareDistance(triangle.a);

    const gp_Vec bp(triangle.b, point);
    const double d3 = ab.Dot(bp);
    const double d4 = ac.Dot(bp);
    if (d3 >= 0.0 && d4 <= d3) return point.SquareDistance(triangle.b);

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double parameter = d1 / (d1 - d3);
        return point.SquareDistance(
            triangle.a.Translated(ab * parameter));
    }

    const gp_Vec cp(triangle.c, point);
    const double d5 = ab.Dot(cp);
    const double d6 = ac.Dot(cp);
    if (d6 >= 0.0 && d5 <= d6) return point.SquareDistance(triangle.c);

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double parameter = d2 / (d2 - d6);
        return point.SquareDistance(
            triangle.a.Translated(ac * parameter));
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const gp_Vec bc(triangle.b, triangle.c);
        const double parameter = (d4 - d3) /
                                 ((d4 - d3) + (d5 - d6));
        return point.SquareDistance(
            triangle.b.Translated(bc * parameter));
    }

    const gp_Vec normal = ab.Crossed(ac);
    const double normalSquared = normal.SquareMagnitude();
    if (normalSquared <= 1.0e-18) {
        return std::min({pointSegmentDistanceSquared(point, triangle.a,
                                                     triangle.b),
                         pointSegmentDistanceSquared(point, triangle.a,
                                                     triangle.c),
                         pointSegmentDistanceSquared(point, triangle.b,
                                                     triangle.c)});
    }
    const double signedDistance = normal.Dot(ap);
    return signedDistance * signedDistance / normalSquared;
}

void ensureObstacleMesh(Obstacle& obstacle) {
    if (obstacle.meshReady) return;
    obstacle.meshReady = true;
    try {
        BRepMesh_IncrementalMesh mesher(obstacle.shape, 0.5, Standard_False,
                                        0.5, Standard_True);
        for (TopExp_Explorer it(obstacle.shape, TopAbs_FACE); it.More();
             it.Next()) {
            const TopoDS_Face face = TopoDS::Face(it.Current());
            TopLoc_Location location;
            const Handle(Poly_Triangulation) triangulation =
                BRep_Tool::Triangulation(face, location);
            if (triangulation.IsNull()) continue;
            for (int index = 1; index <= triangulation->NbTriangles();
                 ++index) {
                int nodeA = 0;
                int nodeB = 0;
                int nodeC = 0;
                triangulation->Triangle(index).Get(nodeA, nodeB, nodeC);
                obstacle.triangles.push_back({
                    triangulation->Node(nodeA).Transformed(
                        location.Transformation()),
                    triangulation->Node(nodeB).Transformed(
                        location.Transformation()),
                    triangulation->Node(nodeC).Transformed(
                        location.Transformation())});
            }
        }
    } catch (const Standard_Failure&) {
        obstacle.triangles.clear();
    }
}

struct AssemblyGrid {
    magazine::packing::AxisAlignedBox bounds;
    // World-space origin of the cropped local region.  The flood fill starts
    // in the aligned assembly coordinates; after extraction all packing
    // coordinates are expressed relative to this origin.
    gp_Vec origin{0.0, 0.0, 0.0};
    double cellSize{8.0};
    double obstacleInflation{0.5};
    int nx{0};
    int ny{0};
    int nz{0};
    gp_Pnt entryPoint;
    // In multi-face mode the selected faces usually describe the side/bottom
    // walls, while the actual opening is the upper edge of that envelope.
    // Keep a separate gate point so the selected-face centroid can still be
    // used as the seed/reference point without allowing flood fill to escape
    // through an open top.
    gp_Pnt gatePoint;
    gp_Dir gateNormal;
    bool boundaryMode{false};
    bool hasBoundaryBounds{false};
    double boundaryMinX{0.0};
    double boundaryMinY{0.0};
    double boundaryMinZ{0.0};
    double boundaryMaxX{0.0};
    double boundaryMaxY{0.0};
    double boundaryMaxZ{0.0};
    double boundaryPadding{0.0};
    bool enforceGlobalBoundaryPlanes{false};
    // A strict multi-face selection intersects every face's tangential
    // footprint.  STEP tessellation, split faces, and tiny modeling gaps can
    // make that intersection empty even when the selected planes do enclose
    // a real cavity. Extraction may retry with a seed-localized envelope
    // while retaining the global one-sided plane constraints.
    bool enforceTangentialIntersection{true};
    // When the exact intersection of all selected-face tangent footprints is
    // empty, keep the connected interval that contains the user seed on each
    // axis instead of reverting to the union of every selected face.  This is
    // important for assemblies where the selection contains split faces or
    // faces from two separated pockets.
    bool localizedBoundaryWindow{false};
    std::array<double, 3> localizedMinimumSpan{{0.0, 0.0, 0.0}};
    std::array<bool, 2> unconstrainedHorizontalAxis{{false, false}};
    std::array<bool, 2> unconstrainedFarUpper{{false, false}};
    double apertureMinX{-std::numeric_limits<double>::infinity()};
    double apertureMaxX{std::numeric_limits<double>::infinity()};
    double apertureMinY{-std::numeric_limits<double>::infinity()};
    double apertureMaxY{std::numeric_limits<double>::infinity()};
    std::vector<unsigned char> reachable;
    std::vector<Obstacle> obstacles;
    std::vector<std::vector<std::size_t>> obstacleCandidates;
    std::vector<FaceConstraint> boundaryConstraints;

    std::size_t index(int x, int y, int z) const {
        return (static_cast<std::size_t>(z) *
                    static_cast<std::size_t>(ny) +
                static_cast<std::size_t>(y)) *
                   static_cast<std::size_t>(nx) +
               static_cast<std::size_t>(x);
    }

    bool inBounds(int x, int y, int z) const {
        return x >= 0 && x < nx && y >= 0 && y < ny && z >= 0 && z < nz;
    }

    gp_Pnt center(int x, int y, int z) const {
        return gp_Pnt(origin.X() + (x + 0.5) * cellSize,
                      origin.Y() + (y + 0.5) * cellSize,
                      origin.Z() + (z + 0.5) * cellSize);
    }

    bool onMagazineSide(const gp_Pnt& point) const {
        const gp_Vec fromEntry(boundaryMode ? gatePoint : entryPoint, point);
        // A whole extra grid cell above the opening lets flood fill walk
        // around a wall's top edge and then back into exterior vehicle air.
        // Sphere clearance applies its own radius margin separately.
        return fromEntry.Dot(gp_Vec(gateNormal)) >= -1.0e-6;
    }

    bool insideBoundaryEnvelope(const gp_Pnt& point) const {
        for (const auto& constraint : boundaryConstraints) {
            const gp_Vec fromFace(constraint.point, point);
            if (fromFace.Dot(gp_Vec(constraint.normal)) >= -1.0e-4) continue;

            // Once the user confirms multiple faces, they describe the walls
            // of one target cavity. Applying their one-sided planes globally
            // prevents a flood path from going around a finite face edge into
            // the surrounding vehicle. A single selected face remains a local
            // hint and keeps the finite-footprint behavior used by point mode.
            if (enforceGlobalBoundaryPlanes) return false;

            // A selected CAD face is finite. Applying its plane to the entire
            // assembly incorrectly turns a local face patch into an infinite
            // wall and can collapse a real cavity into a thin band. Only use
            // the one-sided constraint while the point is within the face's
            // tangential footprint. The actual solid geometry remains the
            // authoritative collision boundary outside that footprint.
            if (constraint.bounds.IsVoid()) return false;
            double minX = 0.0;
            double minY = 0.0;
            double minZ = 0.0;
            double maxX = 0.0;
            double maxY = 0.0;
            double maxZ = 0.0;
            constraint.bounds.Get(minX, minY, minZ, maxX, maxY, maxZ);
            const double tangentPadding = 2.0 * cellSize;
            const double coordinates[3] = {point.X(), point.Y(), point.Z()};
            const double lower[3] = {minX, minY, minZ};
            const double upper[3] = {maxX, maxY, maxZ};
            bool insideTangentialFootprint = true;
            for (int axis = 0; axis < 3; ++axis) {
                if (axis == constraint.normalAxis) continue;
                if (coordinates[axis] < lower[axis] - tangentPadding ||
                    coordinates[axis] > upper[axis] + tangentPadding) {
                    insideTangentialFootprint = false;
                    break;
                }
            }
            if (insideTangentialFootprint) return false;
        }
        return true;
    }

    bool withinAperture(const gp_Pnt& point, double margin = 0.0) const {
        return point.X() >= apertureMinX + margin &&
               point.X() <= apertureMaxX - margin &&
               point.Y() >= apertureMinY + margin &&
               point.Y() <= apertureMaxY - margin;
    }

    bool withinBoundaryBox(const gp_Pnt& point, double margin = 0.0,
                           bool ignoreTangentialIntersection = false) const {
        if (!boundaryMode || !hasBoundaryBounds) return true;

        if (enforceGlobalBoundaryPlanes && !boundaryConstraints.empty() &&
            enforceTangentialIntersection && !ignoreTangentialIntersection) {
            // The union AABB of selected faces is a poor cavity proxy: a long
            // chassis plate can enlarge it to most of the vehicle.  In
            // confirmed multi-face mode each face contributes only the two
            // tangential extents; their intersection is the finite window
            // shared by the selected cavity walls.  The plane inequalities in
            // insideBoundaryEnvelope() constrain the remaining normal axes.
            const double tangentPadding = 2.0 * cellSize;
            const double coordinates[3] = {point.X(), point.Y(), point.Z()};
            for (const auto& constraint : boundaryConstraints) {
                if (constraint.bounds.IsVoid()) return false;
                double minX = 0.0;
                double minY = 0.0;
                double minZ = 0.0;
                double maxX = 0.0;
                double maxY = 0.0;
                double maxZ = 0.0;
                constraint.bounds.Get(minX, minY, minZ, maxX, maxY, maxZ);
                const double lower[3] = {minX, minY, minZ};
                const double upper[3] = {maxX, maxY, maxZ};
                for (int axis = 0; axis < 3; ++axis) {
                    if (axis == constraint.normalAxis) continue;
                    if (coordinates[axis] < lower[axis] - tangentPadding + margin ||
                        coordinates[axis] > upper[axis] + tangentPadding - margin) {
                        return false;
                    }
                }
            }
            return true;
        }

        return point.X() >= boundaryMinX - boundaryPadding + margin &&
               point.X() <= boundaryMaxX + boundaryPadding - margin &&
               point.Y() >= boundaryMinY - boundaryPadding + margin &&
               point.Y() <= boundaryMaxY + boundaryPadding - margin &&
               point.Z() >= boundaryMinZ - boundaryPadding + margin &&
               point.Z() <= boundaryMaxZ + boundaryPadding - margin;
    }

    bool sphereOnMagazineSide(const gp_Pnt& point, double radius) const {
        const gp_Vec fromEntry(boundaryMode ? gatePoint : entryPoint, point);
        return fromEntry.Dot(gp_Vec(gateNormal)) >= radius - 1.0e-6;
    }

    bool pointInsideObstacle(const gp_Pnt& point) const {
        auto testObstacle = [&](const Obstacle& obstacle) {
            if (point.X() < obstacle.minX - obstacleInflation ||
                point.X() > obstacle.maxX + obstacleInflation ||
                point.Y() < obstacle.minY - obstacleInflation ||
                point.Y() > obstacle.maxY + obstacleInflation ||
                point.Z() < obstacle.minZ - obstacleInflation ||
                point.Z() > obstacle.maxZ + obstacleInflation) {
                return false;
            }
            if (!obstacle.classifier) {
                try {
                    obstacle.classifier =
                        std::make_shared<BRepClass3d_SolidClassifier>(
                            TopoDS::Solid(obstacle.shape));
                } catch (const Standard_Failure&) {
                    // Keep malformed parts for exact distance checks.  They
                    // cannot be used for point classification.
                    return false;
                }
            }
            try {
                obstacle.classifier->Perform(point, Precision::Confusion());
                const TopAbs_State state = obstacle.classifier->State();
                return state == TopAbs_IN || state == TopAbs_ON;
            } catch (const Standard_Failure&) {
                // The exact sphere query below remains the final guard. A
                // malformed part must not abort the whole assembly analysis.
            }
            return false;
        };

        const int x = static_cast<int>(std::floor(
            (point.X() - origin.X()) / cellSize));
        const int y = static_cast<int>(std::floor(
            (point.Y() - origin.Y()) / cellSize));
        const int z = static_cast<int>(std::floor(
            (point.Z() - origin.Z()) / cellSize));
        if (inBounds(x, y, z) && !obstacleCandidates.empty()) {
            const auto& candidates = obstacleCandidates[index(x, y, z)];
            for (const std::size_t obstacleIndex : candidates) {
                if (obstacleIndex < obstacles.size() &&
                    testObstacle(obstacles[obstacleIndex])) {
                    return true;
                }
            }
            return false;
        }
        for (const Obstacle& obstacle : obstacles) {
            if (testObstacle(obstacle)) return true;
        }
        return false;
    }

    void rebuildObstacleIndex() {
        obstacleCandidates.clear();
        if (nx <= 0 || ny <= 0 || nz <= 0 || obstacles.empty()) return;
        obstacleCandidates.resize(
            static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) *
            static_cast<std::size_t>(nz));
        for (std::size_t obstacleIndex = 0; obstacleIndex < obstacles.size();
             ++obstacleIndex) {
            const Obstacle& obstacle = obstacles[obstacleIndex];
            const int minX = std::clamp(
                static_cast<int>(std::floor(
                    (obstacle.minX - obstacleInflation - origin.X()) /
                    cellSize)),
                0, nx - 1);
            const int minY = std::clamp(
                static_cast<int>(std::floor(
                    (obstacle.minY - obstacleInflation - origin.Y()) /
                    cellSize)),
                0, ny - 1);
            const int minZ = std::clamp(
                static_cast<int>(std::floor(
                    (obstacle.minZ - obstacleInflation - origin.Z()) /
                    cellSize)),
                0, nz - 1);
            const int maxX = std::clamp(
                static_cast<int>(std::floor(
                    (obstacle.maxX + obstacleInflation - origin.X()) /
                    cellSize)),
                0, nx - 1);
            const int maxY = std::clamp(
                static_cast<int>(std::floor(
                    (obstacle.maxY + obstacleInflation - origin.Y()) /
                    cellSize)),
                0, ny - 1);
            const int maxZ = std::clamp(
                static_cast<int>(std::floor(
                    (obstacle.maxZ + obstacleInflation - origin.Z()) /
                    cellSize)),
                0, nz - 1);
            if (minX > maxX || minY > maxY || minZ > maxZ) continue;
            for (int z = minZ; z <= maxZ; ++z) {
                for (int y = minY; y <= maxY; ++y) {
                    for (int x = minX; x <= maxX; ++x) {
                        obstacleCandidates[index(x, y, z)].push_back(
                            obstacleIndex);
                    }
                }
            }
        }
    }

    bool freeCell(int x, int y, int z,
                  bool ignoreBoundaryConstraints = false) const {
        if (!inBounds(x, y, z)) return false;
        const gp_Pnt point = center(x, y, z);
        return onMagazineSide(point) && withinAperture(point) &&
               withinBoundaryBox(point, 0.0, ignoreBoundaryConstraints) &&
               (ignoreBoundaryConstraints || insideBoundaryEnvelope(point)) &&
               !pointInsideObstacle(point);
    }

    bool transitionBlocked(const gp_Pnt& from, const gp_Pnt& to) const {
        const gp_Vec delta(from, to);
        for (int i = 1; i <= 5; ++i) {
            const double fraction = static_cast<double>(i) / 6.0;
            const gp_Pnt point = from.Translated(delta * fraction);
            if (pointInsideObstacle(point)) return true;
        }
        return false;
    }

    bool sphereClear(const magazine::packing::Vec3& localCenter,
                     double radius, double tolerance) const {
        const gp_Pnt centerPoint(origin.X() + localCenter.x,
                                 origin.Y() + localCenter.y,
                                 origin.Z() + localCenter.z);
        if (!sphereOnMagazineSide(centerPoint, radius) ||
            !withinAperture(centerPoint, radius) ||
            !withinBoundaryBox(centerPoint, radius) ||
            !insideBoundaryEnvelope(centerPoint) ||
            pointInsideObstacle(centerPoint)) {
            return false;
        }
        const int cx = static_cast<int>(std::floor(localCenter.x / cellSize));
        const int cy = static_cast<int>(std::floor(localCenter.y / cellSize));
        const int cz = static_cast<int>(std::floor(localCenter.z / cellSize));
        if (!inBounds(cx, cy, cz) || reachable[index(cx, cy, cz)] == 0) {
            return false;
        }

        for (const Obstacle& obstacle : obstacles) {
            const double clearance = radius + tolerance;
            const double dx = std::max({obstacle.minX - centerPoint.X(),
                                        0.0,
                                        centerPoint.X() - obstacle.maxX});
            const double dy = std::max({obstacle.minY - centerPoint.Y(),
                                        0.0,
                                        centerPoint.Y() - obstacle.maxY});
            const double dz = std::max({obstacle.minZ - centerPoint.Z(),
                                        0.0,
                                        centerPoint.Z() - obstacle.maxZ});
            if (dx * dx + dy * dy + dz * dz > clearance * clearance) {
                continue;
            }
            try {
                ensureObstacleMesh(const_cast<Obstacle&>(obstacle));
                if (!obstacle.triangles.empty()) {
                    const double clearanceSquared = clearance * clearance;
                    for (const Triangle& triangle : obstacle.triangles) {
                        if (pointTriangleDistanceSquared(centerPoint, triangle) <=
                            clearanceSquared) {
                            return false;
                        }
                    }
                } else {
                    // Keep a conservative OCCT fallback for malformed parts
                    // that cannot be triangulated.
                    const TopoDS_Shape sphere =
                        BRepPrimAPI_MakeSphere(centerPoint, radius).Shape();
                    BRepExtrema_DistShapeShape distance(sphere, obstacle.shape);
                    if (!distance.IsDone() || distance.Value() <= tolerance) {
                        return false;
                    }
                }
            } catch (const Standard_Failure&) {
                return false;
            }
        }
        return true;
    }
};

gp_Dir faceNormal(const TopoDS_Face& face, const gp_Dir& fallback) {
    BRepAdaptor_Surface surface(face, true);
    gp_Dir normal = fallback;
    if (surface.GetType() == GeomAbs_Plane) {
        normal = surface.Plane().Axis().Direction();
    } else {
        const Handle(Geom_Surface) geometry = BRep_Tool::Surface(face);
        const double u = 0.5 * (surface.FirstUParameter() +
                                surface.LastUParameter());
        const double v = 0.5 * (surface.FirstVParameter() +
                                surface.LastVParameter());
        if (!geometry.IsNull() && std::isfinite(u) && std::isfinite(v)) {
            GeomLProp_SLProps properties(geometry, u, v, 1, 1.0e-9);
            if (properties.IsNormalDefined()) normal = properties.Normal();
        }
    }
    if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
    return normal;
}

void localizeBoundaryWindowToSeed(const std::shared_ptr<AssemblyGrid>& grid,
                                  const gp_Pnt& seed) {
    if (!grid->boundaryMode || !grid->hasBoundaryBounds ||
        grid->enforceTangentialIntersection || grid->boundaryConstraints.empty()) {
        return;
    }

    const double coordinates[3] = {seed.X(), seed.Y(), seed.Z()};
    const double originalLower[3] = {grid->boundaryMinX, grid->boundaryMinY,
                                     grid->boundaryMinZ};
    const double originalUpper[3] = {grid->boundaryMaxX, grid->boundaryMaxY,
                                     grid->boundaryMaxZ};
    const double mergePadding = 2.0 * grid->cellSize;
    double localizedLower[3] = {originalLower[0], originalLower[1],
                                originalLower[2]};
    double localizedUpper[3] = {originalUpper[0], originalUpper[1],
                                originalUpper[2]};
    bool changed = false;

    for (int axis = 0; axis < 3; ++axis) {
        if (axis < 2 && grid->enforceGlobalBoundaryPlanes) {
            bool lowerWall = false;
            bool upperWall = false;
            for (const auto& constraint : grid->boundaryConstraints) {
                if (constraint.normalAxis != axis) continue;
                const double component = axis == 0
                    ? constraint.normal.X() : constraint.normal.Y();
                lowerWall = lowerWall || component > 0.5;
                upperWall = upperWall || component < -0.5;
            }
            if (lowerWall || upperWall) {
                // Planes normal to this axis already give a physical side of
                // the cavity. Tangential projections of split faces on other
                // walls must not replace that side with a narrow interval.
                if (lowerWall && upperWall) continue;
                if (originalUpper[axis] - originalLower[axis] >=
                    grid->localizedMinimumSpan[axis]) {
                    continue;
                }
                // Only one selected side is known and its AABB is too thin.
                // Search across the model for the real opposite solid wall.
                localizedLower[axis] = 0.0;
                localizedUpper[axis] = axis == 0
                    ? grid->bounds.widthMm : grid->bounds.depthMm;
                grid->unconstrainedHorizontalAxis[axis] = true;
                grid->unconstrainedFarUpper[axis] = lowerWall;
                changed = true;
                continue;
            }
        }
        std::vector<std::pair<double, double>> intervals;
        for (const auto& constraint : grid->boundaryConstraints) {
            if (axis == constraint.normalAxis || constraint.bounds.IsVoid()) {
                continue;
            }
            double minX = 0.0;
            double minY = 0.0;
            double minZ = 0.0;
            double maxX = 0.0;
            double maxY = 0.0;
            double maxZ = 0.0;
            constraint.bounds.Get(minX, minY, minZ, maxX, maxY, maxZ);
            const double lower[3] = {minX, minY, minZ};
            const double upper[3] = {maxX, maxY, maxZ};
            if (upper[axis] >= lower[axis]) {
                intervals.emplace_back(lower[axis], upper[axis]);
            }
        }
        if (intervals.empty()) continue;

        std::sort(intervals.begin(), intervals.end(),
                  [](const auto& lhs, const auto& rhs) {
                      if (lhs.first != rhs.first) return lhs.first < rhs.first;
                      return lhs.second < rhs.second;
                  });
        std::vector<std::pair<double, double>> merged;
        for (const auto& interval : intervals) {
            if (merged.empty() ||
                interval.first > merged.back().second + mergePadding) {
                merged.push_back(interval);
            } else {
                merged.back().second =
                    std::max(merged.back().second, interval.second);
            }
        }

        std::size_t selected = 0;
        double bestDistance = std::numeric_limits<double>::infinity();
        for (std::size_t index = 0; index < merged.size(); ++index) {
            const auto& interval = merged[index];
            const double distance =
                coordinates[axis] < interval.first
                    ? interval.first - coordinates[axis]
                    : coordinates[axis] > interval.second
                          ? coordinates[axis] - interval.second
                          : 0.0;
            if (distance < bestDistance) {
                bestDistance = distance;
                selected = index;
            }
            if (distance <= mergePadding) break;
        }
        // A single interval can still be only the thickness of one selected
        // side wall. In the fallback path the finite AABB is merely a seed
        // window; the selected face planes and exact solids provide the real
        // cavity boundary. Keep a separated interval when there are several,
        // but allow a single narrow interval to grow around the seed below.
        double lower = merged.size() > 1
            ? merged[selected].first
            : originalLower[axis];
        double upper = merged.size() > 1
            ? merged[selected].second
            : originalUpper[axis];
        if (coordinates[axis] < lower) lower = coordinates[axis];
        if (coordinates[axis] > upper) upper = coordinates[axis];
        const double minimumSpan = grid->localizedMinimumSpan[axis];
        // A single connected projection may be a narrow split of a wider
        // cavity, so it is safe to grow it around the selected seed.  When
        // projections are genuinely separated, keep only the seed interval;
        // growing it would bridge two independent pockets again.
        if (merged.size() == 1 && minimumSpan > 0.0 &&
            upper - lower < minimumSpan) {
            const double modelLower = 0.0;
            const double modelUpper = axis == 0
                ? grid->bounds.widthMm
                : axis == 1
                      ? grid->bounds.depthMm
                      : grid->bounds.heightMm;
            const double halfSpan = 0.5 * minimumSpan;
            lower = coordinates[axis] - halfSpan;
            upper = coordinates[axis] + halfSpan;
            if (lower < modelLower) {
                upper += modelLower - lower;
                lower = modelLower;
            }
            if (upper > modelUpper) {
                lower -= upper - modelUpper;
                upper = modelUpper;
            }
            lower = std::max(modelLower, lower);
            upper = std::min(modelUpper, upper);
        }
        // Do not clip an expanded fallback back to the selected-face AABB.
        // A wall face can be only a narrow split/step of a much wider cavity;
        // the selected planes and exact solid checks remain the authoritative
        // boundaries after this seed-local window is enlarged.
        const double modelLower = 0.0;
        const double modelUpper = axis == 0
            ? grid->bounds.widthMm
            : axis == 1
                  ? grid->bounds.depthMm
                  : grid->bounds.heightMm;
        const double expandedLower = std::max(modelLower, lower);
        const double expandedUpper = std::min(modelUpper, upper);
        localizedLower[axis] = expandedLower;
        localizedUpper[axis] = expandedUpper;
        changed = changed || expandedLower > originalLower[axis] + 1.0e-7 ||
                  expandedUpper < originalUpper[axis] - 1.0e-7 ||
                  expandedLower < originalLower[axis] - 1.0e-7 ||
                  expandedUpper > originalUpper[axis] + 1.0e-7;
    }

    if (!changed) return;

    grid->boundaryMinX = localizedLower[0];
    grid->boundaryMinY = localizedLower[1];
    grid->boundaryMinZ = localizedLower[2];
    grid->boundaryMaxX = localizedUpper[0];
    grid->boundaryMaxY = localizedUpper[1];
    grid->boundaryMaxZ = localizedUpper[2];
    // The virtual gate follows the localized upper rim so a taller, separate
    // selected pocket cannot lift the top boundary of the seeded one.
    grid->gatePoint = gp_Pnt(grid->gatePoint.X(), grid->gatePoint.Y(),
                             grid->boundaryMaxZ);
    grid->localizedBoundaryWindow = true;
}

LocalFrame makeFrame(const CadImportResult& model,
                     const gp_Dir& gravity,
                     const TopoDS_Face& entryFace) {
    gp_Vec up(-gravity.X(), -gravity.Y(), -gravity.Z());
    if (up.SquareMagnitude() <= 1.0e-18) {
        throw std::invalid_argument("gravity direction must be non-zero");
    }
    up.Normalize();
    gp_Vec x;
    if (!entryFace.IsNull()) {
        x = gp_Vec(faceNormal(entryFace, gp::DZ()));
        x -= up * x.Dot(up);
    }
    if (x.SquareMagnitude() <= 1.0e-12) {
        x = gp_Vec(1.0, 0.0, 0.0);
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

    double minX = std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double minZ = std::numeric_limits<double>::infinity();
    for (const double sourceX : {model.minX, model.maxX}) {
        for (const double sourceY : {model.minY, model.maxY}) {
            for (const double sourceZ : {model.minZ, model.maxZ}) {
                const gp_Pnt point = gp_Pnt(sourceX, sourceY, sourceZ)
                                         .Transformed(frame.sourceToLocal);
                minX = std::min(minX, point.X());
                minY = std::min(minY, point.Y());
                minZ = std::min(minZ, point.Z());
            }
        }
    }
    frame.sourceToLocal.SetTranslationPart(gp_Vec(-minX, -minY, -minZ));
    return frame;
}

void setBounds(CadImportResult& model) {
    Bnd_Box bounds;
    BRepBndLib::Add(model.shape, bounds);
    if (bounds.IsVoid()) throw std::invalid_argument("assembly has no bounds");
    bounds.Get(model.minX, model.minY, model.minZ,
               model.maxX, model.maxY, model.maxZ);
}

std::vector<Obstacle> collectObstacles(const TopoDS_Shape& shape,
                                        const gp_Pnt& entryPoint,
                                        double obstacleRange,
                                        const std::vector<std::size_t>&
                                            boundarySolidIndices,
                                        std::size_t& sourceSolidCount) {
    std::vector<Obstacle> obstacles;
    std::size_t solidIndex = 0;
    for (TopExp_Explorer it(shape, TopAbs_SOLID); it.More(); it.Next()) {
        const bool boundaryEssential =
            std::find(boundarySolidIndices.begin(), boundarySolidIndices.end(),
                      solidIndex) != boundarySolidIndices.end();
        ++sourceSolidCount;
        ++solidIndex;
        Obstacle obstacle;
        obstacle.shape = it.Current();
        obstacle.boundaryEssential = boundaryEssential;
        BRepBndLib::Add(obstacle.shape, obstacle.bounds);
        if (obstacle.bounds.IsVoid()) continue;
        obstacle.bounds.Get(obstacle.minX, obstacle.minY, obstacle.minZ,
                            obstacle.maxX, obstacle.maxY, obstacle.maxZ);
        if (!boundaryEssential &&
            (entryPoint.X() < obstacle.minX - obstacleRange ||
            entryPoint.X() > obstacle.maxX + obstacleRange ||
            entryPoint.Y() < obstacle.minY - obstacleRange ||
            entryPoint.Y() > obstacle.maxY + obstacleRange ||
            entryPoint.Z() < obstacle.minZ - obstacleRange ||
            entryPoint.Z() > obstacle.maxZ + obstacleRange)) {
            continue;
        }
        // Solid classifiers are built lazily from point queries.  Large STEP
        // assemblies often contain hundreds of remote parts; constructing a
        // classifier for every one before flood filling wastes both time and
        // memory.
        obstacles.push_back(std::move(obstacle));
    }
    return obstacles;
}

bool solidContainsFace(const TopoDS_Shape& solid, const TopoDS_Face& selectedFace) {
    for (TopExp_Explorer it(solid, TopAbs_FACE); it.More(); it.Next()) {
        const TopoDS_Face candidate = TopoDS::Face(it.Current());
        if (candidate.IsSame(selectedFace)) {
            return true;
        }
    }
    return false;
}

std::vector<std::size_t> findBoundarySolidIndices(
    const TopoDS_Shape& sourceShape,
    const std::vector<TopoDS_Face>& boundaryFaces) {
    std::vector<std::size_t> boundarySolidIndices;
    if (boundaryFaces.empty()) return boundarySolidIndices;
    std::size_t solidIndex = 0;
    for (TopExp_Explorer it(sourceShape, TopAbs_SOLID); it.More(); it.Next()) {
        const TopoDS_Shape sourceSolid = it.Current();
        bool selected = false;
        for (const auto& boundaryFace : boundaryFaces) {
            if (solidContainsFace(sourceSolid, boundaryFace)) {
                selected = true;
                break;
            }
        }
        if (selected) boundarySolidIndices.push_back(solidIndex);
        ++solidIndex;
    }
    return boundarySolidIndices;
}

std::optional<gp_Pnt> findSeed(const std::shared_ptr<AssemblyGrid>& grid,
                               const gp_Pnt& entryPoint,
                               const gp_Dir& flowDirection,
                               double requestedOffset,
                               double maximumDistance) {
    if (grid->boundaryMode) {
        const std::array<double, 7> offsets{{
            0.0,
            requestedOffset,
            -requestedOffset,
            2.0 * requestedOffset,
            -2.0 * requestedOffset,
            4.0 * requestedOffset,
            -4.0 * requestedOffset,
        }};
        for (const double offset : offsets) {
            const gp_Pnt candidate = entryPoint.Translated(
                gp_Vec(flowDirection) * offset);
            if (!grid->onMagazineSide(candidate) ||
                !grid->withinAperture(candidate) ||
                !grid->withinBoundaryBox(candidate) ||
                !grid->insideBoundaryEnvelope(candidate) ||
                grid->pointInsideObstacle(candidate)) {
                continue;
            }
            const int x = static_cast<int>(std::floor(candidate.X() /
                                                       grid->cellSize));
            const int y = static_cast<int>(std::floor(candidate.Y() /
                                                       grid->cellSize));
            const int z = static_cast<int>(std::floor(candidate.Z() /
                                                       grid->cellSize));
            if (grid->inBounds(x, y, z) && grid->freeCell(x, y, z)) {
                return candidate;
            }
        }
    }
    const int attempts = std::max(
        1, static_cast<int>(std::ceil(maximumDistance / grid->cellSize)));
    for (int index = 1; index <= attempts; ++index) {
        const double distance = std::max(requestedOffset,
                                         index * grid->cellSize * 0.5);
        const gp_Pnt candidate = entryPoint.Translated(
            gp_Vec(flowDirection) * distance);
        if (!grid->onMagazineSide(candidate) ||
            grid->pointInsideObstacle(candidate)) {
            continue;
        }
        const int x = static_cast<int>(std::floor(candidate.X() / grid->cellSize));
        const int y = static_cast<int>(std::floor(candidate.Y() / grid->cellSize));
        const int z = static_cast<int>(std::floor(candidate.Z() / grid->cellSize));
        if (grid->inBounds(x, y, z) && grid->freeCell(x, y, z)) return candidate;
    }
    return std::nullopt;
}

std::optional<gp_Pnt> useSelectedSeed(const std::shared_ptr<AssemblyGrid>& grid,
                                      const gp_Pnt& seedPoint,
                                      bool ignoreBoundaryConstraints = false) {
    if (!grid->onMagazineSide(seedPoint) ||
        !grid->withinAperture(seedPoint) ||
        !grid->withinBoundaryBox(seedPoint, 0.0,
                                 ignoreBoundaryConstraints) ||
        (!ignoreBoundaryConstraints &&
         !grid->insideBoundaryEnvelope(seedPoint)) ||
        grid->pointInsideObstacle(seedPoint)) {
        return std::nullopt;
    }
    const int x = static_cast<int>(std::floor(seedPoint.X() / grid->cellSize));
    const int y = static_cast<int>(std::floor(seedPoint.Y() / grid->cellSize));
    const int z = static_cast<int>(std::floor(seedPoint.Z() / grid->cellSize));
    if (!grid->inBounds(x, y, z) ||
        !grid->freeCell(x, y, z, ignoreBoundaryConstraints)) {
        return std::nullopt;
    }
    return seedPoint;
}

std::optional<gp_Pnt> findBoundarySeed(
    const std::shared_ptr<AssemblyGrid>& grid,
    const std::vector<TopoDS_Face>& alignedBoundaryFaces,
    double cellSize,
    bool ignoreBoundaryConstraints = false) {
    const std::array<double, 5> offsets{{
        0.5 * cellSize,
        cellSize,
        2.0 * cellSize,
        4.0 * cellSize,
        8.0 * cellSize,
    }};
    // Search the common selected-face centroid first.  For a set of inner
    // walls this is normally inside the intended cavity; trying individual
    // face centroids first can accidentally choose the exterior side of a
    // wall when the face is split, overlapping, or has a hole.
    const gp_Pnt selectedCenter = grid->entryPoint;
    std::optional<gp_Pnt> bestSeed;
    int bestConstraintScore = -1;
    double bestCenterDistance = std::numeric_limits<double>::infinity();
    auto considerCandidate = [&](const gp_Pnt& candidate) {
        const auto seed = useSelectedSeed(
            grid, candidate, ignoreBoundaryConstraints);
        if (!seed.has_value()) return false;
        if (!ignoreBoundaryConstraints) {
            bestSeed = seed;
            return true;
        }
        int constraintScore = 0;
        for (const auto& constraint : grid->boundaryConstraints) {
            const gp_Vec fromFace(constraint.point, *seed);
            if (fromFace.Dot(gp_Vec(constraint.normal)) >=
                -0.5 * cellSize) {
                ++constraintScore;
            }
        }
        const double centerDistance = selectedCenter.SquareDistance(*seed);
        if (!bestSeed.has_value() || constraintScore > bestConstraintScore ||
            (constraintScore == bestConstraintScore &&
             centerDistance < bestCenterDistance)) {
            bestSeed = seed;
            bestConstraintScore = constraintScore;
            bestCenterDistance = centerDistance;
        }
        return false;
    };
    const int neighborhoodRadius = 6;
    for (int shell = 0; shell <= neighborhoodRadius; ++shell) {
        for (int z = -shell; z <= shell; ++z) {
            for (int y = -shell; y <= shell; ++y) {
                for (int x = -shell; x <= shell; ++x) {
                    if (std::max({std::abs(x), std::abs(y), std::abs(z)}) !=
                        shell) {
                        continue;
                    }
                    const gp_Pnt candidate = selectedCenter.Translated(
                        gp_Vec(x * cellSize, y * cellSize, z * cellSize));
                    if (considerCandidate(candidate) && !ignoreBoundaryConstraints) {
                        return bestSeed;
                    }
                }
            }
        }
    }

    for (const auto& face : alignedBoundaryFaces) {
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(face, properties);
        const gp_Pnt center = properties.CentreOfMass();
        const gp_Dir normal = faceNormal(face, gp::DZ());
        for (const double offset : offsets) {
            for (const double sign : {1.0, -1.0}) {
                const gp_Pnt candidate = center.Translated(
                    gp_Vec(normal) * (sign * offset));
                if (considerCandidate(candidate) && !ignoreBoundaryConstraints) {
                    return bestSeed;
                }
            }
        }
    }
    return bestSeed;
}

void orientBoundaryConstraintsToSeed(
    const std::shared_ptr<AssemblyGrid>& grid, const gp_Pnt& seed) {
    for (auto& constraint : grid->boundaryConstraints) {
        const gp_Vec fromFace(constraint.point, seed);
        if (fromFace.Dot(gp_Vec(constraint.normal)) < -0.5 * grid->cellSize) {
            constraint.normal.Reverse();
        }
    }
}

bool strictTangentialWindowIsNarrow(const AssemblyGrid& grid) {
    if (!grid.enforceGlobalBoundaryPlanes ||
        grid.boundaryConstraints.empty()) return false;
    for (int axis = 0; axis < 2; ++axis) {
        const double selectedSpan = axis == 0
            ? grid.boundaryMaxX - grid.boundaryMinX
            : grid.boundaryMaxY - grid.boundaryMinY;
        bool lowerWall = false;
        bool upperWall = false;
        for (const auto& constraint : grid.boundaryConstraints) {
            if (constraint.normalAxis != axis) continue;
            const double component = axis == 0
                ? constraint.normal.X() : constraint.normal.Y();
            lowerWall = lowerWall || component > 0.5;
            upperWall = upperWall || component < -0.5;
        }
        if ((lowerWall != upperWall) &&
            selectedSpan < grid.localizedMinimumSpan[axis]) {
            return true;
        }
        double lower = -std::numeric_limits<double>::infinity();
        double upper = std::numeric_limits<double>::infinity();
        for (const auto& constraint : grid.boundaryConstraints) {
            if (constraint.normalAxis == axis || constraint.bounds.IsVoid()) {
                continue;
            }
            double minX = 0.0, minY = 0.0, minZ = 0.0;
            double maxX = 0.0, maxY = 0.0, maxZ = 0.0;
            constraint.bounds.Get(minX, minY, minZ, maxX, maxY, maxZ);
            const double minima[2] = {minX, minY};
            const double maxima[2] = {maxX, maxY};
            lower = std::max(lower, minima[axis] - 2.0 * grid.cellSize);
            upper = std::min(upper, maxima[axis] + 2.0 * grid.cellSize);
        }
        if (std::isfinite(lower) && std::isfinite(upper) &&
            upper - lower < grid.localizedMinimumSpan[axis]) {
            return true;
        }
    }
    return false;
}

void floodFill(const std::shared_ptr<AssemblyGrid>& grid,
               const gp_Pnt& seed, std::size_t maximumCells) {
    const int seedX = static_cast<int>(std::floor(seed.X() / grid->cellSize));
    const int seedY = static_cast<int>(std::floor(seed.Y() / grid->cellSize));
    const int seedZ = static_cast<int>(std::floor(seed.Z() / grid->cellSize));
    if (!grid->inBounds(seedX, seedY, seedZ) ||
        !grid->freeCell(seedX, seedY, seedZ)) {
        throw std::invalid_argument("entry seed is not in free assembly space");
    }

    std::deque<CellKey> queue;
    queue.push_back({seedX, seedY, seedZ});
    grid->reachable[grid->index(seedX, seedY, seedZ)] = 1;
    constexpr std::array<std::array<int, 3>, 6> directions{{
        {{1, 0, 0}}, {{-1, 0, 0}}, {{0, 1, 0}},
        {{0, -1, 0}}, {{0, 0, 1}}, {{0, 0, -1}}}};

    while (!queue.empty()) {
        const CellKey current = queue.front();
        queue.pop_front();
        const gp_Pnt currentPoint = grid->center(current.x, current.y,
                                                  current.z);
        for (const auto& direction : directions) {
            const CellKey next{current.x + direction[0],
                               current.y + direction[1],
                               current.z + direction[2]};
            if (!grid->inBounds(next.x, next.y, next.z)) continue;
            const std::size_t nextIndex = grid->index(next.x, next.y, next.z);
            if (grid->reachable[nextIndex] != 0 ||
                !grid->freeCell(next.x, next.y, next.z)) {
                continue;
            }
            if (grid->transitionBlocked(currentPoint,
                                        grid->center(next.x, next.y, next.z))) {
                continue;
            }
            grid->reachable[nextIndex] = 1;
            queue.push_back(next);
            if (queue.size() + 1 > maximumCells) {
                throw std::invalid_argument(
                    "assembly free-space extraction exceeded the cell limit; "
                    "crop the assembly or increase the limit");
            }
        }
    }
}

void rejectUnboundedHorizontalFlood(const std::shared_ptr<AssemblyGrid>& grid) {
    if (!grid->unconstrainedHorizontalAxis[0] &&
        !grid->unconstrainedHorizontalAxis[1]) {
        return;
    }
    for (int z = 0; z < grid->nz; ++z) {
        for (int y = 0; y < grid->ny; ++y) {
            for (int x = 0; x < grid->nx; ++x) {
                if (grid->reachable[grid->index(x, y, z)] == 0) continue;
                if ((grid->unconstrainedHorizontalAxis[0] &&
                     x == (grid->unconstrainedFarUpper[0]
                               ? grid->nx - 1 : 0)) ||
                    (grid->unconstrainedHorizontalAxis[1] &&
                     y == (grid->unconstrainedFarUpper[1]
                               ? grid->ny - 1 : 0))) {
                    throw std::invalid_argument(
                        "selected faces do not bound the cavity on one horizontal axis; "
                        "the reachable region touches the assembly edge at cell (" +
                        std::to_string(x) + ", " + std::to_string(y) + ", " +
                        std::to_string(z) + ") in grid " +
                        std::to_string(grid->nx) + "x" +
                        std::to_string(grid->ny) + "x" +
                        std::to_string(grid->nz) + ". Select an opposing inner wall face");
                }
            }
        }
    }
}

void cropToReachableComponent(const std::shared_ptr<AssemblyGrid>& grid,
                              double paddingMm) {
    int minX = grid->nx;
    int minY = grid->ny;
    int minZ = grid->nz;
    int maxX = -1;
    int maxY = -1;
    int maxZ = -1;
    for (int z = 0; z < grid->nz; ++z) {
        for (int y = 0; y < grid->ny; ++y) {
            for (int x = 0; x < grid->nx; ++x) {
                if (grid->reachable[grid->index(x, y, z)] == 0) continue;
                minX = std::min(minX, x);
                minY = std::min(minY, y);
                minZ = std::min(minZ, z);
                maxX = std::max(maxX, x);
                maxY = std::max(maxY, y);
                maxZ = std::max(maxZ, z);
            }
        }
    }
    if (maxX < minX || maxY < minY || maxZ < minZ) {
        throw std::invalid_argument("assembly extraction found no reachable free space");
    }

    const int oldNx = grid->nx;
    const int oldNy = grid->ny;
    const int newNx = maxX - minX + 1;
    const int newNy = maxY - minY + 1;
    const int newNz = maxZ - minZ + 1;
    std::vector<unsigned char> croppedReachable(
        static_cast<std::size_t>(newNx) * static_cast<std::size_t>(newNy) *
            static_cast<std::size_t>(newNz),
        0);
    for (int z = minZ; z <= maxZ; ++z) {
        for (int y = minY; y <= maxY; ++y) {
            for (int x = minX; x <= maxX; ++x) {
                const std::size_t sourceIndex =
                    (static_cast<std::size_t>(z) * static_cast<std::size_t>(oldNy) +
                     static_cast<std::size_t>(y)) *
                        static_cast<std::size_t>(oldNx) +
                    static_cast<std::size_t>(x);
                const std::size_t targetIndex =
                    (static_cast<std::size_t>(z - minZ) *
                         static_cast<std::size_t>(newNy) +
                     static_cast<std::size_t>(y - minY)) *
                        static_cast<std::size_t>(newNx) +
                    static_cast<std::size_t>(x - minX);
                croppedReachable[targetIndex] = grid->reachable[sourceIndex];
            }
        }
    }
    grid->nx = newNx;
    grid->ny = newNy;
    grid->nz = newNz;
    grid->reachable = std::move(croppedReachable);
    grid->origin = gp_Vec(minX * grid->cellSize,
                          minY * grid->cellSize,
                          minZ * grid->cellSize);
    grid->bounds = {
        (maxX - minX + 1) * grid->cellSize,
        (maxY - minY + 1) * grid->cellSize,
        (maxZ - minZ + 1) * grid->cellSize,
    };

    // Keep only solids that can touch the cropped component.  The padding is
    // the largest supported projectile radius plus a small numerical margin;
    // solids outside it cannot affect a sphere-clearance query.
    const double minWorldX = grid->origin.X() - paddingMm;
    const double minWorldY = grid->origin.Y() - paddingMm;
    const double minWorldZ = grid->origin.Z() - paddingMm;
    const double maxWorldX = grid->origin.X() + grid->bounds.widthMm + paddingMm;
    const double maxWorldY = grid->origin.Y() + grid->bounds.depthMm + paddingMm;
    const double maxWorldZ = grid->origin.Z() + grid->bounds.heightMm + paddingMm;
    grid->obstacles.erase(
        std::remove_if(grid->obstacles.begin(), grid->obstacles.end(),
                       [&](const Obstacle& obstacle) {
                           return !obstacle.boundaryEssential &&
                                  (obstacle.maxX < minWorldX ||
                                  obstacle.minX > maxWorldX ||
                                  obstacle.maxY < minWorldY ||
                                  obstacle.minY > maxWorldY ||
                                  obstacle.maxZ < minWorldZ ||
                                  obstacle.minZ > maxWorldZ);
                       }),
        grid->obstacles.end());
    grid->rebuildObstacleIndex();
}

std::optional<magazine::packing::VerticalInterval> coarseIntervalForColumn(
    const std::shared_ptr<AssemblyGrid>& grid, double x, double y,
    double radius) {
    if (x < radius || x > grid->bounds.widthMm - radius ||
        y < radius || y > grid->bounds.depthMm - radius) {
        return std::nullopt;
    }
    const int cx = static_cast<int>(std::floor(x / grid->cellSize));
    const int cy = static_cast<int>(std::floor(y / grid->cellSize));
    if (!grid->inBounds(cx, cy, 0)) return std::nullopt;
    int first = -1;
    int last = -1;
    bool contiguous = true;
    for (int z = 0; z < grid->nz; ++z) {
        if (grid->reachable[grid->index(cx, cy, z)] == 0) {
            if (first >= 0 && last != z - 1) contiguous = false;
            continue;
        }
        if (first < 0) first = z;
        if (last >= 0 && z != last + 1) contiguous = false;
        last = z;
    }
    if (first < 0 || last < first || !contiguous) return std::nullopt;
    const double lower = first * grid->cellSize + radius;
    const double upper = (last + 1) * grid->cellSize - radius;
    if (lower > upper + 1.0e-7) return std::nullopt;
    return magazine::packing::VerticalInterval{lower, upper};
}

std::optional<magazine::packing::VerticalInterval> intervalForColumn(
    const std::shared_ptr<AssemblyGrid>& grid, double x, double y,
    double radius, double tolerance) {
    if (x < radius || x > grid->bounds.widthMm - radius ||
        y < radius || y > grid->bounds.depthMm - radius) {
        return std::nullopt;
    }
    const int samples = std::max(8, grid->nz);
    const double step = grid->bounds.heightMm / samples;
    int first = -1;
    int last = -1;
    for (int index = 0; index <= samples; ++index) {
        const double z = index * step;
        const bool valid = grid->sphereClear({x, y, z}, radius, tolerance);
        if (valid && first < 0) {
            first = index;
            last = index;
        } else if (valid && last == index - 1) {
            last = index;
        } else if (first >= 0 && !valid) {
            break;
        }
    }
    if (first < 0) return std::nullopt;
    auto refine = [&](double outside, double inside) {
        for (int iteration = 0; iteration < 10; ++iteration) {
            const double middle = 0.5 * (outside + inside);
            if (grid->sphereClear({x, y, middle}, radius, tolerance)) {
                inside = middle;
            } else {
                outside = middle;
            }
        }
        return inside;
    };
    const double lower = first == 0 ? 0.0
                                    : refine((first - 1) * step, first * step);
    const double upper = last == samples
        ? grid->bounds.heightMm
        : refine((last + 1) * step, last * step);
    return magazine::packing::VerticalInterval{lower, upper};
}

} // namespace

AssemblyPackingRegion makeAssemblyPackingRegionImpl(
    const CadImportResult& model, const gp_Dir& gravityDirection,
    const TopoDS_Face& entryFace,
    const std::vector<TopoDS_Face>& boundaryFaces,
    const AssemblyExtractionOptions& options) {
    if (model.shape.IsNull() || model.solidCount == 0) {
        throw std::invalid_argument("assembly contains no solid obstacles");
    }
    if (entryFace.IsNull() && boundaryFaces.empty() &&
        !options.seedPointSource.has_value()) {
        throw std::invalid_argument(
            "assembly extraction requires boundary faces, an entry face, or a free-space seed");
    }
    for (const auto& face : boundaryFaces) {
        if (face.IsNull() || face.ShapeType() != TopAbs_FACE) {
            throw std::invalid_argument("selected assembly boundary contains an invalid face");
        }
    }
    if (options.cellSizeMm <= 0.0 || options.maximumCells == 0 ||
        options.seedOffsetMm <= 0.0 || options.maximumSeedSearchMm <= 0.0 ||
        options.obstacleRangeMm <= 0.0 ||
        options.entryApertureMarginMm < 0.0 ||
        options.minimumEntryApertureWidthMm <= 0.0 ||
        options.minimumEntryApertureDepthMm <= 0.0) {
        throw std::invalid_argument("invalid assembly extraction options");
    }

    const TopoDS_Face frameFace = !boundaryFaces.empty()
        ? boundaryFaces.front()
        : entryFace;
    const LocalFrame frame = makeFrame(model, gravityDirection, frameFace);
    const TopoDS_Shape alignedShape =
        BRepBuilderAPI_Transform(model.shape, frame.sourceToLocal, true).Shape();
    CadImportResult aligned = model;
    aligned.shape = alignedShape;
    setBounds(aligned);

    gp_Vec localFlow(gravityDirection);
    localFlow.Transform(frame.sourceToLocal);
    localFlow.Normalize();
    gp_Dir flow(localFlow);

    GProp_GProps faceProperties;
    gp_Pnt sourceEntryPoint;
    std::vector<TopoDS_Face> alignedBoundaryFaces;
    Bnd_Box boundaryBounds;
    double boundaryArea = 0.0;
    double boundaryCenterX = 0.0;
    double boundaryCenterY = 0.0;
    double boundaryCenterZ = 0.0;
    for (const auto& face : boundaryFaces) {
        BRepGProp::SurfaceProperties(face, faceProperties);
        const gp_Pnt center = faceProperties.CentreOfMass();
        const double area = std::max(0.0, faceProperties.Mass());
        boundaryArea += area;
        boundaryCenterX += center.X() * area;
        boundaryCenterY += center.Y() * area;
        boundaryCenterZ += center.Z() * area;
        const TopoDS_Shape alignedFaceShape =
            BRepBuilderAPI_Transform(face, frame.sourceToLocal, true).Shape();
        if (!alignedFaceShape.IsNull() &&
            alignedFaceShape.ShapeType() == TopAbs_FACE) {
            const TopoDS_Face alignedFace = TopoDS::Face(alignedFaceShape);
            alignedBoundaryFaces.push_back(alignedFace);
            BRepBndLib::Add(alignedFace, boundaryBounds);
        }
    }
    if (options.entryPointSource.has_value()) {
        sourceEntryPoint = *options.entryPointSource;
    } else if (!boundaryFaces.empty()) {
        if (boundaryArea > Precision::Confusion()) {
            sourceEntryPoint = gp_Pnt(boundaryCenterX / boundaryArea,
                                      boundaryCenterY / boundaryArea,
                                      boundaryCenterZ / boundaryArea);
        } else {
            BRepGProp::SurfaceProperties(boundaryFaces.front(), faceProperties);
            sourceEntryPoint = faceProperties.CentreOfMass();
        }
    } else if (!entryFace.IsNull()) {
        BRepGProp::SurfaceProperties(entryFace, faceProperties);
        sourceEntryPoint = faceProperties.CentreOfMass();
    } else {
        sourceEntryPoint = *options.seedPointSource;
    }
    gp_Pnt entryPoint = sourceEntryPoint.Transformed(frame.sourceToLocal);
    std::optional<gp_Pnt> seedPoint;
    if (options.seedPointSource.has_value()) {
        seedPoint = options.seedPointSource->Transformed(frame.sourceToLocal);
        // A ray-selected near boundary can be viewed from any direction. If
        // it lies on the wrong side of the physical gravity gate, move the
        // virtual gate just outside the selected seed while preserving the
        // user gravity direction.
        if (gp_Vec(entryPoint, *seedPoint).Dot(localFlow) <
            -options.cellSizeMm) {
            entryPoint = seedPoint->Translated(
                gp_Vec(localFlow) * -std::max(options.seedOffsetMm,
                                               4.0 * options.cellSizeMm));
        }
    }

    auto grid = std::make_shared<AssemblyGrid>();
    grid->bounds = {aligned.widthMm(), aligned.depthMm(), aligned.heightMm()};
    grid->cellSize = options.cellSizeMm;
    grid->obstacleInflation = options.obstacleInflationMm;
    grid->entryPoint = entryPoint;
    grid->gatePoint = entryPoint;
    grid->boundaryMode = !boundaryFaces.empty();
    // The selected face is only a geometric reference for the opening
    // location.  It may be a rim or a slanted wall and therefore must not
    // define the gate half-space.  The user-specified gravity direction is
    // the physical insertion direction and is the only valid gate normal.
    grid->gateNormal = flow;

    const TopoDS_Face displayEntrySource = !boundaryFaces.empty()
        ? boundaryFaces.front()
        : entryFace;
    const TopoDS_Shape alignedEntry = displayEntrySource.IsNull()
        ? TopoDS_Shape()
        : BRepBuilderAPI_Transform(displayEntrySource, frame.sourceToLocal,
                                    true).Shape();
    Bnd_Box entryBounds;
    if (!boundaryFaces.empty()) {
        entryBounds = boundaryBounds;
    } else if (!alignedEntry.IsNull()) {
        BRepBndLib::Add(alignedEntry, entryBounds);
    }
    double entryMinX = 0.0;
    double entryMinY = 0.0;
    double entryMinZ = 0.0;
    double entryMaxX = 0.0;
    double entryMaxY = 0.0;
    double entryMaxZ = 0.0;
    if (!entryBounds.IsVoid()) {
        entryBounds.Get(entryMinX, entryMinY, entryMinZ,
                        entryMaxX, entryMaxY, entryMaxZ);
    } else {
        entryMinX = entryMaxX = entryPoint.X();
        entryMinY = entryMaxY = entryPoint.Y();
        entryMinZ = entryMaxZ = entryPoint.Z();
    }
    const bool boundaryMode = !boundaryFaces.empty();
    // Boundary-face selection is a cavity hint, not a tiny entry slot. A
    // vertical wall can have almost zero projection on one horizontal axis;
    // capping this margin at 16 mm used to turn that projection into a narrow
    // strip and discard the rest of the intended magazine.
    const double margin = std::max(2.0 * options.cellSizeMm,
                                   options.entryApertureMarginMm);
    // A confirmed multi-face selection already supplies both sides of the
    // intended cavity in most axes. A 48 mm gate margin (and the historical
    // 120 x 160 mm minimum gate) would turn a small magazine into a large
    // rectangular region. Keep a larger fallback only for a single face,
    // where there is no opposite selected wall to establish the extent.
    const double boundaryWindowPadding =
        boundaryMode && boundaryFaces.size() > 1
            ? std::max(2.0 * options.cellSizeMm, 24.0)
            : margin;
    const bool pointMode = options.seedPointSource.has_value() &&
                           !boundaryMode;
    // In boundary-face mode the selected faces define a finite local window,
    // not an infinite half-space.  Store that window before flood filling so
    // both connectivity and final sphere checks reject exterior air.  The
    // same margin used for the virtual gate keeps a single side-wall face
    // from collapsing the intended cavity into a thin slab.
    grid->hasBoundaryBounds = boundaryMode && !boundaryBounds.IsVoid();
    if (grid->hasBoundaryBounds) {
        boundaryBounds.Get(grid->boundaryMinX, grid->boundaryMinY,
                           grid->boundaryMinZ, grid->boundaryMaxX,
                           grid->boundaryMaxY, grid->boundaryMaxZ);
        grid->boundaryPadding = boundaryWindowPadding;
        // If the selected-face intervals split into separate components, keep
        // at least a useful local window for a normal magazine axis.  A single
        // side-wall projection may be expanded during fallback; separated
        // intervals remain seed-localized so distant pockets are not merged.
        grid->localizedMinimumSpan = {
            std::max(4.0 * options.cellSizeMm,
                     options.minimumEntryApertureWidthMm -
                         2.0 * grid->boundaryPadding),
            std::max(4.0 * options.cellSizeMm,
                     options.minimumEntryApertureDepthMm -
                         2.0 * grid->boundaryPadding),
            0.0};
        // The cavity is open on the side opposite gravity.  Use the actual
        // upper edge of the selected-face envelope as the virtual entry gate.
        // Adding a grid-cell clearance here lets sphere centers rise above the
        // physical rim and creates a spurious final projectile layer.  The
        // sphere-side test already subtracts the projectile radius, so no
        // extra clearance is needed.
        grid->gatePoint = gp_Pnt(entryPoint.X(), entryPoint.Y(),
                                 grid->boundaryMaxZ);
    }
    const double entryCenterX = pointMode ? entryPoint.X()
                                          : 0.5 * (entryMinX + entryMaxX);
    const double entryCenterY = pointMode ? entryPoint.Y()
                                          : 0.5 * (entryMinY + entryMaxY);
    double halfWidth = std::max(
        pointMode ? 0.5 * options.minimumEntryApertureWidthMm
                  : 0.5 * (entryMaxX - entryMinX) + margin,
        0.5 * options.minimumEntryApertureWidthMm);
    double halfDepth = std::max(
        pointMode ? 0.5 * options.minimumEntryApertureDepthMm
                  : 0.5 * (entryMaxY - entryMinY) + margin,
        0.5 * options.minimumEntryApertureDepthMm);
    if (pointMode && seedPoint.has_value()) {
        // Keep the selected interior point inside the virtual gate even when
        // the visible opening is slanted relative to the gravity frame.
        halfWidth = std::max(halfWidth,
                             std::abs(seedPoint->X() - entryPoint.X()) +
                                 0.5 * options.minimumEntryApertureWidthMm);
        halfDepth = std::max(halfDepth,
                             std::abs(seedPoint->Y() - entryPoint.Y()) +
                                 0.5 * options.minimumEntryApertureDepthMm);
    }
    grid->apertureMinX = entryCenterX - halfWidth;
    grid->apertureMaxX = entryCenterX + halfWidth;
    grid->apertureMinY = entryCenterY - halfDepth;
    grid->apertureMaxY = entryCenterY + halfDepth;
    if (boundaryMode) {
        // In confirmed boundary mode the finite face envelope is the gate.
        // Leaving the generic entry aperture at infinity avoids imposing a
        // second, unrelated rectangular window on the selected cavity.
        grid->apertureMinX = -std::numeric_limits<double>::infinity();
        grid->apertureMaxX = std::numeric_limits<double>::infinity();
        grid->apertureMinY = -std::numeric_limits<double>::infinity();
        grid->apertureMaxY = std::numeric_limits<double>::infinity();
    }
    grid->nx = std::max(1, static_cast<int>(std::ceil(
        grid->bounds.widthMm / grid->cellSize)));
    grid->ny = std::max(1, static_cast<int>(std::ceil(
        grid->bounds.depthMm / grid->cellSize)));
    grid->nz = std::max(1, static_cast<int>(std::ceil(
        grid->bounds.heightMm / grid->cellSize)));
    const std::size_t totalCells = static_cast<std::size_t>(grid->nx) *
                                   static_cast<std::size_t>(grid->ny) *
                                   static_cast<std::size_t>(grid->nz);
    if (totalCells > options.maximumCells * 2) {
        throw std::invalid_argument(
            "assembly bounding box is too large for the selected cell size; "
            "crop the assembly or use a coarser grid");
    }
    grid->reachable.assign(totalCells, 0);
    std::size_t sourceSolidCount = 0;
    // A selected face may be far from the weighted entry centroid (for
    // example, a long side wall). Extend the candidate cube to cover the
    // complete selected-face envelope, but do not scan the whole vehicle for
    // every grid cell. Bounding-box rejection keeps point tests cheap and
    // classifiers are still lazy.
    // When the GUI supplies a free-space seed, it is the authoritative
    // location of the target cavity. The ray's near boundary is only a gate
    // reference and can be hundreds of millimetres away on a large vehicle;
    // using it as the obstacle-query origin could omit walls around the
    // selected cavity from the local obstacle set.
    const gp_Pnt obstacleReference = seedPoint.value_or(entryPoint);
    // Keep the complete solids that own confirmed boundary faces.  A selected
    // wall can sit outside the entry-centered range (or just outside the
    // reachable-cell crop) even though it is the physical wall that must be
    // visible and must block projectile clearance.
    const std::vector<std::size_t> boundarySolidIndices =
        findBoundarySolidIndices(model.shape, boundaryFaces);
    double selectedEnvelopeRange = 0.0;
    if (!boundaryBounds.IsVoid()) {
        double minX = 0.0;
        double minY = 0.0;
        double minZ = 0.0;
        double maxX = 0.0;
        double maxY = 0.0;
        double maxZ = 0.0;
        boundaryBounds.Get(minX, minY, minZ, maxX, maxY, maxZ);
        selectedEnvelopeRange = std::max({
            std::abs(obstacleReference.X() - minX),
            std::abs(obstacleReference.X() - maxX),
            std::abs(obstacleReference.Y() - minY),
            std::abs(obstacleReference.Y() - maxY),
            std::abs(obstacleReference.Z() - minZ),
            std::abs(obstacleReference.Z() - maxZ)});
    }
    grid->obstacles = collectObstacles(
        alignedShape, obstacleReference,
        std::max(options.obstacleRangeMm,
                 selectedEnvelopeRange + 2.0 * options.cellSizeMm),
        boundarySolidIndices,
        sourceSolidCount);
    if (grid->obstacles.empty()) {
        throw std::invalid_argument("assembly has no measurable solid obstacles");
    }
    grid->rebuildObstacleIndex();

    // Turn each selected face into a one-sided finite constraint. The side
    // that is not occupied by an assembly solid is the cavity side and is
    // retained; the finite tangential footprint prevents a local floor,
    // sidewall, or rim from becoming an infinite plane.
    for (const auto& face : alignedBoundaryFaces) {
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(face, properties);
        const gp_Pnt point = properties.CentreOfMass();
        gp_Dir normal = faceNormal(face, gp::DZ());
        // When OCCT cannot classify the two offset samples (common for an
        // invalid or open STEP assembly), orient the wall toward the common
        // selected-face centroid.  This gives all selected walls a coherent
        // cavity side before the local solid test is applied.
        if (boundaryArea > Precision::Confusion()) {
            // grid->entryPoint is the weighted centroid after the assembly
            // has been transformed into the gravity-aligned local frame.
            const gp_Pnt selectedCenter = grid->entryPoint;
            if (gp_Vec(point, selectedCenter).Dot(gp_Vec(normal)) < 0.0) {
                normal.Reverse();
            }
        }
        const double offset = std::max(2.0, 0.5 * options.cellSizeMm);
        const gp_Pnt plus = point.Translated(gp_Vec(normal) * offset);
        const gp_Pnt minus = point.Translated(gp_Vec(normal) * -offset);
        const bool plusOccupied = grid->pointInsideObstacle(plus);
        const bool minusOccupied = grid->pointInsideObstacle(minus);
        if (plusOccupied == minusOccupied) {
            if (plusOccupied) continue;
            // Some STEP parts are topologically imperfect and cannot be
            // classified reliably. The selected CAD face is still a valid
            // boundary hint; its oriented normal is the outward/free-side
            // direction, so retain it instead of silently dropping the wall.
        } else if (plusOccupied) {
            normal.Reverse();
        }
        Bnd_Box faceBounds;
        BRepBndLib::Add(face, faceBounds);
        const double normalComponents[3] = {std::abs(normal.X()),
                                            std::abs(normal.Y()),
                                            std::abs(normal.Z())};
        const int normalAxis = static_cast<int>(std::max_element(
            std::begin(normalComponents), std::end(normalComponents)) -
                                                std::begin(normalComponents));
        grid->boundaryConstraints.push_back({point, normal, faceBounds,
                                             normalAxis});
    }
    if (boundaryMode && grid->boundaryConstraints.empty()) {
        throw std::invalid_argument(
            "selected boundary faces do not expose a measurable free side; select the inner cavity faces");
    }
    grid->enforceGlobalBoundaryPlanes =
        boundaryMode && grid->boundaryConstraints.size() > 1;
    // A nonempty intersection can still be just a narrow fragment of the
    // cavity. In particular, a short floor patch can clip the normal axis of
    // a selected side wall even though the opposite wall is a real solid.
    // Treat that case like an empty intersection and let the seed, oriented
    // wall planes, and solid collision determine the reachable component.
    if (seedPoint.has_value() && strictTangentialWindowIsNarrow(*grid)) {
        grid->enforceTangentialIntersection = false;
    }

    std::optional<gp_Pnt> seed;
    if (seedPoint.has_value()) {
        // A point explicitly selected by the user is the strongest seed
        // signal.  Accept it before applying face half-spaces; the selected
        // point then determines which side of intersecting faces is the
        // intended cavity side.
        seed = useSelectedSeed(grid, *seedPoint, true);
    }
    if (!seed.has_value() && boundaryMode) {
        // First locate a free point in the finite selected-face envelope,
        // without assuming that all face normals already agree.  This is
        // important for overlapping faces and STEP assemblies with holes or
        // small gaps between patches.
        seed = findBoundarySeed(grid, alignedBoundaryFaces,
                                options.cellSizeMm, true);
    }
    if (seed.has_value() && boundaryMode) {
        orientBoundaryConstraintsToSeed(grid, *seed);
        if (!useSelectedSeed(grid, *seed)) {
            // The seed may lie in a valid cavity while split-face tangent
            // ranges do not overlap exactly.  Keep the oriented global planes
            // and use the union envelope as the finite local window.
            grid->enforceTangentialIntersection = false;
            if (!useSelectedSeed(grid, *seed)) seed.reset();
        }
    }
    if (!seed.has_value() && boundaryMode) {
        // Retain the original constrained search as a conservative fallback
        // for selections where the centroid neighborhood was occupied.
        seed = findBoundarySeed(grid, alignedBoundaryFaces,
                                options.cellSizeMm);
    }
    if (!seed.has_value() && boundaryMode &&
        grid->enforceGlobalBoundaryPlanes &&
        grid->enforceTangentialIntersection) {
        // Do not discard the selected planes when a split-face or modeling
        // tolerance makes their finite tangential footprints fail to overlap.
        // The retry uses the union AABB as a local window, but the same
        // globally oriented one-sided planes and exact solid checks remain in
        // force, so exterior air cannot be accepted merely because a face is
        // large or slightly offset.
        grid->enforceTangentialIntersection = false;
        if (seedPoint.has_value()) {
            seed = useSelectedSeed(grid, *seedPoint);
        }
        if (!seed.has_value()) {
            seed = findBoundarySeed(grid, alignedBoundaryFaces,
                                    options.cellSizeMm);
        }
        if (!seed.has_value()) {
            seed = findSeed(grid, entryPoint, flow,
                            options.seedOffsetMm,
                            options.maximumSeedSearchMm);
        }
    }
    if (!seed.has_value()) {
        seed = findSeed(grid, entryPoint, flow,
                        options.seedOffsetMm,
                        options.maximumSeedSearchMm);
    }
    if (!seed.has_value()) {
        throw std::invalid_argument(
            "could not find free space behind the selected entry; "
            "click a visible point in the intended cavity or select its opening boundary");
    }
    // A failed strict tangent intersection used to fall back to the union
    // AABB of every selected face.  Localize that fallback around the actual
    // free-space seed before flood filling, otherwise separated pockets can
    // be admitted into one apparent magazine.
    localizeBoundaryWindowToSeed(grid, *seed);
    if (!useSelectedSeed(grid, *seed)) {
        throw std::invalid_argument(
            "selected boundary fallback does not contain the chosen interior point; "
            "select boundary faces from one target cavity");
    }
    floodFill(grid, *seed, options.maximumCells);
    rejectUnboundedHorizontalFlood(grid);
    const std::size_t reachableCount = static_cast<std::size_t>(
        std::count(grid->reachable.begin(), grid->reachable.end(), 1));
    cropToReachableComponent(grid, 22.0);

    AssemblyPackingRegion result;
    result.region.bounds = grid->bounds;
    result.region.containsSphere = [grid](const magazine::packing::Vec3& center,
                                          double radius, double tolerance) {
        return grid->sphereClear(center, radius, tolerance);
    };
    result.region.verticalInterval = [grid](double x, double y, double radius)
        -> std::optional<magazine::packing::VerticalInterval> {
        return intervalForColumn(grid, x, y, radius, 1.0e-5);
    };
    result.region.coarseVerticalInterval =
        [grid](double x, double y, double radius)
            -> std::optional<magazine::packing::VerticalInterval> {
        return coarseIntervalForColumn(grid, x, y, radius);
    };
    result.region.entryFace = magazine::packing::EntryFaceInfo{
        {entryPoint.X() - grid->origin.X(),
         entryPoint.Y() - grid->origin.Y(),
         entryPoint.Z() - grid->origin.Z()},
        {flow.X(), flow.Y(), flow.Z()}};
    gp_Trsf cropTransform;
    cropTransform.SetTranslationPart(
        gp_Vec(-grid->origin.X(), -grid->origin.Y(), -grid->origin.Z()));
    // Display the same local obstacle set used by the solver. Showing the
    // entire vehicle made it impossible to visually tell whether the selected
    // cavity was isolated, especially for an assembly with hundreds of
    // unrelated solids.
    BRep_Builder displayBuilder;
    TopoDS_Compound displayCompound;
    displayBuilder.MakeCompound(displayCompound);
    for (const auto& obstacle : grid->obstacles) {
        const TopoDS_Shape croppedObstacle =
            BRepBuilderAPI_Transform(obstacle.shape, cropTransform, true).Shape();
        if (!croppedObstacle.IsNull()) {
            displayBuilder.Add(displayCompound, croppedObstacle);
        }
    }
    result.displayShape = displayCompound.IsNull()
        ? BRepBuilderAPI_Transform(alignedShape, cropTransform, true).Shape()
        : TopoDS_Shape(displayCompound);
    if (!alignedEntry.IsNull() && alignedEntry.ShapeType() == TopAbs_FACE) {
        const TopoDS_Shape croppedEntry =
            BRepBuilderAPI_Transform(alignedEntry, cropTransform, true).Shape();
        if (!croppedEntry.IsNull() && croppedEntry.ShapeType() == TopAbs_FACE) {
            result.displayEntryFace = TopoDS::Face(croppedEntry);
        }
    }
    result.obstacleCount = grid->obstacles.size();
    result.sourceSolidCount = sourceSolidCount;
    result.classifiedObstacleCount = static_cast<std::size_t>(std::count_if(
        grid->obstacles.begin(), grid->obstacles.end(),
        [](const Obstacle& obstacle) {
            return static_cast<bool>(obstacle.classifier);
        }));
    result.reachableCellCount = reachableCount;
    result.selectedBoundaryFaceCount = boundaryFaces.size();
    result.boundaryConstraintCount = grid->boundaryConstraints.size();
    result.boundarySolidCount = boundarySolidIndices.size();
    result.cellSizeMm = options.cellSizeMm;
    result.usedTangentialEnvelopeFallback =
        boundaryMode && !grid->enforceTangentialIntersection;
    result.usedLocalizedFallbackWindow = grid->localizedBoundaryWindow;
    result.sourceTopologyValid = model.topologyValid;
    return result;
}

AssemblyPackingRegion makeAssemblyPackingRegion(
    const CadImportResult& model, const gp_Dir& gravityDirection,
    const std::vector<TopoDS_Face>& boundaryFaces,
    const AssemblyExtractionOptions& options) {
    return makeAssemblyPackingRegionImpl(model, gravityDirection,
                                         TopoDS_Face(), boundaryFaces,
                                         options);
}

} // namespace magazine::cad

namespace magazine::cad {

AssemblyPackingRegion makeAssemblyPackingRegion(
    const CadImportResult& model, const gp_Dir& gravityDirection,
    const TopoDS_Face& entryFace, const AssemblyExtractionOptions& options) {
    return makeAssemblyPackingRegionImpl(model, gravityDirection, entryFace,
                                         {}, options);
}

AssemblyPackingRegion makeAssemblyPackingRegion(
    const CadImportResult& model, const gp_Dir& gravityDirection,
    const gp_Pnt& seedPointSource, const AssemblyExtractionOptions& options) {
    AssemblyExtractionOptions pointOptions = options;
    pointOptions.seedPointSource = seedPointSource;
    return makeAssemblyPackingRegionImpl(model, gravityDirection, TopoDS_Face(),
                                         {}, pointOptions);
}

} // namespace magazine::cad
