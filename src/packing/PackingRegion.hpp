#pragma once

#include "packing/AxisAlignedBox.hpp"

#include <functional>
#include <optional>

namespace magazine::packing {

struct VerticalInterval {
    double floorZ{0.0};
    double ceilingZ{0.0};
};

struct EntryFaceInfo {
    Vec3 center;
    Vec3 inwardNormal;
};

struct PackingRegion {
    AxisAlignedBox bounds;
    std::function<bool(const Vec3&, double, double)> containsSphere;
    std::function<std::optional<VerticalInterval>(double, double, double)>
        verticalInterval;
    std::optional<EntryFaceInfo> entryFace;

    bool validFor(double radius) const {
        return bounds.validFor(radius) && containsSphere && verticalInterval;
    }
};

inline PackingRegion boxPackingRegion(const AxisAlignedBox& box) {
    return {
        box,
        [box](const Vec3& center, double radius, double tolerance) {
            return box.containsSphere(center, radius, tolerance);
        },
        [box](double x, double y, double radius)
            -> std::optional<VerticalInterval> {
            if (x < radius || x > box.widthMm - radius || y < radius ||
                y > box.depthMm - radius || box.heightMm < 2.0 * radius) {
                return std::nullopt;
            }
            return VerticalInterval{radius, box.heightMm - radius};
        },
    };
}

} // namespace magazine::packing
