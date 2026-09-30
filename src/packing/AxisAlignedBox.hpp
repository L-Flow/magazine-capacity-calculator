#pragma once

#include "packing/PackingTypes.hpp"

#include <algorithm>

namespace magazine::packing {

struct AxisAlignedBox {
    double widthMm{0.0};
    double depthMm{0.0};
    double heightMm{0.0};

    bool validFor(double radius) const {
        return radius > 0.0 && widthMm >= 2.0 * radius &&
               depthMm >= 2.0 * radius && heightMm >= 2.0 * radius;
    }

    bool containsSphere(const Vec3& center, double radius,
                        double tolerance = 1.0e-7) const {
        return center.x >= radius - tolerance &&
               center.y >= radius - tolerance &&
               center.z >= radius - tolerance &&
               center.x <= widthMm - radius + tolerance &&
               center.y <= depthMm - radius + tolerance &&
               center.z <= heightMm - radius + tolerance;
    }

    void projectAgainstFloorAndWalls(Vec3& center, double radius) const {
        center.x = std::clamp(center.x, radius, widthMm - radius);
        center.y = std::clamp(center.y, radius, depthMm - radius);
        center.z = std::max(center.z, radius);
    }
};

} // namespace magazine::packing

