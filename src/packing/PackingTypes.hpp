#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace magazine::packing {

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

inline Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 operator-(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 operator*(const Vec3& value, double scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

inline Vec3& operator+=(Vec3& a, const Vec3& b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

inline Vec3& operator-=(Vec3& a, const Vec3& b) {
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

inline double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double squaredLength(const Vec3& value) {
    return dot(value, value);
}

struct PackingResult {
    std::string method;
    double sphereRadiusMm{0.0};
    std::vector<Vec3> centers;
    std::size_t rejectedCount{0};
    std::uint64_t seed{0};
    // The solver may return a valid partial packing when an interactive
    // caller reaches its runtime budget or requests cancellation.
    bool stoppedByTimeLimit{false};
    bool stoppedByCancellation{false};
};

} // namespace magazine::packing

