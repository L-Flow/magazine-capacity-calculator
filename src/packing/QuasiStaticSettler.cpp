#include "packing/QuasiStaticSettler.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_map>

namespace magazine::packing {
namespace {

struct CellKey {
    int x{0};
    int y{0};

    bool operator==(const CellKey& other) const {
        return x == other.x && y == other.y;
    }
};

struct CellKeyHash {
    std::size_t operator()(const CellKey& key) const {
        const auto x = static_cast<std::uint32_t>(key.x);
        const auto y = static_cast<std::uint32_t>(key.y);
        return (static_cast<std::size_t>(x) << 32U) ^ y;
    }
};

class SupportIndex {
public:
    explicit SupportIndex(double cellSize) : cellSize_(cellSize) {}

    void insert(const Vec3& center, std::size_t index) {
        cells_[cellFor(center.x, center.y)].push_back(index);
    }

    double restingHeight(double x, double y,
                         const std::vector<Vec3>& centers, double radius,
                         double floorHeight) const {
        const CellKey origin = cellFor(x, y);
        const double diameter = 2.0 * radius;
        const double diameterSquared = diameter * diameter;
        double height = floorHeight;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                const auto cell = cells_.find({origin.x + dx, origin.y + dy});
                if (cell == cells_.end()) continue;
                for (const std::size_t index : cell->second) {
                    const Vec3& other = centers[index];
                    const double offsetX = x - other.x;
                    const double offsetY = y - other.y;
                    const double horizontalSquared =
                        offsetX * offsetX + offsetY * offsetY;
                    if (horizontalSquared >= diameterSquared) continue;
                    const double vertical = std::sqrt(
                        std::max(0.0, diameterSquared - horizontalSquared));
                    height = std::max(height, other.z + vertical);
                }
            }
        }
        return height;
    }

private:
    CellKey cellFor(double x, double y) const {
        return {static_cast<int>(std::floor(x / cellSize_)),
                static_cast<int>(std::floor(y / cellSize_))};
    }

    double cellSize_;
    std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> cells_;
};

Vec3 restingPosition(double x, double y, const PackingRegion& region,
                     const std::vector<Vec3>& centers,
                     const SupportIndex& supportIndex, double radius) {
    const double supportHeight = supportIndex.restingHeight(
        x, y, centers, radius, 0.0);
    const auto interval = region.verticalInterval(x, y, radius);
    if (!interval.has_value()) {
        return {x, y, std::numeric_limits<double>::infinity()};
    }
    const double height = std::max(supportHeight, interval->floorZ);
    if (height > interval->ceilingZ + 1.0e-7) {
        return {x, y, std::numeric_limits<double>::infinity()};
    }
    return {x, y, height};
}

Vec3 relaxDownhill(Vec3 candidate, const PackingRegion& region,
                   const std::vector<Vec3>& centers, double radius,
                   const SupportIndex& supportIndex,
    const SettlingOptions& options) {
    double step = radius * options.initialLateralStepRadiusFactor;
    candidate = restingPosition(candidate.x, candidate.y, region, centers,
                                supportIndex, radius);

    for (int iteration = 0;
         iteration < options.maximumRelaxationIterations &&
         step >= options.minimumLateralStepMm;
         ++iteration) {
        if (options.cancellationRequested &&
            options.cancellationRequested()) {
            return {candidate.x, candidate.y,
                    std::numeric_limits<double>::infinity()};
        }
        Vec3 best = candidate;
        bool improved = false;
        for (int direction = 0; direction < options.relaxationDirections;
             ++direction) {
            if (options.cancellationRequested &&
                options.cancellationRequested()) {
                return {candidate.x, candidate.y,
                        std::numeric_limits<double>::infinity()};
            }
            const double angle =
                2.0 * 3.14159265358979323846 * direction /
                static_cast<double>(options.relaxationDirections);
            Vec3 trial{candidate.x + step * std::cos(angle),
                       candidate.y + step * std::sin(angle), 0.0};
            if (trial.x < 0.0 || trial.x > region.bounds.widthMm ||
                trial.y < 0.0 || trial.y > region.bounds.depthMm) {
                continue;
            }
            const double supportHeight = supportIndex.restingHeight(
                trial.x, trial.y, centers, radius, 0.0);
            if (supportHeight + options.improvementToleranceMm >= best.z) {
                continue;
            }
            trial = restingPosition(trial.x, trial.y, region, centers,
                                    supportIndex, radius);
            if (trial.z + options.improvementToleranceMm < best.z) {
                best = trial;
                improved = true;
            }
        }
        if (improved) {
            candidate = best;
        } else {
            step *= 0.5;
        }
    }
    return candidate;
}

} // namespace

PackingResult settleWithoutFriction(const AxisAlignedBox& box, double radius,
                                    const SettlingOptions& options) {
    return settleWithoutFriction(boxPackingRegion(box), radius, options);
}

PackingResult settleWithoutFriction(const PackingRegion& region, double radius,
                                    const SettlingOptions& options) {
    if (!region.validFor(radius)) {
        throw std::invalid_argument(
            "packing region is invalid for the selected sphere");
    }
    if (options.failedInsertionsBeforeStop == 0 ||
        options.candidateTrialsPerSphere == 0 ||
        options.relaxationDirections < 4 ||
        options.maximumRelaxationIterations <= 0 ||
        options.initialLateralStepRadiusFactor <= 0.0 ||
        options.minimumLateralStepMm <= 0.0) {
        throw std::invalid_argument("invalid settling options");
    }

    std::mt19937_64 random(options.seed);
    std::uniform_real_distribution<double> xDistribution(
        0.0, region.bounds.widthMm);
    std::uniform_real_distribution<double> yDistribution(
        0.0, region.bounds.depthMm);

    std::vector<Vec3> centers;
    SupportIndex supportIndex(2.0 * radius);
    std::size_t consecutiveFailures = 0;
    std::size_t rejected = 0;

    while (consecutiveFailures < options.failedInsertionsBeforeStop) {
        if (options.cancellationRequested &&
            options.cancellationRequested()) {
            break;
        }
        Vec3 best;
        best.z = std::numeric_limits<double>::infinity();
        for (std::size_t trialIndex = 0;
             trialIndex < options.candidateTrialsPerSphere; ++trialIndex) {
            if (options.cancellationRequested &&
                options.cancellationRequested()) {
                break;
            }
            Vec3 candidate{xDistribution(random), yDistribution(random), 0.0};
            candidate = relaxDownhill(candidate, region, centers, radius,
                                      supportIndex, options);
            if (candidate.z < best.z &&
                region.containsSphere(candidate, radius, 1.0e-5)) {
                best = candidate;
            }
        }

        if (std::isfinite(best.z)) {
            centers.push_back(best);
            supportIndex.insert(best, centers.size() - 1);
            consecutiveFailures = 0;
        } else {
            ++consecutiveFailures;
            ++rejected;
        }
    }

    std::sort(centers.begin(), centers.end(), [](const Vec3& a,
                                                 const Vec3& b) {
        if (a.z != b.z) return a.z < b.z;
        if (a.y != b.y) return a.y < b.y;
        return a.x < b.x;
    });
    return {"frictionless quasi-static settling", radius,
            std::move(centers), rejected, options.seed};
}

} // namespace magazine::packing
