#include "packing/QuasiStaticSettler.hpp"
#include "packing/LatticePacking.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_map>

namespace magazine::packing {
namespace {

using SettlingClock = std::chrono::steady_clock;

bool timeLimitReached(const SettlingClock::time_point& deadline) {
    return deadline != SettlingClock::time_point::max() &&
           SettlingClock::now() >= deadline;
}

bool stopRequested(const SettlingOptions& options,
                   const SettlingClock::time_point& deadline) {
    return timeLimitReached(deadline) ||
           (options.cancellationRequested && options.cancellationRequested());
}

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

    void rebuild(const std::vector<Vec3>& centers) {
        cells_.clear();
        for (std::size_t index = 0; index < centers.size(); ++index) {
            insert(centers[index], index);
        }
    }

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
                     const SupportIndex& supportIndex, double radius,
                     bool exactGeometry,
                     const SettlingClock::time_point& deadline) {
    if (timeLimitReached(deadline)) {
        return {x, y, std::numeric_limits<double>::infinity()};
    }
    const double supportHeight = supportIndex.restingHeight(
        x, y, centers, radius, 0.0);
    if (!exactGeometry && region.coarseVerticalInterval) {
        const auto coarse = region.coarseVerticalInterval(x, y, radius);
        if (coarse.has_value()) {
            const double coarseHeight =
                std::max(supportHeight, coarse->floorZ);
            if (coarseHeight <= coarse->ceilingZ + 1.0e-7) {
                return {x, y, coarseHeight};
            }
        }
    }
    if (timeLimitReached(deadline)) {
        return {x, y, std::numeric_limits<double>::infinity()};
    }
    const auto interval = region.verticalInterval(x, y, radius);
    if (timeLimitReached(deadline)) {
        return {x, y, std::numeric_limits<double>::infinity()};
    }
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
                   const SettlingOptions& options,
                   const SettlingClock::time_point& deadline) {
    double step = radius * options.initialLateralStepRadiusFactor;
    candidate = restingPosition(candidate.x, candidate.y, region, centers,
                                supportIndex, radius, false, deadline);

    for (int iteration = 0;
         iteration < options.maximumRelaxationIterations &&
         step >= options.minimumLateralStepMm;
         ++iteration) {
        if (stopRequested(options, deadline)) {
            return {candidate.x, candidate.y,
                    std::numeric_limits<double>::infinity()};
        }
        Vec3 best = candidate;
        bool improved = false;
        for (int direction = 0; direction < options.relaxationDirections;
             ++direction) {
            if (stopRequested(options, deadline)) {
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
                                    supportIndex, radius, false, deadline);
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
        options.minimumLateralStepMm <= 0.0 ||
        options.systematicSweepSpacingDiameterFactor <= 0.0) {
        throw std::invalid_argument("invalid settling options");
    }

    const auto deadline = options.maximumRuntimeMilliseconds == 0
        ? SettlingClock::time_point::max()
        : SettlingClock::now() + std::chrono::milliseconds(
              options.maximumRuntimeMilliseconds);
    bool stoppedByTimeLimit = false;
    bool stoppedByCancellation = false;
    const auto shouldStop = [&]() {
        if (timeLimitReached(deadline)) stoppedByTimeLimit = true;
        if (options.cancellationRequested &&
            options.cancellationRequested()) {
            stoppedByCancellation = true;
        }
        return stoppedByTimeLimit || stoppedByCancellation;
    };

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
        if (shouldStop()) break;
        Vec3 best;
        best.z = std::numeric_limits<double>::infinity();
        std::vector<Vec3> candidates;
        candidates.reserve(options.candidateTrialsPerSphere);
        for (std::size_t trialIndex = 0;
             trialIndex < options.candidateTrialsPerSphere; ++trialIndex) {
            if (shouldStop()) break;
            Vec3 candidate{xDistribution(random), yDistribution(random), 0.0};
            candidate = relaxDownhill(candidate, region, centers, radius,
                                      supportIndex, options, deadline);
            if (std::isfinite(candidate.z)) candidates.push_back(candidate);
        }

        if (shouldStop()) break;

        std::sort(candidates.begin(), candidates.end(),
                  [](const Vec3& lhs, const Vec3& rhs) {
                      if (lhs.z != rhs.z) return lhs.z < rhs.z;
                      if (lhs.y != rhs.y) return lhs.y < rhs.y;
                      return lhs.x < rhs.x;
                  });
        bool accepted = false;
        if (!region.coarseVerticalInterval) {
            // Even when the region has no cheap voxel interval, the final
            // entity predicate is the authoritative wall check.  A support
            // sphere can raise a candidate into a non-column part of a
            // concave CAD cavity, so verticalInterval alone is insufficient.
            for (const Vec3& candidate : candidates) {
                if (shouldStop()) break;
                if (region.containsSphere(candidate, radius, 1.0e-5)) {
                    best = candidate;
                    accepted = true;
                    break;
                }
            }
        } else {
            for (const Vec3& coarseCandidate : candidates) {
                if (shouldStop()) break;
                // The voxel interval is only a conservative ordering and
                // rejection accelerator. Refine the few lowest candidates
                // against the exact CAD geometry before accepting one.
                const Vec3 candidate = restingPosition(
                    coarseCandidate.x, coarseCandidate.y, region, centers,
                    supportIndex, radius, true, deadline);
                if (std::isfinite(candidate.z) &&
                    region.containsSphere(candidate, radius, 1.0e-5)) {
                    best = candidate;
                    accepted = true;
                    break;
                }
            }
        }
        if (accepted) {
            centers.push_back(best);
            supportIndex.insert(best, centers.size() - 1);
            consecutiveFailures = 0;
        } else {
            ++consecutiveFailures;
            ++rejected;
        }
    }

    // Random drops are useful for avoiding a single lattice bias, but they
    // can stop with large empty holes in a CAD cavity. A small deterministic
    // XY sweep gives those holes a second chance without becoming an
    // unbounded global optimizer. Evaluate a batch with the cheap coarse
    // interval first, then do exact CAD checks only for its lowest candidates.
    // The sweep intentionally does not downhill-relax its XY columns: that
    // relaxation is what makes separated low pockets steal candidates from
    // otherwise empty middle columns.
    if (!shouldStop()) {
        const double spacing = std::max(
            2.0 * radius * options.systematicSweepSpacingDiameterFactor,
            2.0 * options.minimumLateralStepMm);
        for (std::size_t pass = 0; pass < options.systematicSweepPasses;
             ++pass) {
            bool insertedInPass = false;
            std::size_t tested = 0;
            const double offset = (pass % 2 == 0) ? 0.0 : 0.5 * spacing;
            std::vector<Vec3> batch;
            batch.reserve(std::max<std::size_t>(8,
                                                options.candidateTrialsPerSphere));
            auto flushBatch = [&]() {
                bool insertedAny = false;
                while (!batch.empty() && !shouldStop()) {
                    std::sort(batch.begin(), batch.end(),
                              [](const Vec3& lhs, const Vec3& rhs) {
                                  if (lhs.z != rhs.z) return lhs.z < rhs.z;
                                  if (lhs.y != rhs.y) return lhs.y < rhs.y;
                                  return lhs.x < rhs.x;
                              });
                    bool inserted = false;
                    for (std::size_t index = 0; index < batch.size();
                         ++index) {
                        if (shouldStop()) break;
                        const Vec3 coarseCandidate = batch[index];
                        const Vec3 candidate = restingPosition(
                            coarseCandidate.x, coarseCandidate.y, region,
                            centers, supportIndex, radius, true, deadline);
                        if (!std::isfinite(candidate.z) ||
                            !region.containsSphere(candidate, radius, 1.0e-5)) {
                            // This candidate is not usable with the current
                            // packing. Do not retry it in this batch.
                            batch.erase(batch.begin() +
                                        static_cast<std::ptrdiff_t>(index));
                            --index;
                            continue;
                        }
                        centers.push_back(candidate);
                        supportIndex.insert(candidate, centers.size() - 1);
                        insertedInPass = true;
                        insertedAny = true;
                        batch.erase(batch.begin() +
                                    static_cast<std::ptrdiff_t>(index));
                        inserted = true;
                        break;
                    }
                    if (!inserted) break;
                }
                batch.clear();
                return insertedAny;
            };
            for (double x = radius + offset;
                 x <= region.bounds.widthMm - radius &&
                 tested < options.systematicSweepMaximumCandidates;
                 x += spacing) {
                for (double y = radius + offset;
                     y <= region.bounds.depthMm - radius &&
                     tested < options.systematicSweepMaximumCandidates;
                     y += spacing) {
                    ++tested;
                    if (shouldStop()) {
                        break;
                    }
                    const Vec3 candidate =
                        restingPosition(x, y, region, centers, supportIndex,
                                        radius, false, deadline);
                    if (std::isfinite(candidate.z)) batch.push_back(candidate);
                    if (batch.size() >=
                        std::max<std::size_t>(8,
                                              options.candidateTrialsPerSphere)) {
                        flushBatch();
                    }
                }
                if (shouldStop()) break;
            }
            if (!shouldStop()) flushBatch();
            if (!insertedInPass) break;
        }
    }

    // A random drop sequence can permanently block the low positions needed
    // by a later sphere.  Build a second candidate packing from an empty
    // support index using the close-packed horizontal rows that are natural
    // under gravity.  Every center is still lowered against the actual
    // centers already accepted in this candidate and then checked against
    // the exact packing region; this is a deterministic recovery strategy,
    // not a substitution with the FCC/HCP reference result.
    if (!shouldStop()) {
        std::vector<Vec3> structuredCenters;
        SupportIndex structuredSupport(2.0 * radius);
        const double columnSpacing = 2.0 * radius;
        const double rowSpacing = std::sqrt(3.0) * radius;
        const std::size_t candidateLimit =
            std::max<std::size_t>(options.systematicSweepMaximumCandidates,
                                  1);
        std::size_t tested = 0;
        for (std::size_t pass = 0;
             pass < options.systematicSweepPasses && !shouldStop(); ++pass) {
            const bool offsetLayer = (pass & 1U) != 0U;
            const double phaseX = offsetLayer ? radius : 0.0;
            const double phaseY = offsetLayer ? rowSpacing / 3.0 : 0.0;
            bool insertedInPass = false;
            std::size_t row = 0;
            for (double y = radius + phaseY;
                 y <= region.bounds.depthMm - radius &&
                 tested < candidateLimit && !shouldStop();
                 y += rowSpacing, ++row) {
                const double rowOffset = (row & 1U) != 0U ? radius : 0.0;
                for (double x = radius + phaseX + rowOffset;
                     x <= region.bounds.widthMm - radius &&
                     tested < candidateLimit && !shouldStop();
                     x += columnSpacing) {
                    ++tested;
                    const Vec3 coarseCandidate = restingPosition(
                        x, y, region, structuredCenters, structuredSupport,
                        radius, false, deadline);
                    if (!std::isfinite(coarseCandidate.z)) continue;
                    const Vec3 candidate = restingPosition(
                        x, y, region, structuredCenters, structuredSupport,
                        radius, true, deadline);
                    if (!std::isfinite(candidate.z) ||
                        !region.containsSphere(candidate, radius, 1.0e-5)) {
                        continue;
                    }
                    structuredCenters.push_back(candidate);
                    structuredSupport.insert(candidate,
                                             structuredCenters.size() - 1);
                    insertedInPass = true;
                }
            }
            if (!insertedInPass) break;
        }
        if (structuredCenters.size() > centers.size()) {
            centers = std::move(structuredCenters);
            // The support index stores center indices.  Rebuild it whenever
            // a complete candidate packing replaces the current one; keeping
            // the old index would query support heights from stale centers.
            supportIndex.rebuild(centers);
        }
    }

    // Repair local voids left by a candidate packing.  A sphere that rests on
    // two nearby spheres has one of two horizontal circle-intersection
    // positions; testing those positions finds the triangular holes that a
    // rectangular sweep cannot see.  Rebuild the pair candidates after each
    // round because every accepted sphere creates new support pairs.  Keep
    // this as a callable pass so a later deterministic packing receives the
    // same local-hole repair as the random packing.
    const auto repairPairVoids = [&](std::size_t maximumRounds) {
        if (centers.size() < 2) return;
        const double diameter = 2.0 * radius;
        const double diameterSquared = diameter * diameter;
        const std::size_t pairCandidateLimit = std::max<std::size_t>(
            options.systematicSweepMaximumCandidates * 4, 256);
        for (std::size_t round = 0; round < maximumRounds &&
                                    !shouldStop();
             ++round) {
            std::vector<Vec3> pairCandidates;
            pairCandidates.reserve(std::min<std::size_t>(
                pairCandidateLimit, centers.size() * 4));
            for (std::size_t first = 0; first + 1 < centers.size() &&
                                          pairCandidates.size() + 1 <
                                              pairCandidateLimit;
                 ++first) {
                for (std::size_t second = first + 1;
                     second < centers.size() &&
                     pairCandidates.size() + 1 < pairCandidateLimit;
                     ++second) {
                    const double dx = centers[second].x - centers[first].x;
                    const double dy = centers[second].y - centers[first].y;
                    const double horizontalSquared = dx * dx + dy * dy;
                    if (horizontalSquared <= 1.0e-8 ||
                        horizontalSquared > diameterSquared + 1.0e-6) {
                        continue;
                    }
                    const double horizontal = std::sqrt(horizontalSquared);
                    const double offset = std::sqrt(std::max(
                        0.0, diameterSquared - horizontalSquared));
                    const double midX =
                        0.5 * (centers[first].x + centers[second].x);
                    const double midY =
                        0.5 * (centers[first].y + centers[second].y);
                    const double perpendicularX = -dy / horizontal;
                    const double perpendicularY = dx / horizontal;
                    pairCandidates.push_back(
                        {midX + perpendicularX * offset,
                         midY + perpendicularY * offset, 0.0});
                    if (pairCandidates.size() < pairCandidateLimit) {
                        pairCandidates.push_back(
                            {midX - perpendicularX * offset,
                             midY - perpendicularY * offset, 0.0});
                    }
                }
            }
            if (pairCandidates.empty()) break;

            std::sort(pairCandidates.begin(), pairCandidates.end(),
                      [](const Vec3& lhs, const Vec3& rhs) {
                          if (lhs.y != rhs.y) return lhs.y < rhs.y;
                          return lhs.x < rhs.x;
                      });
            bool insertedInRound = false;
            for (const Vec3& pairCandidate : pairCandidates) {
                if (shouldStop()) break;
                const Vec3 candidate = restingPosition(
                    pairCandidate.x, pairCandidate.y, region, centers,
                    supportIndex, radius, true, deadline);
                if (!std::isfinite(candidate.z) ||
                    !region.containsSphere(candidate, radius, 1.0e-5)) {
                    continue;
                }
                centers.push_back(candidate);
                supportIndex.insert(candidate, centers.size() - 1);
                insertedInRound = true;
            }
            if (!insertedInRound) break;
        }
    };

    repairPairVoids(options.systematicSweepPasses);

    // Use the close-packed horizontal phases as search proposals for a
    // fresh gravity build.  The lattice itself is never copied into the
    // result: each proposed XY position is lowered against the newly built
    // support index and checked by the exact packing-region predicate.  This
    // recovers holes caused by early random blocking while preserving the
    // quasi-static settling semantics.
    bool usedDeterministicCompaction = false;
    if (!shouldStop()) {
        const auto compactGuide = [&](const PackingResult& guide) {
            std::vector<Vec3> compactedCenters;
            compactedCenters.reserve(guide.centers.size());
            SupportIndex compactedSupport(2.0 * radius);
            for (const Vec3& proposal : guide.centers) {
                if (shouldStop()) break;
                const Vec3 candidate = restingPosition(
                    proposal.x, proposal.y, region, compactedCenters,
                    compactedSupport, radius, true, deadline);
                if (!std::isfinite(candidate.z) ||
                    !region.containsSphere(candidate, radius, 1.0e-5)) {
                    continue;
                }
                compactedCenters.push_back(candidate);
                compactedSupport.insert(candidate,
                                         compactedCenters.size() - 1);
            }
            return compactedCenters;
        };

        LatticeOptions hcpOptions;
        // Three phase offsets give the guided build enough freedom to align
        // the close-packed rows with stepped walls while remaining bounded
        // for the interactive CAD time budget.
        hcpOptions.phaseDivisions = 3;
        const PackingResult hcpGuide =
            packHcp(region, radius, hcpOptions);
        std::vector<Vec3> compactedCenters = compactGuide(hcpGuide);
        if (compactedCenters.size() > centers.size()) {
            centers = std::move(compactedCenters);
            // The deterministic candidate packing has its own support index;
            // make the main index agree before any later repair pass.
            supportIndex.rebuild(centers);
            usedDeterministicCompaction = true;
        }
    }

    // HCP-guided compaction fills the large-scale layers first.  Run a short
    // contact repair afterwards as well: the rebuilt packing can expose
    // triangular interstices that were absent from the random candidate.
    if (!shouldStop()) repairPairVoids(std::min<std::size_t>(
        options.systematicSweepPasses, 3));

    // A close-packed guide has a fixed horizontal phase.  A short finer
    // sweep after compaction checks the interstices around CAD steps and
    // rounded obstacles that do not coincide with that phase.  Candidates
    // are still lowered against the current support index and accepted only
    // after the exact region predicate, so this cannot create an out-of-wall
    // sphere or an overlap.
    if (!shouldStop()) {
        const double fineSpacing = std::max(
            2.0 * radius * 0.60, 2.0 * options.minimumLateralStepMm);
        const std::size_t fineCandidateLimit = std::min<std::size_t>(
            options.systematicSweepMaximumCandidates, 5000);
        for (std::size_t pass = 0; pass < 2 && !shouldStop(); ++pass) {
            const double offset = (pass == 0) ? 0.0 : 0.5 * fineSpacing;
            bool insertedInPass = false;
            std::size_t tested = 0;
            for (double x = radius + offset;
                 x <= region.bounds.widthMm - radius &&
                 tested < fineCandidateLimit && !shouldStop();
                 x += fineSpacing) {
                for (double y = radius + offset;
                     y <= region.bounds.depthMm - radius &&
                     tested < fineCandidateLimit && !shouldStop();
                     y += fineSpacing) {
                    ++tested;
                    const Vec3 coarseCandidate = restingPosition(
                        x, y, region, centers, supportIndex, radius, false,
                        deadline);
                    if (!std::isfinite(coarseCandidate.z)) continue;
                    const Vec3 candidate = restingPosition(
                        x, y, region, centers, supportIndex, radius, true,
                        deadline);
                    if (!std::isfinite(candidate.z) ||
                        !region.containsSphere(candidate, radius, 1.0e-5)) {
                        continue;
                    }
                    centers.push_back(candidate);
                    supportIndex.insert(candidate, centers.size() - 1);
                    insertedInPass = true;
                }
            }
            if (!insertedInPass) break;
        }
    }

    std::sort(centers.begin(), centers.end(), [](const Vec3& a,
                                                 const Vec3& b) {
        if (a.z != b.z) return a.z < b.z;
        if (a.y != b.y) return a.y < b.y;
        return a.x < b.x;
    });
    return {usedDeterministicCompaction
                ? "frictionless quasi-static settling (deterministic compaction)"
                : "frictionless quasi-static settling",
            radius,
            std::move(centers), rejected, options.seed,
            stoppedByTimeLimit, stoppedByCancellation};
}

} // namespace magazine::packing
