#include "volume_surface/SurfaceNormalField.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace volume_surface {
namespace {

constexpr double kEpsilon = 1.0e-20;

struct BucketKey
{
    int x = 0;
    int y = 0;
    int z = 0;

    [[nodiscard]] bool operator==(const BucketKey& other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct BucketHasher
{
    [[nodiscard]] std::size_t operator()(const BucketKey& key) const noexcept
    {
        std::size_t value = static_cast<std::uint32_t>(key.x);
        value = (value * 0x9e3779b9U) ^ static_cast<std::uint32_t>(key.y);
        value = (value * 0x9e3779b9U) ^ static_cast<std::uint32_t>(key.z);
        return value;
    }
};

struct NeighborCandidate
{
    std::size_t ordinal = 0;
    double distanceSquared = 0.0;
};

double wendlandC2(double normalizedDistance)
{
    if (normalizedDistance >= 1.0) {
        return 0.0;
    }
    const double remaining = 1.0 - normalizedDistance;
    return remaining * remaining * remaining * remaining *
        (4.0 * normalizedDistance + 1.0);
}

openvdb::Vec3d normalizeOrZero(openvdb::Vec3d normal)
{
    const double length = normal.length();
    if (!std::isfinite(length) || length <= kEpsilon) {
        return openvdb::Vec3d(0.0);
    }
    return normal / length;
}

class CoreSpatialIndex final
{
public:
    CoreSpatialIndex(
        const SurfaceTargetCache& target,
        const std::vector<std::size_t>& sampleIndices,
        double cellSize)
        : mTarget(&target)
        , mCellSize(std::max(cellSize, 1.0e-9))
        , mSampleIndices(sampleIndices)
    {
        bool hasOrigin = false;
        for (const std::size_t sampleIndex : sampleIndices) {
            const openvdb::Vec3d position(target.samples[sampleIndex].worldPosition);
            if (!hasOrigin) {
                mOrigin = position;
                hasOrigin = true;
            } else {
                mOrigin.x() = std::min(mOrigin.x(), position.x());
                mOrigin.y() = std::min(mOrigin.y(), position.y());
                mOrigin.z() = std::min(mOrigin.z(), position.z());
            }
        }
        mBuckets.reserve(sampleIndices.size() / 2 + 1);
        for (std::size_t ordinal = 0; ordinal < sampleIndices.size(); ++ordinal) {
            const auto& sample = target.samples[sampleIndices[ordinal]];
            mBuckets[bucketFor(openvdb::Vec3d(sample.worldPosition))].push_back(ordinal);
        }
    }

    template <typename Visitor>
    void visit(
        const openvdb::Vec3d& position,
        double radius,
        Visitor&& visitor) const
    {
        const double queryRadius = std::max(radius, 1.0e-9);
        const double radiusSquared = queryRadius * queryRadius;
        const BucketKey minimum = bucketFor(position - openvdb::Vec3d(queryRadius));
        const BucketKey maximum = bucketFor(position + openvdb::Vec3d(queryRadius));
        for (int z = minimum.z; z <= maximum.z; ++z) {
            for (int y = minimum.y; y <= maximum.y; ++y) {
                for (int x = minimum.x; x <= maximum.x; ++x) {
                    const auto found = mBuckets.find(BucketKey{x, y, z});
                    if (found == mBuckets.end()) {
                        continue;
                    }
                    for (const std::size_t ordinal : found->second) {
                        const auto sampleIndex = mSampleIndices[ordinal];
                        const openvdb::Vec3d delta =
                            openvdb::Vec3d(mTarget->samples[sampleIndex].worldPosition) -
                            position;
                        const double distanceSquared = delta.lengthSqr();
                        if (distanceSquared <= radiusSquared) {
                            visitor(ordinal, distanceSquared);
                        }
                    }
                }
            }
        }
    }

private:
    [[nodiscard]] BucketKey bucketFor(const openvdb::Vec3d& position) const
    {
        return BucketKey{
            static_cast<int>(std::floor((position.x() - mOrigin.x()) / mCellSize)),
            static_cast<int>(std::floor((position.y() - mOrigin.y()) / mCellSize)),
            static_cast<int>(std::floor((position.z() - mOrigin.z()) / mCellSize))};
    }

    const SurfaceTargetCache* mTarget = nullptr;
    double mCellSize = 1.0;
    openvdb::Vec3d mOrigin{0.0};
    const std::vector<std::size_t>& mSampleIndices;
    std::unordered_map<BucketKey, std::vector<std::size_t>, BucketHasher> mBuckets;
};

std::vector<NeighborCandidate> collectNeighbors(
    const CoreSpatialIndex& index,
    const openvdb::Vec3d& position,
    double radius,
    std::size_t maximumNeighbors)
{
    std::vector<NeighborCandidate> candidates;
    candidates.reserve(maximumNeighbors + 8);
    index.visit(position, radius, [&](std::size_t ordinal, double distanceSquared) {
        candidates.push_back({ordinal, distanceSquared});
    });
    if (candidates.size() > maximumNeighbors) {
        std::nth_element(
            candidates.begin(),
            candidates.begin() + static_cast<std::ptrdiff_t>(maximumNeighbors),
            candidates.end(),
            [](const NeighborCandidate& lhs, const NeighborCandidate& rhs) {
                return lhs.distanceSquared < rhs.distanceSquared;
            });
        candidates.resize(maximumNeighbors);
    }
    return candidates;
}

double confidenceFor(const SurfaceTargetSample& sample)
{
    return 0.25 + 0.75 * std::clamp(static_cast<double>(sample.planarity), 0.0, 1.0);
}

double angularWeight(double dot, double sigmaRadians)
{
    const double angle = std::sqrt(std::max(0.0, 2.0 * (1.0 - dot)));
    const double normalized = angle / std::max(sigmaRadians, 1.0e-6);
    return std::exp(-0.5 * normalized * normalized);
}

} // namespace

SurfaceNormalField smoothSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalSmoothingSettings& settings)
{
    if (target.empty()) {
        return {};
    }
    if (!std::isfinite(settings.radius) || settings.radius <= 0.0 ||
        !std::isfinite(settings.strength) || settings.strength < 0.0 ||
        settings.strength > 1.0 || settings.iterations == 0 ||
        !std::isfinite(settings.angleSigmaRadians) ||
        settings.angleSigmaRadians <= 0.0 ||
        !std::isfinite(settings.sheetThickness) || settings.sheetThickness <= 0.0 ||
        settings.maximumNeighbors == 0) {
        throw std::invalid_argument("surface normal smoothing settings are invalid");
    }

    SurfaceNormalField result;
    result.normals.resize(target.samples.size());
    result.coreCount = target.coreCount;
    for (std::size_t index = 0; index < target.samples.size(); ++index) {
        result.normals[index] = target.samples[index].normal;
    }

    std::vector<std::size_t> coreSampleIndices;
    coreSampleIndices.reserve(target.coreCount);
    for (std::size_t index = 0; index < target.samples.size(); ++index) {
        if (target.samples[index].kind == SurfaceTargetSampleKind::Core &&
            normalizeOrZero(openvdb::Vec3d(target.samples[index].normal)).lengthSqr() > kEpsilon) {
            coreSampleIndices.push_back(index);
        }
    }
    if (coreSampleIndices.empty()) {
        return result;
    }

    CoreSpatialIndex spatialIndex(target, coreSampleIndices, settings.radius);
    std::vector<openvdb::Vec3d> currentNormals(coreSampleIndices.size());
    std::vector<openvdb::Vec3d> nextNormals(coreSampleIndices.size());
    for (std::size_t ordinal = 0; ordinal < coreSampleIndices.size(); ++ordinal) {
        currentNormals[ordinal] = normalizeOrZero(openvdb::Vec3d(
            target.samples[coreSampleIndices[ordinal]].normal));
    }

    const double strength = std::clamp(settings.strength, 0.0, 1.0);
    const std::size_t maximumNeighbors = std::max<std::size_t>(1, settings.maximumNeighbors);
    for (std::size_t iteration = 0; iteration < settings.iterations; ++iteration) {
        tbb::parallel_for(
            tbb::blocked_range<std::size_t>(0, coreSampleIndices.size()),
            [&](const tbb::blocked_range<std::size_t>& range) {
                for (std::size_t ordinal = range.begin(); ordinal != range.end(); ++ordinal) {
                    const std::size_t sampleIndex = coreSampleIndices[ordinal];
                    const auto& sample = target.samples[sampleIndex];
                    const openvdb::Vec3d position(sample.worldPosition);
                    const openvdb::Vec3d base = currentNormals[ordinal];
                    if (base.lengthSqr() <= kEpsilon) {
                        nextNormals[ordinal] = base;
                        continue;
                    }

                    openvdb::Vec3d weightedNormal = base;
                    double weightSum = 1.0;
                    const auto candidates = collectNeighbors(
                        spatialIndex,
                        position,
                        settings.radius,
                        maximumNeighbors);
                    for (const auto& candidate : candidates) {
                        if (candidate.ordinal == ordinal) {
                            continue;
                        }
                        openvdb::Vec3d neighborNormal = currentNormals[candidate.ordinal];
                        if (neighborNormal.lengthSqr() <= kEpsilon) {
                            continue;
                        }
                        const auto& neighborSample = target.samples[
                            coreSampleIndices[candidate.ordinal]];
                        const openvdb::Vec3d delta =
                            openvdb::Vec3d(neighborSample.worldPosition) - position;
                        const double distance = std::sqrt(
                            std::max(candidate.distanceSquared, 0.0));
                        const double distanceWeight = wendlandC2(
                            distance / settings.radius);
                        if (distanceWeight <= 1.0e-12) {
                            continue;
                        }
                        double normalDot = neighborNormal.dot(base);
                        if (normalDot < 0.0) {
                            neighborNormal = -neighborNormal;
                            normalDot = -normalDot;
                        }
                        const double offset = std::abs(delta.dot(base));
                        const double sheetNormalized = offset / settings.sheetThickness;
                        const double sheetWeight = std::exp(
                            -0.5 * sheetNormalized * sheetNormalized);
                        const double weight = distanceWeight *
                            angularWeight(std::clamp(normalDot, 0.0, 1.0),
                                settings.angleSigmaRadians) *
                            sheetWeight * confidenceFor(neighborSample);
                        if (weight <= 1.0e-12) {
                            continue;
                        }
                        weightedNormal += neighborNormal * weight;
                        weightSum += weight;
                    }

                    const openvdb::Vec3d average = normalizeOrZero(
                        weightedNormal / std::max(weightSum, 1.0e-12));
                    const double blend = strength * confidenceFor(sample);
                    nextNormals[ordinal] = normalizeOrZero(
                        base * (1.0 - blend) + average * blend);
                }
            });
        currentNormals.swap(nextNormals);
    }

    for (std::size_t ordinal = 0; ordinal < coreSampleIndices.size(); ++ordinal) {
        const auto normal = currentNormals[ordinal];
        if (normal.lengthSqr() <= kEpsilon) {
            continue;
        }
        result.normals[coreSampleIndices[ordinal]] = openvdb::Vec3f(normal);
    }
    result.smoothedCoreCount = coreSampleIndices.size();

    std::vector<std::uint8_t> transitionFlags(target.samples.size(), 0);
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, target.samples.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            for (std::size_t sampleIndex = range.begin();
                 sampleIndex != range.end();
                 ++sampleIndex) {
                if (target.samples[sampleIndex].kind != SurfaceTargetSampleKind::Transition) {
                    continue;
                }
                const auto& sample = target.samples[sampleIndex];
                const openvdb::Vec3d position(sample.worldPosition);
                const openvdb::Vec3d rawNormal = normalizeOrZero(
                    openvdb::Vec3d(sample.normal));
                if (rawNormal.lengthSqr() <= kEpsilon) {
                    continue;
                }
                const auto candidates = collectNeighbors(
                    spatialIndex,
                    position,
                    settings.radius,
                    maximumNeighbors);
                openvdb::Vec3d weightedNormal(0.0);
                double weightSum = 0.0;
                for (const auto& candidate : candidates) {
                    const openvdb::Vec3d neighborNormal = currentNormals[candidate.ordinal];
                    if (neighborNormal.lengthSqr() <= kEpsilon) {
                        continue;
                    }
                    const auto& neighborSample = target.samples[
                        coreSampleIndices[candidate.ordinal]];
                    const openvdb::Vec3d delta =
                        openvdb::Vec3d(neighborSample.worldPosition) - position;
                    const double distanceWeight = wendlandC2(
                        std::sqrt(std::max(candidate.distanceSquared, 0.0)) /
                        settings.radius);
                    double normalDot = neighborNormal.dot(rawNormal);
                    openvdb::Vec3d oriented = neighborNormal;
                    if (normalDot < 0.0) {
                        oriented = -oriented;
                        normalDot = -normalDot;
                    }
                    const double offset = std::abs(delta.dot(rawNormal));
                    const double sheetNormalized = offset / settings.sheetThickness;
                    const double sheetWeight = std::exp(
                        -0.5 * sheetNormalized * sheetNormalized);
                    const double weight = distanceWeight *
                        angularWeight(std::clamp(normalDot, 0.0, 1.0),
                            settings.angleSigmaRadians) *
                        sheetWeight * neighborSample.supportWeight;
                    if (weight <= 1.0e-12) {
                        continue;
                    }
                    weightedNormal += oriented * weight;
                    weightSum += weight;
                }
                if (weightSum <= 1.0e-12) {
                    continue;
                }
                const openvdb::Vec3d average = normalizeOrZero(
                    weightedNormal / weightSum);
                const double blend = strength * confidenceFor(sample);
                const openvdb::Vec3d filtered = normalizeOrZero(
                    rawNormal * (1.0 - blend) + average * blend);
                if (filtered.lengthSqr() > kEpsilon) {
                    result.normals[sampleIndex] = openvdb::Vec3f(filtered);
                    transitionFlags[sampleIndex] = 1;
                }
            }
        });

    result.smoothedTransitionCount = static_cast<std::size_t>(
        std::count(transitionFlags.begin(), transitionFlags.end(),
            static_cast<std::uint8_t>(1)));

    return result;
}

} // namespace volume_surface
