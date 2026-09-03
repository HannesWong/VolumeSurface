#include "volume_surface/SurfaceNormalField.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <openvdb/tools/Interpolation.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace volume_surface {
namespace {

constexpr double kEpsilon = 1.0e-20;
constexpr std::array<openvdb::Coord, 6> kAxisNeighbors{{
    openvdb::Coord(-1, 0, 0),
    openvdb::Coord(1, 0, 0),
    openvdb::Coord(0, -1, 0),
    openvdb::Coord(0, 1, 0),
    openvdb::Coord(0, 0, -1),
    openvdb::Coord(0, 0, 1)}};

bool isInside(
    const openvdb::FloatGrid& grid,
    double value,
    double isoValue)
{
    return grid.getGridClass() == openvdb::GRID_FOG_VOLUME
        ? value >= isoValue
        : value <= isoValue;
}

struct CoordHasher
{
    [[nodiscard]] std::size_t operator()(
        const openvdb::Coord& coordinate) const noexcept
    {
        std::size_t value = static_cast<std::uint32_t>(coordinate.x());
        value = (value * 0x9e3779b9U) ^
            static_cast<std::uint32_t>(coordinate.y());
        value = (value * 0x9e3779b9U) ^
            static_cast<std::uint32_t>(coordinate.z());
        return value;
    }
};

struct NeighborhoodDefinition
{
    int halfExtent = 1;
    std::size_t maximumSamples = 9;
};

struct NeighborhoodCandidate
{
    std::size_t sampleIndex = 0;
    std::uint8_t hops = 0;
    double projectedDistance = 0.0;
};

using Matrix3 = std::array<std::array<double, 3>, 3>;

openvdb::Vec3d normalizeOrZero(openvdb::Vec3d value)
{
    const double length = value.length();
    if (!std::isfinite(length) || length <= kEpsilon) {
        return openvdb::Vec3d(0.0);
    }
    return value / length;
}

openvdb::Vec3d smallestEigenvector(Matrix3 matrix)
{
    Matrix3 eigenvectors{{
        {{1.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0}},
        {{0.0, 0.0, 1.0}}}};
    for (int iteration = 0; iteration < 24; ++iteration) {
        int p = 0;
        int q = 1;
        double largest = std::abs(matrix[0][1]);
        if (std::abs(matrix[0][2]) > largest) {
            p = 0;
            q = 2;
            largest = std::abs(matrix[0][2]);
        }
        if (std::abs(matrix[1][2]) > largest) {
            p = 1;
            q = 2;
            largest = std::abs(matrix[1][2]);
        }
        if (largest <= 1.0e-14) {
            break;
        }

        const double app = matrix[p][p];
        const double aqq = matrix[q][q];
        const double apq = matrix[p][q];
        const double angle = 0.5 * std::atan2(2.0 * apq, aqq - app);
        const double cosine = std::cos(angle);
        const double sine = std::sin(angle);
        for (int k = 0; k < 3; ++k) {
            if (k == p || k == q) {
                continue;
            }
            const double mkp = matrix[k][p];
            const double mkq = matrix[k][q];
            matrix[k][p] = matrix[p][k] = cosine * mkp - sine * mkq;
            matrix[k][q] = matrix[q][k] = sine * mkp + cosine * mkq;
        }
        matrix[p][p] = cosine * cosine * app -
            2.0 * sine * cosine * apq + sine * sine * aqq;
        matrix[q][q] = sine * sine * app +
            2.0 * sine * cosine * apq + cosine * cosine * aqq;
        matrix[p][q] = matrix[q][p] = 0.0;
        for (int k = 0; k < 3; ++k) {
            const double vkp = eigenvectors[k][p];
            const double vkq = eigenvectors[k][q];
            eigenvectors[k][p] = cosine * vkp - sine * vkq;
            eigenvectors[k][q] = sine * vkp + cosine * vkq;
        }
    }

    int smallest = 0;
    if (matrix[1][1] < matrix[smallest][smallest]) {
        smallest = 1;
    }
    if (matrix[2][2] < matrix[smallest][smallest]) {
        smallest = 2;
    }
    return normalizeOrZero(openvdb::Vec3d(
        eigenvectors[0][smallest],
        eigenvectors[1][smallest],
        eigenvectors[2][smallest]));
}

NeighborhoodDefinition fitNeighborhoodDefinition(
    SurfaceFitNeighborhood neighborhood)
{
    switch (neighborhood) {
        case SurfaceFitNeighborhood::Grid5x5:
            return {2, 25};
        case SurfaceFitNeighborhood::Grid9x9:
            return {4, 81};
        case SurfaceFitNeighborhood::Grid3x3:
            return {1, 9};
    }
    return {1, 9};
}

NeighborhoodDefinition normalNeighborhoodDefinition(
    SurfaceNormalNeighborhood neighborhood)
{
    switch (neighborhood) {
        case SurfaceNormalNeighborhood::Grid5x5:
            return {2, 25};
        case SurfaceNormalNeighborhood::Grid3x3:
            return {1, 9};
        case SurfaceNormalNeighborhood::None:
            return {0, 1};
    }
    return {0, 1};
}

void tangentBasis(
    const openvdb::Vec3d& normal,
    openvdb::Vec3d& tangentU,
    openvdb::Vec3d& tangentV)
{
    const openvdb::Vec3d axis = std::abs(normal.x()) < 0.9
        ? openvdb::Vec3d(1.0, 0.0, 0.0)
        : openvdb::Vec3d(0.0, 1.0, 0.0);
    tangentU = normalizeOrZero(normal.cross(axis));
    tangentV = normalizeOrZero(normal.cross(tangentU));
}

int localIndex(int x, int y, int z, int halfExtent, int side)
{
    return (z + halfExtent) * side * side +
        (y + halfExtent) * side + x + halfExtent;
}

void decodeLocalIndex(
    int index,
    int halfExtent,
    int side,
    int& x,
    int& y,
    int& z)
{
    const int plane = side * side;
    z = index / plane - halfExtent;
    const int remainder = index % plane;
    y = remainder / side - halfExtent;
    x = remainder % side - halfExtent;
}

std::vector<NeighborhoodCandidate> collectConnectedNeighborhood(
    const SurfaceTargetCache& target,
    const std::unordered_map<openvdb::Coord, std::size_t, CoordHasher>& coreIndices,
    std::size_t centerIndex,
    const NeighborhoodDefinition& definition)
{
    const int halfExtent = definition.halfExtent;
    const int side = halfExtent * 2 + 1;
    const int centerCell = localIndex(0, 0, 0, halfExtent, side);
    std::array<int, 729> localOrdinals{};
    localOrdinals.fill(-1);
    const openvdb::Coord center = target.samples[centerIndex].coordinate;
    for (int z = -halfExtent; z <= halfExtent; ++z) {
        for (int y = -halfExtent; y <= halfExtent; ++y) {
            for (int x = -halfExtent; x <= halfExtent; ++x) {
                const auto found = coreIndices.find(center.offsetBy(x, y, z));
                if (found != coreIndices.end()) {
                    localOrdinals[localIndex(x, y, z, halfExtent, side)] =
                        static_cast<int>(found->second);
                }
            }
        }
    }
    if (localOrdinals[centerCell] < 0) {
        return {};
    }

    const openvdb::Vec3d seedNormal = [&]() {
        openvdb::Vec3d value = normalizeOrZero(
            openvdb::Vec3d(target.samples[centerIndex].normal));
        return value.lengthSqr() > kEpsilon
            ? value
            : openvdb::Vec3d(0.0, 0.0, 1.0);
    }();
    openvdb::Vec3d tangentU;
    openvdb::Vec3d tangentV;
    tangentBasis(seedNormal, tangentU, tangentV);

    std::array<std::uint8_t, 729> visited{};
    std::array<std::uint8_t, 729> hops{};
    std::array<int, 729> queue{};
    int queueBegin = 0;
    int queueEnd = 0;
    visited[centerCell] = 1;
    queue[queueEnd++] = centerCell;

    std::vector<NeighborhoodCandidate> candidates;
    candidates.reserve(definition.maximumSamples + 16);
    while (queueBegin < queueEnd) {
        const int cell = queue[queueBegin++];
        int x = 0;
        int y = 0;
        int z = 0;
        decodeLocalIndex(cell, halfExtent, side, x, y, z);
        const std::size_t sampleIndex = static_cast<std::size_t>(localOrdinals[cell]);
        const openvdb::Vec3d delta =
            openvdb::Vec3d(target.samples[sampleIndex].worldPosition) -
            openvdb::Vec3d(target.samples[centerIndex].worldPosition);
        const double projectedDistance = std::sqrt(
            std::max(0.0, std::pow(delta.dot(tangentU), 2.0) +
                std::pow(delta.dot(tangentV), 2.0)));
        candidates.push_back({
            sampleIndex,
            hops[cell],
            projectedDistance});

        if (hops[cell] >= static_cast<std::uint8_t>(halfExtent)) {
            continue;
        }
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const int nx = x + dx;
                    const int ny = y + dy;
                    const int nz = z + dz;
                    if (nx < -halfExtent || nx > halfExtent ||
                        ny < -halfExtent || ny > halfExtent ||
                        nz < -halfExtent || nz > halfExtent) {
                        continue;
                    }
                    const int neighborCell = localIndex(
                        nx,
                        ny,
                        nz,
                        halfExtent,
                        side);
                    if (visited[neighborCell] || localOrdinals[neighborCell] < 0) {
                        continue;
                    }
                    visited[neighborCell] = 1;
                    hops[neighborCell] = static_cast<std::uint8_t>(hops[cell] + 1);
                    queue[queueEnd++] = neighborCell;
                }
            }
        }
    }

    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [](const NeighborhoodCandidate& left, const NeighborhoodCandidate& right) {
            if (left.hops != right.hops) {
                return left.hops < right.hops;
            }
            if (left.projectedDistance != right.projectedDistance) {
                return left.projectedDistance < right.projectedDistance;
            }
            return left.sampleIndex < right.sampleIndex;
        });
    if (candidates.size() > definition.maximumSamples) {
        candidates.resize(definition.maximumSamples);
    }
    return candidates;
}

std::vector<NeighborhoodCandidate> collectTransitionNeighborhood(
    const SurfaceTargetCache& target,
    const std::unordered_map<openvdb::Coord, std::size_t, CoordHasher>& coreIndices,
    std::size_t centerIndex,
    const NeighborhoodDefinition& definition)
{
    const int halfExtent = definition.halfExtent;
    const openvdb::Coord center = target.samples[centerIndex].coordinate;
    const openvdb::Vec3d seedNormal = [&]() {
        openvdb::Vec3d value = normalizeOrZero(
            openvdb::Vec3d(target.samples[centerIndex].normal));
        return value.lengthSqr() > kEpsilon
            ? value
            : openvdb::Vec3d(0.0, 0.0, 1.0);
    }();
    openvdb::Vec3d tangentU;
    openvdb::Vec3d tangentV;
    tangentBasis(seedNormal, tangentU, tangentV);

    std::vector<NeighborhoodCandidate> candidates;
    candidates.reserve(definition.maximumSamples + 8);
    for (int z = -halfExtent; z <= halfExtent; ++z) {
        for (int y = -halfExtent; y <= halfExtent; ++y) {
            for (int x = -halfExtent; x <= halfExtent; ++x) {
                const auto found = coreIndices.find(center.offsetBy(x, y, z));
                if (found == coreIndices.end()) {
                    continue;
                }
                const std::size_t sampleIndex = found->second;
                const openvdb::Vec3d delta =
                    openvdb::Vec3d(target.samples[sampleIndex].worldPosition) -
                    openvdb::Vec3d(target.samples[centerIndex].worldPosition);
                const double projectedDistance = std::sqrt(
                    std::max(0.0, std::pow(delta.dot(tangentU), 2.0) +
                        std::pow(delta.dot(tangentV), 2.0)));
                candidates.push_back({
                    sampleIndex,
                    static_cast<std::uint8_t>(
                        std::max({std::abs(x), std::abs(y), std::abs(z)})),
                    projectedDistance});
            }
        }
    }
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [](const NeighborhoodCandidate& left, const NeighborhoodCandidate& right) {
            if (left.hops != right.hops) {
                return left.hops < right.hops;
            }
            if (left.projectedDistance != right.projectedDistance) {
                return left.projectedDistance < right.projectedDistance;
            }
            return left.sampleIndex < right.sampleIndex;
        });
    if (candidates.size() > definition.maximumSamples) {
        candidates.resize(definition.maximumSamples);
    }
    return candidates;
}

bool fitTrendNormal(
    const SurfaceTargetCache& target,
    const std::vector<NeighborhoodCandidate>& candidates,
    std::size_t centerIndex,
    std::size_t robustIterations,
    openvdb::Vec3d& normal)
{
    if (candidates.size() < 3) {
        return false;
    }
    const openvdb::Vec3d seedNormal = normalizeOrZero(
        openvdb::Vec3d(target.samples[centerIndex].normal));
    if (seedNormal.lengthSqr() <= kEpsilon) {
        return false;
    }
    std::vector<double> baseWeights(candidates.size(), 1.0);
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        const auto& sample = target.samples[candidates[index].sampleIndex];
        const double planarity = std::clamp(
            static_cast<double>(sample.planarity),
            0.0,
            1.0);
        baseWeights[index] =
            (0.5 + 0.5 * planarity) /
            (1.0 + 0.5 * static_cast<double>(candidates[index].hops));
    }
    std::vector<double> weights = baseWeights;
    const std::size_t iterations = std::max<std::size_t>(1, robustIterations);
    for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
        openvdb::Vec3d mean(0.0);
        double weightSum = 0.0;
        for (std::size_t index = 0; index < candidates.size(); ++index) {
            const openvdb::Vec3d position(
                target.samples[candidates[index].sampleIndex].worldPosition);
            mean += position * weights[index];
            weightSum += weights[index];
        }
        if (weightSum <= kEpsilon) {
            return false;
        }
        mean /= weightSum;
        Matrix3 covariance{{
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}},
            {{0.0, 0.0, 0.0}}}};
        for (std::size_t index = 0; index < candidates.size(); ++index) {
            const openvdb::Vec3d position(
                target.samples[candidates[index].sampleIndex].worldPosition);
            const openvdb::Vec3d delta = position - mean;
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    covariance[row][column] += weights[index] *
                        delta[row] * delta[column];
                }
            }
        }
        normal = smallestEigenvector(covariance);
        if (normal.lengthSqr() <= kEpsilon) {
            return false;
        }
        if (normal.dot(seedNormal) < 0.0) {
            normal = -normal;
        }
        if (iteration + 1 >= iterations) {
            break;
        }

        std::vector<double> residuals;
        residuals.reserve(candidates.size());
        for (const auto& candidate : candidates) {
            const openvdb::Vec3d position(
                target.samples[candidate.sampleIndex].worldPosition);
            residuals.push_back(std::abs((position - mean).dot(normal)));
        }
        std::vector<double> sortedResiduals = residuals;
        std::nth_element(
            sortedResiduals.begin(),
            sortedResiduals.begin() + sortedResiduals.size() / 2,
            sortedResiduals.end());
        const double median = sortedResiduals[sortedResiduals.size() / 2];
        const double scale = std::max(1.0e-6, median * 1.4826);
        for (std::size_t index = 0; index < candidates.size(); ++index) {
            const double normalized = residuals[index] / (2.5 * scale);
            const double robustWeight = normalized <= 1.0
                ? 1.0
                : 1.0 / std::max(normalized, 1.0);
            weights[index] = baseWeights[index] * robustWeight;
        }
    }
    return true;
}

} // namespace

void collectCoreIndices(
    const SurfaceTargetCache& target,
    const std::vector<openvdb::Vec3f>& normals,
    std::vector<std::size_t>& coreSampleIndices,
    std::unordered_map<openvdb::Coord, std::size_t, CoordHasher>& coreIndices)
{
    coreSampleIndices.clear();
    coreIndices.clear();
    coreSampleIndices.reserve(target.coreCount);
    coreIndices.reserve(target.coreCount * 2 + 1);
    for (std::size_t index = 0; index < target.samples.size(); ++index) {
        if (target.samples[index].kind != SurfaceTargetSampleKind::Core ||
            index >= normals.size() ||
            normalizeOrZero(openvdb::Vec3d(normals[index])).lengthSqr() <= kEpsilon) {
            continue;
        }
        coreIndices.emplace(target.samples[index].coordinate, index);
        coreSampleIndices.push_back(index);
    }
}

openvdb::Vec3d surfacePointForOrientation(
    const openvdb::FloatGrid& grid,
    const SurfaceTargetSample& sample,
    double isoValue,
    const openvdb::FloatGrid::ConstAccessor& accessor)
{
    openvdb::Vec3d point(sample.worldPosition);
    if (sample.kind != SurfaceTargetSampleKind::Core) {
        return point;
    }

    const openvdb::Vec3d center = grid.indexToWorld(
        sample.coordinate.asVec3d());
    const bool centerInside = isInside(grid, sample.density, isoValue);
    openvdb::Vec3d crossingSum(0.0);
    std::size_t crossingCount = 0;
    for (const auto& direction : kAxisNeighbors) {
        const openvdb::Coord neighborCoordinate = sample.coordinate + direction;
        const double neighborValue = accessor.getValue(neighborCoordinate);
        if (centerInside == isInside(grid, neighborValue, isoValue)) {
            continue;
        }
        const double denominator = neighborValue - static_cast<double>(sample.density);
        const double interpolation = std::abs(denominator) > 1.0e-12
            ? std::clamp((isoValue - static_cast<double>(sample.density)) / denominator, 0.0, 1.0)
            : 0.5;
        const openvdb::Vec3d neighbor = grid.indexToWorld(
            neighborCoordinate.asVec3d());
        crossingSum += center + (neighbor - center) * interpolation;
        ++crossingCount;
    }
    return crossingCount > 0
        ? crossingSum / static_cast<double>(crossingCount)
        : point;
}

void orientNormalsOutward(
    const openvdb::FloatGrid& grid,
    const SurfaceTargetCache& target,
    double isoValue,
    std::vector<openvdb::Vec3f>& normals)
{
    const openvdb::Vec3d voxelSize = grid.voxelSize();
    const double minimumVoxelSize = std::min({
        std::abs(voxelSize.x()),
        std::abs(voxelSize.y()),
        std::abs(voxelSize.z())});
    const double probeDistance = std::max(1.0e-6, minimumVoxelSize * 0.25);
    if (!std::isfinite(probeDistance) || probeDistance <= 0.0) {
        return;
    }

    const bool fogVolume = grid.getGridClass() == openvdb::GRID_FOG_VOLUME;
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, target.samples.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            const auto accessor = grid.getConstAccessor();
            openvdb::tools::GridSampler<
                openvdb::FloatGrid,
                openvdb::tools::BoxSampler> sampler(grid);
            for (std::size_t index = range.begin(); index != range.end(); ++index) {
                openvdb::Vec3d normal = normalizeOrZero(
                    openvdb::Vec3d(normals[index]));
                if (normal.lengthSqr() <= kEpsilon) {
                    continue;
                }
                const openvdb::Vec3d point = surfacePointForOrientation(
                    grid,
                    target.samples[index],
                    isoValue,
                    accessor);
                const double plus = sampler.wsSample(point + normal * probeDistance);
                const double minus = sampler.wsSample(point - normal * probeDistance);
                const double difference = plus - minus;
                if (!std::isfinite(difference) || std::abs(difference) <= 1.0e-4) {
                    continue;
                }
                const bool pointsTowardIncreasingDensity = difference > 0.0;
                const bool pointsInward = fogVolume
                    ? pointsTowardIncreasingDensity
                    : !pointsTowardIncreasingDensity;
                if (pointsInward) {
                    normals[index] = openvdb::Vec3f(-normal);
                }
            }
        });
}

SurfaceNormalField fitSurfaceTargetNormalsImpl(
    const SurfaceTargetCache& target,
    const SurfaceNormalFitSettings& settings,
    const openvdb::FloatGrid* orientationGrid)
{
    if (target.empty()) {
        return {};
    }
    if (settings.robustIterations == 0 || !std::isfinite(settings.isoValue)) {
        throw std::invalid_argument("surface fit settings are invalid");
    }

    const NeighborhoodDefinition definition = fitNeighborhoodDefinition(
        settings.neighborhood);
    SurfaceNormalField result;
    result.normals.resize(target.samples.size());
    result.coreCount = target.coreCount;
    for (std::size_t index = 0; index < target.samples.size(); ++index) {
        result.normals[index] = target.samples[index].normal;
    }

    std::vector<std::size_t> coreSampleIndices;
    std::unordered_map<openvdb::Coord, std::size_t, CoordHasher> coreIndices;
    collectCoreIndices(target, result.normals, coreSampleIndices, coreIndices);
    if (coreSampleIndices.empty()) {
        return result;
    }

    std::vector<std::uint8_t> coreFlags(coreSampleIndices.size(), 0);
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, coreSampleIndices.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            for (std::size_t ordinal = range.begin(); ordinal != range.end(); ++ordinal) {
                const std::size_t sampleIndex = coreSampleIndices[ordinal];
                const auto candidates = collectConnectedNeighborhood(
                    target,
                    coreIndices,
                    sampleIndex,
                    definition);
                openvdb::Vec3d trendNormal(0.0);
                if (!fitTrendNormal(
                        target,
                        candidates,
                        sampleIndex,
                        settings.robustIterations,
                        trendNormal)) {
                    continue;
                }
                const openvdb::Vec3d filtered = normalizeOrZero(trendNormal);
                if (filtered.lengthSqr() <= kEpsilon) {
                    continue;
                }
                result.normals[sampleIndex] = openvdb::Vec3f(filtered);
                coreFlags[ordinal] = 1;
            }
        });
    result.smoothedCoreCount = static_cast<std::size_t>(
        std::count(coreFlags.begin(), coreFlags.end(), static_cast<std::uint8_t>(1)));

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
                const auto candidates = collectTransitionNeighborhood(
                    target,
                    coreIndices,
                    sampleIndex,
                    definition);
                const openvdb::Vec3d rawNormal = normalizeOrZero(
                    openvdb::Vec3d(result.normals[sampleIndex]));
                openvdb::Vec3d normalSum(0.0);
                double weightSum = 0.0;
                for (const auto& candidate : candidates) {
                    const openvdb::Vec3d candidateNormal = normalizeOrZero(
                        openvdb::Vec3d(result.normals[candidate.sampleIndex]));
                    if (candidateNormal.lengthSqr() <= kEpsilon) {
                        continue;
                    }
                    const auto& coreSample = target.samples[candidate.sampleIndex];
                    openvdb::Vec3d oriented = candidateNormal;
                    if (oriented.dot(rawNormal) < 0.0) {
                        oriented = -oriented;
                    }
                    const double weight =
                        (1.0 / (1.0 + static_cast<double>(candidate.hops))) *
                        std::clamp(static_cast<double>(coreSample.supportWeight), 0.0, 1.0);
                    normalSum += oriented * weight;
                    weightSum += weight;
                }
                const openvdb::Vec3d averaged = normalizeOrZero(
                    normalSum / std::max(weightSum, 1.0e-12));
                if (averaged.lengthSqr() <= kEpsilon) {
                    continue;
                }
                const openvdb::Vec3d filtered = normalizeOrZero(averaged);
                if (filtered.lengthSqr() <= kEpsilon) {
                    continue;
                }
                result.normals[sampleIndex] = openvdb::Vec3f(filtered);
                transitionFlags[sampleIndex] = 1;
            }
        });
    result.smoothedTransitionCount = static_cast<std::size_t>(
        std::count(
            transitionFlags.begin(),
            transitionFlags.end(),
            static_cast<std::uint8_t>(1)));
    if (orientationGrid) {
        orientNormalsOutward(
            *orientationGrid,
            target,
            settings.isoValue,
            result.normals);
    }
    return result;
}

SurfaceNormalField fitSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalFitSettings& settings)
{
    return fitSurfaceTargetNormalsImpl(target, settings, nullptr);
}

SurfaceNormalField fitSurfaceTargetNormals(
    const openvdb::FloatGrid& grid,
    const SurfaceTargetCache& target,
    const SurfaceNormalFitSettings& settings)
{
    return fitSurfaceTargetNormalsImpl(target, settings, &grid);
}

SurfaceNormalField smoothSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalField& seed,
    const SurfaceNormalSmoothingSettings& settings)
{
    if (target.empty()) {
        return {};
    }
    if (seed.normals.size() != target.samples.size()) {
        throw std::invalid_argument("surface normal seed size does not match target");
    }
    if (!std::isfinite(settings.strength) || settings.strength < 0.0 ||
        settings.strength > 1.0 || settings.robustIterations == 0) {
        throw std::invalid_argument("surface normal trend settings are invalid");
    }

    SurfaceNormalField result = seed;
    if (settings.neighborhood == SurfaceNormalNeighborhood::None) {
        result.smoothedCoreCount = 0;
        result.smoothedTransitionCount = 0;
        return result;
    }

    const NeighborhoodDefinition definition = normalNeighborhoodDefinition(
        settings.neighborhood);
    std::vector<std::size_t> coreSampleIndices;
    std::unordered_map<openvdb::Coord, std::size_t, CoordHasher> coreIndices;
    collectCoreIndices(target, seed.normals, coreSampleIndices, coreIndices);
    if (coreSampleIndices.empty()) {
        return result;
    }

    std::vector<std::uint8_t> coreFlags(coreSampleIndices.size(), 0);
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, coreSampleIndices.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            for (std::size_t ordinal = range.begin(); ordinal != range.end(); ++ordinal) {
                const std::size_t sampleIndex = coreSampleIndices[ordinal];
                const openvdb::Vec3d rawNormal = normalizeOrZero(
                    openvdb::Vec3d(seed.normals[sampleIndex]));
                const auto candidates = collectConnectedNeighborhood(
                    target,
                    coreIndices,
                    sampleIndex,
                    definition);
                openvdb::Vec3d normalSum(0.0);
                double weightSum = 0.0;
                for (const auto& candidate : candidates) {
                    openvdb::Vec3d normal = normalizeOrZero(
                        openvdb::Vec3d(seed.normals[candidate.sampleIndex]));
                    if (normal.lengthSqr() <= kEpsilon) {
                        continue;
                    }
                    if (normal.dot(rawNormal) < 0.0) {
                        normal = -normal;
                    }
                    const auto& sample = target.samples[candidate.sampleIndex];
                    const double weight =
                        (0.5 + 0.5 * std::clamp(static_cast<double>(sample.planarity), 0.0, 1.0)) /
                        (1.0 + static_cast<double>(candidate.hops));
                    normalSum += normal * weight;
                    weightSum += weight;
                }
                const openvdb::Vec3d averaged = normalizeOrZero(
                    normalSum / std::max(weightSum, 1.0e-12));
                const openvdb::Vec3d filtered = normalizeOrZero(
                    rawNormal * (1.0 - settings.strength) +
                    averaged * settings.strength);
                if (filtered.lengthSqr() <= kEpsilon) {
                    continue;
                }
                result.normals[sampleIndex] = openvdb::Vec3f(filtered);
                coreFlags[ordinal] = 1;
            }
        });
    result.smoothedCoreCount = static_cast<std::size_t>(
        std::count(coreFlags.begin(), coreFlags.end(), static_cast<std::uint8_t>(1)));

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
                const openvdb::Vec3d rawNormal = normalizeOrZero(
                    openvdb::Vec3d(seed.normals[sampleIndex]));
                if (rawNormal.lengthSqr() <= kEpsilon) {
                    continue;
                }
                const auto candidates = collectTransitionNeighborhood(
                    target,
                    coreIndices,
                    sampleIndex,
                    definition);
                openvdb::Vec3d normalSum(0.0);
                double weightSum = 0.0;
                for (const auto& candidate : candidates) {
                    openvdb::Vec3d normal = normalizeOrZero(
                        openvdb::Vec3d(seed.normals[candidate.sampleIndex]));
                    if (normal.lengthSqr() <= kEpsilon) {
                        continue;
                    }
                    if (normal.dot(rawNormal) < 0.0) {
                        normal = -normal;
                    }
                    const auto& coreSample = target.samples[candidate.sampleIndex];
                    const double weight =
                        (1.0 / (1.0 + static_cast<double>(candidate.hops))) *
                        std::clamp(static_cast<double>(coreSample.supportWeight), 0.0, 1.0);
                    normalSum += normal * weight;
                    weightSum += weight;
                }
                const openvdb::Vec3d averaged = normalizeOrZero(
                    normalSum / std::max(weightSum, 1.0e-12));
                const openvdb::Vec3d filtered = normalizeOrZero(
                    rawNormal * (1.0 - settings.strength) +
                    averaged * settings.strength);
                if (filtered.lengthSqr() <= kEpsilon) {
                    continue;
                }
                result.normals[sampleIndex] = openvdb::Vec3f(filtered);
                transitionFlags[sampleIndex] = 1;
            }
        });
    result.smoothedTransitionCount = static_cast<std::size_t>(
        std::count(
            transitionFlags.begin(),
            transitionFlags.end(),
            static_cast<std::uint8_t>(1)));
    return result;
}

SurfaceNormalField smoothSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalSmoothingSettings& settings)
{
    SurfaceNormalFitSettings fitSettings;
    fitSettings.neighborhood = settings.neighborhood == SurfaceNormalNeighborhood::Grid5x5
        ? SurfaceFitNeighborhood::Grid5x5
        : SurfaceFitNeighborhood::Grid3x3;
    fitSettings.robustIterations = settings.robustIterations;
    const SurfaceNormalField seed = fitSurfaceTargetNormals(target, fitSettings);
    return smoothSurfaceTargetNormals(target, seed, settings);
}

} // namespace volume_surface
