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

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace volume_surface {
namespace {

constexpr double kEpsilon = 1.0e-20;

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
    double indexDistance = 0.0;
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

openvdb::Vec3d largestEigenvector(const Matrix3& matrix)
{
    Matrix3 negated = matrix;
    for (auto& row : negated) {
        for (double& value : row) {
            value = -value;
        }
    }
    return smallestEigenvector(negated);
}

void addNormalTensor(Matrix3& tensor, const openvdb::Vec3d& value, double weight)
{
    const openvdb::Vec3d normal = normalizeOrZero(value);
    if (!std::isfinite(weight) || weight <= 0.0 ||
        normal.lengthSqr() <= kEpsilon) {
        return;
    }
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            tensor[row][column] += weight * normal[row] * normal[column];
        }
    }
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
        const double indexDistance = std::sqrt(
            static_cast<double>(x * x + y * y + z * z));
        candidates.push_back({
            sampleIndex,
            hops[cell],
            indexDistance});

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
            if (left.indexDistance != right.indexDistance) {
                return left.indexDistance < right.indexDistance;
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
                const double indexDistance = std::sqrt(
                    static_cast<double>(x * x + y * y + z * z));
                candidates.push_back({
                    sampleIndex,
                    static_cast<std::uint8_t>(
                        std::max({std::abs(x), std::abs(y), std::abs(z)})),
                    indexDistance});
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
            if (left.indexDistance != right.indexDistance) {
                return left.indexDistance < right.indexDistance;
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
    std::size_t robustIterations,
    openvdb::Vec3d& normal)
{
    if (candidates.size() < 3) {
        return false;
    }
    std::vector<double> baseWeights(candidates.size(), 1.0);
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        baseWeights[index] =
            1.0 /
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
    std::vector<std::size_t>& coreSampleIndices,
    std::unordered_map<openvdb::Coord, std::size_t, CoordHasher>& coreIndices)
{
    coreSampleIndices.clear();
    coreIndices.clear();
    coreSampleIndices.reserve(target.coreCount);
    coreIndices.reserve(target.coreCount * 2 + 1);
    for (std::size_t index = 0; index < target.samples.size(); ++index) {
        if (target.samples[index].kind != SurfaceTargetSampleKind::Core ||
            !openvdb::Vec3d(target.samples[index].worldPosition).isFinite()) {
            continue;
        }
        coreIndices.emplace(target.samples[index].coordinate, index);
        coreSampleIndices.push_back(index);
    }
}

SurfaceNormalField fitSurfaceTargetNormalsImpl(
    const SurfaceTargetCache& target,
    const SurfaceNormalFitSettings& settings,
    const openvdb::FloatGrid* orientationGrid)
{
    // The grid argument is retained for API compatibility. Fit orientation is
    // intentionally derived from point positions only.
    (void)orientationGrid;
    if (target.empty()) {
        return {};
    }
    if (settings.robustIterations == 0 || !std::isfinite(settings.isoValue)) {
        throw std::invalid_argument("surface fit settings are invalid");
    }

    const NeighborhoodDefinition definition = fitNeighborhoodDefinition(
        settings.neighborhood);
    SurfaceNormalField result;
    result.normals.assign(target.samples.size(), openvdb::Vec3f(0.0f));
    result.coreCount = target.coreCount;

    std::vector<std::size_t> coreSampleIndices;
    std::unordered_map<openvdb::Coord, std::size_t, CoordHasher> coreIndices;
    collectCoreIndices(target, coreSampleIndices, coreIndices);
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
                Matrix3 tensor{{
                    {{0.0, 0.0, 0.0}},
                    {{0.0, 0.0, 0.0}},
                    {{0.0, 0.0, 0.0}}}};
                double weightSum = 0.0;
                for (const auto& candidate : candidates) {
                    const openvdb::Vec3d candidateNormal = normalizeOrZero(
                        openvdb::Vec3d(result.normals[candidate.sampleIndex]));
                    if (candidateNormal.lengthSqr() <= kEpsilon) {
                        continue;
                    }
                    const auto& coreSample = target.samples[candidate.sampleIndex];
                    const double weight =
                        (1.0 / (1.0 + static_cast<double>(candidate.hops))) *
                        std::clamp(static_cast<double>(coreSample.supportWeight), 0.0, 1.0);
                    addNormalTensor(tensor, candidateNormal, weight);
                    weightSum += weight;
                }
                if (weightSum <= kEpsilon) {
                    continue;
                }
                const openvdb::Vec3d filtered = largestEigenvector(tensor);
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
    collectCoreIndices(target, coreSampleIndices, coreIndices);
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
                Matrix3 tensor{{
                    {{0.0, 0.0, 0.0}},
                    {{0.0, 0.0, 0.0}},
                    {{0.0, 0.0, 0.0}}}};
                double weightSum = 0.0;
                const openvdb::Vec3d centerNormal = normalizeOrZero(
                    openvdb::Vec3d(seed.normals[sampleIndex]));
                if (centerNormal.lengthSqr() > kEpsilon) {
                    addNormalTensor(
                        tensor,
                        centerNormal,
                        std::max(0.0, 1.0 - settings.strength));
                    weightSum += std::max(0.0, 1.0 - settings.strength);
                }
                for (const auto& candidate : candidates) {
                    openvdb::Vec3d normal = normalizeOrZero(
                        openvdb::Vec3d(seed.normals[candidate.sampleIndex]));
                    if (normal.lengthSqr() <= kEpsilon) {
                        continue;
                    }
                    const double weight = settings.strength /
                        (1.0 + static_cast<double>(candidate.hops));
                    addNormalTensor(tensor, normal, weight);
                    weightSum += weight;
                }
                if (weightSum <= kEpsilon) {
                    continue;
                }
                const openvdb::Vec3d filtered = largestEigenvector(tensor);
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
                Matrix3 tensor{{
                    {{0.0, 0.0, 0.0}},
                    {{0.0, 0.0, 0.0}},
                    {{0.0, 0.0, 0.0}}}};
                double weightSum = 0.0;
                const openvdb::Vec3d centerNormal = normalizeOrZero(
                    openvdb::Vec3d(seed.normals[sampleIndex]));
                if (centerNormal.lengthSqr() > kEpsilon) {
                    addNormalTensor(
                        tensor,
                        centerNormal,
                        std::max(0.0, 1.0 - settings.strength));
                    weightSum += std::max(0.0, 1.0 - settings.strength);
                }
                for (const auto& candidate : candidates) {
                    openvdb::Vec3d normal = normalizeOrZero(
                        openvdb::Vec3d(seed.normals[candidate.sampleIndex]));
                    if (normal.lengthSqr() <= kEpsilon) {
                        continue;
                    }
                    const auto& coreSample = target.samples[candidate.sampleIndex];
                    const double weight = settings.strength *
                        (1.0 / (1.0 + static_cast<double>(candidate.hops))) *
                        std::clamp(static_cast<double>(coreSample.supportWeight), 0.0, 1.0);
                    addNormalTensor(tensor, normal, weight);
                    weightSum += weight;
                }
                if (weightSum <= kEpsilon) {
                    continue;
                }
                const openvdb::Vec3d filtered = largestEigenvector(tensor);
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

SurfaceNormalField orientSurfaceTargetNormals(
    const SurfaceTargetCache& target,
    const SurfaceNormalField& axes,
    std::size_t seedSampleIndex,
    const openvdb::Vec3d& seedDirection,
    const SurfaceNormalOrientationSettings& settings)
{
    if (target.empty()) {
        return {};
    }
    if (axes.normals.size() != target.samples.size()) {
        throw std::invalid_argument("surface normal axis size does not match target");
    }
    if (!seedDirection.isFinite() || seedDirection.lengthSqr() <= kEpsilon ||
        !std::isfinite(settings.minimumAlignment) ||
        settings.minimumAlignment < 0.0 || settings.minimumAlignment > 1.0) {
        throw std::invalid_argument("surface normal orientation settings are invalid");
    }

    SurfaceNormalField result;
    result.normals.assign(target.samples.size(), openvdb::Vec3f(0.0f));
    result.coreCount = target.coreCount;

    std::vector<std::size_t> coreSampleIndices;
    std::unordered_map<openvdb::Coord, std::size_t, CoordHasher> coreIndices;
    collectCoreIndices(target, coreSampleIndices, coreIndices);
    if (coreSampleIndices.empty()) {
        return result;
    }

    const openvdb::Vec3d requestedDirection = seedDirection / seedDirection.length();
    std::size_t seedCoreIndex = std::numeric_limits<std::size_t>::max();
    double nearestSeedDistance = std::numeric_limits<double>::infinity();
    const openvdb::Vec3d seedPosition = seedSampleIndex < target.samples.size()
        ? openvdb::Vec3d(target.samples[seedSampleIndex].worldPosition)
        : openvdb::Vec3d(0.0);
    for (const std::size_t sampleIndex : coreSampleIndices) {
        const openvdb::Vec3d axis = normalizeOrZero(
            openvdb::Vec3d(axes.normals[sampleIndex]));
        if (axis.lengthSqr() <= kEpsilon) {
            continue;
        }
        if (seedSampleIndex == sampleIndex) {
            seedCoreIndex = sampleIndex;
            break;
        }
        const double distance =
            (openvdb::Vec3d(target.samples[sampleIndex].worldPosition) - seedPosition).lengthSqr();
        if (distance < nearestSeedDistance) {
            nearestSeedDistance = distance;
            seedCoreIndex = sampleIndex;
        }
    }
    if (seedCoreIndex == std::numeric_limits<std::size_t>::max()) {
        return result;
    }

    struct NeighborOffset
    {
        int dx = 0;
        int dy = 0;
        int dz = 0;
        int distanceSquared = 0;
    };

    std::array<NeighborOffset, 26> neighborOffsets{};
    std::size_t neighborOffsetCount = 0;
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                neighborOffsets[neighborOffsetCount++] = {
                    dx,
                    dy,
                    dz,
                    dx * dx + dy * dy + dz * dz};
            }
        }
    }

    struct CachedNeighborhood
    {
        std::array<std::int32_t, 26> neighbors{};
        bool built = false;
    };

    struct OrientationCandidate
    {
        std::size_t ordinal = 0;
        std::size_t parentOrdinal = 0;
        int localDistanceSquared = 0;
        bool backtracks = false;
        double lateralDistanceSquared = 0.0;
        double forwardDot = 0.0;
        double alignment = 0.0;
    };

    std::vector<std::int32_t> coreOrdinalBySample(
        target.samples.size(),
        static_cast<std::int32_t>(-1));
    for (std::size_t ordinal = 0; ordinal < coreSampleIndices.size(); ++ordinal) {
        coreOrdinalBySample[coreSampleIndices[ordinal]] =
            static_cast<std::int32_t>(ordinal);
    }

    // Cache coordinate adjacency once per Core ordinal and reuse it for all
    // later support checks and frontier expansions.
    std::vector<CachedNeighborhood> neighborhoodCache(coreSampleIndices.size());
    const auto buildNeighborhood = [&](std::size_t ordinal) {
        CachedNeighborhood& cache = neighborhoodCache[ordinal];
        if (cache.built) {
            return;
        }
        cache.neighbors.fill(-1);
        const openvdb::Coord center = target.samples[
            coreSampleIndices[ordinal]].coordinate;
        for (std::size_t slot = 0; slot < neighborOffsetCount; ++slot) {
            const NeighborOffset& offset = neighborOffsets[slot];
            const auto found = coreIndices.find(center.offsetBy(
                offset.dx,
                offset.dy,
                offset.dz));
            if (found == coreIndices.end()) {
                continue;
            }
            const std::int32_t neighborOrdinal = coreOrdinalBySample[found->second];
            if (neighborOrdinal >= 0) {
                cache.neighbors[slot] = neighborOrdinal;
            }
        }
        cache.built = true;
    };

    // A pending point is discovered but cannot propagate until it has enough
    // independent local support to determine its sign.
    std::vector<std::uint8_t> oriented(target.samples.size(), 0);
    std::vector<std::uint8_t> orientationState(
        coreSampleIndices.size(),
        static_cast<std::uint8_t>(0));
    std::vector<std::int32_t> orientationDepth(
        coreSampleIndices.size(),
        static_cast<std::int32_t>(-1));
    std::vector<std::size_t> predecessor(
        coreSampleIndices.size(),
        std::numeric_limits<std::size_t>::max());
    std::vector<OrientationCandidate> preferredCandidate(
        coreSampleIndices.size());
    std::vector<std::uint8_t> hasPreferredCandidate(
        coreSampleIndices.size(),
        static_cast<std::uint8_t>(0));

    const auto candidateIsPreferred = [](
        const OrientationCandidate& candidate,
        const OrientationCandidate& current) {
        if (candidate.localDistanceSquared != current.localDistanceSquared) {
            return candidate.localDistanceSquared < current.localDistanceSquared;
        }
        if (candidate.backtracks != current.backtracks) {
            return !candidate.backtracks;
        }
        if (candidate.lateralDistanceSquared != current.lateralDistanceSquared) {
            return candidate.lateralDistanceSquared < current.lateralDistanceSquared;
        }
        if (candidate.forwardDot != current.forwardDot) {
            return candidate.forwardDot > current.forwardDot;
        }
        if (candidate.alignment != current.alignment) {
            return candidate.alignment > current.alignment;
        }
        return candidate.parentOrdinal < current.parentOrdinal;
    };

    openvdb::Vec3d seedAxis = normalizeOrZero(
        openvdb::Vec3d(axes.normals[seedCoreIndex]));
    if (seedAxis.dot(requestedDirection) < 0.0) {
        seedAxis = -seedAxis;
    }
    result.normals[seedCoreIndex] = openvdb::Vec3f(seedAxis);
    oriented[seedCoreIndex] = 1;
    const std::int32_t seedOrdinal = coreOrdinalBySample[seedCoreIndex];
    orientationState[static_cast<std::size_t>(seedOrdinal)] = 2;
    orientationDepth[static_cast<std::size_t>(seedOrdinal)] = 0;

    std::vector<std::size_t> frontier{seedCoreIndex};
    std::vector<std::uint8_t> reevaluateFlags(
        coreSampleIndices.size(),
        static_cast<std::uint8_t>(0));
    std::int32_t currentDepth = 0;
    while (!frontier.empty()) {
        std::vector<std::size_t> reevaluate;
        reevaluate.reserve(frontier.size() * 8);
        for (const std::size_t fromSampleIndex : frontier) {
            const std::int32_t fromOrdinal = coreOrdinalBySample[fromSampleIndex];
            if (fromOrdinal < 0) {
                continue;
            }
            const std::size_t sourceOrdinal = static_cast<std::size_t>(fromOrdinal);
            buildNeighborhood(sourceOrdinal);
            const openvdb::Vec3d sourceNormal = normalizeOrZero(
                openvdb::Vec3d(result.normals[fromSampleIndex]));
            if (sourceNormal.lengthSqr() <= kEpsilon) {
                continue;
            }

            const std::size_t parentOrdinal = predecessor[sourceOrdinal];
            const bool hasParent = parentOrdinal !=
                std::numeric_limits<std::size_t>::max();
            const openvdb::Coord center = target.samples[fromSampleIndex].coordinate;
            const openvdb::Vec3d current(
                static_cast<double>(center.x()),
                static_cast<double>(center.y()),
                static_cast<double>(center.z()));
            const openvdb::Vec3d previous = hasParent
                ? openvdb::Vec3d(
                    static_cast<double>(target.samples[
                        coreSampleIndices[parentOrdinal]].coordinate.x()),
                    static_cast<double>(target.samples[
                        coreSampleIndices[parentOrdinal]].coordinate.y()),
                    static_cast<double>(target.samples[
                        coreSampleIndices[parentOrdinal]].coordinate.z()))
                : openvdb::Vec3d(0.0);
            const openvdb::Vec3d pathStep = hasParent
                ? current - previous
                : openvdb::Vec3d(0.0);
            const double pathLengthSquared = pathStep.lengthSqr();

            const CachedNeighborhood& cache = neighborhoodCache[sourceOrdinal];
            for (std::size_t slot = 0; slot < neighborOffsetCount; ++slot) {
                const std::int32_t neighborOrdinal = cache.neighbors[slot];
                if (neighborOrdinal < 0) {
                    continue;
                }
                const std::size_t candidateOrdinal =
                    static_cast<std::size_t>(neighborOrdinal);
                if (orientationState[candidateOrdinal] == 2) {
                    continue;
                }
                const std::size_t candidateSampleIndex =
                    coreSampleIndices[candidateOrdinal];
                const openvdb::Vec3d candidateAxis = normalizeOrZero(
                    openvdb::Vec3d(axes.normals[candidateSampleIndex]));
                if (candidateAxis.lengthSqr() <= kEpsilon) {
                    continue;
                }
                const double alignment = std::abs(candidateAxis.dot(sourceNormal));
                if (alignment < settings.minimumAlignment) {
                    continue;
                }
                if (orientationState[candidateOrdinal] == 0) {
                    orientationState[candidateOrdinal] = 1;
                    buildNeighborhood(candidateOrdinal);
                }

                const NeighborOffset& offset = neighborOffsets[slot];
                const openvdb::Vec3d step(
                    static_cast<double>(offset.dx),
                    static_cast<double>(offset.dy),
                    static_cast<double>(offset.dz));
                double forwardDot = 0.0;
                double lateralDistanceSquared = 0.0;
                if (hasParent && pathLengthSquared > kEpsilon) {
                    forwardDot = pathStep.dot(step);
                    const double projection = forwardDot / pathLengthSquared;
                    const openvdb::Vec3d lateral = step - pathStep * projection;
                    lateralDistanceSquared = lateral.lengthSqr();
                }
                const OrientationCandidate candidate{
                    candidateOrdinal,
                    sourceOrdinal,
                    offset.distanceSquared,
                    hasParent && forwardDot < 0.0,
                    lateralDistanceSquared,
                    forwardDot,
                    alignment};
                if (!hasPreferredCandidate[candidateOrdinal] || candidateIsPreferred(
                        candidate,
                        preferredCandidate[candidateOrdinal])) {
                    preferredCandidate[candidateOrdinal] = candidate;
                    hasPreferredCandidate[candidateOrdinal] = 1;
                    predecessor[candidateOrdinal] = sourceOrdinal;
                }
                if (!reevaluateFlags[candidateOrdinal]) {
                    reevaluateFlags[candidateOrdinal] = 1;
                    reevaluate.push_back(candidateOrdinal);
                }
            }
        }

        std::stable_sort(
            reevaluate.begin(),
            reevaluate.end(),
            [&](const std::size_t left, const std::size_t right) {
                if (hasPreferredCandidate[left] && hasPreferredCandidate[right]) {
                    if (candidateIsPreferred(
                            preferredCandidate[left],
                            preferredCandidate[right])) {
                        return true;
                    }
                    if (candidateIsPreferred(
                            preferredCandidate[right],
                            preferredCandidate[left])) {
                        return false;
                    }
                }
                return left < right;
            });

        std::vector<std::size_t> nextFrontier;
        nextFrontier.reserve(reevaluate.size());
        const std::int32_t nextDepth = currentDepth + 1;
        for (const std::size_t candidateOrdinal : reevaluate) {
            reevaluateFlags[candidateOrdinal] = 0;
            if (orientationState[candidateOrdinal] != 1) {
                continue;
            }
            const std::size_t candidateSampleIndex =
                coreSampleIndices[candidateOrdinal];
            const openvdb::Vec3d candidateAxis = normalizeOrZero(
                openvdb::Vec3d(axes.normals[candidateSampleIndex]));
            if (candidateAxis.lengthSqr() <= kEpsilon) {
                continue;
            }

            const CachedNeighborhood& cache = neighborhoodCache[candidateOrdinal];
            double signedVote = 0.0;
            double voteWeight = 0.0;
            std::size_t supportCount = 0;
            for (std::size_t slot = 0; slot < neighborOffsetCount; ++slot) {
                const std::int32_t neighborOrdinal = cache.neighbors[slot];
                if (neighborOrdinal < 0) {
                    continue;
                }
                const std::size_t supportOrdinal =
                    static_cast<std::size_t>(neighborOrdinal);
                if (orientationState[supportOrdinal] != 2 ||
                    orientationDepth[supportOrdinal] > currentDepth) {
                    continue;
                }
                const std::size_t supportSampleIndex =
                    coreSampleIndices[supportOrdinal];
                const openvdb::Vec3d sourceNormal = normalizeOrZero(
                    openvdb::Vec3d(result.normals[supportSampleIndex]));
                if (sourceNormal.lengthSqr() <= kEpsilon) {
                    continue;
                }
                const double alignment = candidateAxis.dot(sourceNormal);
                const double absoluteAlignment = std::abs(alignment);
                if (absoluteAlignment < settings.minimumAlignment) {
                    continue;
                }
                const double distanceWeight = 1.0 /
                    static_cast<double>(neighborOffsets[slot].distanceSquared);
                const double weight = distanceWeight * absoluteAlignment * absoluteAlignment;
                signedVote += weight * alignment;
                voteWeight += weight;
                ++supportCount;
            }

            const bool seedRing = currentDepth == 0 && supportCount > 0;
            if (voteWeight <= kEpsilon ||
                (!seedRing && supportCount < 2) ||
                std::abs(signedVote) < settings.minimumAlignment * voteWeight) {
                continue;
            }

            result.normals[candidateSampleIndex] = openvdb::Vec3f(
                signedVote < 0.0 ? -candidateAxis : candidateAxis);
            oriented[candidateSampleIndex] = 1;
            orientationState[candidateOrdinal] = 2;
            orientationDepth[candidateOrdinal] = nextDepth;
            nextFrontier.push_back(candidateSampleIndex);
        }
        frontier.swap(nextFrontier);
        currentDepth = nextDepth;
    }

    // Reconcile loop and branch conflicts after the wavefront pass. The
    // selected seed remains fixed while each other Core point votes against
    // the current oriented normals of its 26-neighborhood.
    for (std::size_t pass = 0; pass < 2; ++pass) {
        const std::vector<openvdb::Vec3f> currentNormals = result.normals;
        std::vector<openvdb::Vec3f> nextNormals = currentNormals;
        for (const std::size_t sampleIndex : coreSampleIndices) {
            if (!oriented[sampleIndex] || sampleIndex == seedCoreIndex) {
                continue;
            }
            const openvdb::Vec3d axis = normalizeOrZero(
                openvdb::Vec3d(axes.normals[sampleIndex]));
            if (axis.lengthSqr() <= kEpsilon) {
                continue;
            }
            const openvdb::Coord center = target.samples[sampleIndex].coordinate;
            double signedVote = 0.0;
            double voteWeight = 0.0;
            for (int dz = -1; dz <= 1; ++dz) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        const auto found = coreIndices.find(center.offsetBy(dx, dy, dz));
                        if (found == coreIndices.end() || !oriented[found->second]) {
                            continue;
                        }
                        const openvdb::Vec3d neighborAxis = normalizeOrZero(
                            openvdb::Vec3d(axes.normals[found->second]));
                        const openvdb::Vec3d neighborNormal = normalizeOrZero(
                            openvdb::Vec3d(currentNormals[found->second]));
                        if (neighborAxis.lengthSqr() <= kEpsilon ||
                            neighborNormal.lengthSqr() <= kEpsilon) {
                            continue;
                        }
                        const double alignment = std::abs(axis.dot(neighborAxis));
                        if (alignment < settings.minimumAlignment) {
                            continue;
                        }
                        const double weight = alignment * alignment;
                        signedVote += weight * axis.dot(neighborNormal);
                        voteWeight += weight;
                    }
                }
            }
            if (voteWeight <= kEpsilon ||
                std::abs(signedVote) < settings.minimumAlignment * voteWeight) {
                continue;
            }
            nextNormals[sampleIndex] = openvdb::Vec3f(
                signedVote < 0.0 ? -axis : axis);
        }
        result.normals.swap(nextNormals);
    }

    std::size_t orientedCoreCount = 0;
    for (const std::size_t sampleIndex : coreSampleIndices) {
        if (oriented[sampleIndex]) {
            ++orientedCoreCount;
        }
    }
    result.smoothedCoreCount = orientedCoreCount;

    // Transition samples are oriented from the nearest already-oriented Core
    // region. Direct Core support seeds a breadth-first propagation through
    // all connected Transition layers, so a raw transition axis is never
    // used as an independent sign source.
    std::unordered_map<openvdb::Coord, std::size_t, CoordHasher> allIndices;
    allIndices.reserve(target.samples.size() * 2 + 1);
    for (std::size_t sampleIndex = 0; sampleIndex < target.samples.size(); ++sampleIndex) {
        if (normalizeOrZero(openvdb::Vec3d(axes.normals[sampleIndex])).lengthSqr() > kEpsilon) {
            allIndices.emplace(target.samples[sampleIndex].coordinate, sampleIndex);
        }
    }

    std::vector<std::uint8_t> orientedTransition(target.samples.size(), 0);
    std::vector<std::size_t> transitionQueue;
    transitionQueue.reserve(target.transitionCount);
    const auto enqueueTransition = [&](std::size_t sampleIndex) {
        if (orientedTransition[sampleIndex]) {
            return;
        }
        orientedTransition[sampleIndex] = 1;
        transitionQueue.push_back(sampleIndex);
    };

    std::size_t orientedTransitionCount = 0;
    for (std::size_t sampleIndex = 0; sampleIndex < target.samples.size(); ++sampleIndex) {
        if (target.samples[sampleIndex].kind != SurfaceTargetSampleKind::Transition) {
            continue;
        }
        const openvdb::Coord center = target.samples[sampleIndex].coordinate;
        openvdb::Vec3d sum(0.0);
        double weightSum = 0.0;
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const auto found = coreIndices.find(center.offsetBy(dx, dy, dz));
                    if (found == coreIndices.end() || !oriented[found->second]) {
                        continue;
                    }
                    const openvdb::Vec3d normal = normalizeOrZero(
                        openvdb::Vec3d(result.normals[found->second]));
                    const double weight = 1.0 /
                        (1.0 + static_cast<double>(std::max({
                            std::abs(dx), std::abs(dy), std::abs(dz)})));
                    sum += normal * weight;
                    weightSum += weight;
                }
            }
        }
        const openvdb::Vec3d orientedNormal = normalizeOrZero(
            sum / std::max(weightSum, 1.0e-12));
        if (orientedNormal.lengthSqr() <= kEpsilon) {
            continue;
        }
        result.normals[sampleIndex] = openvdb::Vec3f(orientedNormal);
        enqueueTransition(sampleIndex);
        ++orientedTransitionCount;
    }

    std::size_t queueHead = 0;
    while (queueHead < transitionQueue.size()) {
        const std::size_t fromIndex = transitionQueue[queueHead++];
        const openvdb::Coord center = target.samples[fromIndex].coordinate;
        const openvdb::Vec3d fromNormal = normalizeOrZero(
            openvdb::Vec3d(result.normals[fromIndex]));
        if (fromNormal.lengthSqr() <= kEpsilon) {
            continue;
        }
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const auto found = allIndices.find(center.offsetBy(dx, dy, dz));
                    if (found == allIndices.end()) {
                        continue;
                    }
                    const std::size_t neighborIndex = found->second;
                    if (target.samples[neighborIndex].kind != SurfaceTargetSampleKind::Transition ||
                        orientedTransition[neighborIndex]) {
                        continue;
                    }
                    openvdb::Vec3d neighbor = normalizeOrZero(
                        openvdb::Vec3d(axes.normals[neighborIndex]));
                    if (neighbor.lengthSqr() <= kEpsilon) {
                        continue;
                    }
                    if (neighbor.dot(fromNormal) < 0.0) {
                        neighbor = -neighbor;
                    }
                    result.normals[neighborIndex] = openvdb::Vec3f(neighbor);
                    enqueueTransition(neighborIndex);
                    ++orientedTransitionCount;
                }
            }
        }
    }

    result.smoothedTransitionCount = orientedTransitionCount;
    return result;
}

SurfaceNormalAdjacencyStatistics analyzeSurfaceTargetNormalAdjacency(
    const SurfaceTargetCache& target,
    const SurfaceNormalField& field,
    std::size_t seedSampleIndex,
    double weakAlignment)
{
    if (target.empty()) {
        return {};
    }
    if (field.normals.size() != target.samples.size()) {
        throw std::invalid_argument("surface normal adjacency field size does not match target");
    }
    if (!std::isfinite(weakAlignment) || weakAlignment < 0.0 || weakAlignment > 1.0) {
        throw std::invalid_argument("surface normal adjacency threshold is invalid");
    }

    SurfaceNormalAdjacencyStatistics result;
    std::vector<std::size_t> coreSampleIndices;
    std::unordered_map<openvdb::Coord, std::size_t, CoordHasher> coreIndices;
    collectCoreIndices(target, coreSampleIndices, coreIndices);
    std::unordered_map<openvdb::Coord, std::size_t, CoordHasher> allIndices;
    allIndices.reserve(target.samples.size() * 2 + 1);
    std::vector<std::uint8_t> valid(target.samples.size(), 0);
    std::vector<std::uint8_t> validCore(target.samples.size(), 0);
    for (const std::size_t sampleIndex : coreSampleIndices) {
        const openvdb::Vec3d normal = normalizeOrZero(
            openvdb::Vec3d(field.normals[sampleIndex]));
        if (normal.lengthSqr() <= kEpsilon) {
            continue;
        }
        valid[sampleIndex] = 1;
        validCore[sampleIndex] = 1;
        ++result.validCoreSampleCount;
    }
    for (std::size_t sampleIndex = 0; sampleIndex < target.samples.size(); ++sampleIndex) {
        const auto& sample = target.samples[sampleIndex];
        if (!openvdb::Vec3d(sample.worldPosition).isFinite()) {
            continue;
        }
        allIndices.emplace(sample.coordinate, sampleIndex);
        if (sample.kind != SurfaceTargetSampleKind::Transition) {
            continue;
        }
        const openvdb::Vec3d normal = normalizeOrZero(
            openvdb::Vec3d(field.normals[sampleIndex]));
        if (normal.lengthSqr() > kEpsilon) {
            valid[sampleIndex] = 1;
            ++result.validTransitionSampleCount;
        }
    }
    if (result.validCoreSampleCount == 0 && result.validTransitionSampleCount == 0) {
        return result;
    }

    std::size_t resolvedSeed = std::numeric_limits<std::size_t>::max();
    if (seedSampleIndex < target.samples.size() && validCore[seedSampleIndex]) {
        resolvedSeed = seedSampleIndex;
    } else if (seedSampleIndex < target.samples.size()) {
        const openvdb::Vec3d seedPosition =
            openvdb::Vec3d(target.samples[seedSampleIndex].worldPosition);
        if (seedPosition.isFinite()) {
            double nearestDistance = std::numeric_limits<double>::infinity();
            for (const std::size_t sampleIndex : coreSampleIndices) {
                if (!valid[sampleIndex]) {
                    continue;
                }
                const double distance =
                    (openvdb::Vec3d(target.samples[sampleIndex].worldPosition) -
                        seedPosition).lengthSqr();
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    resolvedSeed = sampleIndex;
                }
            }
        }
    }

    std::vector<std::int32_t> componentIds(target.samples.size(), -1);
    std::vector<std::size_t> pending;
    pending.reserve(result.validCoreSampleCount);
    for (const std::size_t start : coreSampleIndices) {
        if (!valid[start] || componentIds[start] >= 0) {
            continue;
        }
        const auto component = static_cast<std::int32_t>(result.connectedComponentCount++);
        componentIds[start] = component;
        pending.clear();
        pending.push_back(start);
        for (std::size_t cursor = 0; cursor < pending.size(); ++cursor) {
            const std::size_t current = pending[cursor];
            const openvdb::Coord center = target.samples[current].coordinate;
            for (int dz = -1; dz <= 1; ++dz) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        const auto found = coreIndices.find(center.offsetBy(dx, dy, dz));
                        if (found == coreIndices.end() || !valid[found->second] ||
                            componentIds[found->second] >= 0) {
                            continue;
                        }
                        componentIds[found->second] = component;
                        pending.push_back(found->second);
                    }
                }
            }
        }
    }

    std::int32_t seededComponentId = -1;
    if (resolvedSeed < componentIds.size() && componentIds[resolvedSeed] >= 0) {
        seededComponentId = componentIds[resolvedSeed];
        for (const std::size_t sampleIndex : coreSampleIndices) {
            if (validCore[sampleIndex] && componentIds[sampleIndex] == seededComponentId) {
                ++result.seededComponentSampleCount;
            }
        }
    }
    result.unseededComponentSampleCount =
        result.validCoreSampleCount - result.seededComponentSampleCount;

    std::vector<std::uint8_t> transitionCoreSupport(target.samples.size(), 0);
    for (std::size_t sampleIndex = 0; sampleIndex < target.samples.size(); ++sampleIndex) {
        if (!valid[sampleIndex] ||
            target.samples[sampleIndex].kind != SurfaceTargetSampleKind::Transition) {
            continue;
        }
        const openvdb::Coord center = target.samples[sampleIndex].coordinate;
        for (int dz = -1; dz <= 1 && !transitionCoreSupport[sampleIndex]; ++dz) {
            for (int dy = -1; dy <= 1 && !transitionCoreSupport[sampleIndex]; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const auto found = coreIndices.find(center.offsetBy(dx, dy, dz));
                    if (found != coreIndices.end() && validCore[found->second]) {
                        transitionCoreSupport[sampleIndex] = 1;
                        break;
                    }
                }
            }
        }
    }

    constexpr std::size_t kMaximumRecordedIssues = 12;
    const auto appendWorstIssue = [&](std::vector<SurfaceNormalAdjacencyStatistics::Issue>& issues,
                                      const SurfaceNormalAdjacencyStatistics::Issue& issue) {
        issues.push_back(issue);
        std::sort(
            issues.begin(),
            issues.end(),
            [](const auto& left, const auto& right) {
                return left.signedAlignment < right.signedAlignment;
            });
        if (issues.size() > kMaximumRecordedIssues) {
            issues.pop_back();
        }
    };
    const auto recordIssue = [&](std::size_t firstIndex, std::size_t secondIndex) {
        const openvdb::Vec3d first = normalizeOrZero(
            openvdb::Vec3d(field.normals[firstIndex]));
        const openvdb::Vec3d second = normalizeOrZero(
            openvdb::Vec3d(field.normals[secondIndex]));
        const double signedAlignment = std::clamp(first.dot(second), -1.0, 1.0);
        if (signedAlignment >= 0.0) {
            return;
        }
        SurfaceNormalAdjacencyStatistics::Issue issue;
        issue.firstSampleIndex = firstIndex;
        issue.secondSampleIndex = secondIndex;
        issue.firstCoordinate = target.samples[firstIndex].coordinate;
        issue.secondCoordinate = target.samples[secondIndex].coordinate;
        issue.firstNormal = openvdb::Vec3f(first);
        issue.secondNormal = openvdb::Vec3f(second);
        issue.signedAlignment = signedAlignment;
        issue.absoluteAlignment = std::abs(signedAlignment);
        issue.transitionLinked =
            target.samples[firstIndex].kind == SurfaceTargetSampleKind::Transition ||
            target.samples[secondIndex].kind == SurfaceTargetSampleKind::Transition;
        issue.unseededComponent =
            (validCore[firstIndex] && seededComponentId >= 0 &&
                componentIds[firstIndex] != seededComponentId) ||
            (validCore[secondIndex] && seededComponentId >= 0 &&
                componentIds[secondIndex] != seededComponentId);
        issue.transitionWithoutCoreSupport =
            (target.samples[firstIndex].kind == SurfaceTargetSampleKind::Transition &&
                !transitionCoreSupport[firstIndex]) ||
            (target.samples[secondIndex].kind == SurfaceTargetSampleKind::Transition &&
                !transitionCoreSupport[secondIndex]);
        if (issue.unseededComponent) {
            ++result.opposingUnseededEdgeCount;
        }
        if (issue.transitionWithoutCoreSupport) {
            ++result.opposingTransitionWithoutCoreSupportEdgeCount;
        }
        appendWorstIssue(result.worstOpposingEdges, issue);
        appendWorstIssue(
            issue.transitionLinked
                ? result.worstTransitionOpposingEdges
                : result.worstCoreOpposingEdges,
            issue);
    };

    double signedAlignmentSum = 0.0;
    double absoluteAlignmentSum = 0.0;
    for (const std::size_t sampleIndex : coreSampleIndices) {
        if (!valid[sampleIndex]) {
            continue;
        }
        const openvdb::Coord center = target.samples[sampleIndex].coordinate;
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx < 0 || (dx == 0 && dy < 0) ||
                        (dx == 0 && dy == 0 && dz <= 0)) {
                        continue;
                    }
                    const auto found = coreIndices.find(center.offsetBy(dx, dy, dz));
                    if (found == coreIndices.end() || !valid[found->second]) {
                        continue;
                    }
                    const openvdb::Vec3d first = normalizeOrZero(
                        openvdb::Vec3d(field.normals[sampleIndex]));
                    const openvdb::Vec3d second = normalizeOrZero(
                        openvdb::Vec3d(field.normals[found->second]));
                    const double signedAlignment = std::clamp(first.dot(second), -1.0, 1.0);
                    const double absoluteAlignment = std::abs(signedAlignment);
                    ++result.adjacencyEdgeCount;
                    if (signedAlignment < 0.0) {
                        ++result.opposingEdgeCount;
                        recordIssue(sampleIndex, found->second);
                    }
                    if (absoluteAlignment < weakAlignment) {
                        ++result.weakEdgeCount;
                    }
                    result.minimumSignedAlignment = std::min(
                        result.minimumSignedAlignment,
                        signedAlignment);
                    result.minimumAbsoluteAlignment = std::min(
                        result.minimumAbsoluteAlignment,
                        absoluteAlignment);
                    signedAlignmentSum += signedAlignment;
                    absoluteAlignmentSum += absoluteAlignment;
                }
            }
        }
    }
    if (result.adjacencyEdgeCount > 0) {
        const double edgeCount = static_cast<double>(result.adjacencyEdgeCount);
        result.meanSignedAlignment = signedAlignmentSum / edgeCount;
        result.meanAbsoluteAlignment = absoluteAlignmentSum / edgeCount;
    }

    for (std::size_t sampleIndex = 0; sampleIndex < target.samples.size(); ++sampleIndex) {
        if (!valid[sampleIndex]) {
            continue;
        }
        const openvdb::Coord center = target.samples[sampleIndex].coordinate;
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx < 0 || (dx == 0 && dy < 0) ||
                        (dx == 0 && dy == 0 && dz <= 0)) {
                        continue;
                    }
                    const auto found = allIndices.find(center.offsetBy(dx, dy, dz));
                    if (found == allIndices.end() || !valid[found->second]) {
                        continue;
                    }
                    const auto& firstSample = target.samples[sampleIndex];
                    const auto& secondSample = target.samples[found->second];
                    if (firstSample.kind != SurfaceTargetSampleKind::Transition &&
                        secondSample.kind != SurfaceTargetSampleKind::Transition) {
                        continue;
                    }
                    const openvdb::Vec3d first = normalizeOrZero(
                        openvdb::Vec3d(field.normals[sampleIndex]));
                    const openvdb::Vec3d second = normalizeOrZero(
                        openvdb::Vec3d(field.normals[found->second]));
                    const double signedAlignment = std::clamp(first.dot(second), -1.0, 1.0);
                    ++result.transitionAdjacencyEdgeCount;
                    if (signedAlignment < 0.0) {
                        ++result.transitionOpposingEdgeCount;
                        recordIssue(sampleIndex, found->second);
                    }
                }
            }
        }
    }
    return result;
}

} // namespace volume_surface
