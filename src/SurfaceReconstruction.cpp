#include "volume_surface/SurfaceReconstruction.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <openvdb/tools/Morphology.h>
#include <openvdb/tools/Interpolation.h>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

namespace volume_surface {
namespace {

using Vec3d = openvdb::Vec3d;
using Coord = openvdb::Coord;

constexpr std::array<Coord, 6> kAxisNeighbors{{
    Coord(-1, 0, 0), Coord(1, 0, 0),
    Coord(0, -1, 0), Coord(0, 1, 0),
    Coord(0, 0, -1), Coord(0, 0, 1)}};

struct CoordHasher
{
    [[nodiscard]] std::size_t operator()(const Coord& coordinate) const noexcept
    {
        std::size_t value = static_cast<std::uint32_t>(coordinate.x());
        value = (value * 0x9e3779b9U) ^ static_cast<std::uint32_t>(coordinate.y());
        value = (value * 0x9e3779b9U) ^ static_cast<std::uint32_t>(coordinate.z());
        return value;
    }
};

bool coordLess(const Coord& lhs, const Coord& rhs)
{
    if (lhs.x() != rhs.x()) return lhs.x() < rhs.x();
    if (lhs.y() != rhs.y()) return lhs.y() < rhs.y();
    return lhs.z() < rhs.z();
}

bool isInside(const openvdb::FloatGrid& grid, double value, double isoValue)
{
    return grid.getGridClass() == openvdb::GRID_FOG_VOLUME
        ? value >= isoValue
        : value <= isoValue;
}

struct ReconstructionSample
{
    Coord coordinate;
    Vec3d position{};
    Vec3d normal{};
    double supportWeight = 0.0;
    bool core = false;
    std::uint32_t component = std::numeric_limits<std::uint32_t>::max();
};

struct SpatialEntry
{
    Coord cell;
    std::uint32_t sampleIndex = 0;
};

struct SpatialRange
{
    std::size_t begin = 0;
    std::size_t end = 0;
};

class SampleSpatialIndex
{
public:
    SampleSpatialIndex(
        const std::vector<ReconstructionSample>& samples,
        double cellSize)
        : mSamples(samples)
        , mCellSize(cellSize)
    {
        mEntries.reserve(samples.size());
        for (std::size_t index = 0; index < samples.size(); ++index) {
            mEntries.push_back({cellFor(samples[index].position),
                static_cast<std::uint32_t>(index)});
        }
        tbb::parallel_sort(
            mEntries.begin(),
            mEntries.end(),
            [](const SpatialEntry& lhs, const SpatialEntry& rhs) {
                if (lhs.cell != rhs.cell) return coordLess(lhs.cell, rhs.cell);
                return lhs.sampleIndex < rhs.sampleIndex;
            });

        mRanges.reserve(mEntries.size() / 4 + 1);
        std::size_t begin = 0;
        while (begin < mEntries.size()) {
            std::size_t end = begin + 1;
            while (end < mEntries.size() && mEntries[end].cell == mEntries[begin].cell) {
                ++end;
            }
            mRanges.emplace(mEntries[begin].cell, SpatialRange{begin, end});
            begin = end;
        }
    }

    template <typename Callback>
    void forEachCandidate(const Vec3d& position, double radius, Callback&& callback) const
    {
        const Coord minimum = cellFor(position - Vec3d(radius));
        const Coord maximum = cellFor(position + Vec3d(radius));
        for (int z = minimum.z(); z <= maximum.z(); ++z) {
            for (int y = minimum.y(); y <= maximum.y(); ++y) {
                for (int x = minimum.x(); x <= maximum.x(); ++x) {
                    const auto found = mRanges.find(Coord(x, y, z));
                    if (found == mRanges.end()) continue;
                    for (std::size_t offset = found->second.begin;
                         offset < found->second.end;
                         ++offset) {
                        const auto sampleIndex = mEntries[offset].sampleIndex;
                        const Vec3d delta = mSamples[sampleIndex].position - position;
                        if (delta.lengthSqr() <= radius * radius) {
                            callback(sampleIndex, delta.lengthSqr());
                        }
                    }
                }
            }
        }
    }

private:
    [[nodiscard]] Coord cellFor(const Vec3d& position) const
    {
        const auto toInteger = [](double value) {
            const double floored = std::floor(value);
            if (floored <= static_cast<double>(std::numeric_limits<int>::min())) {
                return std::numeric_limits<int>::min();
            }
            if (floored >= static_cast<double>(std::numeric_limits<int>::max())) {
                return std::numeric_limits<int>::max();
            }
            return static_cast<int>(floored);
        };
        return Coord(
            toInteger(position.x() / mCellSize),
            toInteger(position.y() / mCellSize),
            toInteger(position.z() / mCellSize));
    }

    const std::vector<ReconstructionSample>& mSamples;
    double mCellSize;
    std::vector<SpatialEntry> mEntries;
    std::unordered_map<Coord, SpatialRange, CoordHasher> mRanges;
};

bool solveLinearSystem6(
    std::array<std::array<double, 7>, 6> matrix,
    std::array<double, 6>& solution)
{
    for (std::size_t column = 0; column < 6; ++column) {
        std::size_t pivot = column;
        double pivotMagnitude = std::abs(matrix[pivot][column]);
        for (std::size_t row = column + 1; row < 6; ++row) {
            const double magnitude = std::abs(matrix[row][column]);
            if (magnitude > pivotMagnitude) {
                pivot = row;
                pivotMagnitude = magnitude;
            }
        }
        if (!std::isfinite(pivotMagnitude) || pivotMagnitude < 1.0e-14) {
            return false;
        }
        if (pivot != column) std::swap(matrix[pivot], matrix[column]);
        const double pivotValue = matrix[column][column];
        for (std::size_t row = column + 1; row < 6; ++row) {
            const double factor = matrix[row][column] / pivotValue;
            if (factor == 0.0) continue;
            for (std::size_t index = column; index <= 6; ++index) {
                matrix[row][index] -= factor * matrix[column][index];
            }
        }
    }

    for (std::size_t row = 6; row-- > 0;) {
        double value = matrix[row][6];
        for (std::size_t column = row + 1; column < 6; ++column) {
            value -= matrix[row][column] * solution[column];
        }
        const double diagonal = matrix[row][row];
        if (!std::isfinite(diagonal) || std::abs(diagonal) < 1.0e-14) {
            return false;
        }
        solution[row] = value / diagonal;
    }
    return std::all_of(
        solution.begin(), solution.end(), [](double value) { return std::isfinite(value); });
}

bool solveLinearSystem3(
    std::array<std::array<double, 4>, 3> matrix,
    std::array<double, 3>& solution)
{
    for (std::size_t column = 0; column < 3; ++column) {
        std::size_t pivot = column;
        double pivotMagnitude = std::abs(matrix[pivot][column]);
        for (std::size_t row = column + 1; row < 3; ++row) {
            const double magnitude = std::abs(matrix[row][column]);
            if (magnitude > pivotMagnitude) {
                pivot = row;
                pivotMagnitude = magnitude;
            }
        }
        if (!std::isfinite(pivotMagnitude) || pivotMagnitude < 1.0e-14) return false;
        if (pivot != column) std::swap(matrix[pivot], matrix[column]);
        const double pivotValue = matrix[column][column];
        for (std::size_t row = column + 1; row < 3; ++row) {
            const double factor = matrix[row][column] / pivotValue;
            for (std::size_t index = column; index <= 3; ++index) {
                matrix[row][index] -= factor * matrix[column][index];
            }
        }
    }
    for (std::size_t row = 3; row-- > 0;) {
        double value = matrix[row][3];
        for (std::size_t column = row + 1; column < 3; ++column) {
            value -= matrix[row][column] * solution[column];
        }
        solution[row] = value / matrix[row][row];
    }
    return std::all_of(
        solution.begin(), solution.end(), [](double value) { return std::isfinite(value); });
}

double wendlandC2(double normalizedDistance)
{
    if (normalizedDistance >= 1.0) return 0.0;
    const double oneMinus = 1.0 - normalizedDistance;
    return oneMinus * oneMinus * oneMinus * oneMinus *
        (4.0 * normalizedDistance + 1.0);
}

std::vector<ReconstructionSample> localizeCoreSamples(
    const openvdb::FloatGrid& grid,
    const SurfaceTargetCache& target,
    double isoValue,
    const std::vector<openvdb::Vec3f>* normalOverrides)
{
    std::vector<ReconstructionSample> samples(target.samples.size());
    const bool hasNormalOverrides = normalOverrides &&
        normalOverrides->size() == target.samples.size();
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, target.samples.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            const auto accessor = grid.getConstAccessor();
            for (std::size_t index = range.begin(); index != range.end(); ++index) {
                const auto& source = target.samples[index];
                auto& destination = samples[index];
                destination.coordinate = source.coordinate;
                destination.position = Vec3d(source.worldPosition);
                destination.normal = hasNormalOverrides
                    ? Vec3d((*normalOverrides)[index])
                    : Vec3d(source.normal);
                if (!destination.normal.isFinite() ||
                    destination.normal.lengthSqr() <= 1.0e-20) {
                    // An invalid fitted override stays invalid; do not leak the
                    // source gradient back into a geometry-only reconstruction.
                    destination.normal = Vec3d(0.0);
                }
                destination.supportWeight = source.supportWeight;
                destination.core =
                    source.kind == SurfaceTargetSampleKind::Core;
                if (!destination.core) continue;

                const double centerValue = static_cast<double>(source.density);
                Vec3d crossingSum(0.0);
                std::size_t crossingCount = 0;
                const Vec3d centerWorld = destination.position;
                for (const Coord& direction : kAxisNeighbors) {
                    const Coord neighborCoordinate = source.coordinate + direction;
                    const double neighborValue = accessor.getValue(neighborCoordinate);
                    if (isInside(grid, centerValue, isoValue) ==
                        isInside(grid, neighborValue, isoValue)) {
                        continue;
                    }
                    const double denominator = neighborValue - centerValue;
                    const double t = std::abs(denominator) > 1.0e-12
                        ? std::clamp((isoValue - centerValue) / denominator, 0.0, 1.0)
                        : 0.5;
                    const Vec3d neighborWorld =
                        grid.indexToWorld(neighborCoordinate.asVec3d());
                    crossingSum += centerWorld + (neighborWorld - centerWorld) * t;
                    ++crossingCount;
                }
                if (crossingCount > 0) {
                    destination.position = crossingSum /
                        static_cast<double>(crossingCount);
                }
            }
        });
    return samples;
}

void labelSurfaceComponents(std::vector<ReconstructionSample>& samples)
{
    std::unordered_map<Coord, std::uint32_t, CoordHasher> coreIndex;
    coreIndex.reserve(samples.size());
    for (std::size_t index = 0; index < samples.size(); ++index) {
        if (samples[index].core) {
            coreIndex.emplace(
                samples[index].coordinate,
                static_cast<std::uint32_t>(index));
        }
    }

    std::uint32_t componentCount = 0;
    std::deque<std::uint32_t> pending;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        if (!samples[index].core ||
            samples[index].component != std::numeric_limits<std::uint32_t>::max()) {
            continue;
        }
        const auto component = componentCount++;
        samples[index].component = component;
        pending.push_back(static_cast<std::uint32_t>(index));
        while (!pending.empty()) {
            const auto currentIndex = pending.front();
            pending.pop_front();
            const Coord current = samples[currentIndex].coordinate;
            for (int dz = -1; dz <= 1; ++dz) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
                        const auto found = coreIndex.find(current.offsetBy(dx, dy, dz));
                        if (found == coreIndex.end()) continue;
                        auto& neighbor = samples[found->second];
                        if (neighbor.component !=
                            std::numeric_limits<std::uint32_t>::max()) {
                            continue;
                        }
                        neighbor.component = component;
                        pending.push_back(found->second);
                    }
                }
            }
        }
    }

    for (auto& sample : samples) {
        if (sample.core) continue;
        double nearestDistanceSquared = std::numeric_limits<double>::infinity();
        std::uint32_t nearestComponent =
            std::numeric_limits<std::uint32_t>::max();
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    const auto found = coreIndex.find(sample.coordinate.offsetBy(dx, dy, dz));
                    if (found == coreIndex.end()) continue;
                    const auto& core = samples[found->second];
                    const double distanceSquared =
                        (core.position - sample.position).lengthSqr();
                    if (distanceSquared < nearestDistanceSquared) {
                        nearestDistanceSquared = distanceSquared;
                        nearestComponent = core.component;
                    }
                }
            }
        }
        sample.component = nearestComponent;
    }
}

struct FieldSample
{
    double value = std::numeric_limits<double>::quiet_NaN();
    Vec3d normal{};
    bool valid = false;
};

struct NeighborCandidate
{
    std::uint32_t index = 0;
    double distanceSquared = 0.0;
};

class MLSField
{
public:
    MLSField(
        const std::vector<ReconstructionSample>& samples,
        const SurfaceReconstructionSettings& settings)
        : mSamples(samples)
        , mSettings(settings)
        , mIndex(samples, settings.mlsRadius)
    {
    }

    [[nodiscard]] bool nearestCorePosition(
        const Vec3d& query,
        Vec3d& position) const
    {
        std::uint32_t nearestIndex = std::numeric_limits<std::uint32_t>::max();
        double nearestDistanceSquared = std::numeric_limits<double>::infinity();
        mIndex.forEachCandidate(
            query,
            mSettings.mlsRadius,
            [&](std::uint32_t index, double distanceSquared) {
                const auto& sample = mSamples[index];
                if (!sample.core || distanceSquared >= nearestDistanceSquared) {
                    return;
                }
                nearestIndex = index;
                nearestDistanceSquared = distanceSquared;
            });
        if (nearestIndex == std::numeric_limits<std::uint32_t>::max()) {
            return false;
        }
        position = mSamples[nearestIndex].position;
        return true;
    }

    [[nodiscard]] FieldSample evaluate(const Vec3d& query) const
    {
        std::vector<NeighborCandidate> candidates;
        candidates.reserve(mSettings.maximumSamplesPerFit * 2 + 8);
        mIndex.forEachCandidate(
            query,
            mSettings.mlsRadius,
            [&](std::uint32_t index, double distanceSquared) {
                candidates.push_back({index, distanceSquared});
            });
        if (candidates.empty()) return {};

        std::uint32_t nearestCore = std::numeric_limits<std::uint32_t>::max();
        double nearestCoreDistance = std::numeric_limits<double>::infinity();
        for (const auto& candidate : candidates) {
            const auto& sample = mSamples[candidate.index];
            if (!sample.core || sample.normal.lengthSqr() <= 1.0e-20) continue;
            if (candidate.distanceSquared < nearestCoreDistance) {
                nearestCore = candidate.index;
                nearestCoreDistance = candidate.distanceSquared;
            }
        }
        if (nearestCore == std::numeric_limits<std::uint32_t>::max()) return {};

        const auto referenceComponent = mSamples[nearestCore].component;
        Vec3d referenceNormal = mSamples[nearestCore].normal;
        referenceNormal.normalize();
        Vec3d anchorPosition(0.0);
        Vec3d anchorNormal(0.0);
        double anchorWeightSum = 0.0;
        for (const auto& candidate : candidates) {
            const auto& sample = mSamples[candidate.index];
            if (!sample.core || sample.component != referenceComponent ||
                sample.normal.lengthSqr() <= 1.0e-20) continue;
            Vec3d normal = sample.normal;
            normal.normalize();
            const double normalDot = normal.dot(referenceNormal);
            if (normalDot < 0.25) continue;
            const double weight = wendlandC2(
                std::sqrt(candidate.distanceSquared) / mSettings.mlsRadius) *
                normalDot;
            anchorPosition += sample.position * weight;
            anchorNormal += normal * weight;
            anchorWeightSum += weight;
        }
        if (anchorWeightSum <= 1.0e-12) return {};
        anchorPosition /= anchorWeightSum;
        if (anchorNormal.lengthSqr() <= 1.0e-20) return {};
        anchorNormal.normalize();
        if (anchorNormal.dot(referenceNormal) < 0.0) anchorNormal = -anchorNormal;

        Vec3d tangentAxis = std::abs(anchorNormal.x()) < 0.9
            ? Vec3d(1.0, 0.0, 0.0)
            : Vec3d(0.0, 1.0, 0.0);
        Vec3d tangentU = anchorNormal.cross(tangentAxis);
        if (tangentU.lengthSqr() <= 1.0e-20) return {};
        tangentU.normalize();
        Vec3d tangentV = anchorNormal.cross(tangentU);
        tangentV.normalize();

        if (candidates.size() > mSettings.maximumSamplesPerFit) {
            std::nth_element(
                candidates.begin(),
                candidates.begin() + static_cast<std::ptrdiff_t>(
                    mSettings.maximumSamplesPerFit),
                candidates.end(),
                [](const NeighborCandidate& lhs, const NeighborCandidate& rhs) {
                    return lhs.distanceSquared < rhs.distanceSquared;
                });
            candidates.resize(mSettings.maximumSamplesPerFit);
        }

        std::array<std::array<double, 7>, 6> matrix{};
        double totalWeight = 0.0;
        for (const auto& candidate : candidates) {
            const auto& sample = mSamples[candidate.index];
            if (sample.component != referenceComponent ||
                sample.normal.lengthSqr() <= 1.0e-20) continue;
            Vec3d normal = sample.normal;
            normal.normalize();
            const double normalDot = normal.dot(anchorNormal);
            if (normalDot < 0.25) continue;
            const double confidence = sample.core
                ? 1.0
                : std::clamp(
                    sample.supportWeight * mSettings.transitionGeometryWeight,
                    0.0,
                    1.0);
            const double kernel = wendlandC2(
                std::sqrt(candidate.distanceSquared) / mSettings.mlsRadius);
            const double weight = kernel * confidence * normalDot;
            if (weight <= 1.0e-10) continue;

            const Vec3d delta = sample.position - anchorPosition;
            const double u = delta.dot(tangentU) / mSettings.mlsRadius;
            const double v = delta.dot(tangentV) / mSettings.mlsRadius;
            const double h = sample.core
                ? delta.dot(anchorNormal) / mSettings.mlsRadius
                : 0.0;
            const std::array<double, 6> basis{1.0, u, v, u * u, u * v, v * v};
            for (std::size_t row = 0; row < 6; ++row) {
                for (std::size_t column = 0; column < 6; ++column) {
                    matrix[row][column] += weight * basis[row] * basis[column];
                }
                matrix[row][6] += weight * basis[row] * h;
            }
            totalWeight += weight;
        }

        if (totalWeight <= 1.0e-12) return {};
        const double regularization = std::max(1.0e-12,
            mSettings.regularization * totalWeight);
        for (std::size_t index = 0; index < 6; ++index) {
            matrix[index][index] += regularization;
        }
        std::array<double, 6> coefficients{};
        if (!solveLinearSystem6(matrix, coefficients)) return {};

        const Vec3d queryDelta = query - anchorPosition;
        const double queryU = queryDelta.dot(tangentU) / mSettings.mlsRadius;
        const double queryV = queryDelta.dot(tangentV) / mSettings.mlsRadius;
        const double queryH = queryDelta.dot(anchorNormal);
        const double fittedH = mSettings.mlsRadius * (
            coefficients[0] + coefficients[1] * queryU + coefficients[2] * queryV +
            coefficients[3] * queryU * queryU + coefficients[4] * queryU * queryV +
            coefficients[5] * queryV * queryV);
        const double fittedAnchorH = mSettings.mlsRadius * coefficients[0];
        const double derivativeU = coefficients[1] +
            2.0 * coefficients[3] * queryU + coefficients[4] * queryV;
        const double derivativeV = coefficients[2] +
            coefficients[4] * queryU + 2.0 * coefficients[5] * queryV;
        Vec3d fittedNormal = anchorNormal - tangentU * derivativeU - tangentV * derivativeV;
        if (fittedNormal.lengthSqr() <= 1.0e-20) fittedNormal = anchorNormal;
        fittedNormal.normalize();

        FieldSample result;
        result.value = queryH - (fittedH - fittedAnchorH);
        result.normal = fittedNormal;
        result.valid = std::isfinite(result.value) && result.normal.lengthSqr() > 1.0e-20;
        return result;
    }

private:
    const std::vector<ReconstructionSample>& mSamples;
    const SurfaceReconstructionSettings& mSettings;
    SampleSpatialIndex mIndex;
};

Coord cellCoordinateFor(const Vec3d& position, const Vec3d& origin, double cellSize)
{
    return Coord(
        static_cast<int>(std::floor((position.x() - origin.x()) / cellSize)),
        static_cast<int>(std::floor((position.y() - origin.y()) / cellSize)),
        static_cast<int>(std::floor((position.z() - origin.z()) / cellSize)));
}

Vec3d cellVertexWorld(
    const Vec3d& origin,
    const Coord& coordinate,
    double cellSize)
{
    return origin + Vec3d(
        static_cast<double>(coordinate.x()),
        static_cast<double>(coordinate.y()),
        static_cast<double>(coordinate.z())) * cellSize;
}

struct CrossingCell
{
    Coord coordinate;
    std::array<std::uint32_t, 8> vertexSamples{};
    std::uint32_t meshVertex = 0;
};

std::array<Coord, 8> cellCorners(const Coord& cell)
{
    return {{
        cell,
        cell.offsetBy(1, 0, 0),
        cell.offsetBy(1, 1, 0),
        cell.offsetBy(0, 1, 0),
        cell.offsetBy(0, 0, 1),
        cell.offsetBy(1, 0, 1),
        cell.offsetBy(1, 1, 1),
        cell.offsetBy(0, 1, 1)}};
}

struct EdgeDescription
{
    std::uint8_t startCorner = 0;
    std::uint8_t endCorner = 0;
};

constexpr std::array<EdgeDescription, 12> kCellEdges{{
    {0, 1}, {1, 2}, {3, 2}, {0, 3},
    {4, 5}, {5, 6}, {7, 6}, {4, 7},
    {0, 4}, {1, 5}, {3, 7}, {2, 6}}};

bool solveQef(
    const std::vector<std::pair<Vec3d, Vec3d>>& hermite,
    const Vec3d& minimum,
    const Vec3d& maximum,
    double regularization,
    Vec3d& result,
    Vec3d& normal)
{
    if (hermite.empty()) return false;
    std::array<std::array<double, 4>, 3> matrix{};
    Vec3d averagePosition(0.0);
    Vec3d normalSum(0.0);
    for (const auto& [position, inputNormal] : hermite) {
        Vec3d n = inputNormal;
        if (n.lengthSqr() <= 1.0e-20) continue;
        n.normalize();
        const double d = n.dot(position);
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                matrix[row][column] += n[row] * n[column];
            }
            matrix[row][3] += n[row] * d;
        }
        averagePosition += position;
        normalSum += n;
    }
    const double count = static_cast<double>(hermite.size());
    averagePosition /= count;
    for (std::size_t index = 0; index < 3; ++index) {
        matrix[index][index] += regularization;
        matrix[index][3] += regularization * averagePosition[index];
    }
    std::array<double, 3> solution{};
    if (!solveLinearSystem3(matrix, solution)) {
        result = averagePosition;
    } else {
        result = Vec3d(solution[0], solution[1], solution[2]);
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
        result[axis] = std::clamp(result[axis], minimum[axis], maximum[axis]);
    }
    normal = normalSum;
    if (normal.lengthSqr() <= 1.0e-20) normal = Vec3d(0.0, 0.0, 1.0);
    normal.normalize();
    return result.isFinite();
}

void appendOrientedQuad(
    SurfaceMesh& mesh,
    const std::array<std::uint32_t, 4>& quad,
    const Vec3d& expectedNormal)
{
    if (quad[0] == quad[1] || quad[0] == quad[2] || quad[0] == quad[3] ||
        quad[1] == quad[2] || quad[1] == quad[3] || quad[2] == quad[3]) {
        return;
    }
    const auto& a = mesh.vertices[quad[0]].position;
    const auto& b = mesh.vertices[quad[1]].position;
    const auto& c = mesh.vertices[quad[2]].position;
    const Vec3d ab(
        static_cast<double>(b[0] - a[0]),
        static_cast<double>(b[1] - a[1]),
        static_cast<double>(b[2] - a[2]));
    const Vec3d ac(
        static_cast<double>(c[0] - a[0]),
        static_cast<double>(c[1] - a[1]),
        static_cast<double>(c[2] - a[2]));
    const bool reverse = ab.cross(ac).dot(expectedNormal) < 0.0;
    const auto appendTriangle = [&](std::uint32_t i0, std::uint32_t i1, std::uint32_t i2) {
        mesh.indices.push_back(i0);
        mesh.indices.push_back(i1);
        mesh.indices.push_back(i2);
    };
    if (reverse) {
        appendTriangle(quad[0], quad[2], quad[1]);
        appendTriangle(quad[0], quad[3], quad[2]);
    } else {
        appendTriangle(quad[0], quad[1], quad[2]);
        appendTriangle(quad[0], quad[2], quad[3]);
    }
}

void calculateMeshNormals(SurfaceMesh& mesh)
{
    for (auto& vertex : mesh.vertices) vertex.normal = {0.0f, 0.0f, 0.0f};
    for (std::size_t index = 0; index + 2 < mesh.indices.size(); index += 3) {
        auto& a = mesh.vertices[mesh.indices[index]];
        auto& b = mesh.vertices[mesh.indices[index + 1]];
        auto& c = mesh.vertices[mesh.indices[index + 2]];
        const Vec3d ab(
            static_cast<double>(b.position[0] - a.position[0]),
            static_cast<double>(b.position[1] - a.position[1]),
            static_cast<double>(b.position[2] - a.position[2]));
        const Vec3d ac(
            static_cast<double>(c.position[0] - a.position[0]),
            static_cast<double>(c.position[1] - a.position[1]),
            static_cast<double>(c.position[2] - a.position[2]));
        const Vec3d normal = ab.cross(ac);
        for (auto* vertex : {&a, &b, &c}) {
            vertex->normal[0] += static_cast<float>(normal.x());
            vertex->normal[1] += static_cast<float>(normal.y());
            vertex->normal[2] += static_cast<float>(normal.z());
        }
    }
    for (auto& vertex : mesh.vertices) {
        const double length = std::sqrt(
            static_cast<double>(vertex.normal[0]) * vertex.normal[0] +
            static_cast<double>(vertex.normal[1]) * vertex.normal[1] +
            static_cast<double>(vertex.normal[2]) * vertex.normal[2]);
        if (length > 1.0e-20) {
            for (float& component : vertex.normal) {
                component = static_cast<float>(component / length);
            }
        } else {
            vertex.normal = {0.0f, 0.0f, 1.0f};
        }
    }
}

} // namespace

SurfaceReconstructionResult reconstructSurfaceMLS(
    const openvdb::FloatGrid& sourceGrid,
    const SurfaceTargetCache& target,
    const SurfaceReconstructionSettings& settings)
{
    return reconstructSurfaceMLS(sourceGrid, target, settings, nullptr);
}

SurfaceReconstructionResult reconstructSurfaceMLS(
    const openvdb::FloatGrid& sourceGrid,
    const SurfaceTargetCache& target,
    const SurfaceReconstructionSettings& settings,
    const std::vector<openvdb::Vec3f>* normalOverrides)
{
    if (sourceGrid.getGridClass() != openvdb::GRID_FOG_VOLUME &&
        sourceGrid.getGridClass() != openvdb::GRID_LEVEL_SET) {
        throw std::invalid_argument("MLS reconstruction requires a fog volume or level set grid");
    }
    if (!std::isfinite(settings.isoValue) ||
        !std::isfinite(settings.mlsRadius) || settings.mlsRadius <= 0.0 ||
        !std::isfinite(settings.targetCellSize) || settings.targetCellSize <= 0.0 ||
        settings.maximumCellCount == 0 || settings.maximumSamplesPerFit < 6 ||
        settings.projectionIterations == 0 ||
        !std::isfinite(settings.transitionGeometryWeight) ||
        settings.transitionGeometryWeight < 0.0 ||
        !std::isfinite(settings.minimumFogDensityFraction) ||
        settings.minimumFogDensityFraction < 0.0 ||
        settings.minimumFogDensityFraction > 1.0 ||
        !std::isfinite(settings.sourceTopologyAdaptivity) ||
        settings.sourceTopologyAdaptivity < 0.0 ||
        settings.sourceTopologyAdaptivity > 1.0 ||
        !std::isfinite(settings.regularization) || settings.regularization <= 0.0 ||
        !std::isfinite(settings.projectionMaximumDisplacement) ||
        settings.projectionMaximumDisplacement <= 0.0) {
        throw std::invalid_argument("MLS reconstruction settings are invalid");
    }
    if (target.empty() || target.coreCount == 0) {
        throw std::invalid_argument("MLS reconstruction requires core surface samples");
    }

    SurfaceReconstructionResult result;
    const auto totalStart = std::chrono::steady_clock::now();
    const auto anchorStart = totalStart;
    auto samples = localizeCoreSamples(
        sourceGrid,
        target,
        settings.isoValue,
        normalOverrides);
    labelSurfaceComponents(samples);
    result.timings.anchorLocalizationMilliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - anchorStart).count();

    const auto indexStart = std::chrono::steady_clock::now();
    MLSField field(samples, settings);
    result.timings.spatialIndexMilliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - indexStart).count();

    const auto topologyStart = std::chrono::steady_clock::now();
    auto sourceTopology = extractIsoSurface(
        sourceGrid,
        settings.isoValue,
        settings.sourceTopologyAdaptivity);
    if (sourceTopology.empty()) {
        result.timings.meshExtractionMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - topologyStart).count();
        result.timings.totalMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - totalStart).count();
        return result;
    }

    auto sourceTopologySplit = splitSurfaceMeshComponents(sourceTopology);
    result.surfaceComponentCount = sourceTopologySplit.componentCount;
    result.excludedTriangleCount = sourceTopologySplit.excludedTriangleCount;
    sourceTopology = std::move(sourceTopologySplit.primary);
    if (sourceTopology.empty()) {
        result.timings.meshExtractionMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - topologyStart).count();
        result.timings.totalMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - totalStart).count();
        return result;
    }

    result.projectionVertexCount = sourceTopology.vertices.size();
    result.candidateCellCount = result.projectionVertexCount;
    result.fieldSampleCount = result.projectionVertexCount;
    result.crossingCellCount = sourceTopology.triangleCount();
    result.sourceCrossingCellCount = sourceTopology.triangleCount();
    result.emittedFaceCount = sourceTopology.triangleCount();

    const openvdb::tools::GridSampler<
        openvdb::FloatGrid,
        openvdb::tools::BoxSampler> projectionSampler(sourceGrid);
    const double maximumDisplacement = settings.projectionMaximumDisplacement;
    const double maximumStep = std::min(
        settings.mlsRadius,
        maximumDisplacement /
            static_cast<double>(std::max<std::size_t>(1, settings.projectionIterations)));
    const double minimumDensity = settings.isoValue *
        settings.minimumFogDensityFraction;
    const bool useDensityGuard = sourceGrid.getGridClass() == openvdb::GRID_FOG_VOLUME;
    std::vector<std::uint8_t> projectionFallbackFlags(
        sourceTopology.vertices.size(),
        0);
    std::vector<std::uint8_t> projectionRejectedFlags(
        sourceTopology.vertices.size(),
        0);
    std::vector<std::uint8_t> projectionDensityRejectedFlags(
        sourceTopology.vertices.size(),
        0);
    std::vector<double> projectionDisplacements(
        sourceTopology.vertices.size(),
        0.0);
    const auto projectionStart = std::chrono::steady_clock::now();
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, sourceTopology.vertices.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            for (std::size_t index = range.begin(); index != range.end(); ++index) {
                auto& vertex = sourceTopology.vertices[index];
                const Vec3d originalPosition(
                    static_cast<double>(vertex.position[0]),
                    static_cast<double>(vertex.position[1]),
                    static_cast<double>(vertex.position[2]));
                Vec3d position = originalPosition;
                bool projected = false;
                bool rejected = false;
                bool densityRejected = false;
                for (std::size_t iteration = 0;
                     iteration < settings.projectionIterations;
                     ++iteration) {
                    const auto sample = field.evaluate(position);
                    if (!sample.valid) {
                        rejected = true;
                        break;
                    }
                    const double correction = std::clamp(
                        sample.value,
                        -settings.mlsRadius,
                        settings.mlsRadius);
                    if (std::abs(correction) <= 1.0e-9) {
                        projected = true;
                        break;
                    }
                    Vec3d step = -sample.normal * correction;
                    const double stepLength = step.length();
                    if (!std::isfinite(stepLength) || stepLength <= 1.0e-12) {
                        rejected = true;
                        break;
                    }
                    if (stepLength > maximumStep) {
                        step *= maximumStep / stepLength;
                    }
                    const double currentDisplacement =
                        (position - originalPosition).length();
                    const double remainingDisplacement = maximumDisplacement -
                        currentDisplacement;
                    if (remainingDisplacement <= 1.0e-12) {
                        rejected = true;
                        break;
                    }
                    if (step.length() > remainingDisplacement) {
                        step *= remainingDisplacement / step.length();
                    }

                    Vec3d candidate = position + step;
                    bool accepted = false;
                    for (int lineSearch = 0; lineSearch < 6; ++lineSearch) {
                        const double density = projectionSampler.wsSample(candidate);
                        const bool densitySafe = !useDensityGuard ||
                            (std::isfinite(density) && density >= minimumDensity);
                        if (candidate.isFinite() && densitySafe) {
                            accepted = true;
                            break;
                        }
                        step *= 0.5;
                        candidate = position + step;
                    }
                    if (!accepted) {
                        rejected = true;
                        densityRejected = useDensityGuard;
                        break;
                    }
                    position = candidate;
                    projected = true;
                }
                if (!projected) {
                    projectionFallbackFlags[index] = 1;
                }
                if (rejected) {
                    projectionRejectedFlags[index] = 1;
                }
                if (densityRejected) {
                    projectionDensityRejectedFlags[index] = 1;
                }
                projectionDisplacements[index] =
                    (position - originalPosition).length();
                vertex.position = {
                    static_cast<float>(position.x()),
                    static_cast<float>(position.y()),
                    static_cast<float>(position.z())};
            }
        });
    result.timings.fieldSamplingMilliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - projectionStart).count();
    result.sourceTopologyProjectionFallbackCount = static_cast<std::size_t>(
        std::count(
            projectionFallbackFlags.begin(),
            projectionFallbackFlags.end(),
            static_cast<std::uint8_t>(1)));
    result.projectionRejectedCount = static_cast<std::size_t>(
        std::count(
            projectionRejectedFlags.begin(),
            projectionRejectedFlags.end(),
            static_cast<std::uint8_t>(1)));
    result.projectionDensityRejectedCount = static_cast<std::size_t>(
        std::count(
            projectionDensityRejectedFlags.begin(),
            projectionDensityRejectedFlags.end(),
            static_cast<std::uint8_t>(1)));
    result.sourceSupportFallbackCount = result.projectionDensityRejectedCount;
    result.projectionMaximumDisplacement = *std::max_element(
        projectionDisplacements.begin(),
        projectionDisplacements.end());
    calculateMeshNormals(sourceTopology);
    if (!sourceTopology.vertices.empty()) {
        const float infinity = std::numeric_limits<float>::infinity();
        sourceTopology.bounds.minimum = {infinity, infinity, infinity};
        sourceTopology.bounds.maximum = {-infinity, -infinity, -infinity};
        for (const auto& vertex : sourceTopology.vertices) {
            for (std::size_t axis = 0; axis < 3; ++axis) {
                sourceTopology.bounds.minimum[axis] = std::min(
                    sourceTopology.bounds.minimum[axis],
                    vertex.position[axis]);
                sourceTopology.bounds.maximum[axis] = std::max(
                    sourceTopology.bounds.maximum[axis],
                    vertex.position[axis]);
            }
        }
    }
    result.mesh = std::move(sourceTopology);
    result.usedSourceTopologyFallback = false;
    result.missingNeighborFaceCount = 0;
    result.timings.meshExtractionMilliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - topologyStart).count();
    result.timings.totalMilliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - totalStart).count();
    return result;

#if 0
    // The former direct MLS/DC topology path is retained only for historical
    // comparison; source-topology projection above is the sole runtime path.
    Vec3d minimum(
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity());
    Vec3d maximum(
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity());
    for (std::size_t index = 0; index < samples.size(); ++index) {
        if (!samples[index].core) continue;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], samples[index].position[axis]);
            maximum[axis] = std::max(maximum[axis], samples[index].position[axis]);
        }
    }
    const double padding = settings.targetCellSize *
        static_cast<double>(settings.candidatePaddingCells + 1);
    const Vec3d origin(
        std::floor((minimum.x() - padding) / settings.targetCellSize) * settings.targetCellSize,
        std::floor((minimum.y() - padding) / settings.targetCellSize) * settings.targetCellSize,
        std::floor((minimum.z() - padding) / settings.targetCellSize) * settings.targetCellSize);

    auto candidateMask = openvdb::BoolGrid::create(false);
    for (const auto& sample : samples) {
        if (sample.component != std::numeric_limits<std::uint32_t>::max()) {
            candidateMask->tree().setValueOn(
                cellCoordinateFor(sample.position, origin, settings.targetCellSize));
        }
    }
    if (settings.candidatePaddingCells > 0) {
        openvdb::tools::dilateActiveValues(
            candidateMask->tree(),
            static_cast<int>(settings.candidatePaddingCells),
            openvdb::tools::NN_FACE_EDGE_VERTEX,
            openvdb::tools::IGNORE_TILES);
    }
    result.candidateCellCount = static_cast<std::size_t>(candidateMask->activeVoxelCount());
    if (result.candidateCellCount > settings.maximumCellCount) {
        throw std::length_error("MLS reconstruction candidate cells exceed the configured limit");
    }

    std::vector<Coord> cells;
    cells.reserve(result.candidateCellCount);
    for (auto iterator = candidateMask->cbeginValueOn(); iterator; ++iterator) {
        cells.push_back(iterator.getCoord());
    }
    std::sort(cells.begin(), cells.end(), coordLess);

    std::unordered_map<Coord, std::uint32_t, CoordHasher> fieldVertexMap;
    fieldVertexMap.reserve(result.candidateCellCount * 2 + 1);
    std::vector<Coord> fieldCoordinates;
    fieldCoordinates.reserve(result.candidateCellCount * 2 + 1);
    for (const Coord& cell : cells) {
        for (const Coord& corner : cellCorners(cell)) {
            if (fieldVertexMap.find(corner) != fieldVertexMap.end()) continue;
            const auto index = static_cast<std::uint32_t>(fieldCoordinates.size());
            fieldVertexMap.emplace(corner, index);
            fieldCoordinates.push_back(corner);
        }
    }
    std::vector<FieldSample> fieldVertices(fieldCoordinates.size());
    std::vector<std::uint8_t> invalidFieldFallbackFlags(fieldCoordinates.size(), 0);
    const double invalidFieldFallbackDistance = std::max(
        settings.mlsRadius,
        settings.targetCellSize * 2.0);
    const auto fieldStart = std::chrono::steady_clock::now();
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, fieldCoordinates.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            for (std::size_t index = range.begin(); index != range.end(); ++index) {
                const Vec3d position = cellVertexWorld(
                    origin,
                    fieldCoordinates[index],
                    settings.targetCellSize);
                fieldVertices[index] = field.evaluate(position);
            }
        });
    const openvdb::tools::GridSampler<
        openvdb::FloatGrid,
        openvdb::tools::BoxSampler> sourceSampler(sourceGrid);
    for (std::size_t index = 0; index < fieldVertices.size(); ++index) {
        auto& sample = fieldVertices[index];
        if (sample.valid) continue;
        const Vec3d position = cellVertexWorld(
            origin,
            fieldCoordinates[index],
            settings.targetCellSize);
        const double sourceValue = sourceSampler.wsSample(position);
        const bool sourceInside = isInside(
            sourceGrid,
            sourceValue,
            settings.isoValue);
        sample.value = sourceInside
            ? -invalidFieldFallbackDistance
            : invalidFieldFallbackDistance;

        const double gradientStep = std::max(
            settings.targetCellSize * 0.5,
            1.0e-6);
        Vec3d sourceNormal(
            sourceSampler.wsSample(position + Vec3d(gradientStep, 0.0, 0.0)) -
                sourceSampler.wsSample(position - Vec3d(gradientStep, 0.0, 0.0)),
            sourceSampler.wsSample(position + Vec3d(0.0, gradientStep, 0.0)) -
                sourceSampler.wsSample(position - Vec3d(0.0, gradientStep, 0.0)),
            sourceSampler.wsSample(position + Vec3d(0.0, 0.0, gradientStep)) -
                sourceSampler.wsSample(position - Vec3d(0.0, 0.0, gradientStep)));
        if (sourceGrid.getGridClass() == openvdb::GRID_FOG_VOLUME) {
            sourceNormal = -sourceNormal;
        }
        if (sourceNormal.lengthSqr() <= 1.0e-20) {
            sourceNormal = Vec3d(0.0, 0.0, 1.0);
        } else {
            sourceNormal.normalize();
        }
        sample.normal = sourceNormal;
        sample.valid = true;
        invalidFieldFallbackFlags[index] = 1;
    }
    result.invalidFieldFallbackCount = static_cast<std::size_t>(std::count(
        invalidFieldFallbackFlags.begin(),
        invalidFieldFallbackFlags.end(),
        static_cast<std::uint8_t>(1)));

    std::vector<CrossingCell> crossingCells;
    crossingCells.reserve(cells.size() / 3 + 1);
    std::unordered_map<Coord, std::uint32_t, CoordHasher> cellVertexMap;
    cellVertexMap.reserve(cells.size() / 2 + 1);
    std::size_t sourceSupportFallbackCount = 0;
    std::size_t sourceCrossingCellCount = 0;
    for (const Coord& cell : cells) {
        const auto corners = cellCorners(cell);
        CrossingCell crossing;
        crossing.coordinate = cell;
        bool hasPositive = false;
        bool hasNegative = false;
        for (std::size_t corner = 0; corner < 8; ++corner) {
            crossing.vertexSamples[corner] = fieldVertexMap.find(corners[corner])->second;
            const auto& sample = fieldVertices[crossing.vertexSamples[corner]];
            if (!sample.valid) continue;
            hasPositive = hasPositive || sample.value >= 0.0;
            hasNegative = hasNegative || sample.value < 0.0;
        }
        if (!hasPositive || !hasNegative) continue;

        bool sourceInside = false;
        bool sourceOutside = false;
        std::array<double, 8> sourceValues{};
        for (std::size_t corner = 0; corner < 8; ++corner) {
            sourceValues[corner] = sourceSampler.wsSample(
                cellVertexWorld(
                    origin,
                    corners[corner],
                    settings.targetCellSize));
            if (isInside(sourceGrid, sourceValues[corner], settings.isoValue)) {
                sourceInside = true;
            } else {
                sourceOutside = true;
            }
        }
        if (sourceInside && sourceOutside) ++sourceCrossingCellCount;

        std::vector<std::pair<Vec3d, Vec3d>> hermite;
        hermite.reserve(12);
        for (const auto& edge : kCellEdges) {
            const auto& first = fieldVertices[crossing.vertexSamples[edge.startCorner]];
            const auto& second = fieldVertices[crossing.vertexSamples[edge.endCorner]];
            if ((first.value >= 0.0) == (second.value >= 0.0)) continue;
            const double denominator = second.value - first.value;
            const double t = std::abs(denominator) > 1.0e-12
                ? std::clamp(-first.value / denominator, 0.0, 1.0)
                : 0.5;
            const Vec3d firstPosition = cellVertexWorld(
                origin, corners[edge.startCorner], settings.targetCellSize);
            const Vec3d secondPosition = cellVertexWorld(
                origin, corners[edge.endCorner], settings.targetCellSize);
            const Vec3d position = firstPosition + (secondPosition - firstPosition) * t;
            Vec3d normal = first.normal * (1.0 - t) + second.normal * t;
            if (normal.lengthSqr() > 1.0e-20) normal.normalize();
            hermite.emplace_back(position, normal);
        }
        if (hermite.empty()) continue;

        const Vec3d cellMinimum = cellVertexWorld(origin, cell, settings.targetCellSize);
        const Vec3d cellMaximum = cellMinimum + Vec3d(settings.targetCellSize);
        Vec3d vertexPosition;
        Vec3d vertexNormal;
        if (!solveQef(
                hermite,
                cellMinimum,
                cellMaximum,
                settings.regularization,
                vertexPosition,
                vertexNormal)) {
            continue;
        }
        if (sourceGrid.getGridClass() == openvdb::GRID_FOG_VOLUME &&
            settings.isoValue > 0.0 &&
            sourceSampler.wsSample(vertexPosition) <
                settings.isoValue * settings.minimumFogDensityFraction) {
            if (sourceInside && sourceOutside) {
                Vec3d sourceIntersectionSum(0.0);
                std::size_t sourceIntersectionCount = 0;
                for (const auto& edge : kCellEdges) {
                    const double firstValue = sourceValues[edge.startCorner];
                    const double secondValue = sourceValues[edge.endCorner];
                    if (isInside(sourceGrid, firstValue, settings.isoValue) ==
                        isInside(sourceGrid, secondValue, settings.isoValue)) {
                        continue;
                    }
                    const double denominator = secondValue - firstValue;
                    const double t = std::abs(denominator) > 1.0e-12
                        ? std::clamp(
                            (settings.isoValue - firstValue) / denominator,
                            0.0,
                            1.0)
                        : 0.5;
                    const Vec3d firstPosition = cellVertexWorld(
                        origin,
                        corners[edge.startCorner],
                        settings.targetCellSize);
                    const Vec3d secondPosition = cellVertexWorld(
                        origin,
                        corners[edge.endCorner],
                        settings.targetCellSize);
                    sourceIntersectionSum += firstPosition +
                        (secondPosition - firstPosition) * t;
                    ++sourceIntersectionCount;
                }
                if (sourceIntersectionCount > 0) {
                    vertexPosition = sourceIntersectionSum /
                        static_cast<double>(sourceIntersectionCount);
                    ++sourceSupportFallbackCount;
                }
            } else {
                Vec3d nearestCorePosition;
                if (field.nearestCorePosition(vertexPosition, nearestCorePosition)) {
                    for (std::size_t axis = 0; axis < 3; ++axis) {
                        nearestCorePosition[axis] = std::clamp(
                            nearestCorePosition[axis],
                            cellMinimum[axis],
                            cellMaximum[axis]);
                    }
                    vertexPosition = nearestCorePosition;
                    ++sourceSupportFallbackCount;
                }
            }
        }
        crossing.meshVertex = static_cast<std::uint32_t>(result.mesh.vertices.size());
        result.mesh.vertices.push_back({
            {static_cast<float>(vertexPosition.x()),
             static_cast<float>(vertexPosition.y()),
             static_cast<float>(vertexPosition.z())},
             {static_cast<float>(vertexNormal.x()),
              static_cast<float>(vertexNormal.y()),
              static_cast<float>(vertexNormal.z())}});
        cellVertexMap.emplace(cell, crossing.meshVertex);
        crossingCells.push_back(crossing);
    }
    result.timings.fieldSamplingMilliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - fieldStart).count();
    result.fieldSampleCount = fieldVertices.size();
    result.crossingCellCount = crossingCells.size();
    result.sourceSupportFallbackCount = sourceSupportFallbackCount;
    result.sourceCrossingCellCount = sourceCrossingCellCount;

    const auto meshStart = std::chrono::steady_clock::now();
    std::size_t missingNeighborFaceCount = 0;
    std::size_t emittedFaceCount = 0;
    const auto appendEdgeFaces = [&](const CrossingCell& cell, int axis) {
        const auto& c = cell.coordinate;
        Coord edgeStart;
        std::array<Coord, 4> incidentCells;
        std::uint32_t startVertex = 0;
        std::uint32_t endVertex = 0;
        if (axis == 0) {
            edgeStart = c.offsetBy(0, 1, 1);
            incidentCells = {{c, c.offsetBy(0, 1, 0), c.offsetBy(0, 1, 1), c.offsetBy(0, 0, 1)}};
        } else if (axis == 1) {
            edgeStart = c.offsetBy(1, 0, 1);
            incidentCells = {{c, c.offsetBy(1, 0, 0), c.offsetBy(1, 0, 1), c.offsetBy(0, 0, 1)}};
        } else {
            edgeStart = c.offsetBy(1, 1, 0);
            incidentCells = {{c, c.offsetBy(1, 0, 0), c.offsetBy(1, 1, 0), c.offsetBy(0, 1, 0)}};
        }
        const Coord edgeEnd = axis == 0
            ? edgeStart.offsetBy(1, 0, 0)
            : axis == 1
                ? edgeStart.offsetBy(0, 1, 0)
                : edgeStart.offsetBy(0, 0, 1);
        const auto startField = fieldVertexMap.find(edgeStart);
        const auto endField = fieldVertexMap.find(edgeEnd);
        if (startField == fieldVertexMap.end() || endField == fieldVertexMap.end()) return;
        const auto& first = fieldVertices[startField->second];
        const auto& second = fieldVertices[endField->second];
        if (!first.valid || !second.valid ||
            (first.value >= 0.0) == (second.value >= 0.0)) return;
        const double denominator = second.value - first.value;
        const double t = std::abs(denominator) > 1.0e-12
            ? std::clamp(-first.value / denominator, 0.0, 1.0)
            : 0.5;
        Vec3d expectedNormal = first.normal * (1.0 - t) + second.normal * t;
        if (expectedNormal.lengthSqr() > 1.0e-20) expectedNormal.normalize();

        std::array<std::uint32_t, 4> quad{};
        for (std::size_t index = 0; index < 4; ++index) {
            const auto found = cellVertexMap.find(incidentCells[index]);
            if (found == cellVertexMap.end()) {
                ++missingNeighborFaceCount;
                return;
            }
            quad[index] = found->second;
        }
        appendOrientedQuad(result.mesh, quad, expectedNormal);
        ++emittedFaceCount;
    };
    for (const auto& cell : crossingCells) {
        appendEdgeFaces(cell, 0);
        appendEdgeFaces(cell, 1);
        appendEdgeFaces(cell, 2);
    }
    calculateMeshNormals(result.mesh);
    if (!result.mesh.vertices.empty()) {
        const float infinity = std::numeric_limits<float>::infinity();
        result.mesh.bounds.minimum = {infinity, infinity, infinity};
        result.mesh.bounds.maximum = {-infinity, -infinity, -infinity};
        for (const auto& vertex : result.mesh.vertices) {
            for (std::size_t axis = 0; axis < 3; ++axis) {
                result.mesh.bounds.minimum[axis] = std::min(
                    result.mesh.bounds.minimum[axis], vertex.position[axis]);
                result.mesh.bounds.maximum[axis] = std::max(
                    result.mesh.bounds.maximum[axis], vertex.position[axis]);
            }
        }
    }

    const std::size_t topologyRepairThreshold = std::max<std::size_t>(
        32,
        crossingCells.size() / 100);
    if (missingNeighborFaceCount > topologyRepairThreshold) {
        auto sourceTopology = extractIsoSurface(
            sourceGrid,
            settings.isoValue,
            settings.sourceTopologyAdaptivity);
        if (!sourceTopology.empty()) {
            std::vector<std::uint8_t> projectionFallbackFlags(
                sourceTopology.vertices.size(),
                0);
            tbb::parallel_for(
                tbb::blocked_range<std::size_t>(
                    0,
                    sourceTopology.vertices.size()),
                [&](const tbb::blocked_range<std::size_t>& range) {
                    for (std::size_t index = range.begin(); index != range.end(); ++index) {
                        auto& vertex = sourceTopology.vertices[index];
                        Vec3d position(
                            static_cast<double>(vertex.position[0]),
                            static_cast<double>(vertex.position[1]),
                            static_cast<double>(vertex.position[2]));
                        bool projected = false;
                        for (int iteration = 0; iteration < 2; ++iteration) {
                            const auto sample = field.evaluate(position);
                            if (!sample.valid) break;
                            const double correction = std::clamp(
                                sample.value,
                                -settings.mlsRadius,
                                settings.mlsRadius);
                            position -= sample.normal * correction;
                            projected = true;
                        }
                        if (!projected) {
                            projectionFallbackFlags[index] = 1;
                            continue;
                        }
                        vertex.position = {
                            static_cast<float>(position.x()),
                            static_cast<float>(position.y()),
                            static_cast<float>(position.z())};
                    }
                });
            calculateMeshNormals(sourceTopology);
            if (!sourceTopology.vertices.empty()) {
                const float infinity = std::numeric_limits<float>::infinity();
                sourceTopology.bounds.minimum = {infinity, infinity, infinity};
                sourceTopology.bounds.maximum = {-infinity, -infinity, -infinity};
                for (const auto& vertex : sourceTopology.vertices) {
                    for (std::size_t axis = 0; axis < 3; ++axis) {
                        sourceTopology.bounds.minimum[axis] = std::min(
                            sourceTopology.bounds.minimum[axis],
                            vertex.position[axis]);
                        sourceTopology.bounds.maximum[axis] = std::max(
                            sourceTopology.bounds.maximum[axis],
                            vertex.position[axis]);
                    }
                }
            }
            result.mesh = std::move(sourceTopology);
            result.usedSourceTopologyFallback = true;
            result.sourceTopologyProjectionFallbackCount = static_cast<std::size_t>(
                std::count(
                    projectionFallbackFlags.begin(),
                    projectionFallbackFlags.end(),
                    static_cast<std::uint8_t>(1)));
        }
    }
    if (!result.mesh.empty()) {
        auto meshSplit = splitSurfaceMeshComponents(result.mesh);
        result.surfaceComponentCount = meshSplit.componentCount;
        result.excludedTriangleCount = meshSplit.excludedTriangleCount;
        result.mesh = std::move(meshSplit.primary);
    }
    result.timings.meshExtractionMilliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - meshStart).count();
    result.missingNeighborFaceCount = missingNeighborFaceCount;
    result.emittedFaceCount = emittedFaceCount;
    result.timings.totalMilliseconds =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - totalStart).count();
    return result;
#endif
}

} // namespace volume_surface
