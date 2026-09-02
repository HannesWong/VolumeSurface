#include "volume_surface/SurfaceTarget.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

namespace volume_surface {
namespace {

constexpr std::array<openvdb::Coord, 6> kAxisNeighbors{{
    openvdb::Coord(-1, 0, 0),
    openvdb::Coord(1, 0, 0),
    openvdb::Coord(0, -1, 0),
    openvdb::Coord(0, 1, 0),
    openvdb::Coord(0, 0, -1),
    openvdb::Coord(0, 0, 1)}};

struct CoordHasher
{
    [[nodiscard]] std::size_t operator()(const openvdb::Coord& coordinate) const noexcept
    {
        std::size_t value = static_cast<std::uint32_t>(coordinate.x());
        value = (value * 0x9e3779b9U) ^ static_cast<std::uint32_t>(coordinate.y());
        value = (value * 0x9e3779b9U) ^ static_cast<std::uint32_t>(coordinate.z());
        return value;
    }
};

struct SpatialBucketKey
{
    int x = 0;
    int y = 0;
    int z = 0;

    [[nodiscard]] bool operator==(const SpatialBucketKey& other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct SpatialBucketHasher
{
    [[nodiscard]] std::size_t operator()(const SpatialBucketKey& key) const noexcept
    {
        std::size_t value = static_cast<std::uint32_t>(key.x);
        value = (value * 0x9e3779b9U) ^ static_cast<std::uint32_t>(key.y);
        value = (value * 0x9e3779b9U) ^ static_cast<std::uint32_t>(key.z);
        return value;
    }
};

class CoreSpatialIndex
{
public:
    CoreSpatialIndex(const SurfaceTargetCache& cache, double cellSize)
        : mCache(&cache)
        , mCellSize(std::max(cellSize, 1.0e-9))
    {
        bool hasOrigin = false;
        for (std::size_t index = 0; index < cache.samples.size(); ++index) {
            if (cache.samples[index].kind != SurfaceTargetSampleKind::Core) {
                continue;
            }
            const openvdb::Vec3d position(cache.samples[index].worldPosition);
            if (!hasOrigin) {
                mOrigin = position;
                hasOrigin = true;
            } else {
                mOrigin.x() = std::min(mOrigin.x(), position.x());
                mOrigin.y() = std::min(mOrigin.y(), position.y());
                mOrigin.z() = std::min(mOrigin.z(), position.z());
            }
        }
        mBuckets.reserve(cache.coreCount / 2 + 1);
        if (!hasOrigin) {
            return;
        }
        for (std::size_t index = 0; index < cache.samples.size(); ++index) {
            if (cache.samples[index].kind == SurfaceTargetSampleKind::Core) {
                mBuckets[bucketFor(openvdb::Vec3d(cache.samples[index].worldPosition))]
                    .push_back(index);
            }
        }
    }

    template <typename Visitor>
    void visit(const openvdb::Vec3d& position, double radius, Visitor&& visitor) const
    {
        const double queryRadius = std::max(radius, 1.0e-9);
        const double radiusSquared = queryRadius * queryRadius;
        const SpatialBucketKey minimum = bucketFor(position - openvdb::Vec3d(queryRadius));
        const SpatialBucketKey maximum = bucketFor(position + openvdb::Vec3d(queryRadius));
        for (int z = minimum.z; z <= maximum.z; ++z) {
            for (int y = minimum.y; y <= maximum.y; ++y) {
                for (int x = minimum.x; x <= maximum.x; ++x) {
                    const auto found = mBuckets.find(SpatialBucketKey{x, y, z});
                    if (found == mBuckets.end()) {
                        continue;
                    }
                    for (const std::size_t sampleIndex : found->second) {
                        const openvdb::Vec3d delta =
                            openvdb::Vec3d(mCache->samples[sampleIndex].worldPosition) - position;
                        const double distanceSquared = delta.lengthSqr();
                        if (distanceSquared <= radiusSquared) {
                            visitor(sampleIndex, distanceSquared);
                        }
                    }
                }
            }
        }
    }

private:
    [[nodiscard]] SpatialBucketKey bucketFor(const openvdb::Vec3d& position) const
    {
        return SpatialBucketKey{
            static_cast<int>(std::floor((position.x() - mOrigin.x()) / mCellSize)),
            static_cast<int>(std::floor((position.y() - mOrigin.y()) / mCellSize)),
            static_cast<int>(std::floor((position.z() - mOrigin.z()) / mCellSize))};
    }

    const SurfaceTargetCache* mCache = nullptr;
    double mCellSize = 1.0;
    openvdb::Vec3d mOrigin{0.0};
    std::unordered_map<SpatialBucketKey, std::vector<std::size_t>, SpatialBucketHasher>
        mBuckets;
};

openvdb::Vec3d smallestEigenvector(
    const std::array<std::array<double, 3>, 3>& covariance)
{
    auto matrix = covariance;
    std::array<std::array<double, 3>, 3> eigenvectors{{
        {{1.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0}},
        {{0.0, 0.0, 1.0}}}};

    for (int iteration = 0; iteration < 12; ++iteration) {
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
        const double tau = (aqq - app) / (2.0 * apq);
        const double t = (tau >= 0.0 ? 1.0 : -1.0) /
            (std::abs(tau) + std::sqrt(1.0 + tau * tau));
        const double cosine = 1.0 / std::sqrt(1.0 + t * t);
        const double sine = t * cosine;

        for (int k = 0; k < 3; ++k) {
            if (k == p || k == q) {
                continue;
            }
            const double mkp = matrix[k][p];
            const double mkq = matrix[k][q];
            matrix[k][p] = matrix[p][k] = cosine * mkp - sine * mkq;
            matrix[k][q] = matrix[q][k] = sine * mkp + cosine * mkq;
        }
        matrix[p][p] = app - t * apq;
        matrix[q][q] = aqq + t * apq;
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
    openvdb::Vec3d result(
        eigenvectors[0][smallest],
        eigenvectors[1][smallest],
        eigenvectors[2][smallest]);
    const double length = result.length();
    if (!std::isfinite(length) || length <= 1.0e-20) {
        return openvdb::Vec3d(0.0);
    }
    return result / length;
}

bool fitCoreNormal(
    const SurfaceTargetCache& cache,
    const CoreSpatialIndex& spatialIndex,
    std::size_t centerIndex,
    double radius,
    openvdb::Vec3d& normal)
{
    const openvdb::Vec3d center(cache.samples[centerIndex].worldPosition);
    const double fitRadius = std::max(radius, 1.0e-9);
    openvdb::Vec3d firstMoment(0.0);
    std::array<std::array<double, 3>, 3> secondMoment{{
        {{0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}},
        {{0.0, 0.0, 0.0}}}};
    double weightSum = 0.0;
    std::size_t neighborCount = 0;
    spatialIndex.visit(center, fitRadius, [&](std::size_t index, double distanceSquared) {
        const double distance = std::sqrt(std::max(distanceSquared, 0.0));
        const double normalizedDistance = distance / fitRadius;
        const double remaining = std::max(0.0, 1.0 - normalizedDistance);
        const double weight = remaining * remaining * remaining * remaining *
            (4.0 * normalizedDistance + 1.0);
        if (weight <= 1.0e-12) {
            return;
        }
        const openvdb::Vec3d delta =
            openvdb::Vec3d(cache.samples[index].worldPosition) - center;
        firstMoment += delta * weight;
        secondMoment[0][0] += weight * delta.x() * delta.x();
        secondMoment[0][1] += weight * delta.x() * delta.y();
        secondMoment[0][2] += weight * delta.x() * delta.z();
        secondMoment[1][0] += weight * delta.y() * delta.x();
        secondMoment[1][1] += weight * delta.y() * delta.y();
        secondMoment[1][2] += weight * delta.y() * delta.z();
        secondMoment[2][0] += weight * delta.z() * delta.x();
        secondMoment[2][1] += weight * delta.z() * delta.y();
        secondMoment[2][2] += weight * delta.z() * delta.z();
        weightSum += weight;
        ++neighborCount;
    });

    constexpr std::size_t kMinimumNormalNeighbors = 6;
    if (neighborCount < kMinimumNormalNeighbors || weightSum <= 1.0e-12) {
        return false;
    }
    const openvdb::Vec3d mean = firstMoment / weightSum;
    std::array<std::array<double, 3>, 3> covariance = secondMoment;
    covariance[0][0] -= weightSum * mean.x() * mean.x();
    covariance[0][1] -= weightSum * mean.x() * mean.y();
    covariance[0][2] -= weightSum * mean.x() * mean.z();
    covariance[1][0] -= weightSum * mean.y() * mean.x();
    covariance[1][1] -= weightSum * mean.y() * mean.y();
    covariance[1][2] -= weightSum * mean.y() * mean.z();
    covariance[2][0] -= weightSum * mean.z() * mean.x();
    covariance[2][1] -= weightSum * mean.z() * mean.y();
    covariance[2][2] -= weightSum * mean.z() * mean.z();
    normal = smallestEigenvector(covariance);
    return normal.lengthSqr() > 1.0e-20;
}

void interpolateTransitionNormal(
    SurfaceTargetCache& cache,
    const CoreSpatialIndex& spatialIndex,
    std::size_t sampleIndex,
    double radius)
{
    auto& sample = cache.samples[sampleIndex];
    const openvdb::Vec3d center(sample.worldPosition);
    const double fitRadius = std::max(radius, 1.0e-9);
    const openvdb::Vec3d reference(sample.normal);
    const bool hasReference = reference.lengthSqr() > 1.0e-20;
    openvdb::Vec3d normalSum(0.0);
    openvdb::Vec3d firstNormal(0.0);
    double weightSum = 0.0;
    spatialIndex.visit(center, fitRadius, [&](std::size_t index, double distanceSquared) {
        openvdb::Vec3d candidate(cache.samples[index].normal);
        const double candidateLength = candidate.length();
        if (!std::isfinite(candidateLength) || candidateLength <= 1.0e-20) {
            return;
        }
        candidate /= candidateLength;
        if (firstNormal.lengthSqr() <= 1.0e-20) {
            firstNormal = candidate;
        }
        if (hasReference) {
            if (candidate.dot(reference) < 0.0) {
                candidate = -candidate;
            }
        } else if (candidate.dot(firstNormal) < 0.0) {
            candidate = -candidate;
        }
        const double normalizedDistance = std::sqrt(std::max(distanceSquared, 0.0)) / fitRadius;
        const double remaining = std::max(0.0, 1.0 - normalizedDistance);
        const double weight = remaining * remaining * remaining * remaining *
            (4.0 * normalizedDistance + 1.0);
        normalSum += candidate * weight;
        weightSum += weight;
    });
    const double length = normalSum.length();
    if (weightSum > 1.0e-12 && std::isfinite(length) && length > 1.0e-20) {
        sample.normal = openvdb::Vec3f(normalSum / length);
    }
}

struct TransitionChunk
{
    std::unordered_map<openvdb::Coord, std::uint8_t, CoordHasher> layers;
};

constexpr std::array<char, 8> kSurfaceTargetCacheMagic{
    'V', 'S', 'T', 'A', 'R', 'G', 'T', '1'};
constexpr std::uint32_t kSurfaceTargetCacheVersion = 2;
constexpr std::uint64_t kMaximumCacheStringLength = 1ULL << 20;

template <typename Value>
bool writeBinary(std::ofstream& stream, const Value& value)
{
    stream.write(
        reinterpret_cast<const char*>(&value),
        static_cast<std::streamsize>(sizeof(Value)));
    return static_cast<bool>(stream);
}

template <typename Value>
bool readBinary(std::ifstream& stream, Value& value)
{
    stream.read(
        reinterpret_cast<char*>(&value),
        static_cast<std::streamsize>(sizeof(Value)));
    return static_cast<bool>(stream);
}

bool writeString(std::ofstream& stream, const std::string& value)
{
    const auto length = static_cast<std::uint64_t>(value.size());
    return writeBinary(stream, length) &&
        static_cast<bool>(stream.write(
            value.data(),
            static_cast<std::streamsize>(value.size())));
}

bool readString(std::ifstream& stream, std::string& value)
{
    std::uint64_t length = 0;
    if (!readBinary(stream, length) ||
        length > kMaximumCacheStringLength ||
        length > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()) ||
        length > static_cast<std::uint64_t>(std::numeric_limits<std::streamsize>::max())) {
        return false;
    }
    value.resize(static_cast<std::size_t>(length));
    return static_cast<bool>(stream.read(
        value.data(),
        static_cast<std::streamsize>(value.size())));
}

bool metadataMatches(
    const SurfaceTargetCacheMetadata& expected,
    const SurfaceTargetCacheMetadata& actual)
{
    const auto& lhs = expected.settings;
    const auto& rhs = actual.settings;
    return expected.sourcePath == actual.sourcePath &&
        expected.gridName == actual.gridName &&
        expected.sourceFileSize == actual.sourceFileSize &&
        expected.sourceWriteTime == actual.sourceWriteTime &&
        expected.gridClass == actual.gridClass &&
        expected.hasActiveBounds == actual.hasActiveBounds &&
        expected.activeBoundsMinimum == actual.activeBoundsMinimum &&
        expected.activeBoundsMaximum == actual.activeBoundsMaximum &&
        lhs.isoValue == rhs.isoValue &&
        lhs.transitionLayers == rhs.transitionLayers &&
        lhs.normalRadius == rhs.normalRadius &&
        lhs.planarityRadius == rhs.planarityRadius &&
        lhs.planarityAngularScaleRadians == rhs.planarityAngularScaleRadians &&
        lhs.maximumSampleCount == rhs.maximumSampleCount;
}

bool setCacheError(std::string* error, const char* message)
{
    if (error) {
        *error = message;
    }
    return false;
}

bool isInside(
    const openvdb::FloatGrid& grid,
    double value,
    double isoValue)
{
    return grid.getGridClass() == openvdb::GRID_FOG_VOLUME
        ? value >= isoValue
        : value <= isoValue;
}

openvdb::Vec3f calculateNormal(
    const openvdb::FloatGrid& grid,
    const openvdb::FloatGrid::ConstAccessor& accessor,
    const openvdb::Coord& coordinate)
{
    const openvdb::Vec3d indexGradient{
        0.5 * static_cast<double>(
            accessor.getValue(coordinate.offsetBy(1, 0, 0)) -
            accessor.getValue(coordinate.offsetBy(-1, 0, 0))),
        0.5 * static_cast<double>(
            accessor.getValue(coordinate.offsetBy(0, 1, 0)) -
            accessor.getValue(coordinate.offsetBy(0, -1, 0))),
        0.5 * static_cast<double>(
            accessor.getValue(coordinate.offsetBy(0, 0, 1)) -
            accessor.getValue(coordinate.offsetBy(0, 0, -1)))};
    openvdb::Vec3d worldNormal =
        grid.transform().baseMap()->applyIJT(indexGradient, coordinate.asVec3d());
    if (grid.getGridClass() == openvdb::GRID_FOG_VOLUME) {
        worldNormal = -worldNormal;
    }
    const double length = worldNormal.length();
    if (!std::isfinite(length) || length <= 1.0e-20) {
        return openvdb::Vec3f(0.0f, 0.0f, 0.0f);
    }
    worldNormal /= length;
    return openvdb::Vec3f(worldNormal);
}

float transitionSupport(
    const openvdb::FloatGrid& grid,
    double density,
    double isoValue,
    std::size_t layer,
    std::size_t transitionLayers)
{
    const double layerWeight = 1.0 - static_cast<double>(layer) /
        static_cast<double>(std::max<std::size_t>(1, transitionLayers + 1));
    double densityWeight = 1.0;
    if (grid.getGridClass() == openvdb::GRID_FOG_VOLUME && isoValue > 1.0e-20) {
        densityWeight = std::clamp(density / isoValue, 0.0, 1.0);
    }
    return static_cast<float>(std::clamp(layerWeight * densityWeight, 0.0, 1.0));
}

void calculatePlanarity(
    SurfaceTargetCache& cache,
    std::size_t sampleIndex,
    const SurfaceTargetSettings& settings,
    const std::unordered_map<openvdb::Coord, std::size_t, CoordHasher>& coreIndices)
{
    auto& sample = cache.samples[sampleIndex];
    const openvdb::Vec3d centerWorld(sample.worldPosition);
    const openvdb::Vec3d centerNormal(sample.normal);
    if (centerNormal.lengthSqr() <= 1.0e-20) {
        return;
    }
    const double sigma = std::max(settings.planarityRadius * 0.5, 1.0e-12);
    const double inverseTwoSigmaSquared = 0.5 / (sigma * sigma);
    openvdb::Vec3d normalSum(0.0);
    double weightSum = 0.0;

    // The first cache pass uses the 26-neighborhood to keep extraction bounded.
    // A larger-radius geodesic neighborhood can be added without changing the cache schema.
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                }
                const auto found = coreIndices.find(sample.coordinate.offsetBy(dx, dy, dz));
                if (found == coreIndices.end()) {
                    continue;
                }
                const auto& neighbor = cache.samples[found->second];
                openvdb::Vec3d neighborNormal(neighbor.normal);
                if (neighborNormal.lengthSqr() <= 1.0e-20) {
                    continue;
                }
                const openvdb::Vec3d delta =
                    openvdb::Vec3d(neighbor.worldPosition) - centerWorld;
                const double distanceSquared = delta.lengthSqr();
                if (distanceSquared > settings.planarityRadius * settings.planarityRadius) {
                    continue;
                }
                if (neighborNormal.dot(centerNormal) < 0.0) {
                    neighborNormal = -neighborNormal;
                }
                const double weight = std::exp(-distanceSquared * inverseTwoSigmaSquared);
                normalSum += neighborNormal * weight;
                weightSum += weight;
            }
        }
    }

    const double resultantLength = weightSum > 0.0
        ? std::clamp(normalSum.length() / weightSum, 1.0e-12, 1.0)
        : 1.0e-12;
    const double angularDeviation = std::sqrt(-2.0 * std::log(resultantLength));
    const double support = 1.0 - std::exp(-weightSum / 3.0);
    const double normalizedDeviation = angularDeviation /
        std::max(settings.planarityAngularScaleRadians, 1.0e-12);
    sample.angularDeviationRadians = static_cast<float>(angularDeviation);
    sample.planarity = static_cast<float>(
        support * std::exp(-(normalizedDeviation * normalizedDeviation)));
}

} // namespace

SurfaceTargetCache extractSurfaceTarget(
    const openvdb::FloatGrid& grid,
    const SurfaceTargetSettings& settings)
{
    if (grid.getGridClass() != openvdb::GRID_FOG_VOLUME &&
        grid.getGridClass() != openvdb::GRID_LEVEL_SET) {
        throw std::invalid_argument("surface target requires a fog volume or level set grid");
    }
    if (!std::isfinite(settings.isoValue) ||
        settings.transitionLayers == 0 ||
        settings.transitionLayers > std::numeric_limits<std::uint8_t>::max() ||
        !std::isfinite(settings.normalRadius) || settings.normalRadius <= 0.0 ||
        !std::isfinite(settings.planarityRadius) || settings.planarityRadius <= 0.0 ||
        !std::isfinite(settings.planarityAngularScaleRadians) ||
            settings.planarityAngularScaleRadians <= 0.0 ||
        settings.maximumSampleCount == 0) {
        throw std::invalid_argument("surface target settings are invalid");
    }

    const double background = grid.background();
    using LeafNode = openvdb::FloatGrid::TreeType::LeafNodeType;
    std::vector<const LeafNode*> leaves;
    leaves.reserve(static_cast<std::size_t>(grid.tree().leafCount()));
    for (auto iterator = grid.tree().cbeginLeaf(); iterator; ++iterator) {
        leaves.push_back(&*iterator);
    }

    std::vector<std::vector<openvdb::Coord>> coreCoordinatesByLeaf(leaves.size());
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, leaves.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            const auto accessor = grid.getConstAccessor();
            for (std::size_t leafIndex = range.begin(); leafIndex != range.end(); ++leafIndex) {
                auto& localCoordinates = coreCoordinatesByLeaf[leafIndex];
                const LeafNode* leaf = leaves[leafIndex];
                for (auto iterator = leaf->cbeginValueOn(); iterator; ++iterator) {
                    const openvdb::Coord coordinate = iterator.getCoord();
                    if (!isInside(grid, iterator.getValue(), settings.isoValue)) {
                        continue;
                    }
                    bool boundary = false;
                    for (const auto& neighbor : kAxisNeighbors) {
                        if (!isInside(
                                grid,
                                accessor.getValue(coordinate + neighbor),
                                settings.isoValue)) {
                            boundary = true;
                            break;
                        }
                    }
                    if (boundary) {
                        localCoordinates.push_back(coordinate);
                    }
                }
            }
        });

    std::vector<openvdb::Coord> coreCoordinates;
    for (auto& localCoordinates : coreCoordinatesByLeaf) {
        coreCoordinates.insert(
            coreCoordinates.end(),
            localCoordinates.begin(),
            localCoordinates.end());
    }
    tbb::parallel_sort(coreCoordinates.begin(), coreCoordinates.end());
    coreCoordinates.erase(
        std::unique(coreCoordinates.begin(), coreCoordinates.end()),
        coreCoordinates.end());

    std::vector<openvdb::Coord> allCoordinates = coreCoordinates;
    std::unordered_map<openvdb::Coord, std::uint8_t, CoordHasher>
        transitionLayerByCoordinate;
    transitionLayerByCoordinate.reserve(coreCoordinates.size());
    const int layerLimit = static_cast<int>(settings.transitionLayers);
    const std::size_t chunkSize = 1024;
    const std::size_t chunkCount =
        (coreCoordinates.size() + chunkSize - 1) / chunkSize;
    std::vector<TransitionChunk> transitionChunks(chunkCount);
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, chunkCount),
        [&](const tbb::blocked_range<std::size_t>& range) {
            const auto accessor = grid.getConstAccessor();
            for (std::size_t chunkIndex = range.begin();
                 chunkIndex != range.end();
                 ++chunkIndex) {
                const std::size_t begin = chunkIndex * chunkSize;
                const std::size_t end = std::min(begin + chunkSize, coreCoordinates.size());
                auto& localLayers = transitionChunks[chunkIndex].layers;
                for (std::size_t coreIndex = begin; coreIndex < end; ++coreIndex) {
                    const auto& core = coreCoordinates[coreIndex];
                    for (int dz = -layerLimit; dz <= layerLimit; ++dz) {
                        for (int dy = -layerLimit; dy <= layerLimit; ++dy) {
                            for (int dx = -layerLimit; dx <= layerLimit; ++dx) {
                                const int layer = std::abs(dx) + std::abs(dy) + std::abs(dz);
                                if (layer == 0 || layer > layerLimit) {
                                    continue;
                                }
                                const openvdb::Coord candidate = core.offsetBy(dx, dy, dz);
                                const double value = accessor.getValue(candidate);
                                if (value != background &&
                                    !isInside(grid, value, settings.isoValue)) {
                                    const auto layerValue = static_cast<std::uint8_t>(layer);
                                    const auto found = localLayers.find(candidate);
                                    if (found == localLayers.end()) {
                                        localLayers.emplace(candidate, layerValue);
                                    } else {
                                        found->second = std::min(found->second, layerValue);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        });
    for (auto& chunk : transitionChunks) {
        for (const auto& [coordinate, layer] : chunk.layers) {
            allCoordinates.push_back(coordinate);
            const auto found = transitionLayerByCoordinate.find(coordinate);
            if (found == transitionLayerByCoordinate.end()) {
                transitionLayerByCoordinate.emplace(coordinate, layer);
            } else {
                found->second = std::min(found->second, layer);
            }
        }
    }
    tbb::parallel_sort(allCoordinates.begin(), allCoordinates.end());
    allCoordinates.erase(
        std::unique(allCoordinates.begin(), allCoordinates.end()),
        allCoordinates.end());
    if (allCoordinates.size() > settings.maximumSampleCount) {
        throw std::length_error("surface target sample count exceeds configured limit");
    }

    const auto accessor = grid.getConstAccessor();
    SurfaceTargetCache cache;
    cache.samples.reserve(allCoordinates.size());
    std::unordered_map<openvdb::Coord, std::size_t, CoordHasher> coreIndices;
    coreIndices.reserve(coreCoordinates.size());
    for (const auto& coordinate : allCoordinates) {
        const double value = accessor.getValue(coordinate);
        const bool core = isInside(grid, value, settings.isoValue);
        if (!core && value == background) {
            continue;
        }
        const bool isCore = std::binary_search(
            coreCoordinates.begin(),
            coreCoordinates.end(),
            coordinate);
        if (!isCore && core) {
            continue;
        }
        SurfaceTargetSample sample;
        sample.coordinate = coordinate;
        const openvdb::Vec3d worldPosition = grid.indexToWorld(coordinate.asVec3d());
        sample.worldPosition = openvdb::Vec3f(worldPosition);
        sample.normal = openvdb::Vec3f(0.0f);
        sample.density = static_cast<float>(value);
        sample.kind = isCore
            ? SurfaceTargetSampleKind::Core
            : SurfaceTargetSampleKind::Transition;
        if (isCore) {
            sample.supportWeight = 1.0f;
            ++cache.coreCount;
            coreIndices.emplace(coordinate, cache.samples.size());
        } else {
            const auto layer = transitionLayerByCoordinate.find(coordinate);
            sample.transitionLayer = layer == transitionLayerByCoordinate.end()
                ? static_cast<std::uint8_t>(settings.transitionLayers)
                : layer->second;
            sample.supportWeight = transitionSupport(
                grid,
                value,
                settings.isoValue,
                sample.transitionLayer,
                settings.transitionLayers);
            ++cache.transitionCount;
        }
        cache.samples.push_back(sample);
    }

    std::vector<openvdb::Vec3f> rawNormals(cache.samples.size());
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, cache.samples.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            const auto workerAccessor = grid.getConstAccessor();
            for (std::size_t index = range.begin(); index != range.end(); ++index) {
                const auto normal = calculateNormal(
                    grid,
                    workerAccessor,
                    cache.samples[index].coordinate);
                cache.samples[index].normal = normal;
                rawNormals[index] = normal;
            }
        });

    CoreSpatialIndex coreSpatialIndex(cache, settings.normalRadius);
    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, cache.samples.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            for (std::size_t index = range.begin(); index != range.end(); ++index) {
                auto& sample = cache.samples[index];
                if (sample.kind != SurfaceTargetSampleKind::Core) {
                    continue;
                }
                openvdb::Vec3d fittedNormal(0.0);
                if (!fitCoreNormal(
                        cache,
                        coreSpatialIndex,
                        index,
                        settings.normalRadius,
                        fittedNormal)) {
                    continue;
                }
                const openvdb::Vec3d rawNormal(rawNormals[index]);
                if (rawNormal.lengthSqr() > 1.0e-20 && fittedNormal.dot(rawNormal) < 0.0) {
                    fittedNormal = -fittedNormal;
                }
                sample.normal = openvdb::Vec3f(fittedNormal);
            }
        });

    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, cache.samples.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            for (std::size_t index = range.begin(); index != range.end(); ++index) {
                if (cache.samples[index].kind == SurfaceTargetSampleKind::Transition) {
                    interpolateTransitionNormal(
                        cache,
                        coreSpatialIndex,
                        index,
                        settings.normalRadius);
                }
            }
        });

    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, cache.samples.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            for (std::size_t index = range.begin(); index != range.end(); ++index) {
                if (cache.samples[index].kind == SurfaceTargetSampleKind::Core) {
                    calculatePlanarity(cache, index, settings, coreIndices);
                }
            }
        });

    return cache;
}

bool saveSurfaceTargetCache(
    const std::filesystem::path& path,
    const SurfaceTargetCache& cache,
    const SurfaceTargetCacheMetadata& metadata,
    std::string* error)
{
    std::ofstream stream(path, std::ios::binary | std::ios::trunc);
    if (!stream) {
        return setCacheError(error, "unable to open surface target cache for writing");
    }

    const std::uint32_t version = kSurfaceTargetCacheVersion;
    const std::uint8_t hasActiveBounds = metadata.hasActiveBounds ? 1 : 0;
    const std::uint64_t sampleCount = static_cast<std::uint64_t>(cache.samples.size());
    const std::uint64_t coreCount = static_cast<std::uint64_t>(cache.coreCount);
    const std::uint64_t transitionCount =
        static_cast<std::uint64_t>(cache.transitionCount);
    if (!stream.write(
            kSurfaceTargetCacheMagic.data(),
            static_cast<std::streamsize>(kSurfaceTargetCacheMagic.size())) ||
        !writeBinary(stream, version) ||
        !writeString(stream, metadata.sourcePath) ||
        !writeString(stream, metadata.gridName) ||
        !writeBinary(stream, metadata.sourceFileSize) ||
        !writeBinary(stream, metadata.sourceWriteTime) ||
        !writeBinary(stream, metadata.gridClass) ||
        !writeBinary(stream, hasActiveBounds)) {
        return setCacheError(error, "unable to write surface target cache metadata");
    }
    for (const auto value : metadata.activeBoundsMinimum) {
        if (!writeBinary(stream, value)) {
            return setCacheError(error, "unable to write surface target cache bounds");
        }
    }
    for (const auto value : metadata.activeBoundsMaximum) {
        if (!writeBinary(stream, value)) {
            return setCacheError(error, "unable to write surface target cache bounds");
        }
    }
    const auto& settings = metadata.settings;
    if (!writeBinary(stream, settings.isoValue) ||
        !writeBinary(stream, settings.transitionLayers) ||
        !writeBinary(stream, settings.normalRadius) ||
        !writeBinary(stream, settings.planarityRadius) ||
        !writeBinary(stream, settings.planarityAngularScaleRadians) ||
        !writeBinary(stream, settings.maximumSampleCount) ||
        !writeBinary(stream, sampleCount) ||
        !writeBinary(stream, coreCount) ||
        !writeBinary(stream, transitionCount)) {
        return setCacheError(error, "unable to write surface target cache header");
    }

    for (const auto& sample : cache.samples) {
        const std::int32_t x = sample.coordinate.x();
        const std::int32_t y = sample.coordinate.y();
        const std::int32_t z = sample.coordinate.z();
        const std::uint8_t kind = static_cast<std::uint8_t>(sample.kind);
        if (!writeBinary(stream, x) ||
            !writeBinary(stream, y) ||
            !writeBinary(stream, z) ||
            !writeBinary(stream, sample.worldPosition.x()) ||
            !writeBinary(stream, sample.worldPosition.y()) ||
            !writeBinary(stream, sample.worldPosition.z()) ||
            !writeBinary(stream, sample.normal.x()) ||
            !writeBinary(stream, sample.normal.y()) ||
            !writeBinary(stream, sample.normal.z()) ||
            !writeBinary(stream, sample.density) ||
            !writeBinary(stream, sample.planarity) ||
            !writeBinary(stream, sample.angularDeviationRadians) ||
            !writeBinary(stream, sample.supportWeight) ||
            !writeBinary(stream, sample.transitionLayer) ||
            !writeBinary(stream, kind)) {
            return setCacheError(error, "unable to write surface target cache samples");
        }
    }
    stream.flush();
    if (!stream) {
        return setCacheError(error, "unable to finalize surface target cache");
    }
    return true;
}

bool loadSurfaceTargetCache(
    const std::filesystem::path& path,
    const SurfaceTargetCacheMetadata& expectedMetadata,
    SurfaceTargetCache& cache,
    std::string* error)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        return setCacheError(error, "surface target cache does not exist");
    }

    std::array<char, kSurfaceTargetCacheMagic.size()> magic{};
    std::uint32_t version = 0;
    SurfaceTargetCacheMetadata actualMetadata;
    std::uint8_t hasActiveBounds = 0;
    if (!stream.read(
            magic.data(),
            static_cast<std::streamsize>(magic.size())) ||
        magic != kSurfaceTargetCacheMagic ||
        !readBinary(stream, version) ||
        version != kSurfaceTargetCacheVersion ||
        !readString(stream, actualMetadata.sourcePath) ||
        !readString(stream, actualMetadata.gridName) ||
        !readBinary(stream, actualMetadata.sourceFileSize) ||
        !readBinary(stream, actualMetadata.sourceWriteTime) ||
        !readBinary(stream, actualMetadata.gridClass) ||
        !readBinary(stream, hasActiveBounds)) {
        return setCacheError(error, "surface target cache header is invalid");
    }
    actualMetadata.hasActiveBounds = hasActiveBounds != 0;
    for (auto& value : actualMetadata.activeBoundsMinimum) {
        if (!readBinary(stream, value)) {
            return setCacheError(error, "surface target cache bounds are invalid");
        }
    }
    for (auto& value : actualMetadata.activeBoundsMaximum) {
        if (!readBinary(stream, value)) {
            return setCacheError(error, "surface target cache bounds are invalid");
        }
    }
    auto& settings = actualMetadata.settings;
    std::uint64_t sampleCount = 0;
    std::uint64_t coreCount = 0;
    std::uint64_t transitionCount = 0;
    if (!readBinary(stream, settings.isoValue) ||
        !readBinary(stream, settings.transitionLayers) ||
        !readBinary(stream, settings.normalRadius) ||
        !readBinary(stream, settings.planarityRadius) ||
        !readBinary(stream, settings.planarityAngularScaleRadians) ||
        !readBinary(stream, settings.maximumSampleCount) ||
        !readBinary(stream, sampleCount) ||
        !readBinary(stream, coreCount) ||
        !readBinary(stream, transitionCount)) {
        return setCacheError(error, "surface target cache settings are invalid");
    }
    if (!metadataMatches(expectedMetadata, actualMetadata)) {
        return setCacheError(error, "surface target cache metadata does not match the current source");
    }
    if (sampleCount > expectedMetadata.settings.maximumSampleCount ||
        sampleCount > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()) ||
        coreCount > sampleCount ||
        transitionCount > sampleCount ||
        coreCount + transitionCount != sampleCount) {
        return setCacheError(error, "surface target cache sample counts are invalid");
    }

    SurfaceTargetCache loaded;
    loaded.samples.resize(static_cast<std::size_t>(sampleCount));
    for (auto& sample : loaded.samples) {
        std::int32_t x = 0;
        std::int32_t y = 0;
        std::int32_t z = 0;
        std::uint8_t kind = 0;
        float worldX = 0.0f;
        float worldY = 0.0f;
        float worldZ = 0.0f;
        float normalX = 0.0f;
        float normalY = 0.0f;
        float normalZ = 0.0f;
        if (!readBinary(stream, x) ||
            !readBinary(stream, y) ||
            !readBinary(stream, z) ||
            !readBinary(stream, worldX) ||
            !readBinary(stream, worldY) ||
            !readBinary(stream, worldZ) ||
            !readBinary(stream, normalX) ||
            !readBinary(stream, normalY) ||
            !readBinary(stream, normalZ) ||
            !readBinary(stream, sample.density) ||
            !readBinary(stream, sample.planarity) ||
            !readBinary(stream, sample.angularDeviationRadians) ||
            !readBinary(stream, sample.supportWeight) ||
            !readBinary(stream, sample.transitionLayer) ||
            !readBinary(stream, kind) ||
            kind > static_cast<std::uint8_t>(SurfaceTargetSampleKind::Transition)) {
            return setCacheError(error, "surface target cache sample data is invalid");
        }
        sample.coordinate = openvdb::Coord(x, y, z);
        sample.worldPosition = openvdb::Vec3f(worldX, worldY, worldZ);
        sample.normal = openvdb::Vec3f(normalX, normalY, normalZ);
        sample.kind = static_cast<SurfaceTargetSampleKind>(kind);
        if (sample.kind == SurfaceTargetSampleKind::Core) {
            ++loaded.coreCount;
        } else {
            ++loaded.transitionCount;
        }
    }
    if (loaded.coreCount != coreCount || loaded.transitionCount != transitionCount) {
        return setCacheError(error, "surface target cache sample classifications are invalid");
    }
    cache = std::move(loaded);
    return true;
}

} // namespace volume_surface
