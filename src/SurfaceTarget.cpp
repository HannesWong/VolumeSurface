#include "volume_surface/SurfaceTarget.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include <openvdb/math/Stencils.h>

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

struct TransitionChunk
{
    std::unordered_map<openvdb::Coord, std::uint8_t, CoordHasher> layers;
};

constexpr std::array<char, 8> kSurfaceTargetCacheMagic{
    'V', 'S', 'T', 'A', 'R', 'G', 'T', '1'};
constexpr std::uint32_t kSurfaceTargetCacheVersion = 4;
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

struct SurfaceCrossings
{
    std::array<openvdb::Vec3d, 6> positions{};
    std::size_t count = 0;
};

SurfaceCrossings collectCoreSurfaceCrossings(
    const openvdb::FloatGrid& grid,
    const openvdb::FloatGrid::ConstAccessor& accessor,
    const openvdb::Coord& coordinate,
    double centerValue,
    double isoValue)
{
    const openvdb::Vec3d centerWorld = grid.indexToWorld(coordinate.asVec3d());
    const bool centerInside = isInside(grid, centerValue, isoValue);
    SurfaceCrossings crossings;
    for (const auto& direction : kAxisNeighbors) {
        const openvdb::Coord neighborCoordinate = coordinate + direction;
        const double neighborValue = accessor.getValue(neighborCoordinate);
        if (centerInside == isInside(grid, neighborValue, isoValue)) {
            continue;
        }
        const double denominator = neighborValue - centerValue;
        const double interpolation = std::abs(denominator) > 1.0e-12
            ? std::clamp((isoValue - centerValue) / denominator, 0.0, 1.0)
            : 0.5;
        const openvdb::Vec3d neighborWorld =
            grid.indexToWorld(neighborCoordinate.asVec3d());
        crossings.positions[crossings.count++] = centerWorld +
            (neighborWorld - centerWorld) * interpolation;
    }
    return crossings;
}

openvdb::Vec3d calculateSurfaceGradient(
    const openvdb::FloatGrid& grid,
    openvdb::math::BoxStencil<openvdb::FloatGrid>& stencil,
    const openvdb::Vec3d& worldPosition)
{
    const openvdb::Vec3d indexPosition = grid.worldToIndex(worldPosition);
    if (!indexPosition.isFinite()) {
        return openvdb::Vec3d(0.0);
    }
    stencil.moveTo(indexPosition);
    const auto sampledGradient = stencil.gradient(indexPosition);
    openvdb::Vec3d worldGradient{
        static_cast<double>(sampledGradient.x()),
        static_cast<double>(sampledGradient.y()),
        static_cast<double>(sampledGradient.z())};
    if (grid.getGridClass() == openvdb::GRID_FOG_VOLUME) {
        worldGradient = -worldGradient;
    }
    return worldGradient;
}

openvdb::Vec3f normalizeGradient(openvdb::Vec3d worldGradient)
{
    const double length = worldGradient.length();
    if (!std::isfinite(length) || length <= 1.0e-20) {
        return openvdb::Vec3f(0.0f, 0.0f, 0.0f);
    }
    worldGradient /= length;
    return openvdb::Vec3f(worldGradient);
}

openvdb::Vec3f calculateVoxelCentralDifferenceNormal(
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
    openvdb::Vec3d worldGradient =
        grid.transform().baseMap()->applyIJT(indexGradient, coordinate.asVec3d());
    if (grid.getGridClass() == openvdb::GRID_FOG_VOLUME) {
        worldGradient = -worldGradient;
    }
    return normalizeGradient(worldGradient);
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

    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, cache.samples.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            openvdb::math::BoxStencil<openvdb::FloatGrid> workerStencil(grid);
            const auto workerAccessor = grid.getConstAccessor();
            for (std::size_t index = range.begin(); index != range.end(); ++index) {
                const auto& sample = cache.samples[index];
                openvdb::Vec3f normal;
                if (sample.kind == SurfaceTargetSampleKind::Core) {
                    const auto crossings = collectCoreSurfaceCrossings(
                        grid,
                        workerAccessor,
                        sample.coordinate,
                        static_cast<double>(sample.density),
                        settings.isoValue);
                    openvdb::Vec3d normalSum(0.0);
                    for (std::size_t crossingIndex = 0;
                         crossingIndex < crossings.count;
                         ++crossingIndex) {
                        const openvdb::Vec3f crossingNormal = normalizeGradient(
                            calculateSurfaceGradient(
                                grid,
                                workerStencil,
                                crossings.positions[crossingIndex]));
                        const openvdb::Vec3d direction(crossingNormal);
                        if (direction.lengthSqr() <= 1.0e-20) {
                            continue;
                        }
                        normalSum += direction;
                    }
                    normal = normalizeGradient(normalSum);
                } else {
                    normal = normalizeGradient(calculateSurfaceGradient(
                        grid,
                        workerStencil,
                        openvdb::Vec3d(sample.worldPosition)));
                }
                if (normal.lengthSqr() <= 1.0e-20) {
                    normal = calculateVoxelCentralDifferenceNormal(
                        grid,
                        workerAccessor,
                        sample.coordinate);
                }
                cache.samples[index].normal = normal;
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

SurfaceTargetComponentFilterResult retainLargestSurfaceTargetComponent(
    const SurfaceTargetCache& cache)
{
    SurfaceTargetComponentFilterResult result;
    if (cache.samples.empty()) {
        return result;
    }

    std::unordered_map<openvdb::Coord, std::size_t, CoordHasher> coreIndices;
    coreIndices.reserve(cache.coreCount * 2 + 1);
    for (std::size_t sampleIndex = 0; sampleIndex < cache.samples.size(); ++sampleIndex) {
        if (cache.samples[sampleIndex].kind == SurfaceTargetSampleKind::Core) {
            coreIndices.emplace(cache.samples[sampleIndex].coordinate, sampleIndex);
        }
    }
    if (coreIndices.empty()) {
        result.primary = cache;
        return result;
    }

    std::vector<std::int32_t> componentBySample(cache.samples.size(), -1);
    std::vector<std::size_t> componentSizes;
    std::vector<std::size_t> pending;
    constexpr std::array<int, 3> offsets{-1, 0, 1};
    for (const auto& entry : coreIndices) {
        const std::size_t seedIndex = entry.second;
        if (componentBySample[seedIndex] >= 0) {
            continue;
        }
        const auto componentIndex = static_cast<std::int32_t>(componentSizes.size());
        std::size_t componentSize = 0;
        pending.clear();
        pending.push_back(seedIndex);
        componentBySample[seedIndex] = componentIndex;
        while (!pending.empty()) {
            const std::size_t currentIndex = pending.back();
            pending.pop_back();
            ++componentSize;
            const auto center = cache.samples[currentIndex].coordinate;
            for (const int dz : offsets) {
                for (const int dy : offsets) {
                    for (const int dx : offsets) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        const auto found = coreIndices.find(center.offsetBy(dx, dy, dz));
                        if (found == coreIndices.end() ||
                            componentBySample[found->second] >= 0) {
                            continue;
                        }
                        componentBySample[found->second] = componentIndex;
                        pending.push_back(found->second);
                    }
                }
            }
        }
        componentSizes.push_back(componentSize);
    }

    result.componentCount = componentSizes.size();
    const auto primaryComponent = static_cast<std::int32_t>(std::distance(
        componentSizes.begin(),
        std::max_element(componentSizes.begin(), componentSizes.end())));
    result.primary.samples.reserve(cache.samples.size());

    auto transitionBelongsToPrimary = [&](const SurfaceTargetSample& sample) {
        const int layer = std::max(1, static_cast<int>(sample.transitionLayer));
        const auto center = sample.coordinate;
        for (int dz = -layer; dz <= layer; ++dz) {
            for (int dy = -layer; dy <= layer; ++dy) {
                for (int dx = -layer; dx <= layer; ++dx) {
                    if (std::abs(dx) + std::abs(dy) + std::abs(dz) > layer) {
                        continue;
                    }
                    const auto found = coreIndices.find(center.offsetBy(dx, dy, dz));
                    if (found != coreIndices.end() &&
                        componentBySample[found->second] == primaryComponent) {
                        return true;
                    }
                }
            }
        }
        return false;
    };

    for (std::size_t sampleIndex = 0; sampleIndex < cache.samples.size(); ++sampleIndex) {
        const auto& sample = cache.samples[sampleIndex];
        bool keep = false;
        if (sample.kind == SurfaceTargetSampleKind::Core) {
            keep = componentBySample[sampleIndex] == primaryComponent;
            if (!keep) {
                ++result.excludedCoreCount;
            }
        } else {
            keep = transitionBelongsToPrimary(sample);
            if (!keep) {
                ++result.excludedTransitionCount;
            }
        }
        if (!keep) {
            continue;
        }
        result.primary.samples.push_back(sample);
        if (sample.kind == SurfaceTargetSampleKind::Core) {
            ++result.primary.coreCount;
        } else {
            ++result.primary.transitionCount;
        }
    }
    return result;
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
