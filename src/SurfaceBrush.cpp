#include "volume_surface/SurfaceBrush.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace volume_surface {
namespace {

constexpr std::array<openvdb::Coord, 6> kAxisNeighbors{{
    openvdb::Coord(-1, 0, 0),
    openvdb::Coord(1, 0, 0),
    openvdb::Coord(0, -1, 0),
    openvdb::Coord(0, 1, 0),
    openvdb::Coord(0, 0, -1),
    openvdb::Coord(0, 0, 1)
}};

struct QueueEntry
{
    double distance = 0.0;
    std::size_t offset = 0;

    bool operator>(const QueueEntry& other) const noexcept
    {
        return distance > other.distance;
    }
};

double elapsedMilliseconds(const std::chrono::steady_clock::time_point& start)
{
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
}

struct WorldBounds
{
    openvdb::Vec3d minimum{
        std::numeric_limits<double>::infinity()};
    openvdb::Vec3d maximum{
        -std::numeric_limits<double>::infinity()};

    void include(const openvdb::Vec3d& point)
    {
        for (int axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], point[axis]);
            maximum[axis] = std::max(maximum[axis], point[axis]);
        }
    }

    void include(const WorldBounds& bounds)
    {
        include(bounds.minimum);
        include(bounds.maximum);
    }
};

WorldBounds worldBoundsForLeaf(
    const openvdb::FloatGrid& grid,
    const openvdb::CoordBBox& bounds)
{
    WorldBounds result;
    const openvdb::Vec3d minimum = bounds.min().asVec3d() - openvdb::Vec3d(0.5);
    const openvdb::Vec3d maximum = bounds.max().asVec3d() + openvdb::Vec3d(0.5);
    for (int corner = 0; corner < 8; ++corner) {
        result.include(grid.indexToWorld(openvdb::Vec3d{
            (corner & 1) ? maximum.x() : minimum.x(),
            (corner & 2) ? maximum.y() : minimum.y(),
            (corner & 4) ? maximum.z() : minimum.z()}));
    }
    return result;
}

double distanceSquaredToBounds(
    const openvdb::Vec3d& point,
    const WorldBounds& bounds)
{
    double distanceSquared = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        const double distance = point[axis] < bounds.minimum[axis]
            ? bounds.minimum[axis] - point[axis]
            : (point[axis] > bounds.maximum[axis]
                ? point[axis] - bounds.maximum[axis]
                : 0.0);
        distanceSquared += distance * distance;
    }
    return distanceSquared;
}

double distanceSquaredSegmentToBounds(
    const openvdb::Vec3d& start,
    const openvdb::Vec3d& end,
    const WorldBounds& bounds)
{
    const openvdb::Vec3d direction = end - start;
    std::array<double, 8> breakpoints{0.0, 1.0};
    std::size_t breakpointCount = 2;
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(direction[axis]) <= 1.0e-20) {
            continue;
        }
        for (const double boundary : {bounds.minimum[axis], bounds.maximum[axis]}) {
            const double t = (boundary - start[axis]) / direction[axis];
            if (t > 0.0 && t < 1.0) {
                breakpoints[breakpointCount++] = t;
            }
        }
    }
    std::sort(breakpoints.begin(), breakpoints.begin() + breakpointCount);
    breakpointCount = static_cast<std::size_t>(std::unique(
        breakpoints.begin(), breakpoints.begin() + breakpointCount) - breakpoints.begin());

    double bestDistanceSquared = std::min(
        distanceSquaredToBounds(start, bounds),
        distanceSquaredToBounds(end, bounds));
    for (std::size_t interval = 1; interval < breakpointCount; ++interval) {
        const double lower = breakpoints[interval - 1];
        const double upper = breakpoints[interval];
        const double middle = (lower + upper) * 0.5;
        double quadratic = 0.0;
        double linear = 0.0;
        double constant = 0.0;
        for (int axis = 0; axis < 3; ++axis) {
            const double middleValue = start[axis] + direction[axis] * middle;
            double offset = 0.0;
            if (middleValue < bounds.minimum[axis]) {
                offset = start[axis] - bounds.minimum[axis];
            } else if (middleValue > bounds.maximum[axis]) {
                offset = start[axis] - bounds.maximum[axis];
            } else {
                continue;
            }
            quadratic += direction[axis] * direction[axis];
            linear += 2.0 * direction[axis] * offset;
            constant += offset * offset;
        }
        const auto evaluate = [&](double t) {
            return quadratic * t * t + linear * t + constant;
        };
        bestDistanceSquared = std::min(bestDistanceSquared, evaluate(lower));
        bestDistanceSquared = std::min(bestDistanceSquared, evaluate(upper));
        if (quadratic > 1.0e-20) {
            const double minimum = std::clamp(-linear / (2.0 * quadratic), lower, upper);
            bestDistanceSquared = std::min(bestDistanceSquared, evaluate(minimum));
        }
    }
    return bestDistanceSquared;
}

class LocalSurfaceBlock
{
public:
    LocalSurfaceBlock(
        const openvdb::FloatGrid& grid,
        std::vector<openvdb::Coord> coordinates,
        double isoValue,
        double planarityRadius,
        double planarityAngularScale)
        : mGrid(grid)
        , mIsoValue(isoValue)
        , mPlanarityRadius(planarityRadius)
        , mPlanarityAngularScale(planarityAngularScale)
        , mCoordinates(std::move(coordinates))
    {
        std::sort(mCoordinates.begin(), mCoordinates.end());
        mCoordinates.erase(
            std::unique(mCoordinates.begin(), mCoordinates.end()),
            mCoordinates.end());
        mOffsets.reserve(mCoordinates.size());
        for (std::size_t offset = 0; offset < mCoordinates.size(); ++offset) {
            mOffsets.emplace(mCoordinates[offset], offset);
        }
        mValues.resize(mCoordinates.size());
        mSurface.assign(mCoordinates.size(), 0);
        mNormals.resize(mCoordinates.size());
        mPlanarity.assign(mCoordinates.size(), -1.0f);
        mAngularDeviation.assign(mCoordinates.size(), 0.0f);
        loadValues();
        classifySurface();
        calculateSurfaceNormals();
    }

    [[nodiscard]] std::size_t voxelCount() const noexcept { return mCoordinates.size(); }

    [[nodiscard]] bool contains(const openvdb::Coord& coordinate) const noexcept
    {
        return mOffsets.find(coordinate) != mOffsets.end();
    }

    [[nodiscard]] std::size_t offset(const openvdb::Coord& coordinate) const
    {
        const auto found = mOffsets.find(coordinate);
        if (found == mOffsets.end()) {
            throw std::out_of_range("surface block coordinate is outside the selected leaves");
        }
        return found->second;
    }

    [[nodiscard]] openvdb::Coord coordinate(std::size_t offset) const
    {
        return mCoordinates[offset];
    }

    [[nodiscard]] bool isSurface(std::size_t offset) const noexcept
    {
        return mSurface[offset] != 0;
    }

    [[nodiscard]] const openvdb::Vec3f& normal(std::size_t offset) const noexcept
    {
        return mNormals[offset];
    }

    [[nodiscard]] float planarity(std::size_t surfaceOffset)
    {
        if (mPlanarity[surfaceOffset] < 0.0f) {
            calculatePlanarity(surfaceOffset);
        }
        return mPlanarity[surfaceOffset];
    }

    [[nodiscard]] float angularDeviation(std::size_t surfaceOffset)
    {
        static_cast<void>(planarity(surfaceOffset));
        return mAngularDeviation[surfaceOffset];
    }

private:
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

    void loadValues()
    {
        auto accessor = mGrid.getConstAccessor();
        for (std::size_t offset = 0; offset < mCoordinates.size(); ++offset) {
            mValues[offset] = accessor.getValue(mCoordinates[offset]);
        }
    }

    [[nodiscard]] bool isInside(float value) const noexcept
    {
        return mGrid.getGridClass() == openvdb::GRID_FOG_VOLUME
            ? static_cast<double>(value) >= mIsoValue
            : static_cast<double>(value) <= mIsoValue;
    }

    void classifySurface()
    {
        for (std::size_t centerOffset = 0; centerOffset < mCoordinates.size(); ++centerOffset) {
            if (!isInside(mValues[centerOffset])) {
                continue;
            }
            const openvdb::Coord& coordinate = mCoordinates[centerOffset];
            bool hasCompleteNeighborhood = true;
            bool touchesOutside = false;
            for (const openvdb::Coord& neighbor : kAxisNeighbors) {
                const auto neighborOffset = mOffsets.find(coordinate + neighbor);
                if (neighborOffset == mOffsets.end()) {
                    hasCompleteNeighborhood = false;
                    break;
                }
                if (!isInside(mValues[neighborOffset->second])) {
                    touchesOutside = true;
                }
            }
            if (hasCompleteNeighborhood && touchesOutside) {
                mSurface[centerOffset] = 1;
            }
        }
    }

    void calculateSurfaceNormals()
    {
        const auto map = mGrid.transform().baseMap();
        for (std::size_t surfaceOffset = 0;
             surfaceOffset < mCoordinates.size();
             ++surfaceOffset) {
            if (!isSurface(surfaceOffset)) {
                continue;
            }
            const openvdb::Coord center = coordinate(surfaceOffset);
            const openvdb::Vec3d indexGradient{
                0.5 * static_cast<double>(
                    mValues[offset(center.offsetBy(1, 0, 0))] -
                    mValues[offset(center.offsetBy(-1, 0, 0))]),
                0.5 * static_cast<double>(
                    mValues[offset(center.offsetBy(0, 1, 0))] -
                    mValues[offset(center.offsetBy(0, -1, 0))]),
                0.5 * static_cast<double>(
                    mValues[offset(center.offsetBy(0, 0, 1))] -
                    mValues[offset(center.offsetBy(0, 0, -1))])};
            openvdb::Vec3d worldNormal = map->applyIJT(indexGradient, center.asVec3d());
            if (mGrid.getGridClass() == openvdb::GRID_FOG_VOLUME) {
                worldNormal = -worldNormal;
            }
            const double length = worldNormal.length();
            if (length > 1.0e-20 && std::isfinite(length)) {
                worldNormal /= length;
                mNormals[surfaceOffset] = openvdb::Vec3f(worldNormal);
            }
        }
    }

    void calculatePlanarity(std::size_t surfaceOffset)
    {
        const openvdb::Coord center = coordinate(surfaceOffset);
        const openvdb::Vec3d centerWorld = mGrid.indexToWorld(center.asVec3d());
        const openvdb::Vec3d centerNormal(mNormals[surfaceOffset]);
        const openvdb::Vec3d voxelSize = mGrid.voxelSize();
        const int radiusX = static_cast<int>(std::ceil(mPlanarityRadius / voxelSize.x()));
        const int radiusY = static_cast<int>(std::ceil(mPlanarityRadius / voxelSize.y()));
        const int radiusZ = static_cast<int>(std::ceil(mPlanarityRadius / voxelSize.z()));
        const double sigma = std::max(mPlanarityRadius * 0.5, 1.0e-12);
        const double inverseTwoSigmaSquared = 0.5 / (sigma * sigma);
        openvdb::Vec3d normalSum(0.0);
        double weightSum = 0.0;

        for (int dz = -radiusZ; dz <= radiusZ; ++dz) {
            for (int dy = -radiusY; dy <= radiusY; ++dy) {
                for (int dx = -radiusX; dx <= radiusX; ++dx) {
                    const openvdb::Coord neighbor = center.offsetBy(dx, dy, dz);
                    if (!contains(neighbor)) {
                        continue;
                    }
                    const std::size_t neighborOffset = offset(neighbor);
                    if (!isSurface(neighborOffset)) {
                        continue;
                    }
                    openvdb::Vec3d candidateNormal(mNormals[neighborOffset]);
                    if (candidateNormal.lengthSqr() <= 1.0e-20) {
                        continue;
                    }
                    const openvdb::Vec3d delta =
                        mGrid.indexToWorld(neighbor.asVec3d()) - centerWorld;
                    const double distanceSquared = delta.lengthSqr();
                    if (distanceSquared > mPlanarityRadius * mPlanarityRadius) {
                        continue;
                    }
                    if (candidateNormal.dot(centerNormal) < 0.0) {
                        candidateNormal = -candidateNormal;
                    }
                    const double weight = std::exp(-distanceSquared * inverseTwoSigmaSquared);
                    normalSum += candidateNormal * weight;
                    weightSum += weight;
                }
            }
        }

        const double resultantLength = weightSum > 0.0
            ? std::clamp(normalSum.length() / weightSum, 1.0e-12, 1.0)
            : 1.0e-12;
        const double angularDeviation = std::sqrt(-2.0 * std::log(resultantLength));
        const double support = 1.0 - std::exp(-weightSum / 3.0);
        const double normalizedDeviation = angularDeviation / mPlanarityAngularScale;
        mAngularDeviation[surfaceOffset] = static_cast<float>(angularDeviation);
        mPlanarity[surfaceOffset] = static_cast<float>(
            support * std::exp(-(normalizedDeviation * normalizedDeviation)));
    }

    const openvdb::FloatGrid& mGrid;
    double mIsoValue = 0.0;
    double mPlanarityRadius = 0.0;
    double mPlanarityAngularScale = 0.0;
    std::vector<openvdb::Coord> mCoordinates;
    std::unordered_map<openvdb::Coord, std::size_t, CoordHasher> mOffsets;
    std::vector<float> mValues;
    std::vector<std::uint8_t> mSurface;
    std::vector<openvdb::Vec3f> mNormals;
    std::vector<float> mPlanarity;
    std::vector<float> mAngularDeviation;
};

void validateParameters(
    const openvdb::FloatGrid& grid,
    const std::vector<openvdb::Vec3d>& centers,
    const SurfaceBrushParameters& brush,
    const SurfacePropagationSettings& settings)
{
    if (grid.getGridClass() != openvdb::GRID_FOG_VOLUME &&
        grid.getGridClass() != openvdb::GRID_LEVEL_SET) {
        throw std::invalid_argument("Surface brush requires a fog volume or level set grid");
    }
    if (centers.empty()) {
        throw std::invalid_argument("brush stroke must contain at least one center");
    }
    for (const openvdb::Vec3d& center : centers) {
        if (!center.isFinite()) {
            throw std::invalid_argument("brush center must be finite");
        }
    }
    if (!std::isfinite(brush.coreRadius) || brush.coreRadius < 0.0 ||
        !std::isfinite(brush.falloffRadius) || brush.falloffRadius <= 0.0 ||
        !std::isfinite(brush.strength) || brush.strength < 0.0f) {
        throw std::invalid_argument(
            "brush core radius and strength must be nonnegative; falloff radius must be positive");
    }
    if (!std::isfinite(settings.isoValue) ||
        !std::isfinite(settings.planarityRadius) || settings.planarityRadius < 0.0 ||
        !std::isfinite(settings.planarityAngularScaleRadians) ||
            settings.planarityAngularScaleRadians <= 0.0 ||
        !std::isfinite(settings.planeOffsetPenalty) || settings.planeOffsetPenalty < 0.0 ||
        !std::isfinite(settings.normalChangePenalty) || settings.normalChangePenalty < 0.0 ||
        !std::isfinite(settings.normalChangeScaleRadians) ||
            settings.normalChangeScaleRadians <= 0.0 ||
        !std::isfinite(settings.seedSearchDistance) || settings.seedSearchDistance < 0.0 ||
        settings.maximumCandidateVoxelCount == 0) {
        throw std::invalid_argument("surface propagation settings are invalid");
    }
}

void appendCoordinatesInBounds(
    std::vector<openvdb::Coord>& coordinates,
    const openvdb::Coord& minimum,
    const openvdb::Coord& maximum,
    std::size_t rawCoordinateLimit)
{
    for (int z = minimum.z(); z <= maximum.z(); ++z) {
        for (int y = minimum.y(); y <= maximum.y(); ++y) {
            for (int x = minimum.x(); x <= maximum.x(); ++x) {
                if (coordinates.size() == rawCoordinateLimit) {
                    throw std::length_error(
                        "surface brush selected leaves exceed their candidate coordinate limit");
                }
                coordinates.emplace_back(x, y, z);
            }
        }
    }
}

void sortAndUniqueCoordinates(std::vector<openvdb::Coord>& coordinates)
{
    std::sort(coordinates.begin(), coordinates.end());
    coordinates.erase(std::unique(coordinates.begin(), coordinates.end()), coordinates.end());
}

double smootherstepFalloff(double distance, const SurfaceBrushParameters& brush)
{
    if (distance <= brush.coreRadius) {
        return 1.0;
    }
    const double t = std::clamp(
        (distance - brush.coreRadius) / brush.falloffRadius,
        0.0,
        1.0);
    const double smootherstep = t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    return 1.0 - smootherstep;
}

double surfaceTransitionCost(
    LocalSurfaceBlock& block,
    const openvdb::FloatGrid& grid,
    std::size_t currentOffset,
    std::size_t neighborOffset,
    const SurfacePropagationSettings& settings)
{
    const openvdb::Coord currentCoordinate = block.coordinate(currentOffset);
    const openvdb::Coord neighborCoordinate = block.coordinate(neighborOffset);
    const openvdb::Vec3d currentWorld = grid.indexToWorld(currentCoordinate.asVec3d());
    const openvdb::Vec3d neighborWorld = grid.indexToWorld(neighborCoordinate.asVec3d());
    const openvdb::Vec3d displacement = neighborWorld - currentWorld;
    const double physicalLength = displacement.length();
    if (physicalLength <= 1.0e-20 || !std::isfinite(physicalLength)) {
        return std::numeric_limits<double>::infinity();
    }

    const openvdb::Vec3d direction = displacement / physicalLength;
    const openvdb::Vec3d currentNormal(block.normal(currentOffset));
    const openvdb::Vec3d neighborNormal(block.normal(neighborOffset));
    openvdb::Vec3d averageNormal = currentNormal + neighborNormal;
    if (averageNormal.lengthSqr() <= 1.0e-20) {
        averageNormal = currentNormal;
    } else {
        averageNormal.normalize();
    }

    const double normalDot = std::clamp(
        currentNormal.dot(neighborNormal), -1.0, 1.0);
    const double normalAngle = std::acos(normalDot);
    const double planeOffset = std::abs(direction.dot(averageNormal));
    const double confidence = std::sqrt(
        static_cast<double>(block.planarity(currentOffset)) *
        static_cast<double>(block.planarity(neighborOffset)));
    const double normalizedAngle = normalAngle / settings.normalChangeScaleRadians;
    const double penalty = confidence * (
        settings.planeOffsetPenalty * planeOffset * planeOffset +
        settings.normalChangePenalty * normalizedAngle * normalizedAngle);
    const double cost = physicalLength * (1.0 + penalty);
    return std::isfinite(cost)
        ? cost
        : std::numeric_limits<double>::infinity();
}

std::vector<std::size_t> findConstrainedSurfacePath(
    LocalSurfaceBlock& block,
    const openvdb::FloatGrid& grid,
    std::size_t sourceOffset,
    std::size_t destinationOffset,
    const SurfacePropagationSettings& settings)
{
    if (sourceOffset == destinationOffset) {
        return {sourceOffset};
    }

    const std::size_t invalidOffset = block.voxelCount();
    std::vector<double> distances(
        block.voxelCount(),
        std::numeric_limits<double>::infinity());
    std::vector<std::size_t> predecessors(block.voxelCount(), invalidOffset);
    std::vector<std::uint8_t> settled(block.voxelCount(), 0);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> queue;
    distances[sourceOffset] = 0.0;
    queue.push({0.0, sourceOffset});

    while (!queue.empty()) {
        const QueueEntry current = queue.top();
        queue.pop();
        if (settled[current.offset] || current.distance != distances[current.offset]) {
            continue;
        }
        if (current.offset == destinationOffset) {
            break;
        }
        settled[current.offset] = 1;

        const openvdb::Coord currentCoordinate = block.coordinate(current.offset);
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const openvdb::Coord neighborCoordinate =
                        currentCoordinate.offsetBy(dx, dy, dz);
                    if (!block.contains(neighborCoordinate)) {
                        continue;
                    }
                    const std::size_t neighborOffset = block.offset(neighborCoordinate);
                    if (!block.isSurface(neighborOffset) || settled[neighborOffset]) {
                        continue;
                    }
                    const double transitionCost = surfaceTransitionCost(
                        block,
                        grid,
                        current.offset,
                        neighborOffset,
                        settings);
                    const double candidateDistance = current.distance + transitionCost;
                    if (candidateDistance < distances[neighborOffset]) {
                        distances[neighborOffset] = candidateDistance;
                        predecessors[neighborOffset] = current.offset;
                        queue.push({candidateDistance, neighborOffset});
                    }
                }
            }
        }
    }

    if (!std::isfinite(distances[destinationOffset])) {
        return {};
    }

    std::vector<std::size_t> path;
    for (std::size_t offset = destinationOffset;
         offset != invalidOffset;
         offset = predecessors[offset]) {
        path.push_back(offset);
        if (offset == sourceOffset) {
            break;
        }
    }
    if (path.empty() || path.back() != sourceOffset) {
        return {};
    }
    std::reverse(path.begin(), path.end());
    return path;
}

} // namespace

struct SurfaceBrushHierarchy::Impl
{
    struct Leaf
    {
        openvdb::CoordBBox indexBounds;
        WorldBounds worldBounds;
        openvdb::Vec3d center{};
    };

    struct Node
    {
        WorldBounds worldBounds;
        std::size_t begin = 0;
        std::size_t end = 0;
        std::size_t leftChild = std::numeric_limits<std::size_t>::max();
        std::size_t rightChild = std::numeric_limits<std::size_t>::max();

        [[nodiscard]] bool isLeaf() const noexcept
        {
            return leftChild == std::numeric_limits<std::size_t>::max();
        }
    };

    explicit Impl(const openvdb::FloatGrid& sourceGrid)
    {
        for (auto leaf = sourceGrid.tree().cbeginLeaf(); leaf; ++leaf) {
            const openvdb::CoordBBox indexBounds = leaf->getNodeBoundingBox();
            const WorldBounds bounds = worldBoundsForLeaf(sourceGrid, indexBounds);
            leaves.push_back({
                indexBounds,
                bounds,
                (bounds.minimum + bounds.maximum) * 0.5});
        }
        nodes.reserve(leaves.size() * 2);
        if (!leaves.empty()) {
            static_cast<void>(buildNode(0, leaves.size(), 0));
        }
    }

    [[nodiscard]] std::size_t buildNode(
        std::size_t begin,
        std::size_t end,
        std::size_t depth)
    {
        const std::size_t nodeIndex = nodes.size();
        nodes.emplace_back();
        Node& node = nodes.back();
        node.begin = begin;
        node.end = end;
        maximumDepth = std::max(maximumDepth, depth);
        for (std::size_t leafIndex = begin; leafIndex < end; ++leafIndex) {
            node.worldBounds.include(leaves[leafIndex].worldBounds);
        }

        constexpr std::size_t maximumLeavesPerNode = 8;
        if (end - begin <= maximumLeavesPerNode) {
            return nodeIndex;
        }

        const openvdb::Vec3d extent =
            node.worldBounds.maximum - node.worldBounds.minimum;
        int splitAxis = 0;
        if (extent.y() > extent.x()) {
            splitAxis = 1;
        }
        if (extent.z() > extent[splitAxis]) {
            splitAxis = 2;
        }
        const std::size_t middle = begin + (end - begin) / 2;
        std::nth_element(
            leaves.begin() + static_cast<std::ptrdiff_t>(begin),
            leaves.begin() + static_cast<std::ptrdiff_t>(middle),
            leaves.begin() + static_cast<std::ptrdiff_t>(end),
            [splitAxis](const Leaf& left, const Leaf& right) {
                return left.center[splitAxis] < right.center[splitAxis];
            });
        const std::size_t leftChild = buildNode(begin, middle, depth + 1);
        const std::size_t rightChild = buildNode(middle, end, depth + 1);
        nodes[nodeIndex].leftChild = leftChild;
        nodes[nodeIndex].rightChild = rightChild;
        return nodeIndex;
    }

    std::vector<Leaf> leaves;
    std::vector<Node> nodes;
    std::size_t maximumDepth = 0;
};

SurfaceBrushHierarchy::SurfaceBrushHierarchy(const openvdb::FloatGrid& grid)
    : mImpl(std::make_unique<Impl>(grid))
{
}

SurfaceBrushHierarchy::~SurfaceBrushHierarchy() = default;

std::size_t SurfaceBrushHierarchy::leafCount() const noexcept
{
    return mImpl ? mImpl->leaves.size() : 0;
}

std::size_t SurfaceBrushHierarchy::debugMaxDepth() const noexcept
{
    return mImpl ? mImpl->maximumDepth : 0;
}

std::vector<SurfaceBrushHierarchyDebugNode>
SurfaceBrushHierarchy::debugNodesAtDepth(std::size_t depth) const
{
    std::vector<SurfaceBrushHierarchyDebugNode> result;
    if (!mImpl || mImpl->nodes.empty() || depth > mImpl->maximumDepth) {
        return result;
    }

    std::vector<std::pair<std::size_t, std::size_t>> pending{{0, 0}};
    while (!pending.empty()) {
        const auto [nodeIndex, nodeDepth] = pending.back();
        pending.pop_back();
        const Impl::Node& node = mImpl->nodes[nodeIndex];
        if (nodeDepth == depth) {
            result.push_back({
                node.worldBounds.minimum,
                node.worldBounds.maximum,
                nodeDepth,
                node.end - node.begin,
                node.isLeaf()});
            continue;
        }
        if (!node.isLeaf()) {
            pending.push_back({node.leftChild, nodeDepth + 1});
            pending.push_back({node.rightChild, nodeDepth + 1});
        }
    }
    return result;
}

SurfaceBrushHierarchyQuery SurfaceBrushHierarchy::queryCapsule(
    const openvdb::Vec3d& startWorld,
    const openvdb::Vec3d& endWorld,
    double radius) const
{
    if (!startWorld.isFinite() || !endWorld.isFinite() ||
        !std::isfinite(radius) || radius < 0.0) {
        throw std::invalid_argument("surface hierarchy capsule query is invalid");
    }

    SurfaceBrushHierarchyQuery query;
    if (!mImpl || mImpl->nodes.empty()) {
        return query;
    }

    const double radiusSquared = radius * radius;
    std::vector<std::size_t> pendingNodes{0};
    while (!pendingNodes.empty()) {
        const std::size_t nodeIndex = pendingNodes.back();
        pendingNodes.pop_back();
        const Impl::Node& node = mImpl->nodes[nodeIndex];
        ++query.visitedNodeCount;
        if (distanceSquaredSegmentToBounds(startWorld, endWorld, node.worldBounds) >
            radiusSquared) {
            continue;
        }
        if (!node.isLeaf()) {
            pendingNodes.push_back(node.leftChild);
            pendingNodes.push_back(node.rightChild);
            continue;
        }
        for (std::size_t leafIndex = node.begin; leafIndex < node.end; ++leafIndex) {
            const Impl::Leaf& leaf = mImpl->leaves[leafIndex];
            if (distanceSquaredSegmentToBounds(startWorld, endWorld, leaf.worldBounds) <=
                radiusSquared) {
                query.leafBounds.push_back(leaf.indexBounds);
            }
        }
    }
    return query;
}

SurfaceBrushResult propagateSurfaceBrushStroke(
    const openvdb::FloatGrid& grid,
    const std::vector<openvdb::Vec3d>& brushPathWorld,
    const SurfaceBrushParameters& brush,
    const SurfacePropagationSettings& settings,
    const SurfaceBrushHierarchy* hierarchy)
{
    validateParameters(grid, brushPathWorld, brush, settings);

    const auto totalStart = std::chrono::steady_clock::now();
    SurfaceBrushResult result;
    const openvdb::Vec3d voxelSize = grid.voxelSize();
    const double maximumVoxelSize = std::max({voxelSize.x(), voxelSize.y(), voxelSize.z()});
    const double outerRadius = brush.coreRadius + brush.falloffRadius;
    const double planarityRadius = settings.planarityRadius > 0.0
        ? settings.planarityRadius
        : maximumVoxelSize * 2.0;
    const double seedSearchDistance = settings.seedSearchDistance > 0.0
        ? settings.seedSearchDistance
        : voxelSize.length();
    const double paddedRadius = outerRadius + planarityRadius + maximumVoxelSize * 2.0;
    openvdb::Coord minimum(
        std::numeric_limits<int>::max(),
        std::numeric_limits<int>::max(),
        std::numeric_limits<int>::max());
    openvdb::Coord maximum(
        std::numeric_limits<int>::lowest(),
        std::numeric_limits<int>::lowest(),
        std::numeric_limits<int>::lowest());
    for (const openvdb::Vec3d& centerWorld : brushPathWorld) {
        const openvdb::Vec3d centerIndex = grid.worldToIndex(centerWorld);
        for (int axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(
                minimum[axis],
                static_cast<int>(std::floor(
                    centerIndex[axis] - paddedRadius / voxelSize[axis])));
            maximum[axis] = std::max(
                maximum[axis],
                static_cast<int>(std::ceil(
                    centerIndex[axis] + paddedRadius / voxelSize[axis])));
        }
    }
    std::vector<openvdb::Coord> candidateCoordinates;

    if (hierarchy) {
        const auto hierarchyQueryStart = std::chrono::steady_clock::now();
        std::vector<openvdb::CoordBBox> selectedLeafBounds;
        if (brushPathWorld.size() == 1) {
            selectedLeafBounds = hierarchy->queryCapsule(
                brushPathWorld.front(),
                brushPathWorld.front(),
                paddedRadius).leafBounds;
        } else {
            for (std::size_t center = 1; center < brushPathWorld.size(); ++center) {
                SurfaceBrushHierarchyQuery query = hierarchy->queryCapsule(
                    brushPathWorld[center - 1],
                    brushPathWorld[center],
                    paddedRadius);
                selectedLeafBounds.insert(
                    selectedLeafBounds.end(),
                    query.leafBounds.begin(),
                    query.leafBounds.end());
            }
        }
        std::sort(
            selectedLeafBounds.begin(),
            selectedLeafBounds.end(),
            [](const openvdb::CoordBBox& left, const openvdb::CoordBBox& right) {
                if (left.min() != right.min()) {
                    return left.min() < right.min();
                }
                return left.max() < right.max();
            });
        selectedLeafBounds.erase(
            std::unique(
                selectedLeafBounds.begin(),
                selectedLeafBounds.end(),
                [](const openvdb::CoordBBox& left, const openvdb::CoordBBox& right) {
                    return left.min() == right.min() && left.max() == right.max();
                }),
            selectedLeafBounds.end());
        result.candidateLeafCount = selectedLeafBounds.size();
        result.timings.hierarchyQueryMilliseconds = elapsedMilliseconds(hierarchyQueryStart);

        const std::size_t rawCoordinateLimit = settings.maximumCandidateVoxelCount >
            std::numeric_limits<std::size_t>::max() / 4
            ? std::numeric_limits<std::size_t>::max()
            : settings.maximumCandidateVoxelCount * 4;
        for (const openvdb::CoordBBox& leafBounds : selectedLeafBounds) {
            const openvdb::Coord leafMinimum(
                std::max(minimum.x(), leafBounds.min().x() - 1),
                std::max(minimum.y(), leafBounds.min().y() - 1),
                std::max(minimum.z(), leafBounds.min().z() - 1));
            const openvdb::Coord leafMaximum(
                std::min(maximum.x(), leafBounds.max().x() + 1),
                std::min(maximum.y(), leafBounds.max().y() + 1),
                std::min(maximum.z(), leafBounds.max().z() + 1));
            if (leafMinimum.x() > leafMaximum.x() ||
                leafMinimum.y() > leafMaximum.y() ||
                leafMinimum.z() > leafMaximum.z()) {
                continue;
            }
            appendCoordinatesInBounds(
                candidateCoordinates,
                leafMinimum,
                leafMaximum,
                rawCoordinateLimit);
        }
        sortAndUniqueCoordinates(candidateCoordinates);
        if (candidateCoordinates.size() > settings.maximumCandidateVoxelCount) {
            throw std::length_error("surface brush selected leaves exceed their candidate limit");
        }
    } else {
    const std::uint64_t sizeX = static_cast<std::uint64_t>(maximum.x() - minimum.x() + 1);
    const std::uint64_t sizeY = static_cast<std::uint64_t>(maximum.y() - minimum.y() + 1);
    const std::uint64_t sizeZ = static_cast<std::uint64_t>(maximum.z() - minimum.z() + 1);
    if (sizeX > std::numeric_limits<std::uint64_t>::max() / sizeY ||
        sizeX * sizeY > std::numeric_limits<std::uint64_t>::max() / sizeZ) {
        throw std::length_error("surface brush candidate block dimensions overflow");
    }
    const std::uint64_t candidateCount = sizeX * sizeY * sizeZ;
    if (candidateCount > settings.maximumCandidateVoxelCount) {
        throw std::length_error("surface brush candidate block exceeds its configured limit");
    }
    candidateCoordinates.reserve(static_cast<std::size_t>(candidateCount));
    appendCoordinatesInBounds(
        candidateCoordinates,
        minimum,
        maximum,
        settings.maximumCandidateVoxelCount);
    }

    const auto blockStart = std::chrono::steady_clock::now();
    LocalSurfaceBlock block(
        grid,
        std::move(candidateCoordinates),
        settings.isoValue,
        planarityRadius,
        settings.planarityAngularScaleRadians);

    result.candidateVoxelCount = block.voxelCount();
    result.timings.blockMilliseconds = elapsedMilliseconds(blockStart);
    const auto anchorResolveStart = std::chrono::steady_clock::now();
    std::vector<std::size_t> anchorOffsets;
    const int seedRadiusX = static_cast<int>(std::ceil(seedSearchDistance / voxelSize.x()));
    const int seedRadiusY = static_cast<int>(std::ceil(seedSearchDistance / voxelSize.y()));
    const int seedRadiusZ = static_cast<int>(std::ceil(seedSearchDistance / voxelSize.z()));
    for (const openvdb::Vec3d& centerWorld : brushPathWorld) {
        const openvdb::Vec3d centerIndex = grid.worldToIndex(centerWorld);
        const int centerX = static_cast<int>(std::floor(centerIndex.x()));
        const int centerY = static_cast<int>(std::floor(centerIndex.y()));
        const int centerZ = static_cast<int>(std::floor(centerIndex.z()));
        std::size_t bestSeedOffset = block.voxelCount();
        double bestSeedDistance = std::numeric_limits<double>::infinity();
        for (int z = centerZ - seedRadiusZ; z <= centerZ + seedRadiusZ; ++z) {
            for (int y = centerY - seedRadiusY; y <= centerY + seedRadiusY; ++y) {
                for (int x = centerX - seedRadiusX; x <= centerX + seedRadiusX; ++x) {
                    const openvdb::Coord coordinate(x, y, z);
                    if (!block.contains(coordinate)) {
                        continue;
                    }
                    const std::size_t candidateOffset = block.offset(coordinate);
                    if (!block.isSurface(candidateOffset)) {
                        continue;
                    }
                    const openvdb::Vec3d candidateWorld =
                        grid.indexToWorld(coordinate.asVec3d());
                    const double distance = (candidateWorld - centerWorld).length();
                    if (distance <= seedSearchDistance && distance < bestSeedDistance) {
                        bestSeedDistance = distance;
                        bestSeedOffset = candidateOffset;
                    }
                }
            }
        }
        if (bestSeedOffset != block.voxelCount() &&
            (anchorOffsets.empty() || anchorOffsets.back() != bestSeedOffset)) {
            anchorOffsets.push_back(bestSeedOffset);
        }
    }
    if (anchorOffsets.empty()) {
        result.timings.anchorResolveMilliseconds = elapsedMilliseconds(anchorResolveStart);
        result.timings.totalMilliseconds = elapsedMilliseconds(totalStart);
        return result;
    }
    result.timings.anchorResolveMilliseconds = elapsedMilliseconds(anchorResolveStart);

    const auto centerlineRouteStart = std::chrono::steady_clock::now();
    std::vector<std::vector<std::size_t>> centerlineOffsetPaths;
    centerlineOffsetPaths.push_back({anchorOffsets.front()});
    for (std::size_t anchorIndex = 1; anchorIndex < anchorOffsets.size(); ++anchorIndex) {
        std::vector<std::size_t>& currentPath = centerlineOffsetPaths.back();
        const std::vector<std::size_t> connection = findConstrainedSurfacePath(
            block,
            grid,
            currentPath.back(),
            anchorOffsets[anchorIndex],
            settings);
        if (connection.empty()) {
            centerlineOffsetPaths.push_back({anchorOffsets[anchorIndex]});
            continue;
        }
        currentPath.insert(currentPath.end(), connection.begin() + 1, connection.end());
    }

    std::vector<std::uint8_t> centerlineSourceMask(block.voxelCount(), 0);
    std::vector<std::size_t> seedOffsets;
    for (const std::vector<std::size_t>& centerlinePath : centerlineOffsetPaths) {
        std::vector<openvdb::Coord>& outputPath = result.centerlinePaths.emplace_back();
        outputPath.reserve(centerlinePath.size());
        for (const std::size_t offset : centerlinePath) {
            outputPath.push_back(block.coordinate(offset));
            if (!centerlineSourceMask[offset]) {
                centerlineSourceMask[offset] = 1;
                seedOffsets.push_back(offset);
            }
        }
    }
    result.seedCoordinate = result.centerlinePaths.front().front();
    result.timings.centerlineRouteMilliseconds = elapsedMilliseconds(centerlineRouteStart);

    const auto surfaceSweepStart = std::chrono::steady_clock::now();
    std::vector<double> distances(
        block.voxelCount(),
        std::numeric_limits<double>::infinity());
    std::vector<std::uint8_t> settled(block.voxelCount(), 0);
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> queue;
    for (const std::size_t seedOffset : seedOffsets) {
        distances[seedOffset] = 0.0;
        queue.push({0.0, seedOffset});
    }

    while (!queue.empty()) {
        const QueueEntry current = queue.top();
        queue.pop();
        if (settled[current.offset] || current.distance != distances[current.offset]) {
            continue;
        }
        settled[current.offset] = 1;
        if (current.distance >= outerRadius) {
            continue;
        }

        const openvdb::Coord currentCoordinate = block.coordinate(current.offset);
        const openvdb::Vec3d currentWorld = grid.indexToWorld(currentCoordinate.asVec3d());
        const openvdb::Vec3d currentNormal(block.normal(current.offset));
        const float currentPlanarity = block.planarity(current.offset);
        const double falloff = smootherstepFalloff(current.distance, brush);

        SurfaceBrushSample sample;
        sample.coordinate = currentCoordinate;
        sample.worldPosition = {
            static_cast<float>(currentWorld.x()),
            static_cast<float>(currentWorld.y()),
            static_cast<float>(currentWorld.z())};
        sample.normal = {
            static_cast<float>(currentNormal.x()),
            static_cast<float>(currentNormal.y()),
            static_cast<float>(currentNormal.z())};
        sample.angularDeviationRadians = block.angularDeviation(current.offset);
        sample.planarity = currentPlanarity;
        sample.propagationDistance = static_cast<float>(current.distance);
        sample.weight = static_cast<float>(static_cast<double>(brush.strength) * falloff);
        result.samples.push_back(sample);

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) {
                        continue;
                    }
                    const openvdb::Coord neighborCoordinate =
                        currentCoordinate.offsetBy(dx, dy, dz);
                    if (!block.contains(neighborCoordinate)) {
                        continue;
                    }
                    const std::size_t neighborOffset = block.offset(neighborCoordinate);
                    if (!block.isSurface(neighborOffset) || settled[neighborOffset]) {
                        continue;
                    }

                    const double candidateDistance = current.distance + surfaceTransitionCost(
                        block,
                        grid,
                        current.offset,
                        neighborOffset,
                        settings);
                    if (candidateDistance < outerRadius &&
                        candidateDistance < distances[neighborOffset]) {
                        distances[neighborOffset] = candidateDistance;
                        queue.push({candidateDistance, neighborOffset});
                    }
                }
            }
        }
    }

    result.timings.surfaceSweepMilliseconds = elapsedMilliseconds(surfaceSweepStart);
    result.timings.totalMilliseconds = elapsedMilliseconds(totalStart);
    return result;
}

SurfaceBrushResult propagateSurfaceBrushStroke(
    const openvdb::FloatGrid& grid,
    const std::vector<openvdb::Vec3d>& brushPathWorld,
    const SurfaceBrushParameters& brush,
    const SurfacePropagationSettings& settings)
{
    return propagateSurfaceBrushStroke(
        grid,
        brushPathWorld,
        brush,
        settings,
        nullptr);
}

SurfaceBrushResult propagateSurfaceBrush(
    const openvdb::FloatGrid& grid,
    const openvdb::Vec3d& brushCenterWorld,
    const SurfaceBrushParameters& brush,
    const SurfacePropagationSettings& settings)
{
    return propagateSurfaceBrushStroke(
        grid,
        {brushCenterWorld},
        brush,
        settings);
}

} // namespace volume_surface
