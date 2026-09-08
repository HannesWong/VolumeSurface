#include "volume_surface/SurfaceMeshContinuity.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

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

struct CoordinateVertexEntry
{
    openvdb::Coord coordinate{};
    std::uint32_t vertex = 0;
};

struct VertexRange
{
    std::size_t begin = 0;
    std::size_t end = 0;
};

openvdb::Vec3d normalizeOrZero(openvdb::Vec3d value)
{
    const double length = value.length();
    if (!std::isfinite(length) || length <= kEpsilon) {
        return openvdb::Vec3d(0.0);
    }
    return value / length;
}

bool finitePositive(const openvdb::Vec3d& value)
{
    return value.isFinite() && value.x() > 0.0 && value.y() > 0.0 && value.z() > 0.0;
}

int roundedIndex(double value)
{
    if (!std::isfinite(value)) {
        return 0;
    }
    if (value <= static_cast<double>(std::numeric_limits<int>::min())) {
        return std::numeric_limits<int>::min();
    }
    if (value >= static_cast<double>(std::numeric_limits<int>::max())) {
        return std::numeric_limits<int>::max();
    }
    return static_cast<int>(std::llround(value));
}

openvdb::Coord meshVertexCoordinate(
    const openvdb::FloatGrid& grid,
    const SurfaceVertex& vertex)
{
    const openvdb::Vec3d indexPosition = grid.worldToIndex(openvdb::Vec3d(
        static_cast<double>(vertex.position[0]),
        static_cast<double>(vertex.position[1]),
        static_cast<double>(vertex.position[2])));
    return openvdb::Coord(
        roundedIndex(indexPosition.x()),
        roundedIndex(indexPosition.y()),
        roundedIndex(indexPosition.z()));
}

std::vector<std::uint32_t> buildAdjacency(
    const SurfaceMesh& mesh,
    std::vector<std::uint32_t>& offsets)
{
    std::vector<std::pair<std::uint32_t, std::uint32_t>> edges;
    edges.reserve(mesh.indices.size() * 2);
    for (std::size_t index = 0; index + 2 < mesh.indices.size(); index += 3) {
        const std::uint32_t a = mesh.indices[index];
        const std::uint32_t b = mesh.indices[index + 1];
        const std::uint32_t c = mesh.indices[index + 2];
        if (a == b || a == c || b == c) {
            continue;
        }
        edges.emplace_back(a, b);
        edges.emplace_back(b, a);
        edges.emplace_back(a, c);
        edges.emplace_back(c, a);
        edges.emplace_back(b, c);
        edges.emplace_back(c, b);
    }
    tbb::parallel_sort(
        edges.begin(),
        edges.end(),
        [](const auto& left, const auto& right) {
            return left < right;
        });
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());

    offsets.assign(mesh.vertices.size() + 1, 0);
    for (const auto& [from, to] : edges) {
        (void)to;
        if (from < mesh.vertices.size()) {
            ++offsets[static_cast<std::size_t>(from) + 1];
        }
    }
    for (std::size_t index = 1; index < offsets.size(); ++index) {
        offsets[index] += offsets[index - 1];
    }
    std::vector<std::uint32_t> adjacency(offsets.back());
    std::vector<std::uint32_t> cursors = offsets;
    for (const auto& [from, to] : edges) {
        if (from >= mesh.vertices.size() || to >= mesh.vertices.size()) {
            continue;
        }
        adjacency[cursors[from]++] = to;
    }
    return adjacency;
}

bool adjacent(
    const SurfaceMeshContinuityField& field,
    std::uint32_t first,
    std::uint32_t second) noexcept
{
    if (first >= field.meshVertexSmoothness.size() ||
        second >= field.meshVertexSmoothness.size() ||
        first + 1 >= field.adjacencyOffsets.size()) {
        return false;
    }
    const auto begin = field.adjacencyVertices.begin() +
        field.adjacencyOffsets[first];
    const auto end = field.adjacencyVertices.begin() +
        field.adjacencyOffsets[first + 1];
    return std::binary_search(begin, end, second);
}

bool shareNeighbor(
    const SurfaceMeshContinuityField& field,
    std::uint32_t first,
    std::uint32_t second) noexcept
{
    if (first >= field.meshVertexSmoothness.size() ||
        second >= field.meshVertexSmoothness.size() ||
        first + 1 >= field.adjacencyOffsets.size() ||
        second + 1 >= field.adjacencyOffsets.size()) {
        return false;
    }
    const std::size_t firstBegin = field.adjacencyOffsets[first];
    const std::size_t firstEnd = field.adjacencyOffsets[first + 1];
    const std::size_t secondBegin = field.adjacencyOffsets[second];
    const std::size_t secondEnd = field.adjacencyOffsets[second + 1];
    if (firstEnd - firstBegin > secondEnd - secondBegin) {
        return shareNeighbor(field, second, first);
    }
    for (std::size_t offset = firstBegin; offset < firstEnd; ++offset) {
        const std::uint32_t candidate = field.adjacencyVertices[offset];
        if (adjacent(field, second, candidate)) {
            return true;
        }
    }
    return false;
}

double topologyWeight(
    const SurfaceMeshContinuityField& field,
    std::uint32_t first,
    std::uint32_t second) noexcept
{
    if (first == SurfaceMeshContinuityField::InvalidMeshVertex ||
        second == SurfaceMeshContinuityField::InvalidMeshVertex) {
        return 0.0;
    }
    if (first == second || adjacent(field, first, second)) {
        return 1.0;
    }
    return shareNeighbor(field, first, second) ? 0.6 : 0.0;
}

} // namespace

SurfaceMeshContinuityField buildSurfaceMeshContinuity(
    const SurfaceMesh& mesh,
    const SurfaceTargetCache& target,
    const openvdb::FloatGrid& grid,
    const SurfaceMeshContinuitySettings& settings)
{
    if (mesh.vertices.empty() || mesh.indices.empty() || target.empty()) {
        return {};
    }
    if (!std::isfinite(settings.minimumProximity) ||
        settings.minimumProximity < 0.0 || settings.minimumProximity > 1.0) {
        throw std::invalid_argument("mesh continuity proximity threshold is invalid");
    }

    const openvdb::Vec3d voxelSize{
        std::abs(grid.voxelSize().x()),
        std::abs(grid.voxelSize().y()),
        std::abs(grid.voxelSize().z())};
    if (!finitePositive(voxelSize)) {
        throw std::invalid_argument("mesh continuity requires a finite voxel size");
    }
    const double proximityRadius = settings.searchRadius > 0.0 &&
        std::isfinite(settings.searchRadius)
        ? settings.searchRadius
        : 1.75 * std::max({voxelSize.x(), voxelSize.y(), voxelSize.z()});
    if (!std::isfinite(proximityRadius) || proximityRadius <= 0.0) {
        throw std::invalid_argument("mesh continuity search radius is invalid");
    }

    SurfaceMeshContinuityField result;
    result.nearestMeshVertex.assign(
        target.samples.size(),
        SurfaceMeshContinuityField::InvalidMeshVertex);
    result.sampleProximity.assign(target.samples.size(), 0.0f);
    result.sampleSmoothness.assign(target.samples.size(), 0.0f);
    result.meshVertexSmoothness.assign(mesh.vertices.size(), 0.0f);
    result.meshVertexNormals.assign(mesh.vertices.size(), openvdb::Vec3f(0.0f));
    result.adjacencyVertices = buildAdjacency(mesh, result.adjacencyOffsets);

    std::vector<openvdb::Vec3d> meshPositions(mesh.vertices.size());
    std::vector<openvdb::Vec3d> meshNormals(mesh.vertices.size());
    for (std::size_t index = 0; index < mesh.vertices.size(); ++index) {
        const auto& vertex = mesh.vertices[index];
        meshPositions[index] = openvdb::Vec3d(
            static_cast<double>(vertex.position[0]),
            static_cast<double>(vertex.position[1]),
            static_cast<double>(vertex.position[2]));
        meshNormals[index] = normalizeOrZero(openvdb::Vec3d(
            static_cast<double>(vertex.normal[0]),
            static_cast<double>(vertex.normal[1]),
            static_cast<double>(vertex.normal[2])));
        result.meshVertexNormals[index] = openvdb::Vec3f(meshNormals[index]);
    }

    for (std::size_t vertex = 0; vertex < mesh.vertices.size(); ++vertex) {
        const openvdb::Vec3d normal = meshNormals[vertex];
        if (normal.lengthSqr() <= kEpsilon || vertex + 1 >= result.adjacencyOffsets.size()) {
            result.meshVertexSmoothness[vertex] = 0.0f;
            continue;
        }
        const std::size_t begin = result.adjacencyOffsets[vertex];
        const std::size_t end = result.adjacencyOffsets[vertex + 1];
        if (begin == end) {
            result.meshVertexSmoothness[vertex] = 1.0f;
            continue;
        }
        double sum = 0.0;
        for (std::size_t offset = begin; offset < end; ++offset) {
            const std::uint32_t neighbor = result.adjacencyVertices[offset];
            if (neighbor >= meshNormals.size()) {
                continue;
            }
            sum += std::clamp(normal.dot(meshNormals[neighbor]), 0.0, 1.0);
        }
        result.meshVertexSmoothness[vertex] = static_cast<float>(std::clamp(
            sum / static_cast<double>(end - begin),
            0.0,
            1.0));
    }

    std::vector<CoordinateVertexEntry> entries;
    entries.reserve(mesh.vertices.size());
    for (std::size_t index = 0; index < mesh.vertices.size(); ++index) {
        entries.push_back({
            meshVertexCoordinate(grid, mesh.vertices[index]),
            static_cast<std::uint32_t>(index)});
    }
    tbb::parallel_sort(
        entries.begin(),
        entries.end(),
        [](const CoordinateVertexEntry& left, const CoordinateVertexEntry& right) {
            if (left.coordinate != right.coordinate) {
                if (left.coordinate.x() != right.coordinate.x()) {
                    return left.coordinate.x() < right.coordinate.x();
                }
                if (left.coordinate.y() != right.coordinate.y()) {
                    return left.coordinate.y() < right.coordinate.y();
                }
                return left.coordinate.z() < right.coordinate.z();
            }
            return left.vertex < right.vertex;
        });
    std::unordered_map<openvdb::Coord, VertexRange, CoordHasher> ranges;
    ranges.reserve(entries.size() / 2 + 1);
    std::size_t begin = 0;
    while (begin < entries.size()) {
        std::size_t end = begin + 1;
        while (end < entries.size() && entries[end].coordinate == entries[begin].coordinate) {
            ++end;
        }
        ranges.emplace(entries[begin].coordinate, VertexRange{begin, end});
        begin = end;
    }

    tbb::parallel_for(
        tbb::blocked_range<std::size_t>(0, target.samples.size()),
        [&](const tbb::blocked_range<std::size_t>& range) {
            for (std::size_t sampleIndex = range.begin();
                 sampleIndex != range.end();
                 ++sampleIndex) {
                const auto& sample = target.samples[sampleIndex];
                const openvdb::Coord center = sample.coordinate;
                std::uint32_t nearest = SurfaceMeshContinuityField::InvalidMeshVertex;
                double nearestDistance = std::numeric_limits<double>::infinity();
                auto visit = [&](int radius) {
                    for (int dz = -radius; dz <= radius; ++dz) {
                        for (int dy = -radius; dy <= radius; ++dy) {
                            for (int dx = -radius; dx <= radius; ++dx) {
                                const auto found = ranges.find(center.offsetBy(dx, dy, dz));
                                if (found == ranges.end()) {
                                    continue;
                                }
                                for (std::size_t offset = found->second.begin;
                                     offset < found->second.end;
                                     ++offset) {
                                    const std::uint32_t vertex = entries[offset].vertex;
                                    const openvdb::Vec3d delta = meshPositions[vertex] -
                                        openvdb::Vec3d(sample.worldPosition);
                                    const double distance = delta.lengthSqr();
                                    if (distance < nearestDistance) {
                                        nearestDistance = distance;
                                        nearest = vertex;
                                    }
                                }
                            }
                        }
                    }
                };
                visit(1);
                if (nearest == SurfaceMeshContinuityField::InvalidMeshVertex) {
                    // Adaptive source meshes can leave several source-grid
                    // indices between a target sample and the nearest mesh
                    // vertex. Keep the normal case at one ring and widen only
                    // when the local cell is empty.
                    visit(4);
                }
                if (nearest == SurfaceMeshContinuityField::InvalidMeshVertex ||
                    !std::isfinite(nearestDistance)) {
                    continue;
                }
                const double distance = std::sqrt(std::max(0.0, nearestDistance));
                const double normalizedDistance = distance / proximityRadius;
                const double proximity = std::exp(-normalizedDistance * normalizedDistance);
                if (!std::isfinite(proximity) || proximity < settings.minimumProximity) {
                    continue;
                }
                result.nearestMeshVertex[sampleIndex] = nearest;
                result.sampleProximity[sampleIndex] = static_cast<float>(proximity);
                result.sampleSmoothness[sampleIndex] =
                    nearest < result.meshVertexSmoothness.size()
                    ? static_cast<float>(proximity * result.meshVertexSmoothness[nearest])
                    : 0.0f;
            }
        });

    result.supportedSampleCount = static_cast<std::size_t>(std::count_if(
        result.nearestMeshVertex.begin(),
        result.nearestMeshVertex.end(),
        [](std::uint32_t index) {
            return index != SurfaceMeshContinuityField::InvalidMeshVertex;
        }));
    if (result.supportedSampleCount > target.samples.size()) {
        throw std::runtime_error("mesh continuity support count is inconsistent");
    }
    return result;
}

double surfaceMeshContinuityWeightBase(
    const SurfaceMeshContinuityField& field,
    std::size_t firstSampleIndex,
    std::size_t secondSampleIndex) noexcept
{
    if (firstSampleIndex >= field.nearestMeshVertex.size() ||
        secondSampleIndex >= field.nearestMeshVertex.size() ||
        firstSampleIndex >= field.sampleProximity.size() ||
        secondSampleIndex >= field.sampleProximity.size()) {
        return 0.0;
    }
    const std::uint32_t first = field.nearestMeshVertex[firstSampleIndex];
    const std::uint32_t second = field.nearestMeshVertex[secondSampleIndex];
    const double topology = topologyWeight(field, first, second);
    if (topology <= 0.0) {
        return 0.0;
    }
    const double proximity = std::sqrt(std::max(
        0.0,
        static_cast<double>(field.sampleProximity[firstSampleIndex]) *
            static_cast<double>(field.sampleProximity[secondSampleIndex])));
    const double smoothness = (first < field.meshVertexSmoothness.size() &&
        second < field.meshVertexSmoothness.size())
        ? std::sqrt(std::max(
            0.0,
            static_cast<double>(field.meshVertexSmoothness[first]) *
                static_cast<double>(field.meshVertexSmoothness[second])))
        : 0.0;
    return std::clamp(proximity * smoothness * topology, 0.0, 1.0);
}

double surfaceMeshContinuityWeight(
    const SurfaceMeshContinuityField& field,
    std::size_t firstSampleIndex,
    std::size_t secondSampleIndex) noexcept
{
    return surfaceMeshContinuityWeightBase(
        field,
        firstSampleIndex,
        secondSampleIndex);
}

double surfaceMeshContinuityWeight(
    const SurfaceMeshContinuityField& field,
    const SurfaceTargetCache& target,
    std::size_t firstSampleIndex,
    std::size_t secondSampleIndex) noexcept
{
    const double base = surfaceMeshContinuityWeightBase(
        field,
        firstSampleIndex,
        secondSampleIndex);
    if (base <= 0.0 ||
        firstSampleIndex >= target.samples.size() ||
        secondSampleIndex >= target.samples.size() ||
        firstSampleIndex >= field.nearestMeshVertex.size() ||
        secondSampleIndex >= field.nearestMeshVertex.size() ||
        field.meshVertexNormals.empty()) {
        return base;
    }
    const std::uint32_t first = field.nearestMeshVertex[firstSampleIndex];
    const std::uint32_t second = field.nearestMeshVertex[secondSampleIndex];
    if (first >= field.meshVertexNormals.size() ||
        second >= field.meshVertexNormals.size()) {
        return base;
    }
    const openvdb::Vec3d displacement = openvdb::Vec3d(
        target.samples[secondSampleIndex].worldPosition) -
        openvdb::Vec3d(target.samples[firstSampleIndex].worldPosition);
    const double length = displacement.length();
    if (!std::isfinite(length) || length <= kEpsilon) {
        return base;
    }
    const openvdb::Vec3d direction = displacement / length;
    const openvdb::Vec3d firstNormal = normalizeOrZero(
        openvdb::Vec3d(field.meshVertexNormals[first]));
    const openvdb::Vec3d secondNormal = normalizeOrZero(
        openvdb::Vec3d(field.meshVertexNormals[second]));
    if (firstNormal.lengthSqr() <= kEpsilon ||
        secondNormal.lengthSqr() <= kEpsilon) {
        return base;
    }
    const double normalComponent = std::max(
        std::abs(direction.dot(firstNormal)),
        std::abs(direction.dot(secondNormal)));
    const double tangentFactor = std::clamp(
        1.0 - normalComponent * normalComponent,
        0.0,
        1.0);
    return std::clamp(base * tangentFactor, 0.0, 1.0);
}

} // namespace volume_surface
