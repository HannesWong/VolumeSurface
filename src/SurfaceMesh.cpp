#include "volume_surface/SurfaceMesh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <openvdb/tools/Interpolation.h>
#include <openvdb/tools/VolumeToMesh.h>

namespace volume_surface {
namespace {

void appendTriangle(
    SurfaceMesh& mesh,
    std::int32_t a,
    std::int32_t b,
    std::int32_t c)
{
    if (a < 0 || b < 0 || c < 0) {
        throw std::runtime_error("VolumeToMesh returned a negative vertex index");
    }

    const auto vertexCount = mesh.vertices.size();
    const auto ia = static_cast<std::uint32_t>(a);
    const auto ib = static_cast<std::uint32_t>(b);
    const auto ic = static_cast<std::uint32_t>(c);
    if (ia >= vertexCount || ib >= vertexCount || ic >= vertexCount) {
        throw std::runtime_error("VolumeToMesh returned an out-of-range vertex index");
    }

    mesh.indices.push_back(ia);
    mesh.indices.push_back(ib);
    mesh.indices.push_back(ic);
}

void calculateNormals(SurfaceMesh& mesh)
{
    for (auto& vertex : mesh.vertices) {
        vertex.normal = {0.0f, 0.0f, 0.0f};
    }

    for (std::size_t i = 0; i < mesh.indices.size(); i += 3) {
        auto& a = mesh.vertices[mesh.indices[i]];
        auto& b = mesh.vertices[mesh.indices[i + 1]];
        auto& c = mesh.vertices[mesh.indices[i + 2]];

        const double abx = static_cast<double>(b.position[0]) - a.position[0];
        const double aby = static_cast<double>(b.position[1]) - a.position[1];
        const double abz = static_cast<double>(b.position[2]) - a.position[2];
        const double acx = static_cast<double>(c.position[0]) - a.position[0];
        const double acy = static_cast<double>(c.position[1]) - a.position[1];
        const double acz = static_cast<double>(c.position[2]) - a.position[2];

        const std::array<float, 3> faceNormal{
            static_cast<float>(aby * acz - abz * acy),
            static_cast<float>(abz * acx - abx * acz),
            static_cast<float>(abx * acy - aby * acx)};

        for (auto* vertex : {&a, &b, &c}) {
            vertex->normal[0] += faceNormal[0];
            vertex->normal[1] += faceNormal[1];
            vertex->normal[2] += faceNormal[2];
        }
    }

    for (auto& vertex : mesh.vertices) {
        const double length = std::sqrt(
            static_cast<double>(vertex.normal[0]) * vertex.normal[0] +
            static_cast<double>(vertex.normal[1]) * vertex.normal[1] +
            static_cast<double>(vertex.normal[2]) * vertex.normal[2]);
        if (length > 1.0e-20) {
            vertex.normal[0] = static_cast<float>(vertex.normal[0] / length);
            vertex.normal[1] = static_cast<float>(vertex.normal[1] / length);
            vertex.normal[2] = static_cast<float>(vertex.normal[2] / length);
        } else {
            vertex.normal = {0.0f, 0.0f, 1.0f};
        }
    }
}

bool windingPointsIntoTheVolume(
    const openvdb::FloatGrid& grid,
    const SurfaceMesh& mesh)
{
    const auto gridClass = grid.getGridClass();
    if (gridClass != openvdb::GRID_FOG_VOLUME &&
        gridClass != openvdb::GRID_LEVEL_SET) {
        return false;
    }

    const openvdb::Vec3d voxelSize = grid.voxelSize();
    const double probeDistance = 0.75 * std::min({
        std::abs(voxelSize.x()),
        std::abs(voxelSize.y()),
        std::abs(voxelSize.z())});
    if (!std::isfinite(probeDistance) || probeDistance <= 0.0) {
        return false;
    }

    openvdb::tools::GridSampler<openvdb::FloatGrid, openvdb::tools::BoxSampler>
        sampler(grid);
    const std::size_t triangleCount = mesh.indices.size() / 3;
    const std::size_t sampleStride = std::max<std::size_t>(1, triangleCount / 4096);
    double signedEvidence = 0.0;
    double totalEvidence = 0.0;

    for (std::size_t triangle = 0; triangle < triangleCount; triangle += sampleStride) {
        const auto& a = mesh.vertices[mesh.indices[triangle * 3]].position;
        const auto& b = mesh.vertices[mesh.indices[triangle * 3 + 1]].position;
        const auto& c = mesh.vertices[mesh.indices[triangle * 3 + 2]].position;
        const openvdb::Vec3d ab{
            static_cast<double>(b[0]) - a[0],
            static_cast<double>(b[1]) - a[1],
            static_cast<double>(b[2]) - a[2]};
        const openvdb::Vec3d ac{
            static_cast<double>(c[0]) - a[0],
            static_cast<double>(c[1]) - a[1],
            static_cast<double>(c[2]) - a[2]};
        openvdb::Vec3d normal = ab.cross(ac);
        const double normalLength = normal.length();
        if (normalLength <= 1.0e-20) {
            continue;
        }
        normal /= normalLength;

        const openvdb::Vec3d center{
            (static_cast<double>(a[0]) + b[0] + c[0]) / 3.0,
            (static_cast<double>(a[1]) + b[1] + c[1]) / 3.0,
            (static_cast<double>(a[2]) + b[2] + c[2]) / 3.0};
        const double positiveValue = sampler.wsSample(center + normal * probeDistance);
        const double negativeValue = sampler.wsSample(center - normal * probeDistance);
        const double difference = positiveValue - negativeValue;
        if (!std::isfinite(difference)) {
            continue;
        }
        signedEvidence += difference;
        totalEvidence += std::abs(difference);
    }

    if (totalEvidence <= 1.0e-12) {
        return gridClass == openvdb::GRID_FOG_VOLUME;
    }

    const bool normalPointsTowardIncreasingValues = signedEvidence > 0.0;
    return gridClass == openvdb::GRID_FOG_VOLUME
        ? normalPointsTowardIncreasingValues
        : !normalPointsTowardIncreasingValues;
}

void orientSurfaceOutward(
    const openvdb::FloatGrid& grid,
    SurfaceMesh& mesh)
{
    if (!windingPointsIntoTheVolume(grid, mesh)) {
        return;
    }

    for (std::size_t i = 0; i < mesh.indices.size(); i += 3) {
        std::swap(mesh.indices[i + 1], mesh.indices[i + 2]);
    }
}

struct MeshEdgeRecord
{
    std::uint32_t first = 0;
    std::uint32_t second = 0;
    std::uint32_t triangle = 0;
};

void appendMeshEdge(
    std::vector<MeshEdgeRecord>& edges,
    std::uint32_t first,
    std::uint32_t second,
    std::uint32_t triangle)
{
    if (first == second) {
        return;
    }
    if (first > second) {
        std::swap(first, second);
    }
    edges.push_back({first, second, triangle});
}

float triangleArea(const SurfaceMesh& mesh, std::size_t triangleIndex)
{
    const auto ia = mesh.indices[triangleIndex * 3];
    const auto ib = mesh.indices[triangleIndex * 3 + 1];
    const auto ic = mesh.indices[triangleIndex * 3 + 2];
    const auto& a = mesh.vertices[ia].position;
    const auto& b = mesh.vertices[ib].position;
    const auto& c = mesh.vertices[ic].position;
    const double abx = static_cast<double>(b[0]) - a[0];
    const double aby = static_cast<double>(b[1]) - a[1];
    const double abz = static_cast<double>(b[2]) - a[2];
    const double acx = static_cast<double>(c[0]) - a[0];
    const double acy = static_cast<double>(c[1]) - a[1];
    const double acz = static_cast<double>(c[2]) - a[2];
    const double crossX = aby * acz - abz * acy;
    const double crossY = abz * acx - abx * acz;
    const double crossZ = abx * acy - aby * acx;
    return static_cast<float>(0.5 * std::sqrt(
        crossX * crossX + crossY * crossY + crossZ * crossZ));
}

SurfaceMesh compactTriangleSubset(
    const SurfaceMesh& source,
    const std::vector<std::size_t>& triangles)
{
    SurfaceMesh result;
    if (triangles.empty()) {
        return result;
    }

    std::vector<std::uint32_t> remap(source.vertices.size(),
                                     std::numeric_limits<std::uint32_t>::max());
    result.indices.reserve(triangles.size() * 3);
    for (const std::size_t triangleIndex : triangles) {
        for (std::size_t corner = 0; corner < 3; ++corner) {
            const std::uint32_t sourceIndex = source.indices[triangleIndex * 3 + corner];
            auto& mappedIndex = remap[sourceIndex];
            if (mappedIndex == std::numeric_limits<std::uint32_t>::max()) {
                mappedIndex = static_cast<std::uint32_t>(result.vertices.size());
                result.vertices.push_back(source.vertices[sourceIndex]);
            }
            result.indices.push_back(mappedIndex);
        }
    }

    const float infinity = std::numeric_limits<float>::infinity();
    result.bounds.minimum = {infinity, infinity, infinity};
    result.bounds.maximum = {-infinity, -infinity, -infinity};
    for (const auto& vertex : result.vertices) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
            result.bounds.minimum[axis] = std::min(
                result.bounds.minimum[axis], vertex.position[axis]);
            result.bounds.maximum[axis] = std::max(
                result.bounds.maximum[axis], vertex.position[axis]);
        }
    }
    calculateNormals(result);
    return result;
}

} // namespace

SurfaceMesh extractIsoSurface(
    const openvdb::FloatGrid& grid,
    double isoValue,
    double adaptivity)
{
    if (!std::isfinite(isoValue)) {
        throw std::invalid_argument("isoValue must be finite");
    }
    if (!std::isfinite(adaptivity) || adaptivity < 0.0 || adaptivity > 1.0) {
        throw std::invalid_argument("adaptivity must be in [0, 1]");
    }

    openvdb::tools::VolumeToMesh mesher(isoValue, adaptivity);
    mesher(grid);

    if (mesher.pointListSize() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::overflow_error("Surface has more vertices than 32-bit indices can address");
    }

    SurfaceMesh mesh;
    mesh.vertices.resize(mesher.pointListSize());
    if (mesh.vertices.empty()) {
        return mesh;
    }

    const float infinity = std::numeric_limits<float>::infinity();
    mesh.bounds.minimum = {infinity, infinity, infinity};
    mesh.bounds.maximum = {-infinity, -infinity, -infinity};

    const auto& points = mesher.pointList();
    for (std::size_t i = 0; i < mesher.pointListSize(); ++i) {
        auto& position = mesh.vertices[i].position;
        position = {points[i].x(), points[i].y(), points[i].z()};
        for (std::size_t axis = 0; axis < 3; ++axis) {
            mesh.bounds.minimum[axis] = std::min(mesh.bounds.minimum[axis], position[axis]);
            mesh.bounds.maximum[axis] = std::max(mesh.bounds.maximum[axis], position[axis]);
        }
    }

    const auto& pools = mesher.polygonPoolList();
    std::size_t triangleCount = 0;
    for (std::size_t i = 0; i < mesher.polygonPoolListSize(); ++i) {
        triangleCount += pools[i].numTriangles() + pools[i].numQuads() * 2;
    }
    mesh.indices.reserve(triangleCount * 3);

    for (std::size_t i = 0; i < mesher.polygonPoolListSize(); ++i) {
        const auto& pool = pools[i];
        for (std::size_t triangleIndex = 0; triangleIndex < pool.numTriangles(); ++triangleIndex) {
            const auto& triangle = pool.triangle(triangleIndex);
            appendTriangle(mesh, triangle[0], triangle[1], triangle[2]);
        }
        for (std::size_t quadIndex = 0; quadIndex < pool.numQuads(); ++quadIndex) {
            const auto& quad = pool.quad(quadIndex);
            appendTriangle(mesh, quad[0], quad[1], quad[2]);
            appendTriangle(mesh, quad[0], quad[2], quad[3]);
        }
    }

    orientSurfaceOutward(grid, mesh);
    calculateNormals(mesh);
    return mesh;
}

SurfaceMeshComponentSplit splitSurfaceMeshComponents(const SurfaceMesh& mesh)
{
    if (mesh.indices.size() % 3 != 0) {
        throw std::invalid_argument("Surface mesh indices must contain complete triangles");
    }
    for (const std::uint32_t index : mesh.indices) {
        if (index >= mesh.vertices.size()) {
            throw std::out_of_range("Surface mesh index is out of range");
        }
    }

    SurfaceMeshComponentSplit result;
    const std::size_t triangleCount = mesh.triangleCount();
    if (triangleCount == 0) {
        return result;
    }

    std::vector<MeshEdgeRecord> edges;
    edges.reserve(triangleCount * 3);
    for (std::size_t triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex) {
        const auto first = mesh.indices[triangleIndex * 3];
        const auto second = mesh.indices[triangleIndex * 3 + 1];
        const auto third = mesh.indices[triangleIndex * 3 + 2];
        appendMeshEdge(edges, first, second, static_cast<std::uint32_t>(triangleIndex));
        appendMeshEdge(edges, second, third, static_cast<std::uint32_t>(triangleIndex));
        appendMeshEdge(edges, third, first, static_cast<std::uint32_t>(triangleIndex));
    }
    std::sort(
        edges.begin(),
        edges.end(),
        [](const MeshEdgeRecord& lhs, const MeshEdgeRecord& rhs) {
            if (lhs.first != rhs.first) return lhs.first < rhs.first;
            if (lhs.second != rhs.second) return lhs.second < rhs.second;
            return lhs.triangle < rhs.triangle;
        });

    std::vector<std::vector<std::uint32_t>> adjacency(triangleCount);
    for (std::size_t begin = 0; begin < edges.size();) {
        std::size_t end = begin + 1;
        while (end < edges.size() &&
               edges[end].first == edges[begin].first &&
               edges[end].second == edges[begin].second) {
            ++end;
        }
        const std::uint32_t firstTriangle = edges[begin].triangle;
        for (std::size_t index = begin + 1; index < end; ++index) {
            const std::uint32_t otherTriangle = edges[index].triangle;
            adjacency[firstTriangle].push_back(otherTriangle);
            adjacency[otherTriangle].push_back(firstTriangle);
        }
        begin = end;
    }

    std::vector<std::uint8_t> visited(triangleCount, 0);
    std::vector<std::vector<std::size_t>> components;
    for (std::size_t seed = 0; seed < triangleCount; ++seed) {
        if (visited[seed]) {
            continue;
        }
        std::vector<std::size_t> component;
        std::vector<std::size_t> pending{seed};
        visited[seed] = 1;
        while (!pending.empty()) {
            const std::size_t current = pending.back();
            pending.pop_back();
            component.push_back(current);
            for (const std::uint32_t neighbor : adjacency[current]) {
                if (visited[neighbor]) {
                    continue;
                }
                visited[neighbor] = 1;
                pending.push_back(neighbor);
            }
        }
        components.push_back(std::move(component));
    }

    result.componentCount = components.size();
    std::size_t primaryComponent = 0;
    float primaryArea = -1.0f;
    for (std::size_t componentIndex = 0;
         componentIndex < components.size();
         ++componentIndex) {
        float area = 0.0f;
        for (const std::size_t triangleIndex : components[componentIndex]) {
            area += triangleArea(mesh, triangleIndex);
        }
        if (components[componentIndex].size() > components[primaryComponent].size() ||
            (components[componentIndex].size() == components[primaryComponent].size() &&
             area > primaryArea)) {
            primaryComponent = componentIndex;
            primaryArea = area;
        }
    }

    std::vector<std::size_t> excludedTriangles;
    for (std::size_t componentIndex = 0;
         componentIndex < components.size();
         ++componentIndex) {
        if (componentIndex == primaryComponent) {
            result.primaryTriangleCount = components[componentIndex].size();
            result.primary = compactTriangleSubset(mesh, components[componentIndex]);
        } else {
            result.excludedTriangleCount += components[componentIndex].size();
            excludedTriangles.insert(
                excludedTriangles.end(),
                components[componentIndex].begin(),
                components[componentIndex].end());
        }
    }
    result.excluded = compactTriangleSubset(mesh, excludedTriangles);
    return result;
}

} // namespace volume_surface
