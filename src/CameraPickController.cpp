#include "volume_surface/viewer/CameraPickController.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace volume_surface::viewer {
namespace {

constexpr std::size_t kSlotCount = 4;
constexpr std::size_t kLeafTriangleCount = 8;
constexpr double kIntersectionEpsilon = 1.0e-10;

struct Bounds
{
    std::array<float, 3> minimum{
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity()};
    std::array<float, 3> maximum{
        -std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity()};

    void include(const openvdb::Vec3d& point) noexcept
    {
        for (int axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], static_cast<float>(point[axis]));
            maximum[axis] = std::max(maximum[axis], static_cast<float>(point[axis]));
        }
    }

    [[nodiscard]] bool valid() const noexcept
    {
        return minimum[0] <= maximum[0] &&
            minimum[1] <= maximum[1] &&
            minimum[2] <= maximum[2];
    }
};

struct BvhNode
{
    Bounds bounds;
    std::uint32_t first = 0;
    std::uint32_t count = 0;
    std::uint32_t left = 0;
    std::uint32_t right = 0;
    bool leaf = false;
};

struct TriangleData
{
    std::uint32_t triangle = 0;
    std::array<float, 3> centroid{};
};

bool intersectBounds(
    const Bounds& bounds,
    const openvdb::Vec3d& origin,
    const openvdb::Vec3d& direction,
    double maximumDistance) noexcept
{
    if (!bounds.valid()) {
        return false;
    }

    double minimumDistance = 0.0;
    double maximum = maximumDistance;
    for (int axis = 0; axis < 3; ++axis) {
        const double originAxis = origin[axis];
        const double directionAxis = direction[axis];
        if (std::abs(directionAxis) <= kIntersectionEpsilon) {
            if (originAxis < bounds.minimum[axis] ||
                originAxis > bounds.maximum[axis]) {
                return false;
            }
            continue;
        }

        double nearDistance = (static_cast<double>(bounds.minimum[axis]) - originAxis) /
            directionAxis;
        double farDistance = (static_cast<double>(bounds.maximum[axis]) - originAxis) /
            directionAxis;
        if (nearDistance > farDistance) {
            std::swap(nearDistance, farDistance);
        }
        minimumDistance = std::max(minimumDistance, nearDistance);
        maximum = std::min(maximum, farDistance);
        if (minimumDistance > maximum) {
            return false;
        }
    }
    return maximum >= 0.0 && minimumDistance <= maximumDistance;
}

bool intersectTriangle(
    const std::vector<std::array<float, 3>>& vertices,
    const std::vector<std::uint32_t>& indices,
    std::uint32_t triangle,
    const openvdb::Vec3d& origin,
    const openvdb::Vec3d& direction,
    double maximumDistance,
    double& distance,
    std::array<double, 3>& barycentric) noexcept
{
    const std::size_t indexOffset = static_cast<std::size_t>(triangle) * 3;
    if (indexOffset + 2 >= indices.size()) {
        return false;
    }
    const std::uint32_t index0 = indices[indexOffset];
    const std::uint32_t index1 = indices[indexOffset + 1];
    const std::uint32_t index2 = indices[indexOffset + 2];
    if (index0 >= vertices.size() || index1 >= vertices.size() || index2 >= vertices.size()) {
        return false;
    }

    const openvdb::Vec3d vertex0{
        vertices[index0][0], vertices[index0][1], vertices[index0][2]};
    const openvdb::Vec3d vertex1{
        vertices[index1][0], vertices[index1][1], vertices[index1][2]};
    const openvdb::Vec3d vertex2{
        vertices[index2][0], vertices[index2][1], vertices[index2][2]};
    const openvdb::Vec3d edge1 = vertex1 - vertex0;
    const openvdb::Vec3d edge2 = vertex2 - vertex0;
    const openvdb::Vec3d pVector = direction.cross(edge2);
    const double determinant = edge1.dot(pVector);
    if (std::abs(determinant) <= kIntersectionEpsilon) {
        return false;
    }

    const double inverseDeterminant = 1.0 / determinant;
    const openvdb::Vec3d tVector = origin - vertex0;
    const double u = tVector.dot(pVector) * inverseDeterminant;
    if (u < -kIntersectionEpsilon || u > 1.0 + kIntersectionEpsilon) {
        return false;
    }

    const openvdb::Vec3d qVector = tVector.cross(edge1);
    const double v = direction.dot(qVector) * inverseDeterminant;
    if (v < -kIntersectionEpsilon || u + v > 1.0 + kIntersectionEpsilon) {
        return false;
    }

    const double candidateDistance = edge2.dot(qVector) * inverseDeterminant;
    if (!std::isfinite(candidateDistance) ||
        candidateDistance <= kIntersectionEpsilon ||
        candidateDistance >= maximumDistance) {
        return false;
    }
    distance = candidateDistance;
    barycentric = {1.0 - u - v, u, v};
    return true;
}

class MeshBvh final
{
public:
    void build(
        const SurfaceMesh& mesh,
        const SceneCoordinateMapper& mapper)
    {
        mVertices.clear();
        mIndices.clear();
        mTriangles.clear();
        mNodes.clear();
        mVertices.reserve(mesh.vertices.size());
        for (const auto& vertex : mesh.vertices) {
            const auto scenePosition = mapper.toScene(filament::math::float3{
                vertex.position[0], vertex.position[1], vertex.position[2]});
            mVertices.push_back({scenePosition.x, scenePosition.y, scenePosition.z});
        }
        mIndices = mesh.indices;
        const std::size_t triangleCount = mIndices.size() / 3;
        mTriangles.reserve(triangleCount);
        for (std::size_t triangle = 0; triangle < triangleCount; ++triangle) {
            const std::size_t indexOffset = triangle * 3;
            if (mIndices[indexOffset] >= mVertices.size() ||
                mIndices[indexOffset + 1] >= mVertices.size() ||
                mIndices[indexOffset + 2] >= mVertices.size()) {
                continue;
            }
            const auto& vertex0 = mVertices[mIndices[indexOffset]];
            const auto& vertex1 = mVertices[mIndices[indexOffset + 1]];
            const auto& vertex2 = mVertices[mIndices[indexOffset + 2]];
            mTriangles.push_back(TriangleData{
                static_cast<std::uint32_t>(triangle),
                {
                    static_cast<float>(
                        (static_cast<double>(vertex0[0]) + vertex1[0] + vertex2[0]) / 3.0),
                    static_cast<float>(
                        (static_cast<double>(vertex0[1]) + vertex1[1] + vertex2[1]) / 3.0),
                    static_cast<float>(
                        (static_cast<double>(vertex0[2]) + vertex1[2] + vertex2[2]) / 3.0)}});
        }
        mNodes.reserve(mTriangles.size() / kLeafTriangleCount * 2 + 1);
        if (!mTriangles.empty()) {
            buildNode(0, mTriangles.size());
        }
    }

    [[nodiscard]] bool empty() const noexcept
    {
        return mNodes.empty();
    }

    [[nodiscard]] bool intersect(
        const CameraPickRay& ray,
        double maximumDistance,
        double& distance,
        std::size_t& triangleIndex,
        std::array<double, 3>& barycentric) const noexcept
    {
        if (mNodes.empty()) {
            return false;
        }
        std::array<std::uint32_t, 64> stack{};
        std::size_t stackSize = 1;
        stack[0] = 0;
        bool hit = false;
        double nearestDistance = maximumDistance;
        while (stackSize > 0) {
            const std::uint32_t nodeIndex = stack[--stackSize];
            const auto& node = mNodes[nodeIndex];
            if (!intersectBounds(
                    node.bounds,
                    ray.origin,
                    ray.direction,
                    nearestDistance)) {
                continue;
            }
            if (node.leaf) {
                for (std::uint32_t offset = 0; offset < node.count; ++offset) {
                    double candidateDistance = nearestDistance;
                    std::array<double, 3> candidateBarycentric{};
                    if (intersectTriangle(
                            mVertices,
                            mIndices,
                            mTriangles[node.first + offset].triangle,
                            ray.origin,
                            ray.direction,
                            nearestDistance,
                            candidateDistance,
                            candidateBarycentric)) {
                        nearestDistance = candidateDistance;
                        triangleIndex = mTriangles[node.first + offset].triangle;
                        barycentric = candidateBarycentric;
                        hit = true;
                    }
                }
                continue;
            }
            if (stackSize + 2 > stack.size()) {
                return hit;
            }
            stack[stackSize++] = node.left;
            stack[stackSize++] = node.right;
        }
        if (hit) {
            distance = nearestDistance;
        }
        return hit;
    }

private:
    std::uint32_t buildNode(std::size_t first, std::size_t count)
    {
        const std::uint32_t nodeIndex = static_cast<std::uint32_t>(mNodes.size());
        mNodes.emplace_back();
        Bounds bounds;
        Bounds centroidBounds;
        for (std::size_t offset = first; offset < first + count; ++offset) {
            const auto& triangle = mTriangles[offset];
            const std::size_t indexOffset = static_cast<std::size_t>(triangle.triangle) * 3;
            for (std::size_t vertexOffset = 0; vertexOffset < 3; ++vertexOffset) {
                const auto& vertex = mVertices[mIndices[indexOffset + vertexOffset]];
                bounds.include(openvdb::Vec3d{vertex[0], vertex[1], vertex[2]});
            }
            centroidBounds.include(openvdb::Vec3d{
                triangle.centroid[0], triangle.centroid[1], triangle.centroid[2]});
        }

        auto& node = mNodes[nodeIndex];
        node.bounds = bounds;
        if (count <= kLeafTriangleCount) {
            node.first = static_cast<std::uint32_t>(first);
            node.count = static_cast<std::uint32_t>(count);
            node.leaf = true;
            return nodeIndex;
        }

        int splitAxis = 0;
        if (centroidBounds.maximum[1] - centroidBounds.minimum[1] >
            centroidBounds.maximum[0] - centroidBounds.minimum[0]) {
            splitAxis = 1;
        }
        if (centroidBounds.maximum[2] - centroidBounds.minimum[2] >
            centroidBounds.maximum[splitAxis] - centroidBounds.minimum[splitAxis]) {
            splitAxis = 2;
        }
        const std::size_t middle = first + count / 2;
        std::nth_element(
            mTriangles.begin() + static_cast<std::ptrdiff_t>(first),
            mTriangles.begin() + static_cast<std::ptrdiff_t>(middle),
            mTriangles.begin() + static_cast<std::ptrdiff_t>(first + count),
            [splitAxis](const TriangleData& lhs, const TriangleData& rhs) {
                return lhs.centroid[splitAxis] < rhs.centroid[splitAxis];
            });
        const std::uint32_t left = buildNode(first, middle - first);
        const std::uint32_t right = buildNode(middle, first + count - middle);
        mNodes[nodeIndex].left = left;
        mNodes[nodeIndex].right = right;
        return nodeIndex;
    }

    std::vector<std::array<float, 3>> mVertices;
    std::vector<std::uint32_t> mIndices;
    std::vector<TriangleData> mTriangles;
    std::vector<BvhNode> mNodes;
};

} // namespace

struct CameraPickController::Impl
{
    std::array<std::unique_ptr<MeshBvh>, kSlotCount> slots;
};

CameraPickController::CameraPickController()
    : mImpl(std::make_unique<Impl>())
{
}

CameraPickController::~CameraPickController() = default;

CameraPickController::CameraPickController(CameraPickController&&) noexcept = default;

CameraPickController& CameraPickController::operator=(CameraPickController&&) noexcept = default;

void CameraPickController::rebuildSlot(
    std::size_t slotIndex,
    const SurfaceMesh& mesh,
    const SceneCoordinateMapper& mapper)
{
    if (slotIndex >= kSlotCount) {
        return;
    }
    if (mesh.empty()) {
        clearSlot(slotIndex);
        return;
    }
    auto bvh = std::make_unique<MeshBvh>();
    bvh->build(mesh, mapper);
    mImpl->slots[slotIndex] = std::move(bvh);
}

void CameraPickController::clearSlot(std::size_t slotIndex) noexcept
{
    if (slotIndex < kSlotCount) {
        mImpl->slots[slotIndex].reset();
    }
}

CameraPickHit CameraPickController::pick(
    const CameraPickRay& ray,
    const std::array<bool, 4>& visibleSlots) const noexcept
{
    CameraPickHit result;
    const double directionLength = ray.direction.length();
    if (!std::isfinite(directionLength) || directionLength <= kIntersectionEpsilon) {
        return result;
    }

    CameraPickRay normalizedRay = ray;
    normalizedRay.direction /= directionLength;
    double nearestDistance = std::numeric_limits<double>::infinity();
    std::size_t nearestTriangle = 0;
    std::array<double, 3> nearestBarycentric{};
    for (std::size_t slotIndex = 0; slotIndex < kSlotCount; ++slotIndex) {
        if (!visibleSlots[slotIndex] || !mImpl->slots[slotIndex] ||
            mImpl->slots[slotIndex]->empty()) {
            continue;
        }
        double candidateDistance = nearestDistance;
        if (!mImpl->slots[slotIndex]->intersect(
                normalizedRay,
                nearestDistance,
                candidateDistance,
                nearestTriangle,
                nearestBarycentric)) {
            continue;
        }
        if (candidateDistance < nearestDistance) {
            nearestDistance = candidateDistance;
            result.hit = true;
            result.slotIndex = slotIndex;
            result.triangleIndex = nearestTriangle;
            result.barycentric = nearestBarycentric;
            result.rayDistance = candidateDistance;
            result.scenePosition = normalizedRay.origin +
                normalizedRay.direction * candidateDistance;
        }
    }
    return result;
}

} // namespace volume_surface::viewer
