#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <openvdb/openvdb.h>

#include "volume_surface/SurfaceMesh.h"
#include "volume_surface/SurfaceTarget.h"

namespace volume_surface {

struct SurfaceMeshContinuitySettings
{
    // The search radius is measured in source-grid world units. A non-positive
    // value derives a bounded radius from the largest voxel dimension.
    double searchRadius = 0.0;
    // Samples farther than this normalized distance from the source mesh have
    // no topology support. The value is relative to searchRadius.
    double minimumProximity = 0.02;
};

struct SurfaceMeshContinuityField
{
    static constexpr std::uint32_t InvalidMeshVertex =
        static_cast<std::uint32_t>(-1);

    std::vector<std::uint32_t> nearestMeshVertex;
    std::vector<float> sampleProximity;
    std::vector<float> sampleSmoothness;
    std::vector<float> meshVertexSmoothness;
    std::vector<openvdb::Vec3f> meshVertexNormals;
    std::vector<std::uint32_t> adjacencyOffsets;
    std::vector<std::uint32_t> adjacencyVertices;
    std::size_t supportedSampleCount = 0;

    [[nodiscard]] bool matchesSampleCount(std::size_t count) const noexcept
    {
        return nearestMeshVertex.size() == count &&
            sampleProximity.size() == count &&
            sampleSmoothness.size() == count;
    }

    [[nodiscard]] bool empty() const noexcept
    {
        return nearestMeshVertex.empty();
    }
};

SurfaceMeshContinuityField buildSurfaceMeshContinuity(
    const SurfaceMesh& mesh,
    const SurfaceTargetCache& target,
    const openvdb::FloatGrid& grid,
    const SurfaceMeshContinuitySettings& settings = {});

double surfaceMeshContinuityWeight(
    const SurfaceMeshContinuityField& field,
    std::size_t firstSampleIndex,
    std::size_t secondSampleIndex) noexcept;

double surfaceMeshContinuityWeight(
    const SurfaceMeshContinuityField& field,
    const SurfaceTargetCache& target,
    std::size_t firstSampleIndex,
    std::size_t secondSampleIndex) noexcept;

} // namespace volume_surface
