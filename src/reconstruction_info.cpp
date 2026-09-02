#include "volume_surface/SurfaceReconstruction.h"

#include <cstdlib>
#include <algorithm>
#include <array>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <openvdb/io/File.h>
#include <openvdb/tools/Interpolation.h>

namespace {

openvdb::FloatGrid::Ptr loadFloatGrid(
    const std::filesystem::path& path,
    const std::string& gridName)
{
    openvdb::io::File file(path.string());
    file.open();
    auto baseGrid = file.readGrid(gridName);
    file.close();
    auto grid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);
    if (!grid) {
        throw std::runtime_error("Grid is missing or is not a FloatGrid: " + gridName);
    }
    return grid;
}

} // namespace

int main(int argc, char** argv)
{
    try {
        if (argc < 2 || argc > 9) {
            std::cerr
                << "Usage: vdb_surface_reconstruction_info <file.vdb> "
                   "[grid=density] [iso=255] [mlsRadiusMm=5] "
                   "[cellSizeMm=1] [maxCellsMillion=4] "
                   "[minimumFogDensityFraction=0.75] [candidatePadding=2]\n";
            return EXIT_FAILURE;
        }

        openvdb::initialize();
        const std::filesystem::path path = argv[1];
        const std::string gridName = argc >= 3 ? argv[2] : "density";
        const double isoValue = argc >= 4 ? std::stod(argv[3]) : 255.0;
        const double mlsRadiusMillimeters = argc >= 5 ? std::stod(argv[4]) : 5.0;
        const double cellSizeMillimeters = argc >= 6 ? std::stod(argv[5]) : 1.0;
        const std::size_t maximumCellsMillion = argc >= 7
            ? static_cast<std::size_t>(std::stoull(argv[6]))
            : 4;
        const double minimumFogDensityFraction = argc >= 8
            ? std::stod(argv[7])
            : 0.75;
        const std::size_t candidatePadding = argc >= 9
            ? static_cast<std::size_t>(std::stoull(argv[8]))
            : 2;

        const auto grid = loadFloatGrid(path, gridName);
        volume_surface::SurfaceTargetSettings targetSettings;
        targetSettings.isoValue = isoValue;
        const auto target = volume_surface::extractSurfaceTarget(*grid, targetSettings);
        const auto rawMesh = volume_surface::extractIsoSurface(*grid, isoValue, 0.1);

        volume_surface::SurfaceReconstructionSettings settings;
        settings.isoValue = isoValue;
        settings.mlsRadius = mlsRadiusMillimeters * 0.001;
        settings.targetCellSize = cellSizeMillimeters * 0.001;
        settings.maximumCellCount = maximumCellsMillion * 1'000'000;
        settings.minimumFogDensityFraction = minimumFogDensityFraction;
        settings.candidatePaddingCells = candidatePadding;
        const auto result = volume_surface::reconstructSurfaceMLS(*grid, target, settings);

        openvdb::tools::GridSampler<openvdb::FloatGrid, openvdb::tools::BoxSampler>
            sampler(*grid);
        double minimumDensity = std::numeric_limits<double>::infinity();
        double maximumDensity = -std::numeric_limits<double>::infinity();
        double densitySum = 0.0;
        std::size_t belowIsoCount = 0;
        std::array<std::size_t, 4> lowDensityCounts{};
        for (const auto& vertex : result.mesh.vertices) {
            const openvdb::Vec3d position(
                static_cast<double>(vertex.position[0]),
                static_cast<double>(vertex.position[1]),
                static_cast<double>(vertex.position[2]));
            const double density = sampler.wsSample(position);
            minimumDensity = std::min(minimumDensity, density);
            maximumDensity = std::max(maximumDensity, density);
            densitySum += density;
            if (density < isoValue) ++belowIsoCount;
            if (density < 64.0) ++lowDensityCounts[0];
            if (density < 128.0) ++lowDensityCounts[1];
            if (density < 192.0) ++lowDensityCounts[2];
            if (density < 224.0) ++lowDensityCounts[3];
        }
        const double vertexCount = static_cast<double>(result.mesh.vertices.size());
        double rawMinimumDensity = std::numeric_limits<double>::infinity();
        double rawMaximumDensity = -std::numeric_limits<double>::infinity();
        double rawDensitySum = 0.0;
        std::size_t rawBelowIsoCount = 0;
        std::array<std::size_t, 4> rawLowDensityCounts{};
        for (const auto& vertex : rawMesh.vertices) {
            const openvdb::Vec3d position(
                static_cast<double>(vertex.position[0]),
                static_cast<double>(vertex.position[1]),
                static_cast<double>(vertex.position[2]));
            const double density = sampler.wsSample(position);
            rawMinimumDensity = std::min(rawMinimumDensity, density);
            rawMaximumDensity = std::max(rawMaximumDensity, density);
            rawDensitySum += density;
            if (density < isoValue) ++rawBelowIsoCount;
            if (density < 64.0) ++rawLowDensityCounts[0];
            if (density < 128.0) ++rawLowDensityCounts[1];
            if (density < 192.0) ++rawLowDensityCounts[2];
            if (density < 224.0) ++rawLowDensityCounts[3];
        }
        std::unordered_map<std::uint64_t, std::uint8_t> rawEdgeCounts;
        rawEdgeCounts.reserve(rawMesh.indices.size());
        for (std::size_t index = 0; index + 2 < rawMesh.indices.size(); index += 3) {
            const std::array<std::uint32_t, 3> triangle{
                rawMesh.indices[index],
                rawMesh.indices[index + 1],
                rawMesh.indices[index + 2]};
            for (std::size_t edge = 0; edge < 3; ++edge) {
                const auto first = triangle[edge];
                const auto second = triangle[(edge + 1) % 3];
                const auto minimum = std::min(first, second);
                const auto maximum = std::max(first, second);
                const auto key = (static_cast<std::uint64_t>(minimum) << 32) |
                    static_cast<std::uint64_t>(maximum);
                auto& count = rawEdgeCounts[key];
                if (count < 255) ++count;
            }
        }
        std::size_t rawBoundaryEdgeCount = 0;
        std::size_t rawNonManifoldEdgeCount = 0;
        for (const auto& [edge, count] : rawEdgeCounts) {
            static_cast<void>(edge);
            if (count == 1) ++rawBoundaryEdgeCount;
            if (count > 2) ++rawNonManifoldEdgeCount;
        }
        const double rawVertexCount = static_cast<double>(rawMesh.vertices.size());
        std::unordered_map<std::uint64_t, std::uint8_t> edgeCounts;
        edgeCounts.reserve(result.mesh.indices.size());
        for (std::size_t index = 0; index + 2 < result.mesh.indices.size(); index += 3) {
            const std::array<std::uint32_t, 3> triangle{
                result.mesh.indices[index],
                result.mesh.indices[index + 1],
                result.mesh.indices[index + 2]};
            for (std::size_t edge = 0; edge < 3; ++edge) {
                const auto first = triangle[edge];
                const auto second = triangle[(edge + 1) % 3];
                const auto minimum = std::min(first, second);
                const auto maximum = std::max(first, second);
                const auto key = (static_cast<std::uint64_t>(minimum) << 32) |
                    static_cast<std::uint64_t>(maximum);
                auto& count = edgeCounts[key];
                if (count < 255) ++count;
            }
        }
        std::size_t boundaryEdgeCount = 0;
        std::size_t nonManifoldEdgeCount = 0;
        for (const auto& [edge, count] : edgeCounts) {
            static_cast<void>(edge);
            if (count == 1) ++boundaryEdgeCount;
            if (count > 2) ++nonManifoldEdgeCount;
        }

        std::cout << std::fixed << std::setprecision(3)
                  << "grid=" << gridName << '\n'
                  << "iso=" << isoValue << '\n'
                  << "target.samples=" << target.samples.size() << '\n'
                  << "target.core=" << target.coreCount << '\n'
                  << "target.transition=" << target.transitionCount << '\n'
                  << "reconstruction.candidate_cells=" << result.candidateCellCount << '\n'
                  << "reconstruction.crossing_cells=" << result.crossingCellCount << '\n'
                  << "reconstruction.field_samples=" << result.fieldSampleCount << '\n'
                  << "reconstruction.invalid_field_fallback="
                  << result.invalidFieldFallbackCount << '\n'
                  << "reconstruction.source_support_fallback="
                  << result.sourceSupportFallbackCount << '\n'
                  << "reconstruction.used_source_topology_fallback="
                  << (result.usedSourceTopologyFallback ? 1 : 0) << '\n'
                  << "reconstruction.source_topology_projection_fallback="
                  << result.sourceTopologyProjectionFallbackCount << '\n'
                  << "reconstruction.source_crossing_cells="
                  << result.sourceCrossingCellCount << '\n'
                  << "reconstruction.missing_neighbor_faces="
                  << result.missingNeighborFaceCount << '\n'
                  << "reconstruction.emitted_faces="
                  << result.emittedFaceCount << '\n'
                  << "reconstruction.vertices=" << result.mesh.vertices.size() << '\n'
                  << "reconstruction.triangles=" << result.mesh.triangleCount() << '\n'
                  << "reconstruction.bounds.min=["
                  << result.mesh.bounds.minimum[0] << ", "
                  << result.mesh.bounds.minimum[1] << ", "
                  << result.mesh.bounds.minimum[2] << "]\n"
                  << "reconstruction.bounds.max=["
                  << result.mesh.bounds.maximum[0] << ", "
                  << result.mesh.bounds.maximum[1] << ", "
                  << result.mesh.bounds.maximum[2] << "]\n"
                  << "reconstruction.vertex_density.min=" << minimumDensity << '\n'
                  << "reconstruction.vertex_density.max=" << maximumDensity << '\n'
                  << "reconstruction.vertex_density.mean="
                  << (vertexCount > 0.0 ? densitySum / vertexCount : 0.0) << '\n'
                  << "reconstruction.vertex_density.below_iso=" << belowIsoCount << '\n'
                  << "reconstruction.vertex_density.below=["
                  << lowDensityCounts[0] << ", "
                  << lowDensityCounts[1] << ", "
                  << lowDensityCounts[2] << ", "
                  << lowDensityCounts[3] << "]\n"
                  << "reconstruction.boundary_edges=" << boundaryEdgeCount << '\n'
                  << "reconstruction.nonmanifold_edges=" << nonManifoldEdgeCount << '\n'
                  << "raw.vertices=" << rawMesh.vertices.size() << '\n'
                  << "raw.vertex_density.min=" << rawMinimumDensity << '\n'
                  << "raw.vertex_density.max=" << rawMaximumDensity << '\n'
                  << "raw.vertex_density.mean="
                  << (rawVertexCount > 0.0 ? rawDensitySum / rawVertexCount : 0.0) << '\n'
                  << "raw.vertex_density.below_iso=" << rawBelowIsoCount << '\n'
                  << "raw.vertex_density.below=["
                  << rawLowDensityCounts[0] << ", "
                  << rawLowDensityCounts[1] << ", "
                  << rawLowDensityCounts[2] << ", "
                  << rawLowDensityCounts[3] << "]\n"
                  << "raw.boundary_edges=" << rawBoundaryEdgeCount << '\n'
                  << "raw.nonmanifold_edges=" << rawNonManifoldEdgeCount << '\n'
                  << "timing.anchor_ms=" << result.timings.anchorLocalizationMilliseconds << '\n'
                  << "timing.index_ms=" << result.timings.spatialIndexMilliseconds << '\n'
                  << "timing.field_ms=" << result.timings.fieldSamplingMilliseconds << '\n'
                  << "timing.mesh_ms=" << result.timings.meshExtractionMilliseconds << '\n'
                  << "timing.total_ms=" << result.timings.totalMilliseconds << '\n';
        return result.mesh.empty() ? EXIT_FAILURE : EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
