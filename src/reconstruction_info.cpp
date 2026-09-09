#include "volume_surface/SurfaceReconstruction.h"
#include "volume_surface/SurfaceNormalField.h"

#include <cstdlib>
#include <algorithm>
#include <array>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

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

double pointTriangleDistanceSquared(
    const openvdb::Vec3d& point,
    const openvdb::Vec3d& first,
    const openvdb::Vec3d& second,
    const openvdb::Vec3d& third)
{
    const openvdb::Vec3d ab = second - first;
    const openvdb::Vec3d ac = third - first;
    const openvdb::Vec3d ap = point - first;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0) return ap.lengthSqr();

    const openvdb::Vec3d bp = point - second;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3) return bp.lengthSqr();

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double t = d1 / (d1 - d3);
        return (point - (first + ab * t)).lengthSqr();
    }

    const openvdb::Vec3d cp = point - third;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6) return cp.lengthSqr();

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double t = d2 / (d2 - d6);
        return (point - (first + ac * t)).lengthSqr();
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const openvdb::Vec3d bc = third - second;
        const double t = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (point - (second + bc * t)).lengthSqr();
    }

    const double denominator = 1.0 / (va + vb + vc);
    const double v = vb * denominator;
    const double w = vc * denominator;
    return (point - (first + ab * v + ac * w)).lengthSqr();
}

} // namespace

int main(int argc, char** argv)
{
    try {
        if (argc < 2 || argc > 12) {
            std::cerr
                << "Usage: vdb_surface_reconstruction_info <file.vdb> "
                   "[grid=density] [iso=255] [mlsRadiusMm=5] "
                   "[cellSizeMm=1] [maxCellsMillion=4] "
                   "[minimumFogDensityFraction=0.75] [candidatePadding=2] "
                   "[raw|3x3|5x5|9x9] [triangleIndex] [triangleIndex2]\n";
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
        std::string normalMode = "raw";
        std::vector<std::size_t> requestedTriangles;
        bool hasPointQuery = false;
        openvdb::Vec3d pointQueryMillimeters(0.0);
        for (int argument = 9; argument < argc; ++argument) {
            const std::string value = argv[argument];
            if (value == "raw" || value == "3x3" || value == "5x5" || value == "9x9") {
                normalMode = value;
            } else if (value.rfind("point=", 0) == 0) {
                std::string coordinates = value.substr(6);
                std::replace(coordinates.begin(), coordinates.end(), ',', ' ');
                std::istringstream stream(coordinates);
                if (!(stream >> pointQueryMillimeters.x() >>
                    pointQueryMillimeters.y() >> pointQueryMillimeters.z())) {
                    throw std::invalid_argument("point query must be point=x,y,z in millimeters");
                }
                hasPointQuery = true;
            } else {
                requestedTriangles.push_back(
                    static_cast<std::size_t>(std::stoull(value)));
            }
        }

        const auto grid = loadFloatGrid(path, gridName);
        volume_surface::SurfaceTargetSettings targetSettings;
        targetSettings.isoValue = isoValue;
        auto target = volume_surface::extractSurfaceTarget(*grid, targetSettings);
        auto targetComponentFilter =
            volume_surface::retainLargestSurfaceTargetComponent(target);
        target = std::move(targetComponentFilter.primary);
        const auto rawMesh = volume_surface::extractIsoSurface(*grid, isoValue, 0.1);

        volume_surface::SurfaceReconstructionSettings settings;
        settings.isoValue = isoValue;
        settings.mlsRadius = mlsRadiusMillimeters * 0.001;
        settings.targetCellSize = cellSizeMillimeters * 0.001;
        settings.maximumCellCount = maximumCellsMillion * 1'000'000;
        settings.minimumFogDensityFraction = minimumFogDensityFraction;
        settings.candidatePaddingCells = candidatePadding;
        volume_surface::SurfaceNormalField normalField;
        const std::vector<openvdb::Vec3f>* normalOverrides = nullptr;
        if (normalMode != "raw") {
            volume_surface::SurfaceNormalSmoothingSettings normalSettings;
            volume_surface::SurfaceNormalFitSettings fitSettings;
            fitSettings.isoValue = isoValue;
            fitSettings.neighborhood = normalMode == "9x9"
                ? volume_surface::SurfaceFitNeighborhood::Grid9x9
                : normalMode == "5x5"
                    ? volume_surface::SurfaceFitNeighborhood::Grid5x5
                    : volume_surface::SurfaceFitNeighborhood::Grid3x3;
            fitSettings.robustIterations = normalSettings.robustIterations;
            normalField = volume_surface::fitSurfaceTargetNormals(
                *grid,
                target,
                fitSettings);
            normalSettings.neighborhood = volume_surface::SurfaceNormalNeighborhood::None;
            normalField = volume_surface::smoothSurfaceTargetNormals(
                target,
                normalField,
                normalSettings);
            normalOverrides = &normalField.normals;
        }
        if (hasPointQuery) {
            const openvdb::Vec3d query = pointQueryMillimeters * 0.001;
            double nearestTargetDistanceSquared = std::numeric_limits<double>::infinity();
            std::size_t nearestTargetIndex = std::numeric_limits<std::size_t>::max();
            for (std::size_t index = 0; index < target.samples.size(); ++index) {
                const openvdb::Vec3d samplePosition(target.samples[index].worldPosition);
                const double distanceSquared = (query - samplePosition).lengthSqr();
                if (distanceSquared < nearestTargetDistanceSquared) {
                    nearestTargetDistanceSquared = distanceSquared;
                    nearestTargetIndex = index;
                }
            }
            if (nearestTargetIndex != std::numeric_limits<std::size_t>::max()) {
                const auto& sample = target.samples[nearestTargetIndex];
                std::cout << "point_target_index=" << nearestTargetIndex
                          << " distance_mm="
                          << std::sqrt(nearestTargetDistanceSquared) * 1000.0
                          << " coordinate=[" << sample.coordinate.x() << ','
                          << sample.coordinate.y() << ',' << sample.coordinate.z() << ']'
                          << " raw_normal=[" << sample.normal.x() << ','
                          << sample.normal.y() << ',' << sample.normal.z() << ']';
                if (normalField.normals.size() == target.samples.size()) {
                    const auto& normal = normalField.normals[nearestTargetIndex];
                    std::cout << " selected_normal=[" << normal.x() << ','
                              << normal.y() << ',' << normal.z() << ']';
                }
                std::cout << '\n';
            }
        }
        const auto result = volume_surface::reconstructSurfaceMLS(
            *grid,
            target,
            settings,
            normalOverrides);
        const auto sourceTopologyMesh = volume_surface::extractIsoSurface(
            *grid,
            isoValue,
            settings.sourceTopologyAdaptivity);

        if (hasPointQuery) {
            const openvdb::Vec3d query = pointQueryMillimeters * 0.001;
            double nearestDistanceSquared = std::numeric_limits<double>::infinity();
            std::size_t nearestTriangle = std::numeric_limits<std::size_t>::max();
            for (std::size_t triangle = 0; triangle < result.mesh.triangleCount(); ++triangle) {
                const std::size_t firstIndex = triangle * 3;
                const auto first = result.mesh.indices[firstIndex];
                const auto second = result.mesh.indices[firstIndex + 1];
                const auto third = result.mesh.indices[firstIndex + 2];
                const openvdb::Vec3d firstPosition(
                    result.mesh.vertices[first].position[0],
                    result.mesh.vertices[first].position[1],
                    result.mesh.vertices[first].position[2]);
                const openvdb::Vec3d secondPosition(
                    result.mesh.vertices[second].position[0],
                    result.mesh.vertices[second].position[1],
                    result.mesh.vertices[second].position[2]);
                const openvdb::Vec3d thirdPosition(
                    result.mesh.vertices[third].position[0],
                    result.mesh.vertices[third].position[1],
                    result.mesh.vertices[third].position[2]);
                const double distanceSquared = pointTriangleDistanceSquared(
                    query,
                    firstPosition,
                    secondPosition,
                    thirdPosition);
                if (distanceSquared < nearestDistanceSquared) {
                    nearestDistanceSquared = distanceSquared;
                    nearestTriangle = triangle;
                }
            }
            if (nearestTriangle != std::numeric_limits<std::size_t>::max()) {
                requestedTriangles.push_back(nearestTriangle);
                std::cout << "point_query_mm=["
                          << pointQueryMillimeters.x() << ','
                          << pointQueryMillimeters.y() << ','
                          << pointQueryMillimeters.z() << "] nearest_triangle="
                          << nearestTriangle << " distance_mm="
                          << std::sqrt(nearestDistanceSquared) * 1000.0 << '\n';
            }
        }

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
                  << "normal_mode=" << normalMode << '\n'
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
                  << "reconstruction.surface_components="
                  << result.surfaceComponentCount << '\n'
                  << "reconstruction.excluded_triangles="
                  << result.excludedTriangleCount << '\n'
                  << "reconstruction.topology=source\n"
                  << "reconstruction.projection_vertices="
                  << result.projectionVertexCount << '\n'
                  << "reconstruction.projection_rejected="
                  << result.projectionRejectedCount << '\n'
                  << "reconstruction.projection_density_rejected="
                  << result.projectionDensityRejectedCount << '\n'
                  << "reconstruction.projection_max_displacement_mm="
                  << result.projectionMaximumDisplacement * 1000.0 << '\n'
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
        for (const std::size_t triangleIndex : requestedTriangles) {
            const std::size_t firstIndex = triangleIndex * 3;
            if (firstIndex + 2 >= result.mesh.indices.size()) {
                std::cout << "triangle[" << triangleIndex << "]=out_of_range\n";
                continue;
            }
            const auto first = result.mesh.indices[firstIndex];
            const auto second = result.mesh.indices[firstIndex + 1];
            const auto third = result.mesh.indices[firstIndex + 2];
            const auto& firstVertex = result.mesh.vertices[first];
            const auto& secondVertex = result.mesh.vertices[second];
            const auto& thirdVertex = result.mesh.vertices[third];
            const auto printVertex = [&](std::uint32_t vertexIndex) {
                const auto& vertex = result.mesh.vertices[vertexIndex];
                const openvdb::Vec3d position(
                    vertex.position[0], vertex.position[1], vertex.position[2]);
                std::cout << " vertex[" << vertexIndex << "]_mm=["
                          << position.x() * 1000.0 << ','
                          << position.y() * 1000.0 << ','
                          << position.z() * 1000.0 << ']';
                if (vertexIndex < sourceTopologyMesh.vertices.size()) {
                    const auto& sourceVertex = sourceTopologyMesh.vertices[vertexIndex];
                    const openvdb::Vec3d sourcePosition(
                        sourceVertex.position[0],
                        sourceVertex.position[1],
                        sourceVertex.position[2]);
                    std::cout << " source_topology_mm=["
                              << sourcePosition.x() * 1000.0 << ','
                              << sourcePosition.y() * 1000.0 << ','
                              << sourcePosition.z() * 1000.0 << ']'
                              << " projection_move_mm="
                              << (position - sourcePosition).length() * 1000.0;
                }
                std::cout << " density=" << sampler.wsSample(position);
            };
            const openvdb::Vec3d a(firstVertex.position[0], firstVertex.position[1], firstVertex.position[2]);
            const openvdb::Vec3d b(secondVertex.position[0], secondVertex.position[1], secondVertex.position[2]);
            const openvdb::Vec3d c(thirdVertex.position[0], thirdVertex.position[1], thirdVertex.position[2]);
            const openvdb::Vec3d geometricNormal = (b - a).cross(c - a).unit();
            const auto edgeKey = [](std::uint32_t firstVertex, std::uint32_t secondVertex) {
                const auto minimum = std::min(firstVertex, secondVertex);
                const auto maximum = std::max(firstVertex, secondVertex);
                return (static_cast<std::uint64_t>(minimum) << 32) |
                    static_cast<std::uint64_t>(maximum);
            };
            const auto edgeCount = [&](std::uint32_t firstVertex, std::uint32_t secondVertex) {
                const auto found = edgeCounts.find(edgeKey(firstVertex, secondVertex));
                return found == edgeCounts.end() ? 0 : static_cast<unsigned int>(found->second);
            };
            std::cout << "triangle[" << triangleIndex << "]="
                      << " vertices=" << first << ',' << second << ',' << third
                      << " positions_mm=["
                      << a.x() * 1000.0 << ',' << a.y() * 1000.0 << ',' << a.z() * 1000.0 << ";"
                      << b.x() * 1000.0 << ',' << b.y() * 1000.0 << ',' << b.z() * 1000.0 << ";"
                      << c.x() * 1000.0 << ',' << c.y() * 1000.0 << ',' << c.z() * 1000.0 << "]"
                      << " geometric_normal=["
                      << geometricNormal.x() << ',' << geometricNormal.y() << ',' << geometricNormal.z() << "]"
                      << " edge_counts=["
                      << edgeCount(first, second) << ','
                      << edgeCount(second, third) << ','
                      << edgeCount(third, first) << "]"
                      << " source_density=["
                      << sampler.wsSample(a) << ',' << sampler.wsSample(b) << ',' << sampler.wsSample(c) << "]";
            printVertex(first);
            printVertex(second);
            printVertex(third);
            std::cout << '\n';
        }
        return result.mesh.empty() ? EXIT_FAILURE : EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
