#include "volume_surface/SurfaceNormalField.h"
#include "volume_surface/viewer/SurfaceNormalSeedStore.h"
#include "volume_surface/viewer/SurfaceTargetCacheRepository.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include <openvdb/io/File.h>

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

double dotNormalized(
    const openvdb::Vec3f& first,
    const openvdb::Vec3f& second)
{
    const openvdb::Vec3d lhs(first);
    const openvdb::Vec3d rhs(second);
    const double lhsLength = lhs.length();
    const double rhsLength = rhs.length();
    return lhsLength > 1.0e-12 && rhsLength > 1.0e-12
        ? lhs.dot(rhs) / (lhsLength * rhsLength)
        : 0.0;
}

const char* sampleKind(volume_surface::SurfaceTargetSampleKind kind)
{
    return kind == volume_surface::SurfaceTargetSampleKind::Core
        ? "Core"
        : "Transition";
}

} // namespace

int main(int argc, char** argv)
{
    try {
        const std::filesystem::path input = argc >= 2
            ? argv[1]
            : R"(G:\transformed\rightArm\rightArm.0224.vdb)";
        const std::string gridName = argc >= 3 ? argv[2] : "density";
        const double isoValue = argc >= 4 ? std::stod(argv[3]) : 255.0;
        const double minimumAlignment = argc >= 5 ? std::stod(argv[4]) : 0.15;

        openvdb::initialize();
        const auto grid = loadFloatGrid(input, gridName);
        volume_surface::SurfaceTargetSettings targetSettings;
        targetSettings.isoValue = isoValue;
        const auto metadata = volume_surface::viewer::SurfaceTargetCacheRepository::metadataForInput(
            input,
            gridName,
            grid.get(),
            targetSettings);
        const auto cachePath =
            volume_surface::viewer::SurfaceTargetCacheRepository::pathForInput(input, gridName);
        volume_surface::SurfaceTargetCache target;
        std::string error;
        bool loadedCache =
            volume_surface::loadSurfaceTargetCache(cachePath, metadata, target, &error);
        if (!loadedCache) {
            std::cerr << "cache_load=failed reason=" << error << "\n";
            target = volume_surface::extractSurfaceTarget(*grid, targetSettings);
        }

        volume_surface::SurfaceNormalFitSettings fitSettings;
        fitSettings.neighborhood = volume_surface::SurfaceFitNeighborhood::Grid9x9;
        fitSettings.isoValue = isoValue;
        const auto fit = volume_surface::fitSurfaceTargetNormals(
            *grid,
            target,
            fitSettings);
        const auto fitStats = volume_surface::analyzeSurfaceTargetNormalAdjacency(
            target,
            fit);

        const auto seedPath =
            volume_surface::viewer::SurfaceNormalSeedStore::pathForInput(input, gridName);
        volume_surface::viewer::SurfaceNormalSeed seed;
        const bool loadedSeed = volume_surface::viewer::SurfaceNormalSeedStore::load(
            seedPath,
            input,
            gridName,
            isoValue,
            seed,
            error);
        if (!loadedSeed) {
            throw std::runtime_error("orientation seed load failed: " + error);
        }

        std::size_t seedIndex = std::numeric_limits<std::size_t>::max();
        for (std::size_t index = 0; index < target.samples.size(); ++index) {
            if (target.samples[index].coordinate == seed.coordinate) {
                seedIndex = index;
                break;
            }
        }
        if (seedIndex == std::numeric_limits<std::size_t>::max()) {
            throw std::runtime_error("orientation seed coordinate is absent from the cache");
        }

        const auto oriented = volume_surface::orientSurfaceTargetNormals(
            target,
            fit,
            seedIndex,
            seed.normal,
            volume_surface::SurfaceNormalOrientationSettings{minimumAlignment});
        const auto orientedStats = volume_surface::analyzeSurfaceTargetNormalAdjacency(
            target,
            oriented,
            seedIndex);

        std::cout << std::setprecision(10)
                  << "source=" << std::filesystem::absolute(input).lexically_normal().generic_string() << '\n'
                  << "grid=" << gridName << " iso=" << isoValue << '\n'
                  << "orientation.minimum_alignment=" << minimumAlignment << '\n'
                  << "cache=" << (loadedCache ? "loaded" : "rebuilt")
                  << " samples=" << target.samples.size()
                  << " core=" << target.coreCount
                  << " transition=" << target.transitionCount << '\n'
                  << "fit9x9.valid_core=" << fitStats.validCoreSampleCount
                  << " fit9x9.edges=" << fitStats.adjacencyEdgeCount
                  << " fit9x9.opposing=" << fitStats.opposingEdgeCount
                  << " fit9x9.weak=" << fitStats.weakEdgeCount
                  << " fit9x9.components=" << fitStats.connectedComponentCount << '\n'
                  << "oriented.valid_core=" << orientedStats.validCoreSampleCount
                  << " oriented.valid_transition=" << orientedStats.validTransitionSampleCount
                  << " oriented.edges=" << orientedStats.adjacencyEdgeCount
                  << " oriented.opposing=" << orientedStats.opposingEdgeCount
                  << " oriented.transition_edges=" << orientedStats.transitionAdjacencyEdgeCount
                  << " oriented.transition_opposing=" << orientedStats.transitionOpposingEdgeCount
                  << " oriented.unseeded_opposing=" << orientedStats.opposingUnseededEdgeCount
                  << " oriented.transition_without_core_opposing="
                  << orientedStats.opposingTransitionWithoutCoreSupportEdgeCount
                  << " oriented.components=" << orientedStats.connectedComponentCount
                  << " oriented.seeded_core=" << orientedStats.seededComponentSampleCount
                  << " oriented.unseeded=" << orientedStats.unseededComponentSampleCount << '\n'
                  << "seed.sample_index=" << seedIndex
                  << " seed.kind=" << sampleKind(target.samples[seedIndex].kind) << '\n'
                  << "seed.coordinate=" << seed.coordinate.x() << ','
                  << seed.coordinate.y() << ',' << seed.coordinate.z() << '\n'
                  << "seed.normal=" << seed.normal.x() << ','
                  << seed.normal.y() << ',' << seed.normal.z() << '\n';

        const auto printIssue = [&](std::size_t rank,
                                    const volume_surface::SurfaceNormalAdjacencyStatistics::Issue& issue) {
            const double baseDot = issue.firstSampleIndex < fit.normals.size() &&
                    issue.secondSampleIndex < fit.normals.size()
                ? dotNormalized(
                    fit.normals[issue.firstSampleIndex],
                    fit.normals[issue.secondSampleIndex])
                : 0.0;
            const auto& first = target.samples[issue.firstSampleIndex];
            const auto& second = target.samples[issue.secondSampleIndex];
            const char* cause = issue.unseededComponent
                ? "unseeded_component"
                : issue.transitionWithoutCoreSupport
                    ? "transition_without_core_support"
                    : baseDot < 0.0
                        ? "fit_axis_conflict"
                        : "propagation_path_conflict";
            std::cout << "issue[" << rank << "]"
                      << " first=" << issue.firstCoordinate.x() << ','
                      << issue.firstCoordinate.y() << ',' << issue.firstCoordinate.z()
                      << " kind=" << sampleKind(first.kind)
                      << " second=" << issue.secondCoordinate.x() << ','
                      << issue.secondCoordinate.y() << ',' << issue.secondCoordinate.z()
                      << " kind=" << sampleKind(second.kind)
                      << " oriented_dot=" << issue.signedAlignment
                      << " fit_dot=" << baseDot
                      << " raw_dot=" << dotNormalized(first.normal, second.normal)
                      << " transition_linked=" << (issue.transitionLinked ? 1 : 0)
                      << " unseeded=" << (issue.unseededComponent ? 1 : 0)
                      << " transition_without_core_support="
                      << (issue.transitionWithoutCoreSupport ? 1 : 0)
                      << " cause=" << cause << '\n';
        };
        const auto printIssues = [&](const char* label,
                                     const auto& issues) {
            std::cout << label << "_count=" << issues.size() << '\n';
            for (std::size_t index = 0; index < issues.size(); ++index) {
                printIssue(index + 1, issues[index]);
            }
        };
        printIssues("worst_core_issue", orientedStats.worstCoreOpposingEdges);
        printIssues("worst_transition_issue", orientedStats.worstTransitionOpposingEdges);
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
