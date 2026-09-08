#include "volume_surface/SurfaceNormalField.h"
#include "volume_surface/SurfaceMesh.h"
#include "volume_surface/SurfaceMeshContinuity.h"
#include "volume_surface/viewer/SurfaceNormalSeedStore.h"
#include "volume_surface/viewer/SurfaceTargetCacheRepository.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

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

openvdb::Vec3d normalizeOrZero(openvdb::Vec3d value)
{
    const double length = value.length();
    return std::isfinite(length) && length > 1.0e-20
        ? value / length
        : openvdb::Vec3d(0.0);
}

bool surfaceContinuationAllowed(
    const volume_surface::SurfaceTargetCache& target,
    std::size_t firstSampleIndex,
    std::size_t secondSampleIndex,
    const openvdb::Vec3d& firstNormal,
    const openvdb::Vec3d& secondNormal,
    double maximumSurfaceNormalComponent)
{
    if (firstSampleIndex >= target.samples.size() ||
        secondSampleIndex >= target.samples.size()) {
        return false;
    }
    const openvdb::Vec3d displacement = openvdb::Vec3d(
        target.samples[secondSampleIndex].worldPosition) -
        openvdb::Vec3d(target.samples[firstSampleIndex].worldPosition);
    const double length = displacement.length();
    if (!std::isfinite(length) || length <= 1.0e-20) {
        return false;
    }
    const openvdb::Vec3d direction = displacement / length;
    const openvdb::Vec3d firstAxis = normalizeOrZero(firstNormal);
    const openvdb::Vec3d secondAxis = normalizeOrZero(secondNormal);
    return firstAxis.lengthSqr() > 1.0e-20 &&
        secondAxis.lengthSqr() > 1.0e-20 &&
        std::abs(direction.dot(firstAxis)) <= maximumSurfaceNormalComponent &&
        std::abs(direction.dot(secondAxis)) <= maximumSurfaceNormalComponent;
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
        const bool useMeshContinuity = argc < 6 || std::stoi(argv[5]) != 0;
        const bool hasProbe = argc >= 9;
        const openvdb::Coord probeCoordinate = hasProbe
            ? openvdb::Coord(
                std::stoi(argv[6]),
                std::stoi(argv[7]),
                std::stoi(argv[8]))
            : openvdb::Coord();

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

        const auto sourceMesh = volume_surface::extractIsoSurface(
            *grid,
            isoValue,
            0.1);
        const auto meshContinuity = volume_surface::buildSurfaceMeshContinuity(
            sourceMesh,
            target,
            *grid);

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

        volume_surface::SurfaceNormalOrientationSettings orientationSettings;
        orientationSettings.minimumAlignment = minimumAlignment;
        volume_surface::SurfaceNormalExpansionTrace expansionTrace;
        const auto oriented = volume_surface::orientSurfaceTargetNormals(
            target,
            fit,
            seedIndex,
            seed.normal,
            orientationSettings,
            &expansionTrace,
            useMeshContinuity ? &meshContinuity : nullptr);
        const auto orientedStats = volume_surface::analyzeSurfaceTargetNormalAdjacency(
            target,
            oriented,
            seedIndex);

        std::size_t supportedCoreCount = 0;
        std::size_t supportedTransitionCount = 0;
        for (std::size_t index = 0; index < target.samples.size(); ++index) {
            if (index >= meshContinuity.nearestMeshVertex.size() ||
                meshContinuity.nearestMeshVertex[index] ==
                    volume_surface::SurfaceMeshContinuityField::InvalidMeshVertex) {
                continue;
            }
            if (target.samples[index].kind == volume_surface::SurfaceTargetSampleKind::Core) {
                ++supportedCoreCount;
            } else {
                ++supportedTransitionCount;
            }
        }

        std::cout << std::setprecision(10)
                  << "source=" << std::filesystem::absolute(input).lexically_normal().generic_string() << '\n'
                  << "grid=" << gridName << " iso=" << isoValue << '\n'
                  << "orientation.minimum_alignment=" << minimumAlignment << '\n'
                  << "mesh_continuity.enabled=" << (useMeshContinuity ? 1 : 0) << '\n'
                  << "mesh_continuity.supported_samples="
                  << meshContinuity.supportedSampleCount
                  << " supported_core=" << supportedCoreCount
                  << " supported_transition=" << supportedTransitionCount
                  << " mesh_vertices=" << sourceMesh.vertices.size()
                  << " mesh_triangles=" << sourceMesh.triangleCount() << '\n'
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

        if (hasProbe) {
            std::size_t probeIndex = std::numeric_limits<std::size_t>::max();
            for (std::size_t index = 0; index < target.samples.size(); ++index) {
                if (target.samples[index].coordinate == probeCoordinate) {
                    probeIndex = index;
                    break;
                }
            }
            if (probeIndex == std::numeric_limits<std::size_t>::max()) {
                std::cout << "probe.coordinate=" << probeCoordinate.x() << ','
                          << probeCoordinate.y() << ',' << probeCoordinate.z()
                          << " found=0\n";
            } else {
                const auto& probe = target.samples[probeIndex];
                const openvdb::Vec3d fitNormal = normalizeOrZero(
                    openvdb::Vec3d(fit.normals[probeIndex]));
                const openvdb::Vec3d orientedNormal = normalizeOrZero(
                    openvdb::Vec3d(oriented.normals[probeIndex]));
                const std::int32_t depth = expansionTrace.depthBySample[probeIndex];
                const std::int32_t parent = expansionTrace.parentBySample[probeIndex];
                std::cout << "probe.coordinate=" << probeCoordinate.x() << ','
                          << probeCoordinate.y() << ',' << probeCoordinate.z()
                          << " found=1 sample_index=" << probeIndex
                          << " kind=" << sampleKind(probe.kind)
                          << " depth=" << depth
                          << " oriented="
                          << static_cast<int>(expansionTrace.orientedBySample[probeIndex])
                          << " fit_normal=" << fitNormal.x() << ','
                          << fitNormal.y() << ',' << fitNormal.z()
                          << " oriented_normal=" << orientedNormal.x() << ','
                          << orientedNormal.y() << ',' << orientedNormal.z()
                          << " fit_oriented_dot=" << fitNormal.dot(orientedNormal)
                          << '\n';
                if (parent >= 0 && static_cast<std::size_t>(parent) < target.samples.size()) {
                    const auto& parentSample = target.samples[static_cast<std::size_t>(parent)];
                    const openvdb::Vec3d parentNormal = normalizeOrZero(
                        openvdb::Vec3d(oriented.normals[static_cast<std::size_t>(parent)]));
                    const double parentWeight = useMeshContinuity
                        ? volume_surface::surfaceMeshContinuityWeight(
                            meshContinuity,
                            target,
                            static_cast<std::size_t>(parent),
                            probeIndex)
                        : 1.0;
                    std::cout << "probe.parent=" << parentSample.coordinate.x() << ','
                              << parentSample.coordinate.y() << ','
                              << parentSample.coordinate.z()
                              << " sample_index=" << parent
                              << " depth=" << expansionTrace.depthBySample[static_cast<std::size_t>(parent)]
                              << " oriented_normal=" << parentNormal.x() << ','
                              << parentNormal.y() << ',' << parentNormal.z()
                              << " axis_dot=" << fitNormal.dot(parentNormal)
                              << " mesh_weight=" << parentWeight << '\n';
                } else {
                    std::cout << "probe.parent=none\n";
                }

                // Print a bounded ancestor chain so a local orientation flip can be
                // distinguished from a flip inherited from an earlier propagation step.
                std::size_t chainIndex = probeIndex;
                for (std::size_t chainDepth = 0; chainDepth < 64; ++chainDepth) {
                    if (chainIndex >= target.samples.size()) {
                        break;
                    }
                    const auto& chainSample = target.samples[chainIndex];
                    const openvdb::Vec3d chainFit = normalizeOrZero(
                        openvdb::Vec3d(fit.normals[chainIndex]));
                    const openvdb::Vec3d chainOriented = normalizeOrZero(
                        openvdb::Vec3d(oriented.normals[chainIndex]));
                    const std::int32_t chainParent =
                        expansionTrace.parentBySample[chainIndex];
                    std::cout << "probe.chain=" << chainDepth
                              << " coordinate=" << chainSample.coordinate.x() << ','
                              << chainSample.coordinate.y() << ','
                              << chainSample.coordinate.z()
                              << " sample_index=" << chainIndex
                              << " depth=" << expansionTrace.depthBySample[chainIndex]
                              << " fit_oriented_dot=" << chainFit.dot(chainOriented)
                              << " parent_index=" << chainParent << '\n';
                    if (chainParent < 0 ||
                        static_cast<std::size_t>(chainParent) >= target.samples.size()) {
                        break;
                    }
                    chainIndex = static_cast<std::size_t>(chainParent);
                }

                std::unordered_map<openvdb::Coord, std::size_t> localIndices;
                localIndices.reserve(32);
                for (std::size_t index = 0; index < target.samples.size(); ++index) {
                    if (target.samples[index].kind == volume_surface::SurfaceTargetSampleKind::Core) {
                        localIndices.emplace(target.samples[index].coordinate, index);
                    }
                }
                double signedVote = 0.0;
                double voteWeight = 0.0;
                std::size_t supportCount = 0;
                constexpr double maximumSurfaceNormalComponent = 0.75;
                for (int dz = -1; dz <= 1; ++dz) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            if (dx == 0 && dy == 0 && dz == 0) {
                                continue;
                            }
                            const auto found = localIndices.find(
                                probeCoordinate.offsetBy(dx, dy, dz));
                            if (found == localIndices.end()) {
                                continue;
                            }
                            const std::size_t neighborIndex = found->second;
                            const auto& neighbor = target.samples[neighborIndex];
                            const openvdb::Vec3d neighborAxis = normalizeOrZero(
                                openvdb::Vec3d(fit.normals[neighborIndex]));
                            const openvdb::Vec3d neighborNormal = normalizeOrZero(
                                openvdb::Vec3d(oriented.normals[neighborIndex]));
                            const std::int32_t neighborDepth =
                                expansionTrace.depthBySample[neighborIndex];
                            const bool depthSupport = depth >= 1 &&
                                neighborDepth >= 0 && neighborDepth < depth &&
                                neighborNormal.lengthSqr() > 1.0e-20;
                            const double axisAlignment = std::abs(
                                fitNormal.dot(neighborAxis));
                            const bool alignmentPass = neighborAxis.lengthSqr() > 1.0e-20 &&
                                axisAlignment >= minimumAlignment;
                            const bool continuationPass = alignmentPass &&
                                surfaceContinuationAllowed(
                                    target,
                                    probeIndex,
                                    neighborIndex,
                                    fitNormal,
                                    neighborAxis,
                                    maximumSurfaceNormalComponent);
                            const double meshWeight = useMeshContinuity
                                ? volume_surface::surfaceMeshContinuityWeight(
                                    meshContinuity,
                                    target,
                                    probeIndex,
                                    neighborIndex)
                                : 1.0;
                            const bool continuityPass = meshWeight >= 0.05;
                            const bool accepted = depthSupport && alignmentPass &&
                                continuationPass && continuityPass;
                            double contribution = 0.0;
                            if (accepted) {
                                const double distanceWeight = 1.0 /
                                    static_cast<double>(dx * dx + dy * dy + dz * dz);
                                contribution = distanceWeight * axisAlignment * axisAlignment *
                                    meshWeight * meshWeight;
                                signedVote += contribution * fitNormal.dot(neighborNormal);
                                voteWeight += contribution;
                                ++supportCount;
                            }
                            std::cout << "probe.neighbor=" << neighbor.coordinate.x() << ','
                                      << neighbor.coordinate.y() << ',' << neighbor.coordinate.z()
                                      << " depth=" << neighborDepth
                                      << " oriented_dot=" << fitNormal.dot(neighborNormal)
                                      << " axis_abs_dot=" << axisAlignment
                                      << " mesh_weight=" << meshWeight
                                      << " depth_support=" << (depthSupport ? 1 : 0)
                                      << " alignment_pass=" << (alignmentPass ? 1 : 0)
                                      << " continuation_pass=" << (continuationPass ? 1 : 0)
                                      << " continuity_pass=" << (continuityPass ? 1 : 0)
                                      << " accepted=" << (accepted ? 1 : 0)
                                      << " contribution=" << contribution << '\n';
                        }
                    }
                }
                std::cout << "probe.legacy_vote_unused=support_count=" << supportCount
                          << " signed=" << signedVote
                          << " weight=" << voteWeight
                          << " signed_ratio=" << (voteWeight > 1.0e-20
                                ? signedVote / voteWeight
                                : 0.0)
                          << " expected_sign=" << (signedVote < 0.0 ? -1 : 1)
                          << '\n';
            }
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
