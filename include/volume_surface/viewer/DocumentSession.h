#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "volume_surface/SurfaceBrush.h"
#include "volume_surface/SurfaceReconstruction.h"
#include "volume_surface/SurfaceTarget.h"

namespace volume_surface::viewer {

struct DocumentSession {
    std::filesystem::path input;
    std::string gridName;
    openvdb::FloatGrid::Ptr grid;

    std::unique_ptr<volume_surface::SurfaceBrushHierarchy> brushHierarchy;
    double brushHierarchyBuildMilliseconds = 0.0;

    float isoValue = 255.0f;
    float referenceIsoValue = 255.0f;
    float adaptivity = 0.1f;

    volume_surface::SurfaceTargetSettings surfaceTargetSettings;
    std::shared_ptr<volume_surface::SurfaceTargetCache> surfaceTargetCache;
    double surfaceTargetBuildMilliseconds = 0.0;
    std::filesystem::path surfaceTargetCachePath;

    volume_surface::SurfaceReconstructionSettings reconstructionSettings;
    volume_surface::SurfaceReconstructionTimings reconstructionTimings;
    std::size_t reconstructionCandidateCellCount = 0;
    std::size_t reconstructionCrossingCellCount = 0;
    std::size_t reconstructionFieldSampleCount = 0;
    std::size_t reconstructionSourceSupportFallbackCount = 0;
    std::size_t reconstructionProjectionVertexCount = 0;
    std::size_t reconstructionProjectionRejectedCount = 0;
    std::size_t reconstructionProjectionDensityRejectedCount = 0;
    double reconstructionProjectionMaximumDisplacement = 0.0;

    openvdb::FloatGrid::Ptr brushWeightGrid;
    std::filesystem::path brushWeightFieldDirectory;
    std::filesystem::path brushWeightFieldPath;
    std::vector<std::filesystem::path> brushWeightFieldFiles;
    int brushWeightFieldSelection = -1;
    std::array<char, 96> brushWeightFieldName{"default"};
    bool brushWeightFieldDirty = false;
};

} // namespace volume_surface::viewer
