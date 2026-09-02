#pragma once

#include <chrono>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "volume_surface/SurfaceBrush.h"
#include "volume_surface/viewer/WeightPaintingStage.h"

namespace volume_surface::viewer {

using BrushProfileSettings = WeightPaintingSettings;

struct BrushProfilePoint {
    double inputMilliseconds = 0.0;
    openvdb::Vec3d worldPosition{};
};

struct BrushProfileFit {
    double inputMilliseconds = 0.0;
    std::size_t anchorCount = 0;
    bool finalFit = false;
    SurfaceBrushTimings coreTimings;
    double heatmapMilliseconds = 0.0;
    double endToEndMilliseconds = 0.0;
    std::size_t candidateVoxelCount = 0;
    std::size_t candidateLeafCount = 0;
    std::size_t centerlineNodeCount = 0;
    std::size_t sampleCount = 0;
    std::size_t heatmapTriangleCount = 0;
};

struct BrushProfileStroke {
    bool active = false;
    std::chrono::steady_clock::time_point startTime{};
    std::filesystem::path input;
    std::string gridName;
    float isoValue = 0.0f;
    float adaptivity = 0.0f;
    BrushProfileSettings settings;
    std::vector<BrushProfilePoint> points;
    std::vector<BrushProfileFit> fits;
};

class BrushProfileRecorder final {
public:
    void begin(
        const std::filesystem::path& input,
        const std::string& gridName,
        float isoValue,
        float adaptivity,
        const BrushProfileSettings& settings);
    void discard() noexcept;
    void addPoint(const openvdb::Vec3d& worldPosition);
    void addFit(
        std::size_t anchorCount,
        bool finalFit,
        const SurfaceBrushResult& result,
        double heatmapMilliseconds,
        double endToEndMilliseconds,
        std::size_t heatmapTriangleCount);
    void markLastFitFinal() noexcept;

    [[nodiscard]] bool active() const noexcept { return mStroke.active; }
    [[nodiscard]] bool hasFits() const noexcept { return !mStroke.fits.empty(); }
    [[nodiscard]] std::size_t pointCount() const noexcept { return mStroke.points.size(); }
    [[nodiscard]] std::size_t fitCount() const noexcept { return mStroke.fits.size(); }
    [[nodiscard]] const BrushProfileStroke& stroke() const noexcept { return mStroke; }

    bool append(const std::filesystem::path& path, std::string& error);

    static std::vector<BrushProfileStroke> load(
        const std::filesystem::path& path);

private:
    BrushProfileStroke mStroke;
};

} // namespace volume_surface::viewer
