#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include <openvdb/openvdb.h>

namespace filament {
class Engine;
class Scene;
}

namespace volume_surface::viewer {

struct ViewerState;

class BrushPaintingPanel final {
public:
    void draw(
        ViewerState& state,
        filament::Engine& engine,
        filament::Scene& scene) const;

    openvdb::FloatGrid::Ptr createWeightGrid(
        const openvdb::FloatGrid& referenceGrid) const;
    std::filesystem::path weightFieldPathForName(
        const ViewerState& state,
        const std::string& name) const;
    std::string weightFieldName(const ViewerState& state) const;
    void setWeightFieldName(
        ViewerState& state,
        const std::string& name) const;
    void setWeightFieldNameFromPath(
        ViewerState& state,
        const std::filesystem::path& path) const;
    void refreshWeightFieldLibrary(ViewerState& state) const;
    bool saveWeightField(ViewerState& state, bool saveAsNew) const;
    void saveSettings(ViewerState& state) const;
    void loadSettings(ViewerState& state) const;
    void clearHeatmap(
        ViewerState& state,
        filament::Engine& engine,
        filament::Scene& scene) const;
    bool rebuildHeatmapFromWeightField(
        ViewerState& state,
        filament::Engine& engine,
        filament::Scene& scene) const;
    bool loadWeightField(
        ViewerState& state,
        filament::Engine& engine,
        filament::Scene& scene,
        const std::filesystem::path& path) const;
    void discardWeightFieldChanges(
        ViewerState& state,
        filament::Engine& engine,
        filament::Scene& scene) const;
};

} // namespace volume_surface::viewer
