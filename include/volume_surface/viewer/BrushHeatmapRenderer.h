#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <math/vec3.h>
#include <openvdb/openvdb.h>
#include <utils/Entity.h>

#include "volume_surface/SurfaceBrush.h"
#include "volume_surface/SurfaceMesh.h"

namespace filament {
class Engine;
class Scene;
}

namespace volume_surface::viewer {

class BrushHeatmapRenderer final {
public:
    struct Stats {
        std::size_t visibleTriangleCount = 0;
        std::size_t affectedVertexCount = 0;
    };

    BrushHeatmapRenderer();
    BrushHeatmapRenderer(const BrushHeatmapRenderer&) = delete;
    BrushHeatmapRenderer& operator=(const BrushHeatmapRenderer&) = delete;
    BrushHeatmapRenderer(BrushHeatmapRenderer&&) = delete;
    BrushHeatmapRenderer& operator=(BrushHeatmapRenderer&&) = delete;
    ~BrushHeatmapRenderer();

    void create(
        filament::Engine& engine,
        filament::Scene& scene,
        const SurfaceMesh& mesh,
        const openvdb::FloatGrid& grid,
        const filament::math::float3& referenceCenter,
        float displayScale);
    void destroy(filament::Engine& engine, filament::Scene& scene);
    void clear(filament::Engine& engine, filament::Scene& scene);
    void clearGeometry(filament::Engine& engine, filament::Scene& scene);

    bool applySamples(
        filament::Engine& engine,
        filament::Scene& scene,
        const std::vector<SurfaceBrushSample>& samples);
    void setVisible(filament::Scene& scene, bool visible);

    [[nodiscard]] Stats stats() const noexcept;
    [[nodiscard]] utils::Entity entity() const noexcept;
    [[nodiscard]] bool addedToScene() const noexcept;
    [[nodiscard]] bool available() const noexcept;

private:
    struct Resources;
    std::unique_ptr<Resources> mResources;
};

} // namespace volume_surface::viewer
