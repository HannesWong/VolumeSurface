#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "volume_surface/SurfaceNormalField.h"
#include "volume_surface/SurfaceTarget.h"

namespace filament {
class Engine;
class Scene;
}

namespace volume_surface::viewer {

enum class SurfaceFitNeighborhoodDisplayMode : std::uint8_t {
    None,
    Candidates,
    Kept,
    Rejected,
    Weight,
};

class SurfaceFitNeighborhoodDebugRenderer final {
public:
    struct Settings {
        SurfaceFitNeighborhoodDisplayMode displayMode =
            SurfaceFitNeighborhoodDisplayMode::None;
        float pointRadiusMultiplier = 1.2f;
    };

    struct Inputs {
        const SurfaceTargetCache* cache = nullptr;
        const SurfaceFitNeighborhoodInspection* inspection = nullptr;
        openvdb::Vec3d referenceCenter{};
        openvdb::Vec3d voxelSize{1.0, 1.0, 1.0};
        float displayScale = 1.0f;
        float sourcePointScale = 1.0f;
    };

    struct Statistics {
        bool valid = false;
        std::size_t candidateCount = 0;
        std::size_t keptCount = 0;
        std::size_t downweightedCount = 0;
        std::size_t rejectedCount = 0;
    };

    SurfaceFitNeighborhoodDebugRenderer();
    ~SurfaceFitNeighborhoodDebugRenderer();

    SurfaceFitNeighborhoodDebugRenderer(const SurfaceFitNeighborhoodDebugRenderer&) = delete;
    SurfaceFitNeighborhoodDebugRenderer& operator=(
        const SurfaceFitNeighborhoodDebugRenderer&) = delete;
    SurfaceFitNeighborhoodDebugRenderer(SurfaceFitNeighborhoodDebugRenderer&&) noexcept;
    SurfaceFitNeighborhoodDebugRenderer& operator=(
        SurfaceFitNeighborhoodDebugRenderer&&) noexcept;

    [[nodiscard]] Settings& settings() noexcept;
    [[nodiscard]] const Settings& settings() const noexcept;
    [[nodiscard]] const Statistics& statistics() const noexcept;

    void rebuild(
        filament::Engine& engine,
        filament::Scene& scene,
        const Inputs& inputs);

    void updateMaterialSettings(filament::Engine& engine);

    void setVisible(
        filament::Scene& scene,
        bool stageVisible);

    void destroy(
        filament::Engine& engine,
        filament::Scene& scene);

private:
    struct Impl;
    std::unique_ptr<Impl> mImpl;
};

} // namespace volume_surface::viewer
