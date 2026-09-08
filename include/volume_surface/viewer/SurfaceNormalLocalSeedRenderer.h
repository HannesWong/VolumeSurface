#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <openvdb/openvdb.h>

#include "volume_surface/SurfaceNormalLocalSeed.h"

namespace filament {
class Engine;
class Scene;
}

namespace volume_surface::viewer {

class SurfaceNormalLocalSeedRenderer final {
public:
    struct FlipPoint {
        openvdb::Vec3d worldPosition{};
        std::array<std::uint8_t, 4> color{255, 255, 255, 255};
        bool enabled = true;
    };

    struct FlipPointInputs {
        const std::vector<FlipPoint>* points = nullptr;
        openvdb::Vec3d referenceCenter{};
        float displayScale = 1.0f;
        float sourcePointScale = 1.0f;
        float minimumVoxelSize = 1.0f;
    };

    struct Settings {
        bool showAffected = true;
        bool showBoundary = true;
        float affectedPointRadiusMultiplier = 1.0f;
        float boundaryPointRadiusMultiplier = 1.15f;
        float seedPointRadiusMultiplier = 1.8f;
    };

    struct Inputs {
        const SurfaceTargetCache* cache = nullptr;
        const SurfaceNormalLocalSeedPreview* preview = nullptr;
        openvdb::Vec3d referenceCenter{};
        float displayScale = 1.0f;
        float sourcePointScale = 1.0f;
        float minimumVoxelSize = 1.0f;
    };

    SurfaceNormalLocalSeedRenderer();
    ~SurfaceNormalLocalSeedRenderer();

    SurfaceNormalLocalSeedRenderer(const SurfaceNormalLocalSeedRenderer&) = delete;
    SurfaceNormalLocalSeedRenderer& operator=(const SurfaceNormalLocalSeedRenderer&) = delete;
    SurfaceNormalLocalSeedRenderer(SurfaceNormalLocalSeedRenderer&&) noexcept;
    SurfaceNormalLocalSeedRenderer& operator=(SurfaceNormalLocalSeedRenderer&&) noexcept;

    [[nodiscard]] Settings& settings() noexcept;
    [[nodiscard]] const Settings& settings() const noexcept;

    void rebuild(
        filament::Engine& engine,
        filament::Scene& scene,
        const Inputs& inputs);

    void rebuildFlipPoints(
        filament::Engine& engine,
        filament::Scene& scene,
        const FlipPointInputs& inputs);

    void setVisible(filament::Scene& scene, bool stageVisible);
    void destroy(filament::Engine& engine, filament::Scene& scene);

private:
    struct Impl;
    std::unique_ptr<Impl> mImpl;
};

} // namespace volume_surface::viewer
