#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <openvdb/openvdb.h>

#include "volume_surface/SurfaceBrush.h"
#include "volume_surface/SurfaceMesh.h"
#include "volume_surface/SurfaceTarget.h"

namespace filament {
class Engine;
class Scene;
}

namespace volume_surface {

class SurfaceTargetPreview final {
public:
    struct Settings {
        bool showPoints = true;
        bool showNormals = false;
        bool showBvh = false;
        bool showCore = true;
        bool showTransition = true;
        bool frontFacingOnly = true;
        std::size_t pointStride = 32;
        std::size_t bvhDepth = 3;
        float pointScale = 1.0f;
    };

    struct Inputs {
        const SurfaceTargetCache* cache = nullptr;
        const SurfaceMesh* fallbackMesh = nullptr;
        const SurfaceBrushHierarchy* hierarchy = nullptr;
        openvdb::Vec3d referenceCenter{};
        openvdb::Vec3d voxelSize{1.0, 1.0, 1.0};
        float displayScale = 1.0f;
        const std::vector<openvdb::Vec3f>* normalOverrides = nullptr;
    };

    struct Statistics {
        std::size_t corePointCount = 0;
        std::size_t transitionPointCount = 0;
        std::size_t normalSegmentCount = 0;
        std::size_t bvhSegmentCount = 0;
    };

    SurfaceTargetPreview();
    ~SurfaceTargetPreview();

    SurfaceTargetPreview(const SurfaceTargetPreview&) = delete;
    SurfaceTargetPreview& operator=(const SurfaceTargetPreview&) = delete;
    SurfaceTargetPreview(SurfaceTargetPreview&&) noexcept;
    SurfaceTargetPreview& operator=(SurfaceTargetPreview&&) noexcept;

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

} // namespace volume_surface
