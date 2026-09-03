#pragma once

#include <array>
#include <cstddef>
#include <memory>

#include <openvdb/openvdb.h>

namespace filament {
class Engine;
class Scene;
}

namespace volume_surface::viewer {

class SurfaceFitPlaneRenderer final {
public:
    struct Settings {
        bool showPlane = true;
        float depthBias = 0.0f;
        std::array<float, 4> depthSuccessColor{0.10f, 0.88f, 0.35f, 0.78f};
        std::array<float, 4> depthFailColor{0.98f, 0.16f, 0.12f, 0.84f};
    };

    struct Inputs {
        bool valid = false;
        openvdb::Vec3d centerWorld{};
        openvdb::Vec3d normalWorld{};
        openvdb::Vec3d referenceCenter{};
        openvdb::Vec3d voxelSize{1.0, 1.0, 1.0};
        std::size_t neighborhoodSide = 3;
        float displayScale = 1.0f;
    };

    struct Statistics {
        bool hasPlane = false;
        std::size_t vertexCount = 0;
    };

    SurfaceFitPlaneRenderer();
    ~SurfaceFitPlaneRenderer();

    SurfaceFitPlaneRenderer(const SurfaceFitPlaneRenderer&) = delete;
    SurfaceFitPlaneRenderer& operator=(const SurfaceFitPlaneRenderer&) = delete;
    SurfaceFitPlaneRenderer(SurfaceFitPlaneRenderer&&) noexcept;
    SurfaceFitPlaneRenderer& operator=(SurfaceFitPlaneRenderer&&) noexcept;

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
