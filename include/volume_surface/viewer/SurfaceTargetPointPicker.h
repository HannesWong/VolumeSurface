#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include <openvdb/openvdb.h>

namespace filament {
class Engine;
class Renderer;
class Scene;
class View;
}

namespace volume_surface {
struct SurfaceTargetCache;
}

namespace volume_surface::viewer {

class SurfaceTargetPointPicker final {
public:
    struct Inputs {
        const SurfaceTargetCache* cache = nullptr;
        openvdb::Vec3d referenceCenter{};
        openvdb::Vec3d voxelSize{1.0, 1.0, 1.0};
        float displayScale = 1.0f;
        float pointScale = 1.0f;
        std::size_t pointStride = 1;
        bool includeCore = true;
    };

    struct Result {
        bool ready = false;
        bool hit = false;
        std::size_t sampleIndex = 0;
    };

    SurfaceTargetPointPicker();
    ~SurfaceTargetPointPicker();

    SurfaceTargetPointPicker(const SurfaceTargetPointPicker&) = delete;
    SurfaceTargetPointPicker& operator=(const SurfaceTargetPointPicker&) = delete;
    SurfaceTargetPointPicker(SurfaceTargetPointPicker&&) noexcept;
    SurfaceTargetPointPicker& operator=(SurfaceTargetPointPicker&&) noexcept;

    void rebuild(filament::Engine& engine, const Inputs& inputs);

    void updatePointScale(float pointScale) noexcept;

    bool request(
        int pointerX,
        int pointerY,
        double scaleX,
        double scaleY) noexcept;

    void prepare(
        filament::Engine& engine,
        filament::View& mainView);

    void finish(
        filament::Renderer& renderer,
        filament::View& mainView);

    [[nodiscard]] Result consumeResult() noexcept;
    [[nodiscard]] bool hasPendingRequest() const noexcept;
    [[nodiscard]] bool ready() const noexcept;

    void destroy(filament::Engine& engine);

private:
    struct Impl;
    std::unique_ptr<Impl> mImpl;
};

} // namespace volume_surface::viewer
