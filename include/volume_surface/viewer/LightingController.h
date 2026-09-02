#pragma once

#include <memory>

#include <math/vec3.h>

namespace filament {
class Engine;
class Scene;
}

namespace volume_surface::viewer {

class LightingController final {
public:
    LightingController();
    LightingController(const LightingController&) = delete;
    LightingController& operator=(const LightingController&) = delete;
    LightingController(LightingController&&) = delete;
    LightingController& operator=(LightingController&&) = delete;
    ~LightingController();

    void create(
        filament::Engine& engine,
        filament::Scene& scene,
        float indirectIntensity,
        const filament::math::float3& direction);
    void setIntensity(float intensity);
    void setDirection(
        filament::Engine& engine,
        const filament::math::float3& direction);
    void destroy(filament::Engine& engine, filament::Scene& scene);

private:
    struct Resources;
    std::unique_ptr<Resources> mResources;
};

} // namespace volume_surface::viewer
