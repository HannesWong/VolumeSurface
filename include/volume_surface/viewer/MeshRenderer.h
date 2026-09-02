#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>

#include "volume_surface/SurfaceMesh.h"
#include "volume_surface/viewer/SceneCoordinateMapper.h"

#include <utils/Entity.h>

namespace filament {
class Engine;
class Material;
class Scene;
}

namespace volume_surface::viewer {

struct MeshSlot {
    std::string name;
    SurfaceMesh mesh;
    std::array<float, 3> color{};
    float opacity = 1.0f;
    bool visible = false;

    [[nodiscard]] bool available() const noexcept
    {
        return !mesh.empty();
    }
};

class MeshRenderer final {
public:
    MeshRenderer();
    ~MeshRenderer();

    MeshRenderer(const MeshRenderer&) = delete;
    MeshRenderer& operator=(const MeshRenderer&) = delete;
    MeshRenderer(MeshRenderer&&) noexcept;
    MeshRenderer& operator=(MeshRenderer&&) noexcept;

    void create(
        filament::Engine& engine,
        filament::Scene& scene,
        std::size_t slotIndex,
        const MeshSlot& slot,
        const SceneCoordinateMapper& mapper,
        const filament::Material* opaqueMaterial,
        const filament::Material* transparentMaterial);

    void applyStyle(
        filament::Engine& engine,
        std::size_t slotIndex,
        const MeshSlot& slot);

    void setVisible(
        filament::Scene& scene,
        std::size_t slotIndex,
        bool visible);

    [[nodiscard]] utils::Entity entity(std::size_t slotIndex) const noexcept;

    void destroy(
        filament::Engine& engine,
        filament::Scene& scene,
        std::size_t slotIndex);

    void destroyAll(
        filament::Engine& engine,
        filament::Scene& scene);

private:
    struct Impl;
    std::unique_ptr<Impl> mImpl;
};

} // namespace volume_surface::viewer
