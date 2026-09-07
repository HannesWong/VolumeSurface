#include <cstddef>
#include <memory>
#include <vector>

#include <openvdb/openvdb.h>

#include "volume_surface/SurfaceNormalField.h"
#include "volume_surface/SurfaceTarget.h"

namespace filament {
class Engine;
class Scene;
}

namespace volume_surface::viewer {

class SurfaceFitExpansionDebugRenderer final {
public:
    struct Settings {
        bool showNeighborhood = false;
        bool showUnifiedNeighborhood = false;
        bool showCurrentPointNormals = false;
        float pointRadiusMultiplier = 1.0f;
        float selectedPointRadiusMultiplier = 1.2f;
    };

    struct Inputs {
        const SurfaceTargetCache* cache = nullptr;
        const SurfaceNormalExpansionNeighborhood* neighborhood = nullptr;
        const SurfaceNormalExpansionTrace* expansionTrace = nullptr;
        const SurfaceFitNeighborhoodInspection* fitNeighborhood = nullptr;
        const std::vector<openvdb::Vec3f>* normalOverrides = nullptr;
        std::int32_t parentDepthThreshold = 0;
        openvdb::Vec3d referenceCenter{};
        openvdb::Vec3d voxelSize{1.0, 1.0, 1.0};
        float displayScale = 1.0f;
        float sourcePointScale = 1.0f;
    };

    struct Statistics {
        bool valid = false;
        std::size_t targetNextPointCount = 0;
        std::size_t sourceBatchPointCount = 0;
        std::size_t parentPointCount = 0;
    };

    SurfaceFitExpansionDebugRenderer();
    ~SurfaceFitExpansionDebugRenderer();

    SurfaceFitExpansionDebugRenderer(const SurfaceFitExpansionDebugRenderer&) = delete;
    SurfaceFitExpansionDebugRenderer& operator=(const SurfaceFitExpansionDebugRenderer&) = delete;
    SurfaceFitExpansionDebugRenderer(SurfaceFitExpansionDebugRenderer&&) noexcept;
    SurfaceFitExpansionDebugRenderer& operator=(SurfaceFitExpansionDebugRenderer&&) noexcept;

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
