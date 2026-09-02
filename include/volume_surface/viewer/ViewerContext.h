#pragma once

#include "volume_surface/viewer/LightingController.h"
#include "volume_surface/viewer/SceneCoordinateMapper.h"

class FilamentApp2;

namespace filament {
class Engine;
class Scene;
class View;
}

namespace volume_surface::viewer {

struct ViewerContext {
    ::FilamentApp2* app = nullptr;
    filament::Engine* engine = nullptr;
    filament::Scene* scene = nullptr;
    filament::View* view = nullptr;
    SceneCoordinateMapper coordinates;
    LightingController lighting;
};

} // namespace volume_surface::viewer
