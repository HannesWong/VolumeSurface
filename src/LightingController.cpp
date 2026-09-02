#include "volume_surface/viewer/LightingController.h"

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Scene.h>
#include <filament/Skybox.h>
#include <utils/EntityManager.h>

namespace volume_surface::viewer {

namespace {

using filament::Engine;
using filament::IndirectLight;
using filament::LightManager;
using filament::Scene;
using filament::Skybox;
using filament::math::float3;
using utils::Entity;
using utils::EntityManager;

} // namespace

struct LightingController::Resources {
    Skybox* skybox = nullptr;
    IndirectLight* indirectLight = nullptr;
    Entity directionalLight;
};

LightingController::LightingController() = default;

LightingController::~LightingController() = default;

void LightingController::destroy(Engine& engine, Scene& scene)
{
    if (!mResources) {
        return;
    }
    Resources& resources = *mResources;
    if (resources.directionalLight) {
        scene.remove(resources.directionalLight);
        engine.destroy(resources.directionalLight);
        EntityManager::get().destroy(resources.directionalLight);
        resources.directionalLight.clear();
    }
    scene.setIndirectLight(nullptr);
    if (resources.indirectLight) {
        engine.destroy(resources.indirectLight);
        resources.indirectLight = nullptr;
    }
    scene.setSkybox(nullptr);
    if (resources.skybox) {
        engine.destroy(resources.skybox);
        resources.skybox = nullptr;
    }
}

void LightingController::create(
    Engine& engine,
    Scene& scene,
    float indirectIntensity,
    const float3& direction)
{
    if (!mResources) {
        mResources = std::make_unique<Resources>();
    }
    destroy(engine, scene);

    Resources& resources = *mResources;
    resources.skybox = Skybox::Builder()
        .color({0.035f, 0.045f, 0.065f, 1.0f})
        .build(engine);
    scene.setSkybox(resources.skybox);

    const float3 ambientIrradiance[1] = {{0.72f, 0.78f, 0.90f}};
    resources.indirectLight = IndirectLight::Builder()
        .irradiance(1, ambientIrradiance)
        .intensity(indirectIntensity)
        .build(engine);
    scene.setIndirectLight(resources.indirectLight);

    resources.directionalLight = EntityManager::get().create();
    LightManager::Builder(LightManager::Type::DIRECTIONAL)
        .color({1.0f, 0.96f, 0.90f})
        .intensity(100000.0f)
        .direction(direction)
        .castShadows(false)
        .build(engine, resources.directionalLight);
    scene.addEntity(resources.directionalLight);
}

void LightingController::setIntensity(float intensity)
{
    if (!mResources || !mResources->indirectLight) {
        return;
    }
    mResources->indirectLight->setIntensity(intensity);
}

void LightingController::setDirection(Engine& engine, const float3& direction)
{
    if (!mResources || !mResources->directionalLight) {
        return;
    }
    auto& lightManager = engine.getLightManager();
    const auto instance = lightManager.getInstance(mResources->directionalLight);
    lightManager.setDirection(instance, direction);
}

} // namespace volume_surface::viewer
