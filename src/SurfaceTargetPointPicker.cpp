#include "volume_surface/viewer/SurfaceTargetPointPicker.h"

#include "volume_surface/SurfaceTarget.h"
#include "volume_surface_viewer_resources.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <backend/PixelBufferDescriptor.h>
#include <filament/Camera.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderTarget.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>
#include <filament/Texture.h>
#include <filament/VertexBuffer.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <math/vec3.h>
#include <utils/EntityManager.h>

using filament::Engine;
using filament::IndexBuffer;
using filament::Material;
using filament::MaterialInstance;
using filament::RenderTarget;
using filament::RenderableManager;
using filament::Renderer;
using filament::Scene;
using filament::Texture;
using filament::VertexBuffer;
using filament::math::float3;
using filament::math::float4;
using utils::Entity;
using utils::EntityManager;

namespace volume_surface::viewer {
namespace {

struct PointVertex {
    float3 position{};
    std::array<std::uint8_t, 4> color{};
    float4 custom0{};
};

struct Request {
    int x = 0;
    int y = 0;
    double scaleX = 1.0;
    double scaleY = 1.0;
};

struct PendingReadback {
    bool complete = false;
    SurfaceTargetPointPicker::Result result;
};

struct ReadbackContext {
    std::shared_ptr<PendingReadback> pending;
    int x = 0;
    int y = 0;
    int width = 0;
    int height = 0;
    int radius = 0;
};

void releaseUpload(void* buffer, std::size_t, void*)
{
    std::free(buffer);
}

template <typename T>
filament::backend::BufferDescriptor copyForUpload(const std::vector<T>& source)
{
    const std::size_t byteCount = source.size() * sizeof(T);
    void* copy = std::malloc(byteCount);
    if (!copy) {
        throw std::bad_alloc();
    }
    std::memcpy(copy, source.data(), byteCount);
    return filament::backend::BufferDescriptor(copy, byteCount, releaseUpload);
}

float3 transformPosition(
    const openvdb::Vec3d& position,
    const openvdb::Vec3d& referenceCenter,
    float displayScale)
{
    return (float3{
        static_cast<float>(position.x()),
        static_cast<float>(position.y()),
        static_cast<float>(position.z())} -
        float3{
            static_cast<float>(referenceCenter.x()),
            static_cast<float>(referenceCenter.y()),
            static_cast<float>(referenceCenter.z())}) * displayScale +
        float3{0.0f, 0.0f, -4.0f};
}

std::uint32_t decodeId(const std::uint8_t* pixel)
{
    return static_cast<std::uint32_t>(pixel[0]) |
        (static_cast<std::uint32_t>(pixel[1]) << 8u) |
        (static_cast<std::uint32_t>(pixel[2]) << 16u);
}

void readbackCallback(void* buffer, std::size_t size, void* user)
{
    auto* context = static_cast<ReadbackContext*>(user);
    if (!context || !context->pending) {
        delete[] static_cast<std::uint8_t*>(buffer);
        delete context;
        return;
    }
    const std::size_t required = static_cast<std::size_t>(context->width) *
        static_cast<std::size_t>(context->height) * 4u;
    SurfaceTargetPointPicker::Result result;
    result.ready = true;
    if (buffer && size >= required) {
        double nearestDistanceSquared = std::numeric_limits<double>::infinity();
        std::uint32_t nearestId = 0;
        const auto* bytes = static_cast<const std::uint8_t*>(buffer);
        for (int row = 0; row < context->height; ++row) {
            const int topY = context->y - context->radius +
                (context->height - 1 - row);
            for (int column = 0; column < context->width; ++column) {
                const auto id = decodeId(bytes +
                    (static_cast<std::size_t>(row) * context->width + column) * 4u);
                if (id == 0) {
                    continue;
                }
                const int topX = context->x - context->radius + column;
                const double dx = static_cast<double>(topX) - context->x;
                const double dy = static_cast<double>(topY) - context->y;
                const double distanceSquared = dx * dx + dy * dy;
                if (distanceSquared < nearestDistanceSquared) {
                    nearestDistanceSquared = distanceSquared;
                    nearestId = id;
                }
            }
        }
        if (nearestId > 0) {
            result.hit = true;
            result.sampleIndex = static_cast<std::size_t>(nearestId - 1u);
        }
    }
    context->pending->result = result;
    context->pending->complete = true;
    delete[] static_cast<std::uint8_t*>(buffer);
    delete context;
}

} // namespace

struct SurfaceTargetPointPicker::Impl {
    Material* material = nullptr;
    MaterialInstance* materialInstance = nullptr;
    VertexBuffer* vertexBuffer = nullptr;
    IndexBuffer* indexBuffer = nullptr;
    Texture* colorTexture = nullptr;
    Texture* depthTexture = nullptr;
    RenderTarget* renderTarget = nullptr;
    Scene* scene = nullptr;
    filament::Camera* camera = nullptr;
    Entity cameraEntity;
    Entity renderableEntity;
    std::size_t sampleCount = 0;
    float basePointRadius = 0.001f;
    float pointRadius = 0.001f;
    Request request;
    bool requestPending = false;
    bool rendering = false;
    std::shared_ptr<PendingReadback> pendingReadback;
    Scene* originalScene = nullptr;
    RenderTarget* originalRenderTarget = nullptr;
    filament::Camera* originalCamera = nullptr;
    filament::Viewport originalViewport{};
    bool originalPostProcessing = true;
    int readX = 0;
    int readY = 0;
    int readWidth = 0;
    int readHeight = 0;
    int readRadius = 0;

    void destroyGeometry(Engine& engine)
    {
        if (scene && renderableEntity) {
            scene->remove(renderableEntity);
        }
        if (renderableEntity) {
            engine.destroy(renderableEntity);
            EntityManager::get().destroy(renderableEntity);
            renderableEntity.clear();
        }
        if (indexBuffer) {
            engine.destroy(indexBuffer);
            indexBuffer = nullptr;
        }
        if (vertexBuffer) {
            engine.destroy(vertexBuffer);
            vertexBuffer = nullptr;
        }
        if (materialInstance) {
            engine.destroy(materialInstance);
            materialInstance = nullptr;
        }
        sampleCount = 0;
    }

    void destroyTarget(Engine& engine)
    {
        if (renderTarget) {
            engine.destroy(renderTarget);
            renderTarget = nullptr;
        }
        if (colorTexture) {
            engine.destroy(colorTexture);
            colorTexture = nullptr;
        }
        if (depthTexture) {
            engine.destroy(depthTexture);
            depthTexture = nullptr;
        }
    }

    void destroyAll(Engine& engine)
    {
        destroyGeometry(engine);
        destroyTarget(engine);
        if (scene) {
            engine.destroy(scene);
            scene = nullptr;
        }
        if (camera) {
            engine.destroyCameraComponent(cameraEntity);
            EntityManager::get().destroy(cameraEntity);
            camera = nullptr;
            cameraEntity.clear();
        }
        if (material) {
            engine.destroy(material);
            material = nullptr;
        }
        pendingReadback.reset();
        requestPending = false;
        rendering = false;
    }

    void ensureTarget(Engine& engine, int width, int height)
    {
        if (width <= 0 || height <= 0) {
            return;
        }
        const auto* existingColor = renderTarget
            ? renderTarget->getTexture(RenderTarget::AttachmentPoint::COLOR0)
            : nullptr;
        if (existingColor && existingColor->getWidth() == static_cast<std::uint32_t>(width) &&
            existingColor->getHeight() == static_cast<std::uint32_t>(height)) {
            return;
        }
        destroyTarget(engine);
        colorTexture = Texture::Builder()
            .width(static_cast<std::uint32_t>(width))
            .height(static_cast<std::uint32_t>(height))
            .levels(1)
            .sampler(Texture::Sampler::SAMPLER_2D)
            .format(Texture::InternalFormat::RGBA8)
            .usage(Texture::Usage::COLOR_ATTACHMENT | Texture::Usage::BLIT_SRC)
            .build(engine);
        depthTexture = Texture::Builder()
            .width(static_cast<std::uint32_t>(width))
            .height(static_cast<std::uint32_t>(height))
            .levels(1)
            .sampler(Texture::Sampler::SAMPLER_2D)
            .format(Texture::InternalFormat::DEPTH24)
            .usage(Texture::Usage::DEPTH_ATTACHMENT)
            .build(engine);
        renderTarget = RenderTarget::Builder()
            .texture(RenderTarget::AttachmentPoint::COLOR0, colorTexture)
            .texture(RenderTarget::AttachmentPoint::DEPTH, depthTexture)
            .build(engine);
        if (!colorTexture || !depthTexture || !renderTarget) {
            throw std::runtime_error("Failed to create SurfaceTarget point pick render target");
        }
    }
};

SurfaceTargetPointPicker::SurfaceTargetPointPicker()
    : mImpl(std::make_unique<Impl>())
{
}

SurfaceTargetPointPicker::~SurfaceTargetPointPicker() = default;

SurfaceTargetPointPicker::SurfaceTargetPointPicker(SurfaceTargetPointPicker&&) noexcept = default;

SurfaceTargetPointPicker& SurfaceTargetPointPicker::operator=(SurfaceTargetPointPicker&&) noexcept = default;

void SurfaceTargetPointPicker::rebuild(
    Engine& engine,
    const SurfaceTargetPointPicker::Inputs& inputs)
{
    if (!mImpl->scene) {
        mImpl->scene = engine.createScene();
        mImpl->cameraEntity = EntityManager::get().create();
        mImpl->camera = engine.createCamera(mImpl->cameraEntity);
        mImpl->material = Material::Builder()
            .package(
                VOLUME_SURFACE_VIEWER_RESOURCES_SURFACETARGETPOINTPICK_DATA,
                VOLUME_SURFACE_VIEWER_RESOURCES_SURFACETARGETPOINTPICK_SIZE)
            .build(engine);
        if (!mImpl->material || !mImpl->camera) {
            throw std::runtime_error("Failed to create SurfaceTarget point pick resources");
        }
    }
    mImpl->destroyGeometry(engine);
    mImpl->pendingReadback.reset();
    mImpl->requestPending = false;
    if (!inputs.cache || inputs.cache->empty() || !inputs.includeCore) {
        return;
    }

    const std::size_t stride = std::max<std::size_t>(1, inputs.pointStride);
    std::vector<PointVertex> vertices;
    vertices.reserve(inputs.cache->coreCount / stride + 1);
    const double minimumVoxelSize = std::max(
        std::min({
            std::abs(inputs.voxelSize.x()),
            std::abs(inputs.voxelSize.y()),
            std::abs(inputs.voxelSize.z())}),
        1.0e-9);
    mImpl->basePointRadius = static_cast<float>(
        0.5 * minimumVoxelSize * std::max(inputs.displayScale, 1.0e-6f));
    mImpl->pointRadius = mImpl->basePointRadius *
        std::clamp(inputs.pointScale, 0.05f, 20.0f);
    for (std::size_t index = 0; index < inputs.cache->samples.size(); index += stride) {
        const auto& sample = inputs.cache->samples[index];
        if (sample.kind != SurfaceTargetSampleKind::Core) {
            continue;
        }
        vertices.push_back({
            transformPosition(
                openvdb::Vec3d(sample.worldPosition),
                inputs.referenceCenter,
                inputs.displayScale),
            {
                static_cast<std::uint8_t>((index + 1u) & 0xffu),
                static_cast<std::uint8_t>(((index + 1u) >> 8u) & 0xffu),
                static_cast<std::uint8_t>(((index + 1u) >> 16u) & 0xffu),
                255u},
            {mImpl->pointRadius, 0.0f, 0.0f, 0.0f}});
    }
    if (vertices.empty()) {
        return;
    }
    mImpl->sampleCount = vertices.size();
    float3 minimum = vertices.front().position;
    float3 maximum = minimum;
    for (const auto& vertex : vertices) {
        for (int axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], vertex.position[axis]);
            maximum[axis] = std::max(maximum[axis], vertex.position[axis]);
        }
    }
    const float3 extent = maximum - minimum;
    const float3 center = (minimum + maximum) * 0.5f;
    const float3 halfExtent{
        std::max(extent.x * 0.5f, 1.0e-4f),
        std::max(extent.y * 0.5f, 1.0e-4f),
        std::max(extent.z * 0.5f, 1.0e-4f)};
    mImpl->vertexBuffer = VertexBuffer::Builder()
        .vertexCount(static_cast<std::uint32_t>(vertices.size()))
        .bufferCount(1)
        .attribute(
            filament::VertexAttribute::POSITION,
            0,
            VertexBuffer::AttributeType::FLOAT3,
            offsetof(PointVertex, position),
            sizeof(PointVertex))
        .attribute(
            filament::VertexAttribute::COLOR,
            0,
            VertexBuffer::AttributeType::UBYTE4,
            offsetof(PointVertex, color),
            sizeof(PointVertex))
        .attribute(
            filament::VertexAttribute::CUSTOM0,
            0,
            VertexBuffer::AttributeType::FLOAT4,
            offsetof(PointVertex, custom0),
            sizeof(PointVertex))
        .normalized(filament::VertexAttribute::COLOR)
        .build(engine);
    if (!mImpl->vertexBuffer) {
        throw std::runtime_error("Failed to create SurfaceTarget point pick vertices");
    }
    mImpl->vertexBuffer->setBufferAt(engine, 0, copyForUpload(vertices));
    std::vector<std::uint32_t> indices(vertices.size());
    for (std::size_t index = 0; index < indices.size(); ++index) {
        indices[index] = static_cast<std::uint32_t>(index);
    }
    mImpl->indexBuffer = IndexBuffer::Builder()
        .indexCount(static_cast<std::uint32_t>(indices.size()))
        .bufferType(IndexBuffer::IndexType::UINT)
        .build(engine);
    mImpl->indexBuffer->setBuffer(engine, copyForUpload(indices));
    mImpl->materialInstance = mImpl->material->createInstance();
    mImpl->materialInstance->setParameter("POINT_RADIUS", mImpl->pointRadius);
    mImpl->renderableEntity = EntityManager::get().create();
    RenderableManager::Builder(1)
        .boundingBox({center, halfExtent})
        .material(0, mImpl->materialInstance)
        .geometry(
            0,
            RenderableManager::PrimitiveType::POINTS,
            mImpl->vertexBuffer,
            mImpl->indexBuffer)
        .priority(8)
        .culling(false)
        .receiveShadows(false)
        .castShadows(false)
        .build(engine, mImpl->renderableEntity);
    mImpl->scene->addEntity(mImpl->renderableEntity);
}

void SurfaceTargetPointPicker::updatePointScale(float pointScale) noexcept
{
    if (!mImpl->materialInstance) {
        return;
    }
    const float scale = std::clamp(pointScale, 0.05f, 20.0f);
    mImpl->pointRadius = mImpl->basePointRadius * scale;
    mImpl->materialInstance->setParameter(
        "POINT_RADIUS",
        mImpl->pointRadius);
}

bool SurfaceTargetPointPicker::request(
    int pointerX,
    int pointerY,
    double scaleX,
    double scaleY) noexcept
{
    if (mImpl->sampleCount == 0 || mImpl->requestPending || mImpl->rendering ||
        (mImpl->pendingReadback && !mImpl->pendingReadback->complete)) {
        return false;
    }
    mImpl->request = {pointerX, pointerY, scaleX, scaleY};
    mImpl->requestPending = true;
    return true;
}

void SurfaceTargetPointPicker::prepare(Engine& engine, filament::View& mainView)
{
    if (!mImpl->requestPending || !mImpl->scene || !mImpl->camera) {
        return;
    }
    const auto viewport = mainView.getViewport();
    const int width = static_cast<int>(viewport.width);
    const int height = static_cast<int>(viewport.height);
    if (width <= 0 || height <= 0) {
        mImpl->requestPending = false;
        return;
    }
    mImpl->ensureTarget(engine, width, height);
    const int localX = static_cast<int>(std::lround(
        static_cast<double>(mImpl->request.x) * mImpl->request.scaleX)) - viewport.left;
    const int localY = static_cast<int>(std::lround(
        static_cast<double>(mImpl->request.y) * mImpl->request.scaleY));
    if (localX < 0 || localY < 0 || localX >= width || localY >= height) {
        mImpl->requestPending = false;
        return;
    }
    constexpr int radius = 4;
    mImpl->readRadius = radius;
    mImpl->readX = std::clamp(localX, radius, width - radius - 1);
    mImpl->readY = std::clamp(localY, radius, height - radius - 1);
    mImpl->readWidth = std::min(2 * radius + 1, width);
    mImpl->readHeight = std::min(2 * radius + 1, height);
    mImpl->originalScene = mainView.getScene();
    mImpl->originalRenderTarget = mainView.getRenderTarget();
    mImpl->originalCamera = &mainView.getCamera();
    mImpl->originalViewport = viewport;
    mImpl->originalPostProcessing = mainView.isPostProcessingEnabled();
    mImpl->camera->setCustomProjection(
        mImpl->originalCamera->getProjectionMatrix(),
        mImpl->originalCamera->getCullingProjectionMatrix(),
        mImpl->originalCamera->getNear(),
        mImpl->originalCamera->getCullingFar());
    mImpl->camera->setModelMatrix(mImpl->originalCamera->getModelMatrix());
    mainView.setScene(mImpl->scene);
    mainView.setCamera(mImpl->camera);
    mainView.setRenderTarget(mImpl->renderTarget);
    mainView.setViewport({0, 0, static_cast<std::uint32_t>(width),
        static_cast<std::uint32_t>(height)});
    mainView.setPostProcessingEnabled(false);
    mImpl->requestPending = false;
    mImpl->rendering = true;
}

void SurfaceTargetPointPicker::finish(Renderer& renderer, filament::View& mainView)
{
    if (!mImpl->rendering) {
        return;
    }
    auto pending = std::make_shared<PendingReadback>();
    auto* context = new ReadbackContext{
        pending,
        mImpl->readX,
        mImpl->readY,
        mImpl->readWidth,
        mImpl->readHeight,
        mImpl->readRadius};
    const int bottom = static_cast<int>(mainView.getViewport().height) -
        (mImpl->readY + mImpl->readHeight - mImpl->readRadius);
    renderer.readPixels(
        mImpl->renderTarget,
        static_cast<std::uint32_t>(mImpl->readX - mImpl->readRadius),
        static_cast<std::uint32_t>(std::max(0, bottom)),
        static_cast<std::uint32_t>(mImpl->readWidth),
        static_cast<std::uint32_t>(mImpl->readHeight),
        filament::backend::PixelBufferDescriptor(
            new std::uint8_t[static_cast<std::size_t>(mImpl->readWidth) *
                static_cast<std::size_t>(mImpl->readHeight) * 4u],
            static_cast<std::size_t>(mImpl->readWidth) *
                static_cast<std::size_t>(mImpl->readHeight) * 4u,
            filament::backend::PixelBufferDescriptor::PixelDataFormat::RGBA,
            filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE,
            readbackCallback,
            context));
    mImpl->pendingReadback = std::move(pending);
    mainView.setScene(mImpl->originalScene);
    mainView.setCamera(mImpl->originalCamera);
    mainView.setRenderTarget(mImpl->originalRenderTarget);
    mainView.setViewport(mImpl->originalViewport);
    mainView.setPostProcessingEnabled(mImpl->originalPostProcessing);
    mImpl->rendering = false;
}

SurfaceTargetPointPicker::Result SurfaceTargetPointPicker::consumeResult() noexcept
{
    if (!mImpl->pendingReadback || !mImpl->pendingReadback->complete) {
        return {};
    }
    const auto result = mImpl->pendingReadback->result;
    mImpl->pendingReadback.reset();
    return result;
}

bool SurfaceTargetPointPicker::hasPendingRequest() const noexcept
{
    return mImpl->requestPending || mImpl->rendering ||
        (mImpl->pendingReadback && !mImpl->pendingReadback->complete);
}

bool SurfaceTargetPointPicker::ready() const noexcept
{
    return mImpl->pendingReadback && mImpl->pendingReadback->complete;
}

void SurfaceTargetPointPicker::destroy(Engine& engine)
{
    mImpl->destroyAll(engine);
}

} // namespace volume_surface::viewer
