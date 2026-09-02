#include "volume_surface/viewer/SliceRenderer.h"

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <new>
#include <stdexcept>

#include <filament/Engine.h>
#include <filament/Texture.h>

namespace volume_surface::viewer {
namespace {

void releaseUpload(void* buffer, std::size_t, void*)
{
    std::free(buffer);
}

} // namespace

void SliceRenderer::rebuild(
    filament::Engine& engine,
    const DensitySlice& slice)
{
    if (slice.width == 0 || slice.height == 0 || slice.rgba.empty()) {
        destroy(engine);
        return;
    }

    void* pixelCopy = std::malloc(slice.rgba.size());
    if (!pixelCopy) {
        throw std::bad_alloc();
    }
    std::memcpy(pixelCopy, slice.rgba.data(), slice.rgba.size());
    filament::Texture::PixelBufferDescriptor pixels(
        pixelCopy,
        slice.rgba.size(),
        filament::Texture::Format::RGBA,
        filament::Texture::Type::UBYTE,
        releaseUpload);
    filament::Texture* texture = filament::Texture::Builder()
        .width(slice.width)
        .height(slice.height)
        .levels(1)
        .sampler(filament::Texture::Sampler::SAMPLER_2D)
        .format(filament::Texture::InternalFormat::RGBA8)
        .build(engine);
    if (!texture) {
        throw std::runtime_error("Failed to create slice texture");
    }
    texture->setImage(engine, 0, std::move(pixels));

    if (mTexture) {
        engine.destroy(mTexture);
    }
    mTexture = texture;
}

void SliceRenderer::destroy(filament::Engine& engine)
{
    if (mTexture) {
        engine.destroy(mTexture);
        mTexture = nullptr;
    }
}

} // namespace volume_surface::viewer
