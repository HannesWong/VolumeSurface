#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>
#include <new>
#include <stdexcept>
#include <type_traits>
#include <utility>

namespace volume_surface {

template<typename T, std::size_t Alignment = 64>
class AlignedBuffer
{
    static_assert(std::is_trivially_destructible_v<T>);
    static_assert((Alignment & (Alignment - 1)) == 0);

public:
    AlignedBuffer() = default;

    explicit AlignedBuffer(std::size_t size)
    {
        resize(size);
    }

    AlignedBuffer(const AlignedBuffer&) = delete;
    AlignedBuffer& operator=(const AlignedBuffer&) = delete;

    AlignedBuffer(AlignedBuffer&& other) noexcept
        : mData(std::exchange(other.mData, nullptr))
        , mSize(std::exchange(other.mSize, 0))
    {
    }

    AlignedBuffer& operator=(AlignedBuffer&& other) noexcept
    {
        if (this != &other) {
            reset();
            mData = std::exchange(other.mData, nullptr);
            mSize = std::exchange(other.mSize, 0);
        }
        return *this;
    }

    ~AlignedBuffer()
    {
        reset();
    }

    void resize(std::size_t size)
    {
        if (size == mSize) return;
        reset();
        if (size == 0) return;
        if (size > std::numeric_limits<std::size_t>::max() / sizeof(T)) {
            throw std::length_error("aligned buffer size overflow");
        }

        mData = static_cast<T*>(::operator new[](size * sizeof(T), std::align_val_t(Alignment)));
        mSize = size;
        std::fill_n(mData, mSize, T{});
    }

    void reset() noexcept
    {
        if (mData) {
            ::operator delete[](mData, std::align_val_t(Alignment));
            mData = nullptr;
        }
        mSize = 0;
    }

    [[nodiscard]] T* data() noexcept { return mData; }
    [[nodiscard]] const T* data() const noexcept { return mData; }
    [[nodiscard]] std::size_t size() const noexcept { return mSize; }
    [[nodiscard]] bool empty() const noexcept { return mSize == 0; }

    T& operator[](std::size_t index) noexcept { return mData[index]; }
    const T& operator[](std::size_t index) const noexcept { return mData[index]; }

private:
    T* mData = nullptr;
    std::size_t mSize = 0;
};

}
