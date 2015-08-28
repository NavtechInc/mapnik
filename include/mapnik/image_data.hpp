/*****************************************************************************
 *
 * This file is part of Mapnik (c++ mapping toolkit)
 *
 * Copyright (C) 2014 Artem Pavlenko
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *****************************************************************************/

#ifndef MAPNIK_IMAGE_DATA_HPP
#define MAPNIK_IMAGE_DATA_HPP

// mapnik
#include <mapnik/global.hpp>
// stl
#include <algorithm>
#include <cassert>
#include <stdexcept>

namespace mapnik {

namespace detail {

struct buffer
{
    explicit buffer(std::size_t size)
        : size_(size),
          data_(static_cast<unsigned char*>(size_ != 0 ? ::operator new(size_) : nullptr))
    {}

    buffer(buffer && rhs) noexcept
        : size_(std::move(rhs.size_)),
          data_(std::move(rhs.data_))
    {
        rhs.size_ = 0;
        rhs.data_ = nullptr;
    }

    buffer(buffer const& rhs)
        : size_(rhs.size_),
          data_(static_cast<unsigned char*>(size_ != 0 ? ::operator new(size_) : nullptr))
    {
        if (data_) std::copy(rhs.data_, rhs.data_ + rhs.size_, data_);
    }

    buffer& operator=(buffer rhs)
    {
        swap(rhs);
        return *this;
    }

    void swap(buffer & rhs)
    {
        std::swap(size_, rhs.size_);
        std::swap(data_, rhs.data_);
    }

    inline bool operator!() const { return (data_ == nullptr)? false : true; }
    ~buffer()
    {
        ::operator delete(data_);
    }

    inline unsigned char* data() { return data_; }
    inline unsigned char const* data() const { return data_; }
    inline std::size_t size() const { return size_; }
    std::size_t size_;
    unsigned char* data_;

};

template <std::size_t max_size>
struct image_dimensions
{
    image_dimensions(int width, int height)
        : width_(width),
          height_(height)
    {
        if (width < 0 || static_cast<std::size_t>(width) > max_size) throw std::runtime_error("Invalid width for image dimensions requested");
        if (height < 0 || static_cast<std::size_t>(height) > max_size) throw std::runtime_error("Invalid height for image dimensions requested");
    }

    image_dimensions(image_dimensions const& other) = default;
    image_dimensions(image_dimensions && other) = default;
    image_dimensions& operator= (image_dimensions rhs)
    {
        std::swap(width_, rhs.width_);
        std::swap(height_, rhs.height_);
        return *this;
    }
    std::size_t width() const
    {
        return width_;
    }
    std::size_t height() const
    {
        return height_;
    }
    std::size_t width_;
    std::size_t height_;
};

}

template <typename T, std::size_t max_size = 65535>
class image_data
{
public:
    using pixel_type = T;
    static constexpr std::size_t pixel_size = sizeof(pixel_type);

    image_data(int width, int height, bool initialize = true)
        : dimensions_(width, height),
          buffer_(dimensions_.width() * dimensions_.height() * pixel_size),
          pData_(reinterpret_cast<pixel_type*>(buffer_.data()))
    {
        if (pData_ && initialize) std::fill(pData_, pData_ + dimensions_.width() * dimensions_.height(), 0);
    }

    image_data(image_data<pixel_type> const& rhs)
        : dimensions_(rhs.dimensions_),
          buffer_(rhs.buffer_),
          pData_(reinterpret_cast<pixel_type*>(buffer_.data()))
    {}

    image_data(image_data<pixel_type> && rhs) noexcept
        : dimensions_(std::move(rhs.dimensions_)),
        buffer_(std::move(rhs.buffer_)),
        pData_(reinterpret_cast<pixel_type*>(buffer_.data()))
    {
        rhs.dimensions_ = { 0, 0 };
        rhs.pData_ = nullptr;
    }

    image_data<pixel_type>& operator=(image_data<pixel_type> rhs)
    {
        swap(rhs);
        return *this;
    }

    void swap(image_data<pixel_type> & rhs)
    {
        std::swap(dimensions_, rhs.dimensions_);
        std::swap(buffer_, rhs.buffer_);
    }

    inline pixel_type& operator() (std::size_t i, std::size_t j)
    {
        assert(i < dimensions_.width() && j < dimensions_.height());
        return pData_[j * dimensions_.width() + i];
    }
    inline const pixel_type& operator() (std::size_t i, std::size_t j) const
    {
        assert(i < dimensions_.width() && j < dimensions_.height());
        return pData_[j * dimensions_.width() + i];
    }
    inline std::size_t width() const
    {
        return dimensions_.width();
    }
    inline std::size_t height() const
    {
        return dimensions_.height();
    }
    inline unsigned getSize() const
    {
        return dimensions_.height() * dimensions_.width() * pixel_size;
    }
    inline unsigned getRowSize() const
    {
        return dimensions_.width() * pixel_size;
    }
    inline void set(pixel_type const& t)
    {
        std::fill(pData_, pData_ + dimensions_.width() * dimensions_.height(), t);
    }

    inline const pixel_type* getData() const
    {
        return pData_;
    }

    inline pixel_type* getData()
    {
        return pData_;
    }

    inline const unsigned char* getBytes() const
    {
        return buffer_.data();
    }

    inline unsigned char* getBytes()
    {
        return buffer_.data();
    }

    inline const pixel_type* getRow(std::size_t row) const
    {
        return pData_ + row * dimensions_.width();
    }

    inline const pixel_type* getRow(std::size_t row, std::size_t x0) const
    {
        return pData_ + row * dimensions_.width() + x0;
    }

    inline pixel_type* getRow(std::size_t row)
    {
        return pData_ + row * dimensions_.width();
    }

    inline pixel_type* getRow(std::size_t row, std::size_t x0)
    {
        return pData_ + row * dimensions_.width() + x0;
    }

    inline void setRow(std::size_t row, pixel_type const* buf, std::size_t size)
    {
        assert(row < dimensions_.height());
        assert(size <= dimensions_.width());
        std::copy(buf, buf + size, pData_ + row * dimensions_.width());
    }
    inline void setRow(std::size_t row, std::size_t x0, std::size_t x1, pixel_type const* buf)
    {
        assert(row < dimensions_.height());
        assert ((x1 - x0) <= dimensions_.width() );
        std::copy(buf, buf + (x1 - x0), pData_ + row * dimensions_.width() + x0);
    }

private:
    detail::image_dimensions<max_size> dimensions_;
    detail::buffer buffer_;
    pixel_type *pData_;
};

using image_data_rgba8 = image_data<std::uint32_t>;
using image_data_gray8 = image_data<std::uint8_t> ;
using image_data_gray16 = image_data<std::int16_t>;
using image_data_gray32f = image_data<float>;
}

#endif // MAPNIK_IMAGE_DATA_HPP
