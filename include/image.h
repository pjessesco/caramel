//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2025 Jino Park
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#pragma once

#include <vector>
#include <string>

#include <common.h>

namespace Caramel {
    // Caramel considers only RGB(BGR) 3 channel image.
    class Image {
    public:
        static constexpr int CHANNEL_NUM = 3;

        Image(Index width, Index height);
        explicit Image(const std::string &filename);
        void write_exr(const std::string &filename) const;

        void set_pixel_value(int w, int h, Float r, Float g, Float b);
        Vector3f get_pixel_value(int w, int h) const;
        Vector2ui size() const;

        std::vector<std::vector<Float>> get_data_for_sampling(bool sin_weight) const;

    private:
        Image(Index width, Index height, const std::vector<Float> &m_data);

        // Called in constructor
        void read_from_jpg(const std::string &filename);
        void read_from_png(const std::string &filename);
        void read_from_exr(const std::string &filename);
        void read_from_hdr(const std::string &filename);

        Index m_width;
        Index m_height;
        std::vector<Float> m_data;
    };
}