//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2026 Jino Park
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

#include <vector>
#include <string>

#include <common.h>
#include <image.h>
#include <logger.h>
#include <scene_parser.h>
#include <shape.h>
#include <scene.h>
#include <image.h>
#include <integrators.h>
#include <camera.h>
#include <FLIP.h>

namespace Caramel {

    Float avg(const Image &img){
        Float sum = Float0;
        const Index w = img.size()[0];
        const Index h = img.size()[1];
        for(int i=0;i<w;i++){
            for(int j=0;j<h;j++){
                const Vector3f val = img.get_pixel_value(i, j);
                sum += (val[0] + val[1] + val[2]);
            }
        }
        return sum / (w * h * 3);
    }

    Float flip_error(const Image &reference, const Image &test, bool useHDR, Image *diff_image_out) {
        if ((reference.size()[0] != test.size()[0]) || (reference.size()[1] != test.size()[1])) {
            CRM_ERROR("size diff");
        }

        const Index w = reference.size()[0];
        const Index h = reference.size()[1];

        std::vector<float> ref_buf(static_cast<size_t>(w) * h * 3);
        std::vector<float> test_buf(static_cast<size_t>(w) * h * 3);

        for (int i = 0; i < w; i++) {
            for (int j = 0; j < h; j++) {
                const Vector3f ref_val = reference.get_pixel_value(i, j);
                const Vector3f test_val = test.get_pixel_value(i, j);
                const size_t idx = (static_cast<size_t>(j) * w + i) * 3;
                ref_buf[idx + 0] = ref_val[0];
                ref_buf[idx + 1] = ref_val[1];
                ref_buf[idx + 2] = ref_val[2];
                test_buf[idx + 0] = test_val[0];
                test_buf[idx + 1] = test_val[1];
                test_buf[idx + 2] = test_val[2];
            }
        }

        FLIP::Parameters params;
        const bool want_diff_image = (diff_image_out != nullptr);
        float mean_error = 0.0f;
        float *error_map = nullptr;

        FLIP::evaluate(ref_buf.data(), test_buf.data(), static_cast<int>(w), static_cast<int>(h),
                       useHDR, params, want_diff_image, true, mean_error, &error_map);

        if (want_diff_image) {
            *diff_image_out = Image(w, h);
            for (int i = 0; i < w; i++) {
                for (int j = 0; j < h; j++) {
                    const size_t idx = (static_cast<size_t>(j) * w + i) * 3;
                    diff_image_out->set_pixel_value(i, j, error_map[idx + 0], error_map[idx + 1], error_map[idx + 2]);
                }
            }
        }

        delete[] error_map;

        return static_cast<Float>(mean_error);
    }

}