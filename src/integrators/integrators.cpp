//
// This software is released under the MIT license.
//
// Copyright (c) 2022 Jino Park
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

#include <chrono>

#include <image.h>
#include <integrators.h>
#include <light.h>
#include <parallel_for.h>
#include <progress.h>
#include <scene.h>

namespace Caramel{
    Integrator::Integrator(Scene scene) : m_scene{std::move(scene)} {}

    Image Integrator::render(){
        const Index width = m_scene.m_cam.m_w;
        const Index height = m_scene.m_cam.m_h;

        Image img(width, height);

        ProgressBar progress_bar(width);

        LOG("Render start...");

        auto time1 = std::chrono::high_resolution_clock::now();

        parallel_for(0, width, std::function([&](int i){
                         UniformStdSampler sampler(i);
                         for(int j=0;j<height;j++){
                             Vector3f rgb = vec3f_zero;
                             for(Index spp=0;spp<SPP;spp++){
                                 rgb = rgb + get_pixel_value(i + sampler.sample_1d(), j + sampler.sample_1d(), sampler);
                             }
                             rgb = rgb / SPP;

                             img.set_pixel_value(i, j, rgb[0], rgb[1], rgb[2]);
                         }
                         progress_bar.increase();
                     }));

        auto time2 = std::chrono::high_resolution_clock::now();
        LOG("Render done in " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(time2 - time1).count() / 1000.0f) + " seconds");

        return img;
    }
}
