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
#include <functional>

#include <integrators.h>

#include <image.h>
#include <light.h>
#include <parallel_for.h>
#include <progress.h>
#include <scene.h>
#include <logger.h>
#include <camera.h>
#include <sampler.h>

namespace Caramel{
    MCIntegrator::MCIntegrator(Index spp) : Integrator(), m_spp{spp} {}

    Image MCIntegrator::render(const Scene &scene){
        if(scene.m_cam == nullptr){
            CRM_ERROR("Camera is nullptr;");
        }

        auto size = scene.m_cam->get_size();
        Image img(size.first/* width */, size.second/* height */);

        ProgressBar progress_bar(size.first);

        CRM_LOG("Render start...");

        auto time1 = std::chrono::high_resolution_clock::now();

        parallel_for(0, size.first, std::function([&](int i){
                         UniformStdSampler sampler(i);
                         for(int j=0;j<size.second;j++){
                             Vector3f rgb = vec3f_zero;
                             for(Index s=0;s<m_spp;s++){
                                 rgb = rgb + get_pixel_value(scene, i + sampler.sample_1d(), j + sampler.sample_1d(), sampler);
                             }
                             rgb = rgb / m_spp;

                             img.set_pixel_value(i, j, rgb[0], rgb[1], rgb[2]);
                         }
                         progress_bar.increase();
                     }));

        auto time2 = std::chrono::high_resolution_clock::now();
        CRM_LOG("Render done in " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(time2 - time1).count() / 1000.0f) + " seconds");

        return img;
    }

    void MCIntegrator::pre_process(const Scene &scene) {}
}
