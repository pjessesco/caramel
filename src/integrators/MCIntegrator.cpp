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

#include <chrono>
#include <functional>
#include <random>

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

    void MCIntegrator::render(const Scene &scene, Image &output, const RenderConfig &config){
        auto size = scene.m_cam->get_size();
        const Index real_spp = config.spp > 0 ? config.spp : m_spp;

#if ENABLE_PROGRESS
        ProgressBar progress_bar(size.first);
        CRM_LOG("Render start...");
        const auto time1 = std::chrono::high_resolution_clock::now();
#endif

        parallel_for(0, size.first, std::function([&](int i){
                         if(config.should_stop && config.should_stop->load()) return;
                         std::random_device rd;
                         UniformStdSampler sampler(config.random_seed ? static_cast<int>(rd()) : i);
                         for(int j=0;j<size.second;j++){
                             Vector3f rgb = vec3f_zero;
                             for(Index s=0;s<real_spp;s++){
                                 rgb = rgb + get_pixel_value(scene, i + sampler.sample_1d(), j + sampler.sample_1d(), sampler);
                             }
                             rgb = rgb / real_spp;
                             output.set_pixel_value(i, j, rgb[0], rgb[1], rgb[2]);
                         }
#if ENABLE_PROGRESS
                         progress_bar.increase();
#endif
                     }));

#if ENABLE_PROGRESS
        const auto time2 = std::chrono::high_resolution_clock::now();
        CRM_LOG("Render done in " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(time2 - time1).count() / 1000.0f) + " seconds");
#endif
    }

    void MCIntegrator::pre_process(const Scene &scene) {}
}
