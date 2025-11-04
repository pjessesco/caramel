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

//
// How does it works?
//
//             0                   1      2              3          4                    5          : index
//  ############--------------------$$$$$$$===============@@@@@@@@@@@!!!!!!!!!!!!!!!!!!!!!          : m_cdf value
//     * -> return 0               *     * -> return 2            * -> return 2     *             * : Random sample [0, 1)
//                                 -> return 1                                      -> return 5
// <-------------------------------------------------------------------------------------->
// 0                                                                                      1
//
//

#pragma once

#include <vector>
#include <numeric>

#include <common.h>

namespace Caramel{
    class Distrib1D{
    public:
        Distrib1D() = default;
        explicit Distrib1D(const std::vector<Float> &vec){
            m_pdf = vec;
            const size_t size = m_pdf.size();
            m_cdf.resize(size);
            m_cdf[0] = m_pdf[0];

            for(size_t i=1;i<size;i++){
                m_cdf[i] = m_cdf[i-1] + m_pdf[i];
            }
            const Float sum = m_cdf.back();

            for(size_t i=0;i<size;i++){
                m_pdf[i] /= sum;
                m_cdf[i] /= sum;
            }
        }

        Index sample(Float x) const{
            const auto iter = std::ranges::upper_bound(m_cdf, x);
            return iter - m_cdf.begin();
        }

        Float pdf(Index i) const{
            return m_pdf[i];
        }

    private:

        std::vector<Float> m_pdf;
        std::vector<Float> m_cdf;
    };

    class Distrib2D{
    public:
        Distrib2D() = default;
        explicit Distrib2D(const std::vector<std::vector<Float>> &vec){
            std::vector<Float> width_vals;
            for (const auto &w : vec) {
                width_vals.push_back(std::reduce(w.begin(), w.end()));
                m_height_distrib_list.emplace_back(w);
            }
            m_width_distrib = Distrib1D(width_vals);
        }

        Vector2ui sample(const Vector2f &_sample) const{
            const Index w = m_width_distrib.sample(_sample[0]);
            return {w, m_height_distrib_list[w].sample(_sample[1])};
        }

        Vector2ui sample(Float x, Float y) const {
            const Index w = m_width_distrib.sample(x);
            return {w, m_height_distrib_list[w].sample(y)};
        }

        Float pdf(Index i, Index j) const{
            return m_width_distrib.pdf(i) * m_height_distrib_list[i].pdf(j);
        }

    private:
        Distrib1D m_width_distrib;
        std::vector<Distrib1D> m_height_distrib_list;
    };
}