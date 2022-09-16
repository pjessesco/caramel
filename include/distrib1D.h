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
#include <common.h>

namespace Caramel{
    class Distrib1D{
    public:
        Distrib1D() {}
        Distrib1D(const std::vector<Float> &vec){
            m_pdf = vec;
            m_cdf.resize(m_pdf.size());
            m_cdf[0] = m_pdf[0];

            for(Index i=1;i<m_pdf.size();i++){
                m_cdf[i] = m_cdf[i-1] + m_pdf[i];
            }
            const Float sum = m_cdf.back();

            for(int i=0;i<m_pdf.size();i++){
                m_pdf[i] /= sum;
                m_cdf[i] /= sum;
            }
        }

        Index sample(Float x) const{
            auto iter = std::upper_bound(m_cdf.begin(), m_cdf.end(), x);
            return iter - m_cdf.begin();
        }

        Float pdf(Index i) const{
            return m_pdf[i];
        }

        Float cdf(Index i) const{
            return m_cdf[i];
        }

    private:

        std::vector<Float> m_pdf;
        std::vector<Float> m_cdf;
    };
}