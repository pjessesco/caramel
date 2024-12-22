//
// This software is released under the MIT license.
//
// Copyright (c) 2022-2023 Jino Park
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

#include <common.h>
#include <microfacets.h>




namespace Caramel{
    Beckmann::Beckmann(Float alphaX, Float alphaY)
    : m_alphaX{alphaX}, m_alphaY{alphaY}, Microfacets() {}

    Float Beckmann::lambda(const Caramel::Vector3f &vec) const {

        const Float cos_phi = vec_cos_phi(vec);
        const Float sin_phi = vec_sin_phi(vec);

        const Float tmp1 = cos_phi * m_alphaX;
        const Float tmp2 = sin_phi * m_alphaY;

        const Float alpha = std::sqrt(tmp1 * tmp1 + tmp2 * tmp2);

        const Float b = Float1 / (alpha * vec_tan(vec));
        if(b >= static_cast<Float>(1.6)){
            return Float0;
        }

        const Float b_2 = b * b;
        return (Float1 - static_cast<Float>(1.259) * b + static_cast<Float>(0.396) * b_2) /
               (static_cast<Float>(3.535) * b + static_cast<Float>(2.181) * (b_2));

    }

}