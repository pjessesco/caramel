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

#pragma once

#include <common.h>

namespace Caramel{
    class Microfacets{
    public:
        // Masking function : Ratio of visible microfacet area to
        //                    total forward-facing microfacet area
        Float G1(const Vector3f &vec) const{
            return Float1 / (Float1 + lambda(vec));
        }

        // Masking-shadowing function : Gives the fraction of microfacets
        //                              visible from given directions
        Float G(const Vector3f &v1, const Vector3f &v2) const{
            // return G1(v1) * G1(v2);
            return Float1 / (Float1 + lambda(v1) + lambda(v2));
        }

    protected:
        // Helper function : Ratio of invisible masked microfacet area
        //                   to visible microfacet area
        virtual Float lambda(const Vector3f &vec) const = 0;

        virtual Float D(const Vector3f &vec) const = 0;
    };

    class Beckmann : public Microfacets{
    public:
        Beckmann(Float alphaX, Float alphaY);

    protected:
        Float lambda(const Vector3f &vec) const override;

    private:
        Float m_alphaX;
        Float m_alphaY;
    };

}
