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

#include <sampler.h>

#include <common.h>

namespace Caramel{

    UniformStdSampler::UniformStdSampler(uint64_t seed, uint64_t stream) {
        // Initialize PCG32
        // inc must be odd, so we use (stream << 1) | 1
        m_state = 0;
        m_inc = (stream << 1) | 1;

        // Warm up the generator
        next_uint32();
        m_state += seed;
        next_uint32();
    }

    uint32_t UniformStdSampler::next_uint32() {
        // Save old state for output function
        uint64_t oldstate = m_state;

        // LCG state transition
        m_state = oldstate * 6364136223846793005ULL + m_inc;

        // Permutation: XOR-shift + random rotation
        uint32_t xorshifted = static_cast<uint32_t>(((oldstate >> 18u) ^ oldstate) >> 27u);
        uint32_t rot = static_cast<uint32_t>(oldstate >> 59u);
        return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
    }

    Float UniformStdSampler::sample_1d() {
        // Convert to [0, 1) range
        // Use upper 24 bits for float (24-bit mantissa)
        return static_cast<Float>(next_uint32() >> 8) * static_cast<Float>(0x1.0p-24);
    }

}