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

#include <mutex>

// This class is independent with Caramel namespace.

class ProgressBar{
public:
    explicit ProgressBar(int total) : m_current(0), m_total_inv(1.0f / static_cast<float>(total)) {}

    void increase(){
        m_lock.lock();
        m_current++;
        const float done_ratio = m_current * m_total_inv;
        std::string done_str(static_cast<int>(m_len * done_ratio), '=');
        std::string remain_str(static_cast<int>(m_len * (1 - done_ratio)), '-');
        std::cout<<"["<<done_str << remain_str << "] " << done_ratio * 100 << " %\n";
        std::cout.flush();
        m_lock.unlock();
    }

private:
    const int m_len = 100;
    std::mutex m_lock;
    int m_current;
    const float m_total_inv;
};