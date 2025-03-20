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

#pragma once

#include <thread>
#include <atomic>
#include <vector>
#include <queue>
#include <numeric>
#include <semaphore>
#include <functional>

#include <common.h>

namespace Caramel{

    constexpr int THREAD_NUM = 10;

    inline void parallel_for(int start_idx, int end_idx, std::function<void(int)> func){

        std::vector<std::thread> tasks(end_idx - start_idx);
        std::atomic<int> total_done_jobs = 0;
        std::counting_semaphore<THREAD_NUM> sem{THREAD_NUM};

        for(int i=start_idx;i<end_idx;i++){
            sem.acquire();
            std::thread th([&](int idx){
                func(idx);
                total_done_jobs++;
                sem.release();
            }, i);
            th.detach();
        }

        while (total_done_jobs != tasks.size()){}

    }
}