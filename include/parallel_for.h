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
#include <functional>
#include <condition_variable>
#include <mutex>

namespace Caramel{

    // Persistent thread pool with task queue
    class ThreadPool {
    public:
        static ThreadPool& instance() {
            static ThreadPool pool;
            return pool;
        }

        template <typename F>
        void parallel_for(int start_idx, int end_idx, F &&func) {
            if (start_idx >= end_idx) return;

            const int task_count = end_idx - start_idx;
            std::atomic<int> next_task{0};
            std::atomic<int> completed_tasks{0};
            std::mutex completion_mutex;
            std::condition_variable completion_cv;

            auto worker_func = [&]() {
                while (true) {
                    int task_idx = next_task.fetch_add(1);
                    if (task_idx >= task_count) break;
                    func(start_idx + task_idx);
                    if (completed_tasks.fetch_add(1) + 1 == task_count) {
                        completion_cv.notify_one();
                    }
                }
            };

            // Launch worker threads (N-1 workers + main thread)
            // Note: When hardware_concurrency() returns 0 or 1, m_thread_count is 1,
            // so no worker threads are created and main thread does all work.
            std::vector<std::thread> workers;
            const unsigned int worker_count = m_thread_count - 1;
            workers.reserve(worker_count);
            for (unsigned int i = 0; i < worker_count; ++i) {
                workers.emplace_back(worker_func);
            }

            // Main thread participates in work
            worker_func();

            // Wait for completion
            {
                std::unique_lock<std::mutex> lock(completion_mutex);
                completion_cv.wait(lock, [&]() {
                    return completed_tasks >= task_count;
                });
            }

            // Join all worker threads
            for (auto& w : workers) {
                w.join();
            }
        }

    private:
        ThreadPool() : m_thread_count(std::max(1u, std::thread::hardware_concurrency())) {}
        ~ThreadPool() = default;
        ThreadPool(const ThreadPool&) = delete;
        ThreadPool& operator=(const ThreadPool&) = delete;

        unsigned int m_thread_count;
    };

    template <typename F>
    inline void parallel_for(int start_idx, int end_idx, F &&func) {
        ThreadPool::instance().parallel_for(start_idx, end_idx, std::forward<F>(func));
    }

}