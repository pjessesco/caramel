//
// Created by Jino Park on 2022/08/30.
//
#pragma once

#include <thread>
#include <atomic>
#include <vector>
#include <queue>
#include <numeric>
#include <semaphore>

#include <common.h>

namespace Caramel{

    constexpr int THREAD_NUM = 10;

    void parallel_for(int start_idx, int end_idx, std::function<void(int)> func){

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