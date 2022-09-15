//
// Created by Jino Park on 2022/09/15.
//

#include <mutex>

#include <logger.h>

namespace Caramel{

    class ProgressBar{
    public:
        ProgressBar(int total) : m_current(0), m_total_inv(1.0f / total) {

        }

        void increase(){
            m_lock.lock();
            m_current++;
            const float done_ratio = m_current * m_total_inv;
            std::string done_str(static_cast<int>(m_len * done_ratio), '=');
            std::string remain_str(static_cast<int>(m_len * (1 - done_ratio)), '-');
            std::cout<<"["<<done_str << remain_str << "] " << done_ratio * 100 << " %\r";
            std::cout.flush();
            m_lock.unlock();
        }

    private:
        const int m_len = 100;
        std::mutex m_lock;
        int m_current;
        const float m_total_inv;
    };
}