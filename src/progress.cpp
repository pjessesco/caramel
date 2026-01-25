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

#include <progress.h>

#include <iostream>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/ioctl.h>
#include <unistd.h>
#endif

// This class is independent with Caramel namespace.

ProgressBar::ProgressBar(int total) : m_current(0), m_total_inv(1.0f / static_cast<float>(total)) {}

void ProgressBar::increase(){
    static int done_len = 0;
    static const int max_len = get_progress_width();
    std::lock_guard<std::mutex> lock_guard(m_lock);
    m_current++;
    const float done_ratio = m_current * m_total_inv;
    const int new_done_len = done_ratio * max_len;
    if(new_done_len == done_len){
        return;
    }
    done_len = new_done_len;

    const std::string done_str(static_cast<int>(done_len), '=');
    const std::string remain_str(static_cast<int>(max_len - done_len), '-');
    std::cout<<"["<<done_str << remain_str << "] " << static_cast<int>(done_ratio * 100) << " %\r";
    std::cout.flush();
}

int ProgressBar::get_progress_width() {
#ifdef _WIN32
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    HANDLE handle = GetStdHandle(STD_OUTPUT_HANDLE);
    if (GetConsoleScreenBufferInfo(handle, &csbi)) {
        return csbi.srWindow.Right - csbi.srWindow.Left - 9;
    }
#elif defined(__APPLE__) || defined(__linux__)
    struct winsize ws;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0) {
        return std::max(0, ws.ws_col - 10);
    }
#endif
    return 100; // by default
}