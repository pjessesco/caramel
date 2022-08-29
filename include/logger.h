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

#pragma once

#include <iostream>
#include <string>
#include <filesystem>

namespace Caramel {

    class Logger {
    public:
        inline static void print_log(const std::string &file,
                                     int line,
                                     const std::string &msg) {
            std::cout << log_prefix() << "In " << get_filename(file) << " " << line << " : " << msg << std::endl;
        }

        inline static void print_err(const std::string &file,
                                     int line,
                                     const std::string &msg) {
            std::cout << err_prefix() << "In " << get_filename(file) << " " << line << " : " << msg << std::endl;
        }

        inline static void print_warn(const std::string &file,
                                      int line,
                                      const std::string &msg) {
            std::cout << warn_prefix() << "In " << get_filename(file) << " " << line << " : " << msg << std::endl;
        }

    private:
        inline static std::string current_time_string() {
            std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

            auto current_time = std::localtime(&now);
            std::string hour = std::to_string(current_time->tm_hour);
            std::string min = std::to_string(current_time->tm_min);
            std::string sec = std::to_string(current_time->tm_sec);

            return "[" + hour + ":" + min + ":" + sec + "]";
        }

        inline static std::string log_prefix() {
            return current_time_string() + "\033[1;30m[LOG]\033[0m ";
        }

        inline static std::string err_prefix() {
            return current_time_string() + "\033[1;31m[ERR]\033[0m ";
        }
        inline static std::string warn_prefix() {
            return current_time_string() + "\033[1;31m[WARN]\033[0m ";
        }

        inline static std::string get_filename(const std::string &str) {
            std::filesystem::path path(str);
            return (path.parent_path().filename() / path.filename()).string();
        }
    };

    enum class LogType {
        LOG,
        ERR,
        WARN
    };
}

#define ERROR(msg) do {                                  \
    Caramel::Logger::print_err(__FILE__, __LINE__, msg); \
    throw std::runtime_error("error");                   \
} while (0);

#define LOG(msg) do {                                    \
    Caramel::Logger::print_log(__FILE__, __LINE__, msg); \
} while (0);

#define WARNING(msg) do {                                 \
    Caramel::Logger::print_warn(__FILE__, __LINE__, msg); \
} while (0);


#define NOT_IMPLEMENTED() do{         \
    MESSAGE(LogType::ERR, "NOT IMPLEMENTED") \
} while(0);

