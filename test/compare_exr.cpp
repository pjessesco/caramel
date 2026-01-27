#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#define TINYEXR_IMPLEMENTATION
#include "../ext/tinyexr.h"

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: ./compare_exr <ref.exr> <target.exr>" << std::endl;
        return 1;
    }

    const char* ref_path = argv[1];
    const char* target_path = argv[2];

    float* ref_out;
    int ref_width, ref_height;
    const char* err = nullptr;

    if (LoadEXR(&ref_out, &ref_width, &ref_height, ref_path, &err) != TINYEXR_SUCCESS) {
        if (err) {
            std::cerr << "Load Ref Error: " << err << std::endl;
            FreeEXRErrorMessage(err);
        }
        return 1;
    }

    float* target_out;
    int target_width, target_height;
    if (LoadEXR(&target_out, &target_width, &target_height, target_path, &err) != TINYEXR_SUCCESS) {
        if (err) {
            std::cerr << "Load Target Error: " << err << std::endl;
            FreeEXRErrorMessage(err);
        }
        return 1;
    }

    if (ref_width != target_width || ref_height != target_height) {
        std::cerr << "Dimension mismatch: " 
                  << ref_width << "x" << ref_height << " vs " 
                  << target_width << "x" << target_height << std::endl;
        return 1;
    }

    double ref_sum = 0.0;
    double target_sum = 0.0;
    double diff_sum_sq = 0.0;
    int num_pixels = ref_width * ref_height * 4; // RGBA

    for (int i = 0; i < num_pixels; ++i) {
        // Skip Alpha channel if 4 channels (TinyEXR loads RGBA by default usually)
        // But LoadEXR returns interleaved RGBA float array.
        if ((i + 1) % 4 == 0) continue; // Skip Alpha

        float r = ref_out[i];
        float t = target_out[i];

        ref_sum += r;
        target_sum += t;
        diff_sum_sq += (r - t) * (r - t);
    }

    int num_channels = 3;
    int num_samples = ref_width * ref_height * num_channels;

    double ref_avg = ref_sum / num_samples;
    double target_avg = target_sum / num_samples;
    double rmse = std::sqrt(diff_sum_sq / num_samples);

    std::cout << "Reference Avg: " << ref_avg << std::endl;
    std::cout << "Target Avg   : " << target_avg << std::endl;
    std::cout << "Ratio (T/R)  : " << target_avg / ref_avg << std::endl;
    std::cout << "RMSE         : " << rmse << std::endl;

    free(ref_out);
    free(target_out);

    return 0;
}
