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

#include <common.h>
#include <image.h>
#include <logger.h>

#define TINYEXR_IMPLEMENTATION
#include <tinyexr.h>

namespace Caramel{

    Image::Image(unsigned int width, unsigned int height)
        : m_width(width), m_height(height) {
        m_data.reserve(width*height*CHANNEL_NUM);
    }

    void Image::write_exr(const std::string &filename){

        EXRHeader header;
        InitEXRHeader(&header);

        EXRImage image;
        InitEXRImage(&image);

        std::vector<Float> r(m_width*m_height);
        std::vector<Float> g(m_width*m_height);
        std::vector<Float> b(m_width*m_height);

        for(int i=0;i<m_width*m_height;i++){
            r[i] = m_data[i*CHANNEL_NUM+0];
            g[i] = m_data[i*CHANNEL_NUM+1];
            b[i] = m_data[i*CHANNEL_NUM+2];
        }

        Float *image_ptr[CHANNEL_NUM];
        image_ptr[0] = &b[0];
        image_ptr[1] = &g[0];
        image_ptr[2] = &r[0];

        image.images = (unsigned char**)image_ptr;
        image.width = m_width;
        image.height = m_height;

        header.num_channels = CHANNEL_NUM;
        header.channels = (EXRChannelInfo *)malloc(sizeof(EXRChannelInfo) * header.num_channels);
        strncpy(header.channels[0].name, "B", 255); header.channels[0].name[strlen("B")] = '\0';
        strncpy(header.channels[1].name, "G", 255); header.channels[1].name[strlen("G")] = '\0';
        strncpy(header.channels[2].name, "R", 255); header.channels[2].name[strlen("R")] = '\0';

        header.pixel_types = (int*)malloc(sizeof(int)*header.num_channels);
        header.requested_pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
        for (int i = 0; i < header.num_channels; i++) {
            // TINYEXR doesn't support double precision
            header.pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
            header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
        }

        const char* err = nullptr;
        int ret = SaveEXRImageToFile(&image, &header, filename.c_str(), &err);
        if (ret != TINYEXR_SUCCESS) {
            fprintf(stderr, "Save EXR err: %s\n", err);
            CRM_ERROR("Error occurs while save "+filename+" : "+err);
            FreeEXRErrorMessage(err);
        }
        CRM_LOG("Saved exr file :"+filename);

        free(header.channels);
        free(header.pixel_types);
        free(header.requested_pixel_types);
    }

    void Image::set_pixel_value(int w, int h, Float r, Float g, Float b) {
        if(w<0 || m_width<=w || h<0 || m_height<=h){
            CRM_ERROR("unavailable image pixel position");
        }

        m_data[(w + h * m_width)*3] = r;
        m_data[(w + h * m_width)*3 + 1] = g;
        m_data[(w + h * m_width)*3 + 2] = b;

    }

}
