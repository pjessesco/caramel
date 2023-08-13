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

#include <image.h>

#include <common.h>
#include <logger.h>

#define TINYEXR_IMPLEMENTATION
#include <tinyexr.h>

#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_BMP
#define STBI_NO_PSD
#define STBI_NO_PSD
#define STBI_NO_TGA
#define STBI_NO_GIF
#define STBI_NO_PIC
#define STBI_NO_PNM
#include <stb_image.h>

namespace Caramel{

    Image::Image(unsigned int width, unsigned int height)
        : m_width(width), m_height(height) {
        m_data.resize(width*height*CHANNEL_NUM);
    }

    Image::Image(const std::string &filename) {
        if(filename.ends_with(".jpg")){
            CRM_LOG("Load .jpg image");
            read_from_jpg(filename);
        }
        else if(filename.ends_with(".png")){
            CRM_LOG("Load .png image");
            read_from_png(filename);
        }
        else if(filename.ends_with(".exr")){
            CRM_LOG("Load .exr image");
            read_from_exr(filename);
        }
        else if(filename.ends_with(".hdr")){
            CRM_LOG("load .hdr image");
            read_from_hdr(filename);
        }
        else{
            CRM_ERROR("Given filename is not supported format : " + filename);
        }
    }

    void Image::write_exr(const std::string &filename) const{

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

    Vector3f Image::get_pixel_value(int w, int h) const{
        return {m_data[(w + h * m_width)*3],
                m_data[(w + h * m_width)*3 + 1],
                m_data[(w + h * m_width)*3 + 2]};
    }

    Image::Image(unsigned int width, unsigned int height, const std::vector<Float> &data)
    : m_width{width}, m_height{height}, m_data{data} {}

    void Image::read_from_jpg(const std::string &filename){
        int width, height, channel;
        float *data = stbi_loadf(filename.c_str(), &width, &height, &channel, 0);

        if(channel != CHANNEL_NUM){
            CRM_ERROR("Support " + std::to_string(CHANNEL_NUM) + "channel images only");
        }

        std::vector<Float> data_dst;
        data_dst.resize(width * height * channel);
        memcpy(&data_dst[0], data, width * height * channel * sizeof(Float));

        stbi_image_free(data);

        m_width = width;
        m_height = height;
        m_data = data_dst;
    }

    void Image::read_from_png(const std::string &filename){
        int width, height, channel;
        float *data = stbi_loadf(filename.c_str(), &width, &height, &channel, 0);

        if(channel == 4){
            CRM_WARNING("Given image's alpha value will be multiplied");
            std::vector<Float> data_dst;
            data_dst.resize(width * height * 3);
            int j = 0;
            for(int i=0;i<width*height*channel;i++){
                if(i%4==3){
                    data_dst[j-3] *= data[i];
                    data_dst[j-2] *= data[i];
                    data_dst[j-1] *= data[i];
                    continue;
                }
                else{
                    data_dst[j] = data[i];
                    j++;
                }
            }

            stbi_image_free(data);

            m_width = width;
            m_height = height;
            m_data = data_dst;
        }
        else if(channel == CHANNEL_NUM){
            std::vector<Float> data_dst;
            data_dst.resize(width * height * channel);
            memcpy(&data_dst[0], data, width * height * channel * sizeof(Float));

            stbi_image_free(data);

            m_width = width;
            m_height = height;
            m_data = data_dst;
        }
        else{
            CRM_ERROR("Not supported image channel");
        }
    }

    void Image::read_from_exr(const std::string &filename){
        Float *out;
        int width;
        int height;
        const char *err;
        int ret = LoadEXR(&out, &width, &height, filename.c_str(), &err);

        if(ret == TINYEXR_SUCCESS){
            std::vector<Float> data_dst;
            m_data.resize(width*height*3);

            m_width = width;
            m_height = height;

            for(int i=0;i<width;i++){
                for(int j=0;j<height;j++){
                    // Ignore alpha
                    set_pixel_value(i, j, out[(i + j * m_width)*4], out[(i + j * m_width)*4 + 1], out[(i + j * m_width)*4 + 2]);
                }
            }

            free(out);
        }
        else{
            CRM_ERROR("EXR read fail : " + std::string(err));
        }

    }

    void Image::read_from_hdr(const std::string &filename){
        // TODO
    }

}
