//
// Created by Jino on 8/6/23.
//

#include <textures.h>
#include <image.h>

namespace Caramel{

    ImageTexture::ImageTexture(const std::string &path)
    : Texture() {
        m_img = new Image(path);
    }

    ImageTexture::~ImageTexture(){
        delete m_img;
    }

    Vector3f ImageTexture::get_val(const Vector2f &uv) const{
        const auto sz = m_img->size();
        return m_img->get_pixel_value(std::clamp(static_cast<Int>(sz[0] * uv[0]), 0, static_cast<Int>(sz[0]) - 1),
                                      std::clamp(static_cast<Int>(sz[1] * (Float1 - uv[1])), 0, static_cast<Int>(sz[1]) - 1));
    }


}