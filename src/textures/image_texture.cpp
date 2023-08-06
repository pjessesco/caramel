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

        return m_img->get_pixel_value(m_img->m_width * uv[0],
                                      m_img->m_height * uv[1]);
    }


}