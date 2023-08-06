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

        return vec3f_zero;
    }


}