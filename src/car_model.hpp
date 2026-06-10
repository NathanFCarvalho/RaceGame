#pragma once

#include "cgp/cgp.hpp"

#include <string>

struct car_model_meshes {
    cgp::mesh body;
    cgp::mesh windows;
};

car_model_meshes create_car_model_meshes(std::string const& obj_path, float visual_length);
