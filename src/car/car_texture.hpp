#pragma once

#include "cgp/cgp.hpp"
#include <string>

cgp::mesh make_wheel_tire_mesh(float radius);
cgp::mesh make_wheel_rim_mesh(float wheel_radius);
std::string find_supported_tire_texture();
std::string wheel_rim_texture_path();
