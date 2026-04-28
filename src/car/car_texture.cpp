#include "car_texture.hpp"
#include "../environment.hpp"
#include <filesystem>

using namespace cgp;

mesh make_wheel_tire_mesh(float radius)
{
    float const half_width = 0.08f;
    return mesh_primitive_cylinder(radius, {0.0f, 0.0f, -half_width}, {0.0f, 0.0f, half_width}, 2, 32, false);
}

mesh make_wheel_rim_mesh(float wheel_radius)
{
    float const rim_radius = 0.62f * wheel_radius;
    float const inset = 0.055f;

    mesh rim = mesh_primitive_disc(rim_radius, {0.0f, 0.0f, -inset}, {0.0f, 0.0f, -1.0f}, 32);
    rim.push_back(mesh_primitive_disc(rim_radius, {0.0f, 0.0f, inset}, {0.0f, 0.0f, 1.0f}, 32));
    rim.fill_empty_field();
    return rim;
}

std::string find_supported_tire_texture()
{
    std::string const tire_png = project::path + "assets/tire.png";
    if (std::filesystem::exists(tire_png))
        return tire_png;

    std::string const tire_jpg = project::path + "assets/tire.jpg";
    if (std::filesystem::exists(tire_jpg))
        return tire_jpg;

    return "";
}

std::string wheel_rim_texture_path()
{
    return project::path + "assets/wheel.png";
}
