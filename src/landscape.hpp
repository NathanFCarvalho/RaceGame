#pragma once

#include "cgp/cgp.hpp"
#include "environment.hpp"
#include "terrain.hpp"

#include <string>

struct mountains_parameters {
    std::string texture_path;
    cgp::vec3 base_color = {0.42f, 0.25f, 0.12f};
    cgp::vec3 snow_color = {1.0f, 1.0f, 1.0f};
    float snow_start_ratio = 0.62f;
    float horizon_radius = 850.0f;
    float horizon_width = 140.0f;
    float mean_height = 85.0f;
    float noise_frequency = 0.006f;
    int noise_octaves = 6;
    int samples_around_horizon = 220;
    int samples_across_horizon = 18;
};

class mountains {
public:
    void initialize(terrain_structure const& terrain, mountains_parameters parameters);
    void display(environment_structure const& environment) const;

    void set_texture(std::string texture_path);

private:
    cgp::mesh create_mesh(terrain_structure const& terrain) const;
    float height(cgp::vec3 const& position, float ridge_factor) const;

    mountains_parameters parameters;
    cgp::mesh_drawable drawable;
};
