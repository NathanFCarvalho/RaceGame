#include "landscape.hpp"
#include "utils.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

using namespace cgp;

namespace {
float track_parameter(int ku, int samples)
{
    return 2.0f * Pi * ku / samples;
}

void apply_texture(mesh_drawable& drawable, std::string const& texture_path)
{
    if (!texture_path.empty())
        drawable.texture.load_and_initialize_texture_2d_on_gpu(project::path + texture_path);
}

vec3 track_center_average(terrain_structure const& terrain, int samples)
{
    vec3 center = {0.0f, 0.0f, 0.0f};
    int const N = std::max(3, samples);
    for (int ku = 0; ku < N; ++ku)
        center += terrain.track_centerline(track_parameter(ku, N));
    return center / static_cast<float>(N);
}
}

void mountains::initialize(terrain_structure const& terrain, mountains_parameters new_parameters)
{
    parameters = std::move(new_parameters);

    drawable.initialize_data_on_gpu(create_mesh(terrain));
    drawable.material.color = {1.0f, 1.0f, 1.0f};
    drawable.material.phong = {0.35f, 0.55f, 0.0f, 1.0f};
    apply_texture(drawable, parameters.texture_path);
}

void mountains::display(environment_structure const& environment) const
{
    draw(drawable, environment);
}

void mountains::set_texture(std::string texture_path)
{
    parameters.texture_path = std::move(texture_path);
    apply_texture(drawable, parameters.texture_path);
}

mesh mountains::create_mesh(terrain_structure const& terrain) const
{
    mesh surface;
    int const Nu = std::max(8, parameters.samples_around_horizon);
    int const Nv = std::max(2, parameters.samples_across_horizon);
    vec3 const center = track_center_average(terrain, Nu);

    surface.position.resize((Nu + 1) * Nv);
    surface.color.resize((Nu + 1) * Nv);
    surface.uv.resize((Nu + 1) * Nv);

    for (int ku = 0; ku <= Nu; ++ku) {
        float const angle = 2.0f * Pi * ku / Nu;
        vec3 const radial_direction = {std::cos(angle), 0.0f, std::sin(angle)};

        for (int kv = 0; kv < Nv; ++kv) {
            float const v = static_cast<float>(kv) / static_cast<float>(Nv - 1);
            float const radius = parameters.horizon_radius + v * parameters.horizon_width;
            vec3 position = center + radius * radial_direction;
            position.y = height(position, v);
            float const snow_t = clamp_value(
                (position.y - parameters.snow_start_ratio * parameters.mean_height)
                / ((1.0f - parameters.snow_start_ratio) * parameters.mean_height),
                0.0f,
                1.0f);
            vec3 const color = (1.0f - snow_t) * parameters.base_color + snow_t * parameters.snow_color;

            int const idx = ku * Nv + kv;
            surface.position[idx] = position;
            surface.color[idx] = color;
            surface.uv[idx] = {static_cast<float>(ku) / Nu, v};
        }
    }

    for (int ku = 0; ku < Nu; ++ku) {
        for (int kv = 0; kv < Nv - 1; ++kv) {
            unsigned int const idx = ku * Nv + kv;
            unsigned int const next_idx = (ku + 1) * Nv + kv;
            surface.connectivity.push_back({idx, next_idx, next_idx + 1});
            surface.connectivity.push_back({idx, next_idx + 1, idx + 1});
        }
    }

    surface.fill_empty_field();
    surface.normal_update();
    return surface;
}

float mountains::height(vec3 const& position, float ridge_factor) const
{
    vec2 const noise_position = parameters.noise_frequency * vec2{position.x, position.z};
    float const detail_noise = 0.5f + 0.5f * noise_perlin(noise_position, parameters.noise_octaves, 0.52f, 2.15f);
    float const large_noise = 0.5f + 0.5f * noise_perlin(0.32f * noise_position + vec2{17.0f, -9.0f}, 4, 0.55f, 2.0f);
    float const ridge = std::pow(std::max(0.0f, 0.72f * detail_noise + 0.28f * large_noise), 2.45f);
    float const height_variation = 0.45f + 1.35f * std::pow(large_noise, 1.6f);
    float const edge_fade = std::sin(Pi * clamp_value(ridge_factor, 0.0f, 1.0f));
    float const base_slope = 0.35f + 0.65f * ridge_factor;
    return parameters.mean_height * ridge * height_variation * edge_fade * base_slope;
}
