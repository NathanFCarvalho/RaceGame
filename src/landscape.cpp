#include "landscape.hpp"
#include "utils.hpp"

#include <algorithm>
#include <cmath>
#include <random>
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

float smoothstep(float edge0, float edge1, float value)
{
    float const t = clamp_value((value - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

void add_branch_segment(mesh& tree, vec3 const& start, vec3 const& end, float radius_start, float radius_end, vec3 const& color)
{
    int const section_count = 8;
    vec3 const axis = normalize_or(end - start, {0.0f, 1.0f, 0.0f});
    vec3 reference = std::abs(dot(axis, {0.0f, 1.0f, 0.0f})) > 0.92f ? vec3{1.0f, 0.0f, 0.0f} : vec3{0.0f, 1.0f, 0.0f};
    vec3 const side = normalize_or(cross(axis, reference), {1.0f, 0.0f, 0.0f});
    vec3 const forward = normalize_or(cross(side, axis), {0.0f, 0.0f, 1.0f});
    unsigned int const base_index = static_cast<unsigned int>(tree.position.size());

    for (int k = 0; k < section_count; ++k) {
        float const angle = 2.0f * Pi * k / static_cast<float>(section_count);
        vec3 const radial = std::cos(angle) * side + std::sin(angle) * forward;
        tree.position.push_back(start + radius_start * radial);
        tree.position.push_back(end + radius_end * radial);
        tree.color.push_back(color);
        tree.color.push_back(color);
        tree.uv.push_back({static_cast<float>(k) / section_count, 0.0f});
        tree.uv.push_back({static_cast<float>(k) / section_count, 1.0f});
    }

    for (int k = 0; k < section_count; ++k) {
        unsigned int const idx = base_index + static_cast<unsigned int>(2 * k);
        unsigned int const next = base_index + static_cast<unsigned int>(2 * ((k + 1) % section_count));
        tree.connectivity.push_back({idx, next, next + 1});
        tree.connectivity.push_back({idx, next + 1, idx + 1});
    }
}

void add_leaf_cluster(mesh& tree, vec3 const& center, float radius, vec3 const& color)
{
    unsigned int const idx = static_cast<unsigned int>(tree.position.size());
    tree.position.push_back(center + vec3{0.0f, radius, 0.0f});
    tree.position.push_back(center + vec3{radius, 0.0f, 0.0f});
    tree.position.push_back(center + vec3{0.0f, 0.0f, radius});
    tree.position.push_back(center + vec3{-radius, 0.0f, 0.0f});
    tree.position.push_back(center + vec3{0.0f, 0.0f, -radius});
    tree.position.push_back(center + vec3{0.0f, -radius, 0.0f});

    for (int k = 0; k < 6; ++k) {
        tree.color.push_back(color);
        tree.uv.push_back({0.5f, 0.5f});
    }

    tree.connectivity.push_back({idx, idx + 1, idx + 2});
    tree.connectivity.push_back({idx, idx + 2, idx + 3});
    tree.connectivity.push_back({idx, idx + 3, idx + 4});
    tree.connectivity.push_back({idx, idx + 4, idx + 1});
    tree.connectivity.push_back({idx + 5, idx + 2, idx + 1});
    tree.connectivity.push_back({idx + 5, idx + 3, idx + 2});
    tree.connectivity.push_back({idx + 5, idx + 4, idx + 3});
    tree.connectivity.push_back({idx + 5, idx + 1, idx + 4});
}

// groups several simple leaf clusters to make each procedural tree fuller. (made by AI)
void add_leaf_cloud(mesh& tree, std::mt19937& generator, vec3 const& center, float radius, vec3 const& color)
{
    std::uniform_real_distribution<float> offset_distribution(-0.65f, 0.65f);
    std::uniform_real_distribution<float> scale_distribution(0.55f, 0.95f);

    add_leaf_cluster(tree, center, radius, color);
    for (int k = 0; k < 4; ++k) {
        vec3 const offset = radius * vec3{
            offset_distribution(generator),
            0.55f * offset_distribution(generator),
            offset_distribution(generator)
        };
        add_leaf_cluster(tree, center + offset, radius * scale_distribution(generator), color);
    }
}

void add_fractal_branch(
    mesh& tree,
    std::mt19937& generator,
    vec3 const& start,
    vec3 const& direction,
    float length,
    float radius,
    int depth)
{
    std::uniform_real_distribution<float> angle_distribution(0.0f, 2.0f * Pi);
    std::uniform_real_distribution<float> spread_distribution(0.28f, 0.58f);
    std::uniform_real_distribution<float> length_distribution(0.62f, 0.78f);
    std::uniform_real_distribution<float> color_distribution(0.0f, 1.0f);

    vec3 const dir = normalize_or(direction, {0.0f, 1.0f, 0.0f});
    vec3 const end = start + length * dir;
    add_branch_segment(tree, start, end, radius, 0.68f * radius, {0.34f, 0.19f, 0.08f});

    if (depth <= 0) {
        float const green_shift = color_distribution(generator);
        add_leaf_cloud(tree, generator, end, 3.2f * radius, {0.08f + 0.05f * green_shift, 0.34f + 0.22f * green_shift, 0.08f});
        return;
    }

    if (depth <= 2) {
        float const green_shift = color_distribution(generator);
        add_leaf_cloud(
            tree,
            generator,
            start + 0.78f * (end - start),
            1.9f * radius,
            {0.07f + 0.05f * green_shift, 0.30f + 0.20f * green_shift, 0.07f});
    }

    vec3 reference = std::abs(dot(dir, {0.0f, 1.0f, 0.0f})) > 0.92f ? vec3{1.0f, 0.0f, 0.0f} : vec3{0.0f, 1.0f, 0.0f};
    vec3 const side = normalize_or(cross(dir, reference), {1.0f, 0.0f, 0.0f});
    vec3 const forward = normalize_or(cross(side, dir), {0.0f, 0.0f, 1.0f});
    int const child_count = depth >= 2 ? 3 : 2;

    for (int child = 0; child < child_count; ++child) {
        float const angle = angle_distribution(generator) + 2.0f * Pi * child / static_cast<float>(child_count);
        float const spread = spread_distribution(generator);
        vec3 const lateral = std::cos(angle) * side + std::sin(angle) * forward;
        vec3 const child_direction = normalize_or((1.0f - spread) * dir + spread * lateral + vec3{0.0f, 0.18f, 0.0f}, dir);
        add_fractal_branch(
            tree,
            generator,
            end,
            child_direction,
            length * length_distribution(generator),
            radius * 0.64f,
            depth - 1);
    }
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
            float constexpr mountain_base_height = -18.0f;
            bool const first_triangle_visible =
                surface.position[idx].y > mountain_base_height + 1e-4f
                && surface.position[next_idx].y > mountain_base_height + 1e-4f
                && surface.position[next_idx + 1].y > mountain_base_height + 1e-4f;
            bool const second_triangle_visible =
                surface.position[idx].y > mountain_base_height + 1e-4f
                && surface.position[next_idx + 1].y > mountain_base_height + 1e-4f
                && surface.position[idx + 1].y > mountain_base_height + 1e-4f;

            if (first_triangle_visible)
                surface.connectivity.push_back({idx, next_idx, next_idx + 1});
            if (second_triangle_visible)
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
    float const root_fade = smoothstep(0.18f, 0.42f, ridge_factor);
    float const back_fade = 1.0f - smoothstep(0.82f, 1.0f, ridge_factor);
    float const edge_fade = root_fade * back_fade;
    float const base_slope = 0.35f + 0.65f * ridge_factor;
    float constexpr mountain_base_height = -18.0f;
    float const mountain_height = parameters.mean_height * ridge * height_variation * edge_fade * base_slope;
    float const root_offset = mountain_base_height * (1.0f - edge_fade);
    return mountain_height + root_offset;
}

mesh create_fractal_tree_mesh(unsigned int seed)
{
    std::mt19937 generator(seed);
    mesh tree;
    add_fractal_branch(tree, generator, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, 10.0f, 0.8f, 4);
    tree.fill_empty_field();
    tree.normal_update();
    return tree;
}

mesh create_bush_billboard_mesh(float width, float height)
{
    mesh bush;
    int const plane_count = 3;
    for (int k = 0; k < plane_count; ++k) {
        float const angle = Pi * k / static_cast<float>(plane_count);
        vec3 const side = 0.5f * width * vec3{std::cos(angle), 0.0f, std::sin(angle)};
        unsigned int const idx = static_cast<unsigned int>(bush.position.size());

        bush.position.push_back(-side);
        bush.position.push_back(side);
        bush.position.push_back(side + vec3{0.0f, height, 0.0f});
        bush.position.push_back(-side + vec3{0.0f, height, 0.0f});

        bush.uv.push_back({0.0f, 0.0f});
        bush.uv.push_back({1.0f, 0.0f});
        bush.uv.push_back({1.0f, 1.0f});
        bush.uv.push_back({0.0f, 1.0f});

        for (int vertex = 0; vertex < 4; ++vertex)
            bush.color.push_back({1.0f, 1.0f, 1.0f});

        bush.connectivity.push_back({idx, idx + 1, idx + 2});
        bush.connectivity.push_back({idx, idx + 2, idx + 3});
    }

    bush.fill_empty_field();
    bush.normal_update();
    return bush;
}
