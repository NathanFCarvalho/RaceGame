#include "terrain.hpp"
#include "environment.hpp"
#include "utils.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <queue>
#include <random>
#include <string>

using namespace cgp;

namespace {
struct outside_out_grid {
    int samples = 0;
    float min_x = 0.0f;
    float min_z = 0.0f;
    float step = 1.0f;
    std::vector<unsigned char> exterior;

    int vertex_count_per_side() const { return samples + 1; }
    int index(int kx, int kz) const { return kz * vertex_count_per_side() + kx; }
    vec3 point(int kx, int kz, float y = 0.0f) const
    {
        return {min_x + step * kx, y, min_z + step * kz};
    }
    bool is_exterior(int kx, int kz) const
    {
        return exterior[static_cast<size_t>(index(kx, kz))] != 0;
    }
};
}

struct lake_parameters {
    vec2 center;
    vec2 radius;
    float depth = 2.5f;
};

float track_parameter(int ku, int N)
{
    return 2.0f * Pi * ku / N;
}

/** Return a horizontal normal vector perpendicular to a track tangent. */
vec3 horizontal_normal(vec3 const& tangent)
{
    vec3 const flat_tangent = normalize(vec3{tangent.x, 0.0f, tangent.z});
    return normalize(vec3{-flat_tangent.z, 0.0f, flat_tangent.x});
}

/** Compute squared distance using only x/z coordinates. */
float squared_norm_xz(vec3 const& value)
{
    return value.x * value.x + value.z * value.z;
}

float smoothstep(float edge0, float edge1, float value)
{
    float const t = clamp_value((value - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

//  per-map lake placement; map 2 is kept inside the outside_in track region.
lake_parameters lake_settings(int map_id)
{
    if (map_id == 1)
        return {{0.0f, 0.0f}, {72.0f, 42.0f}, 2.8f};

    return {{72.0f, -6.0f}, {22.0f, 12.0f}, 1.7f};
}

float lake_normalized_radius(vec3 const& point, lake_parameters const& lake)
{
    float const dx = (point.x - lake.center.x) / lake.radius.x;
    float const dz = (point.z - lake.center.y) / lake.radius.y;
    return std::sqrt(dx * dx + dz * dz);
}

float lake_basin_offset(vec3 const& point, int map_id)
{
    lake_parameters const lake = lake_settings(map_id);
    float const r = lake_normalized_radius(point, lake);
    if (r >= 1.0f)
        return 0.0f;

    float const basin = 1.0f - smoothstep(0.0f, 1.0f, r);
    return -lake.depth * basin;
}

float lake_wave_height(vec3 const& point, int map_id)
{
    lake_parameters const lake = lake_settings(map_id);
    float const r = lake_normalized_radius(point, lake);
    float const edge_fade = 1.0f - smoothstep(0.75f, 1.0f, r);
    float const wave = 0.055f * std::sin(0.17f * point.x + 0.23f * point.z + static_cast<float>(map_id))
        + 0.035f * std::sin(0.31f * point.x - 0.19f * point.z);
    float const detail = 0.05f * noise_perlin(0.045f * vec2{point.x, point.z} + vec2{3.1f, -7.4f}, 4, 0.5f, 2.0f);
    return edge_fade * (wave + detail);
}

// simple mesh cache to avoid regenerating expensive terrain grids every run. (made with AI)
std::string terrain_cache_directory()
{
    return project::path + "assets/cache/";
}

bool ensure_terrain_cache_directory()
{
    std::error_code error;
    std::filesystem::create_directories(terrain_cache_directory(), error);
    return !error;
}

template <typename T>
void write_binary(std::ofstream& stream, T const& value)
{
    stream.write(reinterpret_cast<char const*>(&value), sizeof(T));
}

template <typename T>
bool read_binary(std::ifstream& stream, T& value)
{
    stream.read(reinterpret_cast<char*>(&value), sizeof(T));
    return static_cast<bool>(stream);
}

void write_vec2(std::ofstream& stream, vec2 const& value)
{
    write_binary(stream, value.x);
    write_binary(stream, value.y);
}

void write_vec3(std::ofstream& stream, vec3 const& value)
{
    write_binary(stream, value.x);
    write_binary(stream, value.y);
    write_binary(stream, value.z);
}

bool read_vec2(std::ifstream& stream, vec2& value)
{
    return read_binary(stream, value.x)
        && read_binary(stream, value.y);
}

bool read_vec3(std::ifstream& stream, vec3& value)
{
    return read_binary(stream, value.x)
        && read_binary(stream, value.y)
        && read_binary(stream, value.z);
}

bool save_mesh_cache(std::string const& filename, mesh const& source)
{
    if (!ensure_terrain_cache_directory())
        return false;

    std::ofstream stream(terrain_cache_directory() + filename, std::ios::binary);
    if (!stream)
        return false;

    uint32_t const magic = 0x5247434Du;
    uint32_t const version = 1u;
    uint32_t const position_count = static_cast<uint32_t>(source.position.size());
    uint32_t const color_count = static_cast<uint32_t>(source.color.size());
    uint32_t const uv_count = static_cast<uint32_t>(source.uv.size());
    uint32_t const triangle_count = static_cast<uint32_t>(source.connectivity.size());

    write_binary(stream, magic);
    write_binary(stream, version);
    write_binary(stream, position_count);
    write_binary(stream, color_count);
    write_binary(stream, uv_count);
    write_binary(stream, triangle_count);

    for (vec3 const& position : source.position)
        write_vec3(stream, position);
    for (vec3 const& color : source.color)
        write_vec3(stream, color);
    for (vec2 const& uv : source.uv)
        write_vec2(stream, uv);
    for (auto const& triangle : source.connectivity) {
        write_binary(stream, triangle.x);
        write_binary(stream, triangle.y);
        write_binary(stream, triangle.z);
    }

    return static_cast<bool>(stream);
}

bool load_mesh_cache(std::string const& filename, mesh& target)
{
    std::ifstream stream(terrain_cache_directory() + filename, std::ios::binary);
    if (!stream)
        return false;

    uint32_t magic = 0;
    uint32_t version = 0;
    uint32_t position_count = 0;
    uint32_t color_count = 0;
    uint32_t uv_count = 0;
    uint32_t triangle_count = 0;

    if (!read_binary(stream, magic)
        || !read_binary(stream, version)
        || magic != 0x5247434Du
        || version != 1u
        || !read_binary(stream, position_count)
        || !read_binary(stream, color_count)
        || !read_binary(stream, uv_count)
        || !read_binary(stream, triangle_count))
        return false;

    target = mesh();
    target.position.resize(position_count);
    target.color.resize(color_count);
    target.uv.resize(uv_count);
    target.connectivity.resize(triangle_count);

    for (vec3& position : target.position) {
        if (!read_vec3(stream, position))
            return false;
    }
    for (vec3& color : target.color) {
        if (!read_vec3(stream, color))
            return false;
    }
    for (vec2& uv : target.uv) {
        if (!read_vec2(stream, uv))
            return false;
    }
    for (auto& triangle : target.connectivity) {
        if (!read_binary(stream, triangle.x)
            || !read_binary(stream, triangle.y)
            || !read_binary(stream, triangle.z))
            return false;
    }

    target.fill_empty_field();
    target.normal_update();
    return true;
}

std::string terrain_mesh_cache_filename(
    char const* name,
    int map_id,
    int N,
    float track_width,
    float extent,
    int samples)
{
    int const width_key = static_cast<int>(std::round(track_width * 100.0f));
    int const extent_key = static_cast<int>(std::round(extent));
    return std::string("terrain_")
        + name
        + "_v5_map" + std::to_string(map_id)
        + "_N" + std::to_string(N)
        + "_w" + std::to_string(width_key)
        + "_e" + std::to_string(extent_key)
        + "_s" + std::to_string(samples)
        + ".bin";
}
/* end of mesh cache done by AI*/

/** Return whether point is inside a polygon projected onto the x/z plane. */
bool point_inside_polygon_xz(vec3 const& point, std::vector<vec3> const& polygon)
{
    if (polygon.size() < 3)
        return false;

    bool inside = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        vec3 const& pi = polygon[i];
        vec3 const& pj = polygon[j];
        bool const crosses_z = (pi.z > point.z) != (pj.z > point.z);
        if (!crosses_z)
            continue;

        float const intersection_x = (pj.x - pi.x) * (point.z - pi.z) / (pj.z - pi.z) + pi.x;
        if (point.x < intersection_x)
            inside = !inside;
    }

    return inside;
}

/** Return the signed polygon area using x/z coordinates. */
float polygon_signed_area_xz(std::vector<vec3> const& polygon)
{
    if (polygon.size() < 3)
        return 0.0f;

    float area = 0.0f;
    for (size_t i = 0; i < polygon.size(); ++i) {
        vec3 const& p = polygon[i];
        vec3 const& q = polygon[(i + 1) % polygon.size()];
        area += p.x * q.z - q.x * p.z;
    }
    return 0.5f * area;
}

/** Ensure that a polygon has the expected winding sign when projected onto x/z. */
void enforce_polygon_winding(std::vector<vec3>& polygon, float expected_sign)
{
    float const area = polygon_signed_area_xz(polygon);
    if (area * expected_sign < 0.0f)
        std::reverse(polygon.begin(), polygon.end());
}

void initialize_centerline_samples(terrain_structure const& terrain);
vec3 sampled_side_direction(terrain_structure const& terrain, int ku);

/** Build one offset border while rejecting samples that fold too close to another centerline segment. */
std::vector<vec3> create_valid_track_border(terrain_structure const& terrain, float lateral_sign)
{
    std::vector<vec3> polygon;
    polygon.reserve(static_cast<size_t>(terrain.N));

    float const half_width = 0.5f * terrain.track_width;
    float const min_centerline_distance_sq = 0.98f * half_width * half_width;

    initialize_centerline_samples(terrain);
    for (int ku = 0; ku < terrain.N; ++ku) {
        vec3 const center = terrain.centerline_samples[static_cast<size_t>(ku)];
        vec3 const side = sampled_side_direction(terrain, ku);
        vec3 const border_point = center + lateral_sign * half_width * side;

        track_projection const border_projection = terrain.closest_track_projection(border_point);
        if (squared_norm_xz(border_point - border_projection.point) >= min_centerline_distance_sq)
            polygon.push_back(border_point);
    }

    return polygon;
}

/** Cache the sampled centerline used by projection and track meshes. */
void initialize_centerline_samples(terrain_structure const& terrain)
{
    if (terrain.centerline_samples_N == terrain.N
        && terrain.centerline_samples_map_id == terrain.map_id
        && terrain.centerline_samples.size() == static_cast<size_t>(terrain.N))
        return;

    terrain.centerline_samples.resize(static_cast<size_t>(terrain.N));
    for (int ku = 0; ku < terrain.N; ++ku)
        terrain.centerline_samples[ku] = terrain.track_centerline(track_parameter(ku, terrain.N));

    terrain.centerline_samples_N = terrain.N;
    terrain.centerline_samples_map_id = terrain.map_id;
}

/* Return a side direction from the cached sampled centerline. */
vec3 sampled_side_direction(terrain_structure const& terrain, int ku)
{
    initialize_centerline_samples(terrain);
    vec3 const previous = terrain.centerline_samples[static_cast<size_t>((ku - 1 + terrain.N) % terrain.N)];
    vec3 const next = terrain.centerline_samples[static_cast<size_t>((ku + 1) % terrain.N)];
    return horizontal_normal(next - previous);
}

outside_out_grid create_outside_out_grid(terrain_structure const& terrain, int samples, float margin)
{
    initialize_centerline_samples(terrain);

    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (vec3 const& center : terrain.centerline_samples) {
        min_x = std::min(min_x, center.x);
        max_x = std::max(max_x, center.x);
        min_z = std::min(min_z, center.z);
        max_z = std::max(max_z, center.z);
    }

    float const center_x = 0.5f * (min_x + max_x);
    float const center_z = 0.5f * (min_z + max_z);
    float const half_extent = 0.5f * std::max(max_x - min_x, max_z - min_z) + margin;

    outside_out_grid grid;
    grid.samples = std::max(8, samples);
    grid.min_x = center_x - half_extent;
    grid.min_z = center_z - half_extent;
    grid.step = 2.0f * half_extent / static_cast<float>(grid.samples);

    int const side_count = grid.vertex_count_per_side();
    int const vertex_count = side_count * side_count;
    std::vector<unsigned char> passable(static_cast<size_t>(vertex_count), 0);
    grid.exterior.assign(static_cast<size_t>(vertex_count), 0);

    float const half_width = 0.5f * terrain.track_width;
    for (int kz = 0; kz < side_count; ++kz) {
        for (int kx = 0; kx < side_count; ++kx) {
            vec3 const point = grid.point(kx, kz);
            track_projection const projection = terrain.closest_track_projection(point);
            float const distance_from_border = std::abs(projection.lateral_distance) - half_width;
            passable[static_cast<size_t>(grid.index(kx, kz))] = distance_from_border > 0.0f ? 1 : 0;
        }
    }

    std::queue<int> frontier;
    auto try_seed = [&](int kx, int kz) {
        int const index = grid.index(kx, kz);
        if (passable[static_cast<size_t>(index)] == 0 || grid.exterior[static_cast<size_t>(index)] != 0)
            return;
        grid.exterior[static_cast<size_t>(index)] = 1;
        frontier.push(index);
    };

    for (int k = 0; k < side_count; ++k) {
        try_seed(k, 0);
        try_seed(k, side_count - 1);
        try_seed(0, k);
        try_seed(side_count - 1, k);
    }

    int const offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    while (!frontier.empty()) {
        int const index = frontier.front();
        frontier.pop();
        int const kx = index % side_count;
        int const kz = index / side_count;

        for (auto const& offset : offsets) {
            int const nx = kx + offset[0];
            int const nz = kz + offset[1];
            if (nx < 0 || nx >= side_count || nz < 0 || nz >= side_count)
                continue;

            int const next_index = grid.index(nx, nz);
            if (passable[static_cast<size_t>(next_index)] == 0 || grid.exterior[static_cast<size_t>(next_index)] != 0)
                continue;

            grid.exterior[static_cast<size_t>(next_index)] = 1;
            frontier.push(next_index);
        }
    }

    return grid;
}

void initialize_outside_out_hills(terrain_structure const& terrain, outside_out_grid const& grid)
{
    if (terrain.outside_out_hills_map_id == terrain.map_id && !terrain.outside_out_hills.empty())
        return;

    terrain.outside_out_hills.clear();

    int const side_count = grid.vertex_count_per_side();
    std::mt19937 generator(1337u + static_cast<unsigned int>(terrain.map_id) * 97u);
    std::uniform_int_distribution<int> grid_distribution(0, side_count - 1);
    std::uniform_real_distribution<float> height_distribution(6.0f, 26.0f);
    std::uniform_real_distribution<float> sigma_distribution(35.0f, 85.0f);

    float const half_width = 0.5f * terrain.track_width;
    int const target_hill_count = 32;
    int const max_attempts = 5000;
    for (int attempt = 0; attempt < max_attempts && terrain.outside_out_hills.size() < static_cast<size_t>(target_hill_count); ++attempt) {
        int const kx = grid_distribution(generator);
        int const kz = grid_distribution(generator);
        if (!grid.is_exterior(kx, kz))
            continue;

        vec3 const point = grid.point(kx, kz);
        track_projection const projection = terrain.closest_track_projection(point);
        float const distance_from_border = std::abs(projection.lateral_distance) - half_width;
        if (distance_from_border < 45.0f)
            continue;

        hill_parameters hill;
        hill.center = {point.x, point.z};
        hill.height = height_distribution(generator);
        hill.sigma = sigma_distribution(generator);
        terrain.outside_out_hills.push_back(hill);
    }

    terrain.outside_out_hills_map_id = terrain.map_id;
}

/** Evaluate the current map centerline at parameter u. */
vec3 terrain_structure::track_centerline(float u) const
{
    if (map_id == 1) {
        float const a = 240.0f;
        float const b = 150.0f;
        float const bend = 0.18f * std::sin(3.0f * u);
        return {
            a * std::cos(u),
            0.0f,
            b * std::sin(u) + 70.0f * bend
        };
    }

    float const t = u / (2.0f * Pi); //remap

    constexpr int NP = 26;
    static constexpr float px[NP] = {
        0.0f, 130.0f, 131.0f, 122.0f, 135.0f, 175.0f, 185.0f, 205.0f, 212.0f, 233.0f, 260.0f, 290.0f, 305.0f,
        400.0f, 458.0f, 151.0f, 170.0f, 151.0f, 149.0f, 140.0f, 138.0f, 144.0f, 135.0f, 115.0f, 92.0f, 63.0f
    };
    static constexpr float pz[NP] = {
        -0.0f, 118.0f, 117.0f, 70.0f, 52.0f, 32.0f, 16.0f, 10.0f, -18.0f, -24.0f, -8.0f, -29.0f, -16.0f,
        -22.0f, -82.0f, -60.0f, -15.0f, -16.0f, -25.0f, -40.0f, -41.0f, 4.0f, 20.0f, 28.0f, 21.0f, -22.0f
    };
    constexpr float track_scale = 1.35f;
    constexpr float track_offset_x = -230.0f;

    float const ft = t * NP;
    int   const i0 = static_cast<int>(ft) % NP;
    float const s = ft - static_cast<int>(ft);      

    int const im1 = (i0 - 1 + NP) % NP;
    int const i1 = (i0 + 1) % NP;
    int const i2 = (i0 + 2) % NP;

    float const alpha = 0.5;    
    float const t0 = 0;
    float const t1 = pow(std::sqrt((px[i0] - px[im1]) * (px[i0] - px[im1]) + (pz[i0] - pz[im1]) * (pz[i0] - pz[im1])), alpha);
    float const t2 = t1 + pow(std::sqrt((px[i1] - px[i0]) * (px[i1] - px[i0]) + (pz[i1] - pz[i0]) * (pz[i1] - pz[i0])), alpha);
    float const t3 = t2 + pow(std::sqrt((px[i2] - px[i1]) * (px[i2] - px[i1]) + (pz[i2] - pz[i1]) * (pz[i2] - pz[i1])), alpha);
	float const s_scaled = s * (t2 - t1) + t1;

    auto interpolation = [&](float p0, float p1, float p2, float p3) -> float { // Catmu0ll-Rom
		float const a0 = (t1 - s_scaled) * p0 / (t1 - t0) + (s_scaled - t0) * p1 / (t1 - t0);
		float const a1 = (t2 - s_scaled) * p1 / (t2 - t1) + (s_scaled - t1) * p2 / (t2 - t1);
		float const a2 = (t3 - s_scaled) * p2 / (t3 - t2) + (s_scaled - t2) * p3 / (t3 - t2);

		float const b0 = (t2 - s_scaled) * a0 / (t2 - t0) + (s_scaled - t0) * a1 / (t2 - t0);
		float const b1 = (t3 - s_scaled) * a1 / (t3 - t1) + (s_scaled - t1) * a2 / (t3 - t1);

		return (t2 - s_scaled) * b0 / (t2 - t1) + (s_scaled - t1) * b1 / (t2 - t1);
        };

    return { track_offset_x + track_scale * interpolation(px[im1], px[i0], px[i1], px[i2]),
             0.0f,
             track_scale * interpolation(pz[im1], pz[i0], pz[i1], pz[i2]) };
}

/** Project a world-space point onto the closest sampled segment of the track centerline. */
track_projection terrain_structure::closest_track_projection(vec3 const& point) const
{
    initialize_centerline_samples(*this);

    track_projection best;
    float best_distance = std::numeric_limits<float>::max();

    for (int ku = 0; ku < N; ++ku) {
        float const u = track_parameter(ku, N);

        vec3 const a = centerline_samples[static_cast<size_t>(ku)];
        vec3 const b = centerline_samples[static_cast<size_t>((ku + 1) % N)];
        vec3 const ab = b - a;
        vec3 const ap = point - a;

        float const ab_length_sq = squared_norm_xz(ab);
        if (ab_length_sq < 1e-6f)
            continue;

        float const t = clamp_value((ap.x * ab.x + ap.z * ab.z) / ab_length_sq, 0.0f, 1.0f);
        vec3 const projection = a + t * ab;
        float const distance_sq = squared_norm_xz(point - projection);

        if (distance_sq < best_distance) {
            vec3 const side = horizontal_normal(ab);
            vec3 const previous = centerline_samples[static_cast<size_t>((ku - 1 + N) % N)];
            vec3 const next = centerline_samples[static_cast<size_t>((ku + 2) % N)];
            vec3 const previous_tangent = normalize(vec3{a.x - previous.x, 0.0f, a.z - previous.z});
            vec3 const next_tangent = normalize(vec3{next.x - b.x, 0.0f, next.z - b.z});
            float const tangent_angle = std::atan2(
                dot(cross(previous_tangent, next_tangent), vec3{0.0f, 1.0f, 0.0f}),
                dot(previous_tangent, next_tangent));
            float const segment_length = std::sqrt(ab_length_sq);

            best_distance = distance_sq;
            best.point = projection;
            best.tangent_direction = normalize(vec3{ab.x, 0.0f, ab.z});
            best.side_direction = side;
            best.u = std::fmod(u + t * 2.0f * Pi / N, 2.0f * Pi);
            best.lateral_distance = dot(point - projection, side);
            best.signed_curvature = tangent_angle / std::max(segment_length, 1e-5f);
        }
    }

    return best;
}

/** Return the normalized horizontal tangent of the track centerline. */
cgp::vec3 terrain_structure::track_tangent(float u, float du) const
{
    vec3 const tangent = track_centerline(u + du) - track_centerline(u - du);
    return normalize_or(vec3{tangent.x, 0.0f, tangent.z}, {1.0f, 0.0f, 0.0f});
}

/** Return the normalized horizontal side direction of the track centerline. */
cgp::vec3 terrain_structure::track_side_direction(float u, float du) const
{
    return horizontal_normal(track_tangent(u, du));
}

/** Initialize which lateral side should be considered outside_out. */
void initialize_outside_out_lateral_sign(terrain_structure const& terrain)
{
    if (terrain.outside_out_sign_initialized)
        return;

    float min_x = std::numeric_limits<float>::max();
    float min_x_u = 0.0f;

    for (int ku = 0; ku < terrain.N; ++ku) {
        float const u = track_parameter(ku, terrain.N);
        vec3 const center = terrain.track_centerline(u);
        if (center.x < min_x) {
            min_x = center.x;
            min_x_u = u;
        }
    }

    float const half_width = 0.5f * terrain.track_width;
    vec3 const center = terrain.track_centerline(min_x_u);
    vec3 const side = terrain.track_side_direction(min_x_u);
    vec3 const point_plus = center + half_width * side;
    vec3 const point_minus = center - half_width * side;

    terrain.outside_out_lateral_sign = point_plus.x < point_minus.x ? 1.0f : -1.0f;
    terrain.outside_out_sign_initialized = true;
}

/** Initialize cached valid offset polygons and determine which one is the inner border. */
void initialize_track_region_polygons(terrain_structure const& terrain)
{
    if (terrain.region_polygons_initialized)
        return;

    terrain.region_polygon_positive = create_valid_track_border(terrain, 1.0f);
    terrain.region_polygon_negative = create_valid_track_border(terrain, -1.0f);

    bool const positive_inside_negative = terrain.region_polygon_positive.size() >= 3
        && terrain.region_polygon_negative.size() >= 3
        && point_inside_polygon_xz(terrain.region_polygon_positive.front(), terrain.region_polygon_negative);
    bool const negative_inside_positive = terrain.region_polygon_positive.size() >= 3
        && terrain.region_polygon_negative.size() >= 3
        && point_inside_polygon_xz(terrain.region_polygon_negative.front(), terrain.region_polygon_positive);

    if (positive_inside_negative && !negative_inside_positive)
        terrain.positive_lateral_border_is_inner = true;
    else if (negative_inside_positive && !positive_inside_negative)
        terrain.positive_lateral_border_is_inner = false;
    else
        terrain.positive_lateral_border_is_inner = terrain.outside_out_lateral_sign < 0.0f;

    enforce_polygon_winding(terrain.region_polygon_positive, 1.0f);
    enforce_polygon_winding(terrain.region_polygon_negative, 1.0f);
    terrain.region_polygons_initialized = true;
}

void terrain_structure::invalidate_region_cache() const
{
    outside_out_lateral_sign = 1.0f;
    outside_out_sign_initialized = false;
    region_polygon_positive.clear();
    region_polygon_negative.clear();
    region_polygons_initialized = false;
    positive_lateral_border_is_inner = false;
    centerline_samples.clear();
    centerline_samples_N = 0;
    centerline_samples_map_id = 0;
    outside_out_hills.clear();
    outside_out_hills_map_id = 0;
}

/** Classify a point relative to the track, preserving the closest projection for callers. */
track_region_query terrain_structure::classify_region_query(vec3 const& point) const
{
    initialize_outside_out_lateral_sign(*this);
    initialize_track_region_polygons(*this);

    track_region_query query;
    query.projection = closest_track_projection(point);

    float const half_width = 0.5f * track_width;
    if (std::abs(query.projection.lateral_distance) <= half_width) {
        query.region = track_region::on_track;
        return query;
    }

    if (region_polygon_positive.size() < 3 || region_polygon_negative.size() < 3) {
        float const lateral_sign = query.projection.lateral_distance >= 0.0f ? 1.0f : -1.0f;
        query.region = lateral_sign == outside_out_lateral_sign
            ? track_region::outside_out
            : track_region::outside_in;
        return query;
    }

    std::vector<vec3> const* inner_polygon = positive_lateral_border_is_inner
        ? &region_polygon_positive
        : &region_polygon_negative;
    std::vector<vec3> const* outer_polygon = positive_lateral_border_is_inner
        ? &region_polygon_negative
        : &region_polygon_positive;

    bool const inside_outer = point_inside_polygon_xz(point, *outer_polygon);
    bool const inside_inner = point_inside_polygon_xz(point, *inner_polygon);

    if (!inside_outer)
        query.region = track_region::outside_out;
    else if (!inside_inner)
        query.region = track_region::on_track;
    else
        query.region = track_region::outside_in;

    return query;
}

track_region terrain_structure::classify_region(vec3 const& point) const
{
    initialize_outside_out_lateral_sign(*this);
    initialize_track_region_polygons(*this);

    if (region_polygon_positive.size() < 3 || region_polygon_negative.size() < 3)
        return classify_region_query(point).region;

    std::vector<vec3> const* inner_polygon = positive_lateral_border_is_inner
        ? &region_polygon_positive
        : &region_polygon_negative;
    std::vector<vec3> const* outer_polygon = positive_lateral_border_is_inner
        ? &region_polygon_negative
        : &region_polygon_positive;

    bool const inside_outer = point_inside_polygon_xz(point, *outer_polygon);
    if (!inside_outer)
        return track_region::outside_out;

    bool const inside_inner = point_inside_polygon_xz(point, *inner_polygon);
    if (!inside_inner)
        return track_region::on_track;

    return track_region::outside_in;
}

float terrain_structure::outside_out_height(vec3 const& point) const
{
    if (outside_out_hills_map_id != map_id || outside_out_hills.empty()) {
        outside_out_grid const grid = create_outside_out_grid(*this, 180, 220.0f);
        initialize_outside_out_hills(*this, grid);
    }

    float gaussian_height = 0.0f;
    for (hill_parameters const& hill : outside_out_hills) {
        float const dx = point.x - hill.center.x;
        float const dz = point.z - hill.center.y;
        float const d = std::sqrt(dx * dx + dz * dz) / hill.sigma;
        gaussian_height += hill.height * std::exp(-d * d);
    }

    track_projection const projection = closest_track_projection(point);
    float const distance_from_track_border = std::abs(projection.lateral_distance) - 0.5f * track_width;
    float const blend = smoothstep(0.0f, 30.0f, distance_from_track_border);
    return blend * gaussian_height;
}

bool terrain_structure::is_outside_out_hill_point(vec3 const& point, float min_track_distance) const
{
    track_projection const projection = closest_track_projection(point);
    float const distance_from_track_border = std::abs(projection.lateral_distance) - 0.5f * track_width;
    if (distance_from_track_border < min_track_distance)
        return false;

    return classify_region(point) == track_region::outside_out;
}

vec3 terrain_structure::outside_out_normal(vec3 const& point) const
{
    float const delta = 2.0f;
    float const h_left = outside_out_height({point.x - delta, 0.0f, point.z});
    float const h_right = outside_out_height({point.x + delta, 0.0f, point.z});
    float const h_back = outside_out_height({point.x, 0.0f, point.z - delta});
    float const h_front = outside_out_height({point.x, 0.0f, point.z + delta});
    return normalize_or({h_left - h_right, 2.0f * delta, h_back - h_front}, {0.0f, 1.0f, 0.0f});
}

/** Return a point ahead of a track projection, used when adversaries need to return toward the centerline. */
cgp::vec3 terrain_structure::track_point_ahead(track_projection const& projection, float lookahead_distance) const
{
    float const ahead_u = projection.u + lookahead_distance / track_radius;
    return track_centerline(ahead_u);
}

/** Keep a car inside the road by testing its four hitbox samples against the track bounds. */
void terrain_structure::resolve_collision(car& car) const
{
    float const half_track_width = 0.5f * track_width;

    std::array<vec3, 4> const hitbox_samples = car.get_hitbox_samples();

    for (int iteration = 0; iteration < 2; ++iteration) {
        float max_penetration = 0.0f;
        vec3 wall_normal = {0.0f, 0.0f, 0.0f};

        for (vec3 const& hitbox_sample : hitbox_samples) {
            vec3 const sample_position = car.position + hitbox_sample;
            track_region_query const region_query = classify_region_query(sample_position);
            track_projection const& projection = region_query.projection;

            if (region_query.region == track_region::on_track)
                continue;

            float const lateral_abs = std::abs(projection.lateral_distance);
            float const penetration = lateral_abs - half_track_width;
            float const side_sign = projection.lateral_distance >= 0.0f ? 1.0f : -1.0f;
            if (penetration > max_penetration) {
                max_penetration = penetration;
                wall_normal = -side_sign * projection.side_direction;
            }
        }

        if (max_penetration <= 0.0f)
            continue;

        car.position += (max_penetration + 0.01f) * wall_normal;

        float const normal_speed = dot(car.velocity, wall_normal);
        if (normal_speed < 0.0f)
            car.velocity -= 2 * normal_speed * wall_normal;

        car.velocity *= wall_friction;
    }
}

/**************************
 Mesh definition
 **************************/

mesh terrain_structure::create_region_ground_mesh(float extent, int samples) const
{
    int const grid_count = std::max(2, samples);
    int const vertex_count = (grid_count + 1) * (grid_count + 1);

    vec3 const outside_in_color = {1.0f, 1.0f, 1.0f};
    vec3 const outside_out_color = {0.18f, 0.42f, 0.18f};
    vec3 const track_underlay_color = outside_out_color;
    float const ground_height = -0.56f;

    mesh ground_mesh;
    ground_mesh.position.resize(vertex_count);
    ground_mesh.color.resize(vertex_count);
    ground_mesh.uv.resize(vertex_count);

    for (int kz = 0; kz <= grid_count; ++kz) {
        for (int kx = 0; kx <= grid_count; ++kx) {
            float const tx = static_cast<float>(kx) / static_cast<float>(grid_count);
            float const tz = static_cast<float>(kz) / static_cast<float>(grid_count);
            float const x = -extent + 2.0f * extent * tx;
            float const z = -extent + 2.0f * extent * tz;
            int const index = kz * (grid_count + 1) + kx;
            vec3 const point = {x, ground_height, z};

            ground_mesh.position[index] = point;
            ground_mesh.uv[index] = {tx, tz};

            track_region const region = classify_region(point);
            if (region == track_region::outside_in)
                ground_mesh.color[index] = outside_in_color;
            else if (region == track_region::outside_out)
                ground_mesh.color[index] = outside_out_color;
            else
                ground_mesh.color[index] = track_underlay_color;
        }
    }

    for (int kz = 0; kz < grid_count; ++kz) {
        for (int kx = 0; kx < grid_count; ++kx) {
            unsigned int const idx = static_cast<unsigned int>(kz * (grid_count + 1) + kx);
            unsigned int const idx_right = idx + 1;
            unsigned int const idx_up = idx + static_cast<unsigned int>(grid_count + 1);
            unsigned int const idx_up_right = idx_up + 1;

            ground_mesh.connectivity.push_back({idx, idx_right, idx_up_right});
            ground_mesh.connectivity.push_back({idx, idx_up_right, idx_up});
        }
    }

    ground_mesh.fill_empty_field();
    ground_mesh.normal_update();
    return ground_mesh;
}

mesh terrain_structure::create_region_ground_mesh(track_region target_region, float extent, int samples) const
{
    int const grid_count = std::max(2, samples);
    char const* region_name = target_region == track_region::outside_in
        ? "ground_outside_in"
        : "ground_outside_out";
    std::string const cache_filename = terrain_mesh_cache_filename(region_name, map_id, N, track_width, extent, grid_count);
    mesh cached_mesh;
    if (load_mesh_cache(cache_filename, cached_mesh))
        return cached_mesh;

    vec3 const outside_in_color = {1.0f, 1.0f, 1.0f};
    vec3 const outside_out_color = {0.18f, 0.42f, 0.18f};
    vec3 const region_color = target_region == track_region::outside_in
        ? outside_in_color
        : outside_out_color;
    float const ground_height = -0.56f;

    mesh ground_mesh;

    auto make_point = [&](int kx, int kz) {
        float const tx = static_cast<float>(kx) / static_cast<float>(grid_count);
        float const tz = static_cast<float>(kz) / static_cast<float>(grid_count);
        vec3 point = {-extent + 2.0f * extent * tx, ground_height, -extent + 2.0f * extent * tz};
        if (target_region == track_region::outside_in)
            point.y += lake_basin_offset(point, map_id);
        return point;
    };

    auto add_triangle = [&](vec3 const& p0, vec3 const& p1, vec3 const& p2) {
        vec3 const centroid = (p0 + p1 + p2) / 3.0f;
        bool const touches_target_region =
            classify_region(centroid) == target_region
            || classify_region(p0) == target_region
            || classify_region(p1) == target_region
            || classify_region(p2) == target_region;
        if (!touches_target_region)
            return;

        unsigned int const idx = static_cast<unsigned int>(ground_mesh.position.size());
        ground_mesh.position.push_back(p0);
        ground_mesh.position.push_back(p1);
        ground_mesh.position.push_back(p2);
        ground_mesh.color.push_back(region_color);
        ground_mesh.color.push_back(region_color);
        ground_mesh.color.push_back(region_color);
        ground_mesh.uv.push_back({(p0.x + extent) / (2.0f * extent), (p0.z + extent) / (2.0f * extent)});
        ground_mesh.uv.push_back({(p1.x + extent) / (2.0f * extent), (p1.z + extent) / (2.0f * extent)});
        ground_mesh.uv.push_back({(p2.x + extent) / (2.0f * extent), (p2.z + extent) / (2.0f * extent)});
        ground_mesh.connectivity.push_back({idx, idx + 1, idx + 2});
    };

    for (int kz = 0; kz < grid_count; ++kz) {
        for (int kx = 0; kx < grid_count; ++kx) {
            vec3 const p00 = make_point(kx, kz);
            vec3 const p10 = make_point(kx + 1, kz);
            vec3 const p01 = make_point(kx, kz + 1);
            vec3 const p11 = make_point(kx + 1, kz + 1);
            add_triangle(p00, p10, p11);
            add_triangle(p00, p11, p01);
        }
    }

    ground_mesh.fill_empty_field();
    ground_mesh.normal_update();
    save_mesh_cache(cache_filename, ground_mesh);
    return ground_mesh;
}

// Codex-added: cached outside_out terrain mesh with Gaussian hills.
mesh terrain_structure::create_outside_out_hills_mesh() const
{
    int const samples = 180;
    std::string const cache_filename = terrain_mesh_cache_filename(
        "outside_out_hills",
        map_id,
        N,
        track_width,
        220.0f,
        samples);
    mesh cached_mesh;
    if (load_mesh_cache(cache_filename, cached_mesh))
        return cached_mesh;

    outside_out_grid const grid = create_outside_out_grid(*this, samples, 220.0f);
    initialize_outside_out_hills(*this, grid);

    int const side_count = grid.vertex_count_per_side();
    int const vertex_count = side_count * side_count;
    float const max_color_height = 32.0f;
    vec3 const low_color = {0.16f, 0.42f, 0.16f};
    vec3 const high_color = {0.46f, 0.32f, 0.16f};

    mesh hills_mesh;
    hills_mesh.position.resize(vertex_count);
    hills_mesh.color.resize(vertex_count);
    hills_mesh.uv.resize(vertex_count);

    for (int kz = 0; kz < side_count; ++kz) {
        for (int kx = 0; kx < side_count; ++kx) {
            int const index = grid.index(kx, kz);
            vec3 point = grid.point(kx, kz);
            float const height = grid.is_exterior(kx, kz) ? outside_out_height(point) : 0.0f;
            point.y = height;

            float const color_t = clamp_value(height / max_color_height, 0.0f, 1.0f);
            hills_mesh.position[index] = point;
            hills_mesh.color[index] = (1.0f - color_t) * low_color + color_t * high_color;
            hills_mesh.uv[index] = {
                static_cast<float>(kx) / static_cast<float>(grid.samples),
                static_cast<float>(kz) / static_cast<float>(grid.samples)
            };
        }
    }

    for (int kz = 0; kz < grid.samples; ++kz) {
        for (int kx = 0; kx < grid.samples; ++kx) {
            bool const exterior00 = grid.is_exterior(kx, kz);
            bool const exterior10 = grid.is_exterior(kx + 1, kz);
            bool const exterior01 = grid.is_exterior(kx, kz + 1);
            bool const exterior11 = grid.is_exterior(kx + 1, kz + 1);

            unsigned int const idx00 = static_cast<unsigned int>(grid.index(kx, kz));
            unsigned int const idx10 = static_cast<unsigned int>(grid.index(kx + 1, kz));
            unsigned int const idx01 = static_cast<unsigned int>(grid.index(kx, kz + 1));
            unsigned int const idx11 = static_cast<unsigned int>(grid.index(kx + 1, kz + 1));

            if (exterior00 && exterior10 && exterior11)
                hills_mesh.connectivity.push_back({idx00, idx10, idx11});
            if (exterior00 && exterior11 && exterior01)
                hills_mesh.connectivity.push_back({idx00, idx11, idx01});
        }
    }

    hills_mesh.fill_empty_field();
    hills_mesh.normal_update();
    save_mesh_cache(cache_filename, hills_mesh);
    return hills_mesh;
}

cgp::mesh terrain_structure::create_lake_mesh() const
{
    lake_parameters const settings = lake_settings(map_id);
    int constexpr angular_count = 192;
    int constexpr radial_count = 28;
    float constexpr lake_height = -0.48f;
    vec3 const shallow_color = {0.20f, 0.70f, 0.92f};
    vec3 const deep_color = {0.02f, 0.22f, 0.62f};

    mesh lake;
    lake.position.resize(1 + angular_count * radial_count);
    lake.color.resize(1 + angular_count * radial_count);
    lake.uv.resize(1 + angular_count * radial_count);

    auto ring_index = [](int radial_index, int angular_index) {
        return 1 + (radial_index - 1) * angular_count + angular_index;
    };

    vec3 center_point = {settings.center.x, lake_height, settings.center.y};
    center_point.y += lake_wave_height(center_point, map_id);
    lake.position[0] = center_point;
    lake.color[0] = deep_color;
    lake.uv[0] = {0.5f, 0.5f};

    for (int kr = 1; kr <= radial_count; ++kr) {
        float const radial = static_cast<float>(kr) / static_cast<float>(radial_count);
        for (int ka = 0; ka < angular_count; ++ka) {
            float const angle = 2.0f * Pi * static_cast<float>(ka) / static_cast<float>(angular_count);
            int const idx = ring_index(kr, ka);
            vec3 point = {
                settings.center.x + radial * settings.radius.x * std::cos(angle),
                lake_height,
                settings.center.y + radial * settings.radius.y * std::sin(angle)
            };

            point.y += lake_wave_height(point, map_id);
            float const color_t = 1.0f - radial;
            lake.position[idx] = point;
            lake.color[idx] = color_t * deep_color + (1.0f - color_t) * shallow_color;
            lake.uv[idx] = {
                0.5f + 0.5f * radial * std::cos(angle),
                0.5f + 0.5f * radial * std::sin(angle)
            };
        }
    }

    for (int ka = 0; ka < angular_count; ++ka) {
        unsigned int const current = static_cast<unsigned int>(ring_index(1, ka));
        unsigned int const next = static_cast<unsigned int>(ring_index(1, (ka + 1) % angular_count));
        lake.connectivity.push_back({0, current, next});
    }

    for (int kr = 2; kr <= radial_count; ++kr) {
        for (int ka = 0; ka < angular_count; ++ka) {
            unsigned int const inner_current = static_cast<unsigned int>(ring_index(kr - 1, ka));
            unsigned int const inner_next = static_cast<unsigned int>(ring_index(kr - 1, (ka + 1) % angular_count));
            unsigned int const outer_current = static_cast<unsigned int>(ring_index(kr, ka));
            unsigned int const outer_next = static_cast<unsigned int>(ring_index(kr, (ka + 1) % angular_count));

            lake.connectivity.push_back({inner_current, outer_current, outer_next});
            lake.connectivity.push_back({inner_current, outer_next, inner_next});
        }
    }

    lake.fill_empty_field();
    lake.normal_update();
    return lake;
}

/** Build the asphalt road mesh, including UV coordinates for repeated asphalt texture. */
mesh terrain_structure::create_asphalt_mesh() const
{
    initialize_centerline_samples(*this);

    mesh asphalt;
    asphalt.position.resize(4 * N);
    asphalt.uv.resize(4 * N);

    float const half_width = 0.5f * track_width;
    float const texture_tile_size = 3.0f;
    float distance_along_track = 0.0f;

    for (int ku = 0; ku < N; ++ku) {
        vec3 const center = centerline_samples[static_cast<size_t>(ku)];
        vec3 const center_next = centerline_samples[static_cast<size_t>((ku + 1) % N)];
        float const segment_length = norm(center_next - center);
        float const v = distance_along_track / texture_tile_size;
        float const v_next = (distance_along_track + segment_length) / texture_tile_size;
        float const u_left = 0.0f;
        float const u_right = track_width / texture_tile_size;

        vec3 const side = sampled_side_direction(*this, ku);
        vec3 const side_next = sampled_side_direction(*this, (ku + 1) % N);

        unsigned int const idx = 4 * ku;

        asphalt.position[idx] = center - half_width * side;
        asphalt.position[idx + 1] = center + half_width * side;
        asphalt.position[idx + 2] = center_next + half_width * side_next;
        asphalt.position[idx + 3] = center_next - half_width * side_next;

        asphalt.uv[idx] = {u_left, v};
        asphalt.uv[idx + 1] = {u_right, v};
        asphalt.uv[idx + 2] = {u_right, v_next};
        asphalt.uv[idx + 3] = {u_left, v_next};

        asphalt.connectivity.push_back({idx, idx + 1, idx + 2});
        asphalt.connectivity.push_back({idx, idx + 2, idx + 3});
        distance_along_track += segment_length;
    }

    asphalt.fill_empty_field();
    return asphalt;
}

/** Build the two vertical wall strips around the track, including UV coordinates for brick texture. */
mesh terrain_structure::create_barrier_mesh() const
{
    initialize_outside_out_lateral_sign(*this);
    initialize_track_region_polygons(*this);

    mesh barrier;
    float const brick_tile_length = 2.0f;
    float const brick_tile_height = 0.4f;

    auto add_polygon_wall = [&](std::vector<vec3> const& polygon) {
        if (polygon.size() < 2)
            return;

        unsigned int const base_index = static_cast<unsigned int>(barrier.position.size());
        float distance_along_wall = 0.0f;
        for (size_t k = 0; k < polygon.size(); ++k) {
            if (k > 0)
                distance_along_wall += norm(polygon[k] - polygon[k - 1]);

            float const uv_u = distance_along_wall / brick_tile_length;
            float const uv_v = barrier_height / brick_tile_height;
            vec3 const base = polygon[k];
            barrier.position.push_back(base);
            barrier.position.push_back(base + vec3{0.0f, barrier_height, 0.0f});
            barrier.uv.push_back({uv_u, 0.0f});
            barrier.uv.push_back({uv_u, uv_v});
        }

        for (size_t k = 0; k < polygon.size(); ++k) {
            unsigned int const idx = base_index + static_cast<unsigned int>(2 * k);
            unsigned int const next_idx = base_index + static_cast<unsigned int>(2 * ((k + 1) % polygon.size()));
            barrier.connectivity.push_back({idx, next_idx, next_idx + 1});
            barrier.connectivity.push_back({idx, next_idx + 1, idx + 1});
        }
    };

    add_polygon_wall(region_polygon_positive);
    add_polygon_wall(region_polygon_negative);

    barrier.fill_empty_field();
    return barrier;
}
