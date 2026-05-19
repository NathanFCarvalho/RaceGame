#include "terrain.hpp"
#include "utils.hpp"

using namespace cgp;

float track_parameter(int ku, int N)
{
    return 2.0f * Pi * ku / N;
}

vec3 horizontal_normal(vec3 const& tangent)
{
    vec3 const flat_tangent = normalize(vec3{tangent.x, 0.0f, tangent.z});
    return normalize(vec3{-flat_tangent.z, 0.0f, flat_tangent.x});
}

float squared_norm_xz(vec3 const& value)
{
    return value.x * value.x + value.z * value.z;
}

vec3 terrain_structure::track_centerline(float u) const
{
    return {
        track_radius * std::cos(u),
        0.0f,
        track_radius * std::sin(u) - track_radius
    };
}

track_projection terrain_structure::closest_track_projection(vec3 const& point) const
{
    track_projection best;
    float best_distance = std::numeric_limits<float>::max();

    for (int ku = 0; ku < N; ++ku) {
        float const u = track_parameter(ku, N);
        float const u_next = track_parameter((ku + 1) % N, N);

        vec3 const a = track_centerline(u);
        vec3 const b = track_centerline(u_next);
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
            vec3 const previous = track_centerline(track_parameter((ku - 1 + N) % N, N));
            vec3 const next = track_centerline(track_parameter((ku + 2) % N, N));
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

cgp::vec3 terrain_structure::track_point_ahead(track_projection const& projection, float lookahead_distance) const
{
    float const ahead_u = projection.u + lookahead_distance / track_radius;
    return track_centerline(ahead_u);
}

void terrain_structure::resolve_collision(car& car) const
{
    float const half_track_width = 0.5f * track_width;

    vec3 const forward = normalize(car.facing_direction);
    vec3 const right = normalize(cross(car.normal, forward));

    float half_length = car.dimensions.collision_half_length;
    std::array<vec3, 8> const hitbox_samples = {{
        half_length * forward + half_length * right,
        half_length * forward - half_length * right,
        -half_length * forward + half_length * right,
        -half_length * forward - half_length * right,
        half_length * forward,
        -half_length * forward,
        half_length * right,
        -half_length * right,
    }};

    for (int iteration = 0; iteration < 2; ++iteration) {
        for (vec3 const& hitbox_sample : hitbox_samples) {
            vec3 const sample_position = car.position + hitbox_sample;
            track_projection const projection = closest_track_projection(sample_position);

            float const lateral_abs = std::abs(projection.lateral_distance);
            if (lateral_abs <= half_track_width)
                continue;

            float const side_sign = projection.lateral_distance >= 0.0f ? 1.0f : -1.0f;
            vec3 const wall_normal = -side_sign * projection.side_direction;

            float const normal_speed = dot(car.velocity, wall_normal);
            if (normal_speed < 0.0f)
                car.velocity -= 2 * normal_speed * wall_normal;

            car.velocity *= wall_friction;
        }
    }
}

/**************************
 Mesh definition
 **************************/

mesh terrain_structure::create_asphalt_mesh() const
{
    mesh asphalt;
    asphalt.position.resize(4 * N);

    float const half_width = 0.5f * track_width;
    float const du = 0.001f;

    for (int ku = 0; ku < N; ++ku) {
        float const u = track_parameter(ku, N);
        float const u_next = track_parameter((ku + 1) % N, N);

        vec3 const center = track_centerline(u);
        vec3 const center_next = track_centerline(u_next);

        vec3 const tangent = track_centerline(u + du) - track_centerline(u - du);
        vec3 const tangent_next = track_centerline(u_next + du) - track_centerline(u_next - du);

        vec3 const side = horizontal_normal(tangent);
        vec3 const side_next = horizontal_normal(tangent_next);

        unsigned int const idx = 4 * ku;

        asphalt.position[idx] = center - half_width * side;
        asphalt.position[idx + 1] = center + half_width * side;
        asphalt.position[idx + 2] = center_next + half_width * side_next;
        asphalt.position[idx + 3] = center_next - half_width * side_next;

        asphalt.connectivity.push_back({idx, idx + 1, idx + 2});
        asphalt.connectivity.push_back({idx, idx + 2, idx + 3});
    }

    asphalt.fill_empty_field();
    return asphalt;
}

mesh terrain_structure::create_barrier_mesh() const
{
    mesh barrier;
    barrier.position.resize(4 * N);

    float const half_width = 0.5f * track_width;
    float const du = 0.001f;

    for (int ku = 0; ku < N; ++ku) {
        float const u = track_parameter(ku, N);
        vec3 const center = track_centerline(u);
        vec3 const tangent = track_centerline(u + du) - track_centerline(u - du);
        vec3 const side = horizontal_normal(tangent);

        vec3 const left_base = center - half_width * side;
        vec3 const right_base = center + half_width * side;

        barrier.position[4 * ku] = left_base;
        barrier.position[4 * ku + 1] = left_base + vec3{0.0f, barrier_height, 0.0f};
        barrier.position[4 * ku + 2] = right_base;
        barrier.position[4 * ku + 3] = right_base + vec3{0.0f, barrier_height, 0.0f};
    }

    for (int ku = 0; ku < N; ++ku) {
        unsigned int const idx = 4 * ku;
        unsigned int const next_idx = 4 * ((ku + 1) % N);

        barrier.connectivity.push_back({idx, next_idx, next_idx + 1});
        barrier.connectivity.push_back({idx, next_idx + 1, idx + 1});

        barrier.connectivity.push_back({idx + 2, idx + 3, next_idx + 3});
        barrier.connectivity.push_back({idx + 2, next_idx + 3, next_idx + 2});
    }

    barrier.fill_empty_field();
    return barrier;
}
