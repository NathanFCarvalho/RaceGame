#include "terrain.hpp"
#include "car/car.hpp"

using namespace cgp;

namespace {

float track_parameter(int ku, int N)
{
    return 2.0f * Pi * ku / N;
}

vec3 horizontal_normal(vec3 const& tangent)
{
    vec3 const flat_tangent = normalize(vec3{tangent.x, 0.0f, tangent.z});
    return normalize(vec3{-flat_tangent.z, 0.0f, flat_tangent.x});
}

float clamp_value(float value, float min_value, float max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

float squared_norm_xz(vec3 const& value)
{
    return value.x * value.x + value.z * value.z;
}

}

vec3 terrain_structure::track_centerline(float u) const
{
    float const t = u / (2.0f * Pi); //remap

    constexpr int NP = 26;
	//               0,     1,   1.1,    2,    3,    4,    5,    6,    7,    8,   9,  10,   11,  12,   13,   14,  15,   16,   17, 18,   19,   20,   21,   22,   23,  24
    float px[NP] = { 0,    13,  13.1, 12.2, 13.5, 17.5, 18.5, 20.5, 21.2, 23.3,  26,  29, 30.5,  40, 45.8, 15.1,  17, 15.1, 14.9, 14, 13.8, 14.4, 13.5, 11.5,  9.2, 6.3 };
    float pz[NP] = { 0, -11.8, -11.7,   -7, -5.2, -3.2, -1.6,   -1,  1.8,  2.4, 0.8, 2.9,  1.6, 2.2,  8.2,    6, 1.5,  1.6,  2.5,  4,  4.1, -0.4,   -2, -2.8, -2.1, 2.2 };

	constexpr float multiplier = 20.0f;


	for (int i = 0; i < NP; ++i) {  
        px[i] *= multiplier;
        pz[i] *= - multiplier;
    }

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

    return { interpolation(px[im1], px[i0], px[i1], px[i2]),
             0.0f,
             interpolation(pz[im1], pz[i0], pz[i1], pz[i2]) };
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

            best_distance = distance_sq;
            best.point = projection;
            best.side_direction = side;
            best.lateral_distance = dot(point - projection, side);
        }
    }

    return best;
}

void terrain_structure::resolve_collision(car_structure& car) const
{
    float const half_track_width = 0.5f * track_width;
    float const wall_friction = 0.85f;

    vec3 const forward = normalize(car.facing_direction);
    vec3 const right = normalize(cross(car.normal, forward));

    std::array<vec3, 8> const hitbox_samples = {{
        car.collision_half_length * forward + car.collision_half_width * right,
        car.collision_half_length * forward - car.collision_half_width * right,
        -car.collision_half_length * forward + car.collision_half_width * right,
        -car.collision_half_length * forward - car.collision_half_width * right,
        car.collision_half_length * forward,
        -car.collision_half_length * forward,
        car.collision_half_width * right,
        -car.collision_half_width * right,
    }};

    for (int iteration = 0; iteration < 2; ++iteration) {
        for (vec3 const& hitbox_sample : hitbox_samples) {
            vec3 const sample_position = car.position + hitbox_sample;
            track_projection const projection = closest_track_projection(sample_position);

            float const lateral_abs = std::abs(projection.lateral_distance);
            if (lateral_abs <= half_track_width)
                continue;

            float const penetration = lateral_abs - half_track_width;
            float const side_sign = projection.lateral_distance >= 0.0f ? 1.0f : -1.0f;
            vec3 const wall_normal = -side_sign * projection.side_direction;

            car.position += penetration * wall_normal;

            float const normal_speed = dot(car.velocity, wall_normal);
            if (normal_speed < 0.0f)
                car.velocity -= normal_speed * wall_normal;

            car.velocity *= wall_friction;
        }
    }
}

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
