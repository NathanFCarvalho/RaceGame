#pragma once

#include "car.hpp"
#include "cgp/cgp.hpp"

#include <vector>

struct track_projection {
    cgp::vec3 point;
    cgp::vec3 tangent_direction;
    cgp::vec3 side_direction;
    float u = 0.0f;
    float lateral_distance = 0.0f;
    float signed_curvature = 0.0f;
};

enum class track_region {
    on_track,
    outside_in,
    outside_out
};

struct track_region_query {
    track_region region = track_region::outside_out;
    track_projection projection;
};

struct hill_parameters {
    cgp::vec2 center;
    float height = 0.0f;
    float sigma = 1.0f;
};

struct terrain_structure {
    int N = 2000;
    int map_id = 1;
    float track_width = 14.0f;
    float track_radius = 40.0f;
    float barrier_height = 0.4f;
    float wall_friction = 0.85f;
    mutable float outside_out_lateral_sign = 1.0f;
    mutable bool outside_out_sign_initialized = false;
    mutable std::vector<cgp::vec3> region_polygon_positive;
    mutable std::vector<cgp::vec3> region_polygon_negative;
    mutable bool region_polygons_initialized = false;
    mutable bool positive_lateral_border_is_inner = false;
    mutable std::vector<cgp::vec3> centerline_samples;
    mutable int centerline_samples_N = 0;
    mutable int centerline_samples_map_id = 0;
    mutable std::vector<hill_parameters> outside_out_hills;
    mutable int outside_out_hills_map_id = 0;

    track_projection closest_track_projection(cgp::vec3 const& point) const;
    track_region classify_region(cgp::vec3 const& point) const;
    track_region_query classify_region_query(cgp::vec3 const& point) const;
    void invalidate_region_cache() const;
    cgp::vec3 track_point_ahead(track_projection const& projection, float lookahead_distance) const;
    void resolve_collision(car& car) const;
    cgp::vec3 track_centerline(float u) const;
    cgp::vec3 track_tangent(float u, float du = 0.001f) const;
    cgp::vec3 track_side_direction(float u, float du = 0.001f) const;
    float outside_out_height(cgp::vec3 const& point) const;
    bool is_outside_out_hill_point(cgp::vec3 const& point, float min_track_distance = 45.0f) const;
    cgp::vec3 outside_out_normal(cgp::vec3 const& point) const;
    cgp::mesh create_region_ground_mesh(float extent = 900.0f, int samples = 160) const;
    cgp::mesh create_region_ground_mesh(track_region target_region, float extent = 900.0f, int samples = 160) const;
    cgp::mesh create_outside_out_hills_mesh() const;
    cgp::mesh create_lake_mesh() const;
    cgp::mesh create_asphalt_mesh() const;
    cgp::mesh create_barrier_mesh() const;
};
