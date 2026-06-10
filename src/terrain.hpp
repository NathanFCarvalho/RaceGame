#pragma once

#include "car.hpp"
#include "cgp/cgp.hpp"

struct track_projection {
    cgp::vec3 point;
    cgp::vec3 tangent_direction;
    cgp::vec3 side_direction;
    float u = 0.0f;
    float lateral_distance = 0.0f;
    float signed_curvature = 0.0f;
};

struct terrain_structure {
    int N = 1000;
    int map_id = 1;
    float track_width = 20.0f;
    float track_radius = 40.0f;
    float barrier_height = 0.4f;
    float wall_friction = 0.85f;

    track_projection closest_track_projection(cgp::vec3 const& point) const;
    cgp::vec3 track_point_ahead(track_projection const& projection, float lookahead_distance) const;
    void resolve_collision(car& car) const;
    cgp::vec3 track_centerline(float u) const;
    cgp::vec3 track_tangent(float u, float du = 0.001f) const;
    cgp::vec3 track_side_direction(float u, float du = 0.001f) const;
    cgp::mesh create_asphalt_mesh() const;
    cgp::mesh create_barrier_mesh() const;
};
