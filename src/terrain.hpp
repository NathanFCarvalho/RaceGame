#pragma once

#include "cgp/cgp.hpp"

struct car_structure;

struct track_projection {
    cgp::vec3 point;
    cgp::vec3 side_direction;
    float lateral_distance = 0.0f;
};

struct terrain_structure {
    int N = 500;
    float track_width = 10.0f;
    float barrier_height = 0.4f;

    cgp::vec3 track_centerline(float u) const;
    track_projection closest_track_projection(cgp::vec3 const& point) const;
    void resolve_collision(car_structure& car) const;
    cgp::mesh create_asphalt_mesh() const;
    cgp::mesh create_barrier_mesh() const;
};
