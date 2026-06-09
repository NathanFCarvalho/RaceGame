#pragma once

#include "cgp/cgp.hpp"

using namespace cgp;


struct car_structure {
	// Car dimension constants
	float length = 1.90f;
	float width = 0.90f;
	float wheel_radius = 0.18f;
	float wheel_base = 3.0f; 

	float wheel_tire_half_width = 0.08f;
	float wheel_forward_offset = 0.27f;
	float wheel_side_offset = 0.24f;

	// Hitbox dimension constants
	float hitbox_margin = 0.1f;
    float collision_half_length = std::max(length/2, wheel_forward_offset + wheel_radius) + hitbox_margin;
    float collision_half_width = std::max(width/2, wheel_side_offset + wheel_tire_half_width) + hitbox_margin;

	cgp::mesh create_wheel_mesh() const;
	cgp::mesh create_wheel_rim_mesh() const;
};



struct movement_contants {
	// Car movement constants
	float steering_speed = 0.4f;
	float max_steering_angle = 0.55f;
	float steering_return_speed = 0.2f;
    float acceleration_value = 5.0f;

	// Drag constants
	float drag_factor = 0.5f;
	float lateral_drag_factor = 3.5f;
};


struct camera_control {
	vec3 position;

	vec3 distance_from_car;
	vec3 camera_smoothed_direction;
	vec3 camera_smoothed_position;

	float back_offset = 3.0f;
	float up_offset = 0.7f;
	float camera_direction_response = 5.0f;
	float camera_position_response = 8.0f;
	float speed_threshold = 0.15f;

	camera_control();
	void position_camera(
		float dt, 
		vec3& car_facing_direction, 
		vec3& car_position, 
		vec3& car_up
	);
};


struct car {
	int idx;
	
	bool in_colision = false;
	car_structure dimensions;
	movement_contants constants;
	
	// Car movement variables
	vec3 position;
	vec3 velocity;
	vec3 acceleration;
	vec3 normal;
	vec3 facing_direction;

	float wheel_acceleration;
	float steering_angle;
	float angular_speed;
	float collision_angular_speed;
	float wheel_spin_angle;

	int throttle_input = 0;
	int steering_input = 0;

	car();
	void update(float dt);
	float forward_speed() const;
    float lateral_speed() const;
	std::array<vec3, 4> get_hitbox_samples() const;
};


struct player_car : car {
	camera_control camera;

	void position_camera(float dt);
	void action_keyboard(input_devices* inputs, window_structure* window);
	void verify_collisions(std::vector<car*> const& cars);
};


struct adversary_car : car {
	float lookahead_distance = 0.5f;
	float heading_gain = 1.4f;
    float adversary_target_speed = 20.0f;
    float min_corner_speed_ratio = 0.55f;
    float steering_dead_zone = 0.03f;

    void follow_target(cgp::vec3 const& target_position);
};
