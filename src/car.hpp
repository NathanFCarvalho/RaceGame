#pragma once

#include "cgp/cgp.hpp"

using namespace cgp;

struct car_structure {
	/******************************************************************************
	Struct for storing car dimension constants and for creating the wheel mesh.
 	******************************************************************************/

	const float wheel_radius = 0.18f;
	const float wheel_base = 3.0f; 

	const float wheel_tire_half_width = 0.10f; // Cylinder height
	const float wheel_forward_offset = 0.47f; // Front wheel positioning
	const float wheel_backwards_offset = 0.57f; // Back wheel positioning
	const float wheel_side_offset = 0.34f;  // Lateral wheel positioning
	const float tire_position = wheel_tire_half_width/2.0 + 0.05f;

	// Hitbox dimension constants
	const float length = 1.90f;
	const float width = 0.90f;

	cgp::mesh create_wheel_mesh() const;
	cgp::mesh create_wheel_rim_mesh() const;
};



struct movement_contants {
	/******************************************************************************
	Struct for storing car movement constants.
 	******************************************************************************/	
 
 	const float steering_speed = 0.4f;
	const float max_steering_angle = 0.55f;
	const float steering_return_speed = 0.2f;
    const float acceleration_value = 5.0f;

	// Drag constants
	const float drag_factor = 0.5f;
	const float lateral_drag_factor = 3.5f;
};


struct camera_control {
	/******************************************************************************
	Struct for managing a smooth camera constrol.
 	******************************************************************************/	

	vec3 position;
	vec3 distance_from_car;

	float back_offset = 3.0f;
	float up_offset = 0.7f;
	float camera_direction_response = 5.0f;
	float camera_position_response = 8.0f;
	float speed_threshold = 0.15f;

	camera_control();
	void position_camera(
		float dt, 
		vec3& car_forward, 
		vec3& car_position, 
		vec3& car_up
	);
};


struct car {
	/******************************************************************************
	Struct for storing car dimension constants and for creating the wheel mesh.
 	******************************************************************************/

	int idx;
	
	bool in_colision = false;
	car_structure dimensions;
	movement_contants constants;
	
	// Car movement variables
	vec3 position;
	vec3 velocity;
	vec3 acceleration;
	vec3 normal;
	vec3 forward;
	vec3 up;
	vec3 right;

	float wheel_acceleration;
	float steering_angle;
	float angular_speed;
	float collision_angular_speed;
	float wheel_spin_angle;

	int throttle_input = 0;
	int steering_input = 0;

	car();
	void update_direction_vectors();
	void update(float dt);
	std::array<vec3, 4> get_hitbox_samples() const;
};

struct player_car : car {
	camera_control camera;

	void position_camera(float dt);
	void action_keyboard(input_devices* inputs, window_structure* window);
	void verify_collisions(std::vector<car*> const& cars);
};


struct adversary_car : car {
	float lookahead_distance = 4.5f;
	float heading_gain = 1.4f;
    float adversary_target_speed = 20.0f;
    float min_corner_speed_ratio = 0.55f;
    float steering_dead_zone = 0.05f;

    void follow_direction(cgp::vec3 const& target_direction);
    void follow_target(cgp::vec3 const& target_position);
    void align_with_track_tangent(cgp::vec3 const& target_tangent);
};

void reset_car_state(car& vehicle);
float adversary_start_lateral_offset(size_t adversary_index, float spacing = 2.0f);
