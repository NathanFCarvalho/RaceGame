#pragma once

#include "cgp/cgp.hpp"

using namespace cgp;

struct car_structure {
	vec3 position;
	vec3 velocity;
	vec3 acceleration;
	vec3 normal;
	vec3 facing_direction;

	int throttle_input;
	int steering_input;

	float wheel_acceleration;
	float steering_angle;
	float angular_speed;
	float wheel_spin_angle;

	float steering_speed = 0.4f;
	float max_steering_angle = 0.55f;
	float steering_return_speed = 0.2f;
    float acceleration_value = 10.0f;

	float wheel_radius = 0.18f;
	float wheel_base = 5.0f;
	float drag_factor = 0.5f;
	float lateral_drag_factor = 3.5f;
	
    float collision_half_length = 0.0f;
    float collision_half_width = 0.0f;

	car_structure();
	void update(float dt);
	void set_trajectory(float t);
	void action_keyboard(input_devices* inputs, window_structure* window);
    float forward_speed() const;
    float lateral_speed() const;
};
