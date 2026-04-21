#pragma once

#include "cgp/cgp.hpp"

using namespace cgp;

struct car_structure {
    vec3 initial_position;
	vec3 current_position;
	vec3 current_velocity;
	vec3 current_acceleration;
	
	car_structure();
	void update(float dt);
	void action_keyboard(input_devices* inputs, window_structure* window);
	void accelerate();
	void decelerate();
};
