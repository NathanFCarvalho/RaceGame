#pragma once

#include "cgp/cgp.hpp"
#include "environment.hpp"
#include "car/car.hpp"
#include "car/car_texture.hpp"
#include <array>

using cgp::mesh_drawable;

struct gui_parameters {
	bool display_frame = true;
	bool display_pines = true;
};

// The structure of the custom scene
struct scene_structure : cgp::scene_inputs_generic {

	// ****************************** //
	// Standard Functions
	// ****************************** //

	void initialize();
	void display_frame();
	void display_gui();
	void display_info();

	void display_car(float dt);
	void position_camera();

	// ****************************** //
	// Context
	// ****************************** //

	environment_structure environment;   // Standard environment controler
	window_structure window;
	input_devices inputs;                // Storage for inputs status (mouse, keyboard, window dimension)
	gui_parameters gui;                  // Standard GUI element storage

	// ****************************** //
	// Camera controler
	// ****************************** //

	camera_controller_orbit_euler camera_control;
	camera_projection_perspective camera_projection;

	vec3 distance_from_car;

	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	timer_basic timer;
	mesh_drawable global_frame;          // The standard global frame
	
	car_structure car;

	cgp::mesh_drawable car_drawable;
    std::array<cgp::mesh_drawable, 4> wheel_tire_drawables;
    std::array<cgp::mesh_drawable, 4> wheel_rim_drawables;
	cgp::mesh_drawable ground;

	// ****************************** //
	// Callback functions
	// ****************************** //

	void mouse_move_event();
	void mouse_click_event();
	void keyboard_event();
	void idle_frame();

};
