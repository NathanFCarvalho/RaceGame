#pragma once

#include "cgp/cgp.hpp"
#include "environment.hpp"
#include "car.hpp"
#include "terrain.hpp"
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

	void display_car(car const& car, cgp::vec3 const& color);
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

	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	timer_basic timer;
	mesh_drawable global_frame;          // The standard global frame
	
	car_structure car_base;
	player_car player;
	adversary_car adversary;

	terrain_structure terrain;

	cgp::mesh_drawable car_drawable;
    std::array<cgp::mesh_drawable, 4> wheel_tire_drawables;
    std::array<cgp::mesh_drawable, 4> wheel_rim_drawables;
	cgp::mesh_drawable ground;
	cgp::mesh_drawable asphalt;
	cgp::mesh_drawable barrier;

	// ****************************** //
	// Callback functions
	// ****************************** //

	void mouse_move_event();
	void mouse_click_event();
	void keyboard_event();
	void idle_frame();

};
