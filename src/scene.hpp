#pragma once

#include "cgp/cgp.hpp"
#include "environment.hpp"
#include "car.hpp"
#include "landscape.hpp"
#include "terrain.hpp"
#include <array>
#include <vector>

using cgp::mesh_drawable;

struct gui_parameters {
	bool display_frame = true;
	bool display_pines = true;
	bool top_view = false;
    bool display_mountains = true;
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
	void initialize_car_on_track(car& car, float lateral_offset = 0.0f);

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
	std::vector<adversary_car> adversaries;

	terrain_structure terrain;
    mountains terrain_mountains;

	cgp::skybox_drawable skybox;
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
