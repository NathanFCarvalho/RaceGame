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
	bool display_hitbox = true;
	bool display_pines = true;
	bool top_view = false;
    bool display_mountains = true;
    bool display_trees = true;
    bool display_bushes = true;
};

struct vegetation_instance {
    cgp::vec3 position;
    float scale = 1.0f;
    float yaw = 0.0f;
    int variant = 0;
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
	void display_hitbox(car const& car, cgp::vec3 const& color);
	void position_camera();
	void initialize_car_on_track(car& car, float lateral_offset = 0.0f);
	void initialize_ground_drawable();
	void initialize_track_drawables();
	void initialize_vegetation();
	void display_vegetation();
	void set_map_id(int map_id);
	void reset_race_start();
	void display_start_menu();
	void start_countdown();
	void update_countdown(float dt);
	void display_countdown_overlay();
	void update_adversary_control(adversary_car& adversary);

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
	mesh_drawable global_frame;          
	
	car_structure car_base;
	player_car player;
	std::vector<adversary_car> adversaries;
	std::vector<cgp::vec3> adversary_colors;

	terrain_structure terrain;
    mountains terrain_mountains;
	bool game_started = false;
	bool race_active = false;
	float countdown_time = 0.0f;
	cgp::vec3 player_color = {0.1f, 0.25f, 0.85f};

	cgp::skybox_drawable skybox;
	cgp::mesh_drawable car_drawable;
	cgp::mesh_drawable car_window_drawable;
    std::array<cgp::mesh_drawable, 4> wheel_tire_drawables;
    std::array<cgp::mesh_drawable, 4> wheel_rim_drawables;
	cgp::mesh_drawable hitbox_point_drawable;
	cgp::mesh_drawable ground_outside_in;
	cgp::mesh_drawable ground_outside_out;
	cgp::mesh_drawable outside_out_hills;
	cgp::mesh_drawable lake;
	std::array<cgp::mesh_drawable, 4> tree_drawables;
	cgp::mesh_drawable bush_drawable;
	std::vector<vegetation_instance> tree_instances;
	std::vector<vegetation_instance> bush_instances;
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
