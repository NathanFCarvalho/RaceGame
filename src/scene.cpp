#include "scene.hpp"

using namespace cgp;



void scene_structure::initialize()
{
	
	camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
	camera_control.set_rotation_axis_y();
	distance_from_car = vec3(-3.0f, 1.0f, 0.0f);
	camera_control.look_at(car.initial_position + distance_from_car, /* position of the camera in the 3D scene */
						   car.initial_position,  /* targeted point in 3D scene */
						   {0,0,1} /* direction of the "up" vector */);
	display_info();
		

	global_frame.initialize_data_on_gpu(mesh_primitive_frame());

	
	// ****************************** //
	// Define the car
	// ****************************** //

	ground.initialize_data_on_gpu(mesh_primitive_quadrangle({ -1.0f,0.0f,-1.0f }, {-1.0f,0.0f, 1.0f }, { 1.0f,0.0f,1.0f }, { 1.0f,0.0f,-1.0f }));
	ground.model.scaling = 50.0f;
	ground.model.translation = { 0.0f, -0.5f, 0.0f };
	ground.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/checkboard.png");

	car_drawable.initialize_data_on_gpu(mesh_primitive_cube(car.initial_position, 1.0f));
	car_drawable.model.scaling = 0.5f;
}


void scene_structure::display_frame()
{
	camera_projection.aspect_ratio = window.aspect_ratio();
	environment.camera_projection = camera_projection.matrix();
	environment.camera_view = camera_control.camera_model.matrix_view();
	environment.light = camera_control.camera_model.position();
	
	draw(ground, environment);

	if (gui.display_frame)
		draw(global_frame, environment);

	float dt = timer.update();
	car.update(dt);

	car_drawable.model.translation = car.current_position;
	draw(car_drawable, environment);

	camera_control.look_at(car.current_position + distance_from_car , car.current_position, {0,0,1});

	environment.uniform_generic.uniform_float["time"] = timer.t;
	
	
}

void scene_structure::display_gui()
{
	ImGui::Checkbox("Frame", &gui.display_frame);
}

void scene_structure::mouse_move_event()
{
	if (!inputs.keyboard.shift)
		camera_control.action_mouse_move();
}
void scene_structure::mouse_click_event()
{
	camera_control.action_mouse_click();
}
void scene_structure::keyboard_event()
{
	camera_control.action_keyboard();
}
void scene_structure::idle_frame()
{
	car.action_keyboard(camera_control.inputs, camera_control.window);
	camera_control.idle_frame();
}

void scene_structure::display_info()
{
	std::cout << "\nCAMERA CONTROL:" << std::endl;
	std::cout << "-----------------------------------------------" << std::endl;
	std::cout << camera_control.doc_usage() << std::endl;
	std::cout << "-----------------------------------------------\n" << std::endl;
}