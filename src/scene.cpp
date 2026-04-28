#include "scene.hpp"
using namespace cgp;

namespace {

void display_vec3_debug(char const* label, vec3 const& value)
{
    ImGui::Text("%s: (%.3f, %.3f, %.3f)", label, value.x, value.y, value.z);
}

}


void scene_structure::initialize()
{
	
	camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
	camera_control.set_rotation_axis_y();
	distance_from_car = vec3(-3.0f, 1.0f, 0.0f);
	camera_control.look_at(car.position + distance_from_car, /* position of the camera in the 3D scene */
						   car.position,  /* targeted point in 3D scene */
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

	car_drawable.initialize_data_on_gpu(mesh_primitive_cube(car.position, 1.0f));
	car_drawable.material.color = {0.8f, 0.15f, 0.1f};
	car_drawable.model.scaling = 0.5f;

    mesh const wheel_tire_mesh = make_wheel_tire_mesh(car.wheel_radius);
    mesh const wheel_rim_mesh = make_wheel_rim_mesh(car.wheel_radius);
    std::string const tire_texture_path = find_supported_tire_texture();
    for (mesh_drawable& tire : wheel_tire_drawables) {
        tire.initialize_data_on_gpu(wheel_tire_mesh);
        if (!tire_texture_path.empty())
            tire.texture.load_and_initialize_texture_2d_on_gpu(tire_texture_path);
        tire.material.color = tire_texture_path.empty() ? vec3{0.08f, 0.08f, 0.08f} : vec3{1.0f, 1.0f, 1.0f};
        tire.material.phong = {0.35f, 0.55f, 0.0f, 1.0f};
    }
    for (mesh_drawable& rim : wheel_rim_drawables) {
        rim.initialize_data_on_gpu(wheel_rim_mesh);
        rim.texture.load_and_initialize_texture_2d_on_gpu(wheel_rim_texture_path());
        rim.material.color = {1.0f, 1.0f, 1.0f};
        rim.material.phong = {0.4f, 0.6f, 0.0f, 1.0f};
    }
}

void scene_structure::display_car(float dt)
{
    car.update(dt);

    rotation_transform body_rotation;
    if (norm(car.facing_direction) > 1e-5f)
        body_rotation = rotation_transform::from_vector_transform({1,0,0}, normalize(car.facing_direction));

    car_drawable.model.rotation = body_rotation;
    car_drawable.model.translation = car.position + vec3{0.0f, 0.15f, 0.0f};
    draw(car_drawable, environment);

    rotation_transform const wheel_spin_rotation =
        rotation_transform::from_axis_angle({0, 0, 1}, -car.wheel_spin_angle);
    rotation_transform const front_wheel_steering_rotation =
        rotation_transform::from_axis_angle(car.normal, car.steering_angle);
    std::array<vec3, 4> const wheel_offsets = {{
        {0.27f, 0.00f, 0.24f},
        {0.27f, 0.00f, -0.24f},
        {-0.27f, 0.00f, 0.24f},
        {-0.27f, 0.00f, -0.24f},
    }};

    for (size_t k = 0; k < wheel_tire_drawables.size(); ++k) {
        rotation_transform wheel_rotation = body_rotation * wheel_spin_rotation;
        if (k < 2)
            wheel_rotation = body_rotation * front_wheel_steering_rotation * wheel_spin_rotation;

        vec3 const wheel_translation = car.position + body_rotation * wheel_offsets[k] + vec3{0.0f, 0.02f, 0.0f};

        wheel_tire_drawables[k].model.rotation = wheel_rotation;
        wheel_tire_drawables[k].model.translation = wheel_translation;
        draw(wheel_tire_drawables[k], environment);

        wheel_rim_drawables[k].model.rotation = wheel_rotation;
        wheel_rim_drawables[k].model.translation = wheel_translation;
        draw(wheel_rim_drawables[k], environment);
    }
}

void scene_structure::position_camera()
{
    vec3 camera_direction = car.facing_direction;
    if (norm(car.velocity) > 1e-5f) {
        vec3 const velocity_direction = normalize(car.velocity);
        float const speed_ratio = norm(car.velocity) / (norm(car.velocity) + 1.0f);
        camera_direction = normalize((1.0f - speed_ratio) * car.facing_direction + speed_ratio * velocity_direction);
    }

	vec3 camera_offset = - 3.0f * camera_direction + 0.7f * car.normal;
	camera_control.look_at(car.position + camera_offset, car.position, {0, 0, 1});
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

	display_car(dt);
    position_camera();
    
}

void scene_structure::display_gui()
{
	ImGui::Checkbox("Frame", &gui.display_frame);

    if (ImGui::CollapsingHeader("Car Debug", ImGuiTreeNodeFlags_DefaultOpen)) {
        float const forward_speed = car.forward_speed();
        float const lateral_speed = car.lateral_speed();
        float const speed_magnitude = norm(car.velocity);
        float const forward_acceleration = dot(car.acceleration, normalize(car.facing_direction));
        vec3 const right_direction = normalize(cross(car.normal, car.facing_direction));
        float const lateral_acceleration = dot(car.acceleration, right_direction);
        float const wheel_spin_rate = forward_speed / car.wheel_radius;

        ImGui::Text("Position: (%.3f, %.3f, %.3f)", car.position.x, car.position.y, car.position.z);
        ImGui::Text("Forward speed: %.3f", forward_speed);
        ImGui::Text("Lateral speed: %.3f", lateral_speed);
        ImGui::Text("Speed magnitude: %.3f", speed_magnitude);
        ImGui::Text("Forward acceleration: %.3f", forward_acceleration);
        ImGui::Text("Lateral acceleration: %.3f", lateral_acceleration);
        ImGui::Text("Steering angle: %.3f rad", car.steering_angle);
        ImGui::Text("Steering input: %.3f", car.steering_input);
        ImGui::Text("Throttle input: %.3f", car.throttle_input);
        ImGui::Text("Wheel acceleration: %.3f", car.wheel_acceleration);
        ImGui::Text("Angular speed: %.3f rad/s", car.angular_speed);
        ImGui::Text("Wheel spin rate: %.3f rad/s", wheel_spin_rate);
        ImGui::Text("Wheel spin angle: %.3f rad", car.wheel_spin_angle);
        ImGui::Text("Throttle accel value: %.3f", car.acceleration_value);
        ImGui::Text("Drag factor: %.3f", car.drag_factor);
        ImGui::Text("Lateral drag factor: %.3f", car.lateral_drag_factor);
        ImGui::Separator();
        display_vec3_debug("Velocity", car.velocity);
        display_vec3_debug("Acceleration", car.acceleration);
        display_vec3_debug("Facing dir", car.facing_direction);
        display_vec3_debug("Normal", car.normal);
    }
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
