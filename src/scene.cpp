#include "scene.hpp"

using namespace cgp;

namespace {

constexpr float car_body_scaling = 0.5f;
constexpr float wheel_forward_offset = 0.27f;
constexpr float wheel_side_offset = 0.24f;
constexpr float wheel_tire_half_width = 0.08f;
constexpr float hitbox_margin = 0.8f;

void display_vec3_debug(char const* label, vec3 const& value)
{
    ImGui::Text("%s: (%.3f, %.3f, %.3f)", label, value.x, value.y, value.z);
}

float clamp_value(float value, float min_value, float max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

}


void scene_structure::initialize()
{
	
	camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
	camera_control.set_rotation_axis_y();
	camera_smoothed_direction = normalize(car.facing_direction);
	camera_smoothed_position = car.position - 3.0f * camera_smoothed_direction + 0.7f * car.normal;
	camera_follow_initialized = true;
	camera_control.look_at(camera_smoothed_position, /* position of the camera in the 3D scene */
						   car.position,  /* targeted point in 3D scene */
						   {0,0,1} /* direction of the "up" vector */);
	display_info();
		

	global_frame.initialize_data_on_gpu(mesh_primitive_frame());

	
	// ****************************** //
	// Define the car
	// ****************************** //

	//ground.initialize_data_on_gpu(mesh_primitive_quadrangle({ -1.0f,0.0f,-1.0f }, {-1.0f,0.0f, 1.0f }, { 1.0f,0.0f,1.0f }, { 1.0f,0.0f,-1.0f }));
	//ground.model.scaling = 50.0f;
	//ground.model.translation = { 0.0f, -0.5f, 0.0f };
	//ground.texture.load_and_initialize_texture_2d_on_gpu(project::path+"assets/checkboard.png");

    mesh asphalt_mesh = terrain.create_asphalt_mesh();
    asphalt.initialize_data_on_gpu(asphalt_mesh);
    asphalt.model.translation = {0.0f, -0.5f, 0.0f};
    asphalt.material.color = {0.32f, 0.32f, 0.32f};
    asphalt.material.phong = {0.25f, 0.35f, 0.0f, 1.0f};

    mesh barrier_mesh = terrain.create_barrier_mesh();
    barrier.initialize_data_on_gpu(barrier_mesh);
    barrier.model.translation = {0.0f, -0.5f, 0.0f};
    barrier.material.color = {1.0f, 1.0f, 1.0f};
    barrier.material.phong = {0.35f, 0.45f, 0.0f, 1.0f};

	car_drawable.initialize_data_on_gpu(mesh_primitive_cube(car.position, 1.0f));
	car_drawable.material.color = {0.8f, 0.15f, 0.1f};
	car_drawable.model.scaling = car_body_scaling;

    float const body_half_size = 0.5f * car_body_scaling;
    car.collision_half_length = std::max(body_half_size, wheel_forward_offset + car.wheel_radius) + hitbox_margin;
    car.collision_half_width = std::max(body_half_size, wheel_side_offset + wheel_tire_half_width) + hitbox_margin;

    mesh const wheel_tire_mesh = mesh_primitive_cylinder(car.wheel_radius, {0.0f, 0.0f, -wheel_tire_half_width}, {0.0f, 0.0f, wheel_tire_half_width}, 2, 32, false);
    
    mesh wheel_rim_mesh = mesh_primitive_disc(car.wheel_radius, {0.0f, 0.0f, -0.055f}, {0.0f, 0.0f, -1.0f}, 32);
    wheel_rim_mesh.push_back(mesh_primitive_disc(car.wheel_radius, {0.0f, 0.0f, 0.055f}, {0.0f, 0.0f, 1.0f}, 32));
    wheel_rim_mesh.fill_empty_field();

    for (mesh_drawable& tire : wheel_tire_drawables) {
        tire.initialize_data_on_gpu(wheel_tire_mesh);
        tire.texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/tire.jpg");
        tire.material.color = vec3{1.0f, 1.0f, 1.0f};
        tire.material.phong = {0.35f, 0.55f, 0.0f, 1.0f};
    }
    for (mesh_drawable& rim : wheel_rim_drawables) {
        rim.initialize_data_on_gpu(wheel_rim_mesh);
        rim.texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/wheel.png");
        rim.material.color = {1.0f, 1.0f, 1.0f};
        rim.material.phong = {0.4f, 0.6f, 0.0f, 1.0f};
    }
}

void scene_structure::display_car(float dt)
{
    car.update(dt);
    terrain.resolve_collision(car);

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
        {wheel_forward_offset, 0.00f, wheel_side_offset},
        {wheel_forward_offset, 0.00f, -wheel_side_offset},
        {-wheel_forward_offset, 0.00f, wheel_side_offset},
        {-wheel_forward_offset, 0.00f, -wheel_side_offset},
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

void scene_structure::position_camera(float dt)
{
    vec3 target_direction = normalize(car.facing_direction);
    float const speed = norm(car.velocity);

    if (speed > 0.15f) {
        vec3 velocity_direction = normalize(car.velocity);
        if (dot(velocity_direction, target_direction) < 0.0f)
            velocity_direction = -velocity_direction;

        float const alignment = clamp_value(dot(velocity_direction, target_direction), 0.0f, 1.0f);
        float const speed_ratio = clamp_value(speed / (speed + 3.0f), 0.0f, 0.75f) * alignment;
        target_direction = normalize((1.0f - speed_ratio) * target_direction + speed_ratio * velocity_direction);
    }

    if (!camera_follow_initialized) {
        camera_smoothed_direction = target_direction;
        camera_smoothed_position = car.position - 3.0f * target_direction + 0.7f * car.normal;
        camera_follow_initialized = true;
    }

    float const direction_alpha = 1.0f - std::exp(-camera_direction_response * dt);
    camera_smoothed_direction = normalize(
        (1.0f - direction_alpha) * camera_smoothed_direction + direction_alpha * target_direction);

	vec3 const target_position = car.position - 3.0f * camera_smoothed_direction + 0.7f * car.normal;
    float const position_alpha = 1.0f - std::exp(-camera_position_response * dt);
    camera_smoothed_position = (1.0f - position_alpha) * camera_smoothed_position + position_alpha * target_position;

	camera_control.look_at(camera_smoothed_position, car.position, {0, 0, 1});
}

void scene_structure::display_frame()
{
	camera_projection.aspect_ratio = window.aspect_ratio();
	environment.camera_projection = camera_projection.matrix();
	environment.camera_view = camera_control.camera_model.matrix_view();
	environment.light = camera_control.camera_model.position();
	
	draw(ground, environment);
    draw(asphalt, environment);
    draw(barrier, environment);

	if (gui.display_frame)
		draw(global_frame, environment);

	float dt = timer.update();

	display_car(dt);
    position_camera(dt);
    
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
        ImGui::Text("Steering input: %d", car.steering_input);
        ImGui::Text("Throttle input: %d", car.throttle_input);
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
