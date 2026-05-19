#include "scene.hpp"
#include "utils.hpp"

using namespace cgp;

namespace {
constexpr bool enable_adversary = true;
constexpr bool camera_follows_adversary = false;

void display_vec3_debug(char const* label, vec3 const& value)
{
    ImGui::Text("%s: (%.3f, %.3f, %.3f)", label, value.x, value.y, value.z);
}
}

void scene_structure::initialize()
{
    // Camera initialization
	camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
	camera_control.set_rotation_axis_y();
    player.camera.position_camera(1.0f, player.facing_direction, player.position, player.normal);
	position_camera();

	display_info();
    
	global_frame.initialize_data_on_gpu(mesh_primitive_frame());

    // Asphalt initialization
    mesh asphalt_mesh = terrain.create_asphalt_mesh();
    asphalt.initialize_data_on_gpu(asphalt_mesh);
    asphalt.model.translation = {0.0f, -0.5f, 0.0f};
    asphalt.material.color = {0.32f, 0.32f, 0.32f};
    asphalt.material.phong = {0.25f, 0.35f, 0.0f, 1.0f};

    // Barrier initialization
    mesh barrier_mesh = terrain.create_barrier_mesh();
    barrier.initialize_data_on_gpu(barrier_mesh);
    barrier.model.translation = {0.0f, -0.5f, 0.0f};
    barrier.material.color = {1.0f, 1.0f, 1.0f};
    barrier.material.phong = {0.35f, 0.45f, 0.0f, 1.0f};

    // Car initialization
	car_drawable.initialize_data_on_gpu(mesh_primitive_cube(player.position, player.dimensions.length));
	car_drawable.material.color = {0.8f, 0.15f, 0.1f};

    // Wheel tire initialization
    mesh const wheel_tire_mesh = car_base.create_wheel_mesh();
    mesh const wheel_rim_mesh = car_base.create_wheel_rim_mesh();

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

void scene_structure::display_car(car const& player, vec3 const& color)
{
    vec3 const forward = normalize_or(player.facing_direction, {1.0f, 0.0f, 0.0f});
    vec3 const up = normalize_or(player.normal, {0.0f, 1.0f, 0.0f});
    rotation_transform const body_rotation = rotation_transform::from_vector_transform({1,0,0}, forward);

    car_drawable.model.rotation = body_rotation;
    car_drawable.model.translation = player.position + vec3{0.0f, 0.15f, 0.0f};
    car_drawable.material.color = color;
    draw(car_drawable, environment);

    rotation_transform const wheel_spin_rotation =
        rotation_transform::from_axis_angle({0, 0, 1}, -player.wheel_spin_angle);
    rotation_transform const front_wheel_steering_rotation =
        rotation_transform::from_axis_angle(up, player.steering_angle);
    std::array<vec3, 4> const wheel_offsets = {{
        {player.dimensions.wheel_forward_offset, 0.00f, player.dimensions.wheel_side_offset},
        {player.dimensions.wheel_forward_offset, 0.00f, -player.dimensions.wheel_side_offset},
        {-player.dimensions.wheel_forward_offset, 0.00f, player.dimensions.wheel_side_offset},
        {-player.dimensions.wheel_forward_offset, 0.00f, -player.dimensions.wheel_side_offset},
    }};

    for (size_t k = 0; k < wheel_tire_drawables.size(); ++k) {
        rotation_transform wheel_rotation = body_rotation * wheel_spin_rotation;
        if (k < 2)
            wheel_rotation = body_rotation * front_wheel_steering_rotation * wheel_spin_rotation;

        vec3 const wheel_translation = player.position + body_rotation * wheel_offsets[k] + vec3{0.0f, 0.02f, 0.0f};

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

    if (camera_follows_adversary) {
        camera_control.look_at(adversary.position - 3.0f*adversary.facing_direction + 1.0f*adversary.normal, /* position of the camera in the 3D scene */
                        adversary.position,  /* targeted point in 3D scene */
                        {0,0,1} /* direction of the "up" vector */);
        return;
    }
    camera_control.look_at(player.camera.position, /* position of the camera in the 3D scene */
						   player.position,  /* targeted point in 3D scene */
						   {0,0,1} /* direction of the "up" vector */);
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

    terrain.resolve_collision(player);
    player.update(dt);
    player.position_camera(dt);

    if (enable_adversary) {
        terrain.resolve_collision(adversary);

        track_projection const proj = terrain.closest_track_projection(adversary.position);
        vec3 const target = terrain.track_point_ahead(proj, adversary.lookahead_distance);
        adversary.follow_target(target);

        adversary.update(dt);
    }

    if (camera_follows_adversary && enable_adversary)
        player.camera.position_camera(dt, adversary.facing_direction, adversary.position, adversary.normal);
    else
        player.camera.position_camera(dt, player.facing_direction, player.position, player.normal);

	display_car(player, {0.1f, 0.25f, 0.85f});
    if (enable_adversary)
        display_car(adversary, {0.1f, 0.25f, 0.1f});
    position_camera();
}

void scene_structure::display_gui()
{
	ImGui::Checkbox("Frame", &gui.display_frame);

    if (ImGui::CollapsingHeader("Car Debug", ImGuiTreeNodeFlags_DefaultOpen)) {
        float const forward_speed = player.forward_speed();
        float const lateral_speed = player.lateral_speed();
        float const speed_magnitude = norm(player.velocity);
        float const forward_acceleration = dot(player.acceleration, normalize(player.facing_direction));
        vec3 const right_direction = normalize(cross(player.normal, player.facing_direction));
        float const lateral_acceleration = dot(player.acceleration, right_direction);
        float const wheel_spin_rate = forward_speed / player.dimensions.wheel_radius;

        ImGui::Text("Forward speed: %.3f", forward_speed);
        ImGui::Text("Lateral speed: %.3f", lateral_speed);
        ImGui::Text("Speed magnitude: %.3f", speed_magnitude);
        ImGui::Text("Forward acceleration: %.3f", forward_acceleration);
        ImGui::Text("Lateral acceleration: %.3f", lateral_acceleration);
        ImGui::Text("Steering angle: %.3f rad", player.steering_angle);
        ImGui::Text("Steering input: %d", player.steering_input);
        ImGui::Text("Throttle input: %d", player.throttle_input);
        ImGui::Text("Wheel acceleration: %.3f", player.wheel_acceleration);
        ImGui::Text("Angular speed: %.3f rad/s", player.angular_speed);
        ImGui::Text("Wheel spin rate: %.3f rad/s", wheel_spin_rate);
        ImGui::Text("Wheel spin angle: %.3f rad", player.wheel_spin_angle);
        ImGui::Separator();
        display_vec3_debug("Velocity", player.velocity);
        display_vec3_debug("Acceleration", player.acceleration);
        display_vec3_debug("Facing dir", player.facing_direction);
        display_vec3_debug("Normal", player.normal);
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
	player.action_keyboard(camera_control.inputs, camera_control.window);
	camera_control.idle_frame();
}

void scene_structure::display_info()
{
	std::cout << "\nCAMERA CONTROL:" << std::endl;
	std::cout << "-----------------------------------------------" << std::endl;
	std::cout << camera_control.doc_usage() << std::endl;
	std::cout << "-----------------------------------------------\n" << std::endl;
}
