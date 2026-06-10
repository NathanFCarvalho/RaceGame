#include "scene.hpp"
#include "car_model.hpp"
#include "utils.hpp"

using namespace cgp;

namespace {
constexpr bool enable_skybox = true;
constexpr bool environment_map = true;
constexpr bool use_obj_car_model = true;
constexpr float window_environment_reflection = 0.85f;
constexpr float hitbox_debug_height = 0.08f;
constexpr float hitbox_point_radius = 0.06f;
constexpr int adversary_car_count = 6;
constexpr float startline_u = 0.0f;
constexpr float countdown_duration = 4.0f;
}

void scene_structure::initialize()
{
    // Camera initialization
	camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
	camera_control.set_rotation_axis_y();

	display_info();
    
	global_frame.initialize_data_on_gpu(mesh_primitive_frame());
    ground.initialize_data_on_gpu(mesh_primitive_quadrangle(
        {-900.0f, -0.56f, -900.0f},
        { 900.0f, -0.56f, -900.0f},
        { 900.0f, -0.56f,  900.0f},
        {-900.0f, -0.56f,  900.0f}));
    ground.material.color = {0.18f, 0.42f, 0.18f};
    ground.material.phong = {0.35f, 0.25f, 0.0f, 1.0f};
    hitbox_point_drawable.initialize_data_on_gpu(mesh_primitive_sphere(1.0f, {0.0f, 0.0f, 0.0f}, 12, 6));
    hitbox_point_drawable.material.color = {1.0f, 0.9f, 0.05f};
    hitbox_point_drawable.material.phong = {0.6f, 0.4f, 0.0f, 1.0f};

    // Skybox initialization.
    if (enable_skybox) {
        image_structure image_skybox_template = image_load_file(project::path+"assets/skybox_02.jpg");
        std::vector<image_structure> image_grid = image_split_grid(image_skybox_template, 4, 3);

        skybox.vao = 0;
        skybox.initialize_data_on_gpu();
        skybox.texture.initialize_cubemap_on_gpu(image_grid[1], image_grid[7], image_grid[5], image_grid[3], image_grid[10], image_grid[4]);
    }
   
    initialize_track_drawables();

    // Car initialization
    car_model_meshes const car_meshes = use_obj_car_model
        ? create_car_model_meshes(project::path + "assets/car.obj", player.dimensions.length)
        : car_model_meshes{mesh_primitive_cube({0.0f, 0.0f, 0.0f}, player.dimensions.length), mesh{}};
    mesh const& car_mesh = car_meshes.body;
	car_drawable.initialize_data_on_gpu(car_mesh);
	car_drawable.material.color = {0.8f, 0.15f, 0.1f};
    car_drawable.material.phong = {0.15f, 0.25f, 0.3f, 10.0f};
    if (car_meshes.windows.position.size() > 0) {
        car_window_drawable.initialize_data_on_gpu(car_meshes.windows);
        car_window_drawable.material.color = {0.005f, 0.006f, 0.008f};
        car_window_drawable.material.phong = {0.04f, 0.05f, 1.0f, 80.0f};
    }

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

    player.idx = 0;
    reset_race_start();
    player.camera.position_camera(1.0f, player.forward, player.position, player.normal);
	position_camera();

    if (environment_map) {
        opengl_shader_structure shader_environment_map;
        shader_environment_map.load(project::path + "shaders/environment_map/environment_map.vert.glsl", project::path + "shaders/environment_map/environment_map.frag.glsl");

        environment.uniform_generic.uniform_mat3["skybox_rotation"] = mat3::build_identity();
        environment.uniform_generic.uniform_float["reflection_strength"] = window_environment_reflection;
        environment.default_expected_uniform = false;

        car_window_drawable.shader = shader_environment_map;
        car_window_drawable.supplementary_texture["image_skybox"] = skybox.texture;
    }

}

void scene_structure::initialize_track_drawables()
{
    if (asphalt.vao != 0)
        asphalt.clear();
    if (barrier.vao != 0)
        barrier.clear();

    mesh asphalt_mesh = terrain.create_asphalt_mesh();
    asphalt.initialize_data_on_gpu(asphalt_mesh);
    asphalt.texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/asphalt.jpg", GL_MIRRORED_REPEAT, GL_MIRRORED_REPEAT);
    asphalt.model.translation = {0.0f, -0.5f, 0.0f};
    asphalt.material.color = {1.0f, 1.0f, 1.0f};
    asphalt.material.phong = {0.25f, 0.35f, 0.0f, 1.0f};

    mesh barrier_mesh = terrain.create_barrier_mesh();
    barrier.initialize_data_on_gpu(barrier_mesh);
    barrier.texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/brick.jpg", GL_MIRRORED_REPEAT, GL_MIRRORED_REPEAT);
    barrier.model.translation = {0.0f, -0.5f, 0.0f};
    barrier.material.color = {1.0f, 1.0f, 1.0f};
    barrier.material.phong = {0.35f, 0.45f, 0.0f, 1.0f};
}

void scene_structure::reset_race_start()
{
    reset_car_state(player);
    player.camera = ::camera_control();
    player.idx = 0;
    initialize_car_on_track(player);

    adversaries.clear();
    adversary_colors.clear();
    adversaries.resize(adversary_car_count);
    adversary_colors.resize(adversaries.size());
    vec3 const player_lateral_normal = terrain.track_side_direction(0.0f);
    for (size_t k = 0; k < adversaries.size(); ++k) {
        adversary_car& adversary = adversaries[k];
        adversary.idx = static_cast<int>(k) + 1;
        adversary.position = player.position
            + adversary_start_lateral_offset(k) * player_lateral_normal;
        adversary.forward = normalize_or(player.forward, {1.0f, 0.0f, 0.0f});
        adversary.update_direction_vectors();
        adversary_colors[k] = {
            rand_uniform(0.15f, 0.95f),
            rand_uniform(0.15f, 0.95f),
            rand_uniform(0.15f, 0.95f)
        };
    }
}

void scene_structure::start_countdown()
{
    game_started = true;
    race_active = false;
    countdown_time = 0.0f;
}

void scene_structure::update_countdown(float dt)
{
    if (!game_started || race_active)
        return;

    countdown_time += dt;
    if (countdown_time >= countdown_duration)
        race_active = true;
}

void scene_structure::display_countdown_overlay()
{
    if (!game_started || race_active)
        return;

    char const* label = "GO";
    if (countdown_time < 1.0f)
        label = "3";
    else if (countdown_time < 2.0f)
        label = "2";
    else if (countdown_time < 3.0f)
        label = "1";

    ImVec2 const display_size = ImGui::GetIO().DisplaySize;
    ImGuiWindowFlags const flags =
        ImGuiWindowFlags_NoDecoration
        | ImGuiWindowFlags_NoMove
        | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoSavedSettings
        | ImGuiWindowFlags_NoBackground
        | ImGuiWindowFlags_NoInputs;

    ImGui::SetNextWindowPos({0.0f, 0.0f});
    ImGui::SetNextWindowSize(display_size);
    ImGui::Begin("Countdown", nullptr, flags);

    ImGui::SetWindowFontScale(6.0f);
    ImVec2 const text_size = ImGui::CalcTextSize(label);
    ImGui::SetCursorPos({
        0.5f * (display_size.x - text_size.x),
        0.42f * (display_size.y - text_size.y)
    });
    ImGui::TextUnformatted(label);
    ImGui::SetWindowFontScale(1.0f);

    ImGui::End();
}

void scene_structure::update_adversary_control(adversary_car& adversary, bool race_started)
{
    if (race_started) {
        track_projection const proj = terrain.closest_track_projection(adversary.position);
        float const centerline_tolerance = terrain.track_width / 6.0f;
        if (std::abs(proj.lateral_distance) <= centerline_tolerance)
            adversary.align_with_track_tangent(proj.tangent_direction);
        else
            adversary.follow_target(terrain.track_point_ahead(proj, adversary.lookahead_distance));
        return;
    }

    adversary.throttle_input = 0;
    adversary.steering_input = 0;
    adversary.forward = player.forward;
    adversary.update_direction_vectors();
}

void scene_structure::initialize_car_on_track(car& vehicle, float lateral_offset)
{
    vec3 const center = terrain.track_centerline(startline_u);
    vec3 const target = terrain.track_tangent(startline_u);
    vec3 const lateral_normal = terrain.track_side_direction(startline_u);

    vehicle.position = center + lateral_offset * lateral_normal;
    vehicle.forward = target;
    vehicle.update_direction_vectors();
}

void scene_structure::display_car(car const& player, vec3 const& color)
{
    rotation_transform const body_rotation = rotation_transform::from_vector_transform({1,0,0}, player.forward);

    car_drawable.model.rotation = body_rotation;
    car_drawable.model.translation = player.position + vec3{0.0f, 0.00f, 0.0f};
    car_drawable.material.color = color;
    draw(car_drawable, environment);

    if (car_window_drawable.vao != 0) {
        car_window_drawable.model.rotation = body_rotation;
        car_window_drawable.model.translation = player.position + vec3{0.0f, 0.00f, 0.0f};
        draw(car_window_drawable, environment);
    }

    rotation_transform const wheel_spin_rotation =
        rotation_transform::from_axis_angle({0, 0, 1}, -player.wheel_spin_angle);
    rotation_transform const front_wheel_steering_rotation =
        rotation_transform::from_axis_angle(player.up, player.steering_angle);
    std::array<vec3, 4> const wheel_offsets = {{
        {player.dimensions.wheel_forward_offset, -0.20f, player.dimensions.wheel_side_offset},
        {player.dimensions.wheel_forward_offset, -0.20f, -player.dimensions.wheel_side_offset},
        {-player.dimensions.wheel_backwards_offset, -0.20f, player.dimensions.wheel_side_offset},
        {-player.dimensions.wheel_backwards_offset, -0.20f, -player.dimensions.wheel_side_offset},
    }};

    for (size_t k = 0; k < wheel_tire_drawables.size(); ++k) {
        rotation_transform wheel_rotation = body_rotation * wheel_spin_rotation;
        if (k < 2)
            wheel_rotation = body_rotation * front_wheel_steering_rotation * wheel_spin_rotation;

        vec3 const wheel_translation = player.position + body_rotation * wheel_offsets[k] + vec3{0.0f, 0.02f, 0.0f};

        wheel_tire_drawables[k].model.rotation = wheel_rotation;
        wheel_tire_drawables[k].model.translation = wheel_translation;
        wheel_tire_drawables[k].model.scaling = 1.0f;
        draw(wheel_tire_drawables[k], environment);

        wheel_rim_drawables[k].model.rotation = wheel_rotation;
        wheel_rim_drawables[k].model.translation = wheel_translation;
        wheel_rim_drawables[k].model.scaling = 1.0f;
        draw(wheel_rim_drawables[k], environment);
    }
}

void scene_structure::display_hitbox(car const& vehicle, vec3 const& color)
{
    std::array<vec3, 4> const local_corners = vehicle.get_hitbox_samples();

    hitbox_point_drawable.material.color = color;
    for (vec3 const& corner : local_corners) {
        hitbox_point_drawable.model.rotation = rotation_transform();
        hitbox_point_drawable.model.translation = vehicle.position + corner + vec3{0.0f, hitbox_debug_height, 0.0f};
        hitbox_point_drawable.model.scaling = hitbox_point_radius;
        hitbox_point_drawable.model.scaling_xyz = {1.0f, 1.0f, 1.0f};
        draw(hitbox_point_drawable, environment);
    }
}

void scene_structure::position_camera()
{
    if (gui.top_view) {
		vec3 const top_position = vec3{ 0.0f, 720.0f, 25.0f };
        vec3 const track_center = vec3{ 0.0f, 0.0f,   25.0f };
        camera_control.look_at(top_position, track_center, { 0, 0, 1 });
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

    if(enable_skybox){
        glDepthMask(GL_FALSE);
        draw(skybox, environment);
        glDepthMask(GL_TRUE);
    }
	
	draw(ground, environment);
    draw(asphalt, environment);
    draw(barrier, environment);

	if (gui.display_frame)
		draw(global_frame, environment);

	float dt = timer.update();
    update_countdown(dt);

    if (game_started && race_active) {
        terrain.resolve_collision(player);
        player.update(dt);

        bool const race_started = player.throttle_input != 0 || norm(player.velocity) > 0.1f;
        for (adversary_car& adversary : adversaries) {
            terrain.resolve_collision(adversary);
            update_adversary_control(adversary, race_started);
            adversary.update(dt);
        }

        std::vector<car*> cars;
        cars.reserve(1 + adversaries.size());
        cars.push_back(&player);
        for (adversary_car& adversary : adversaries)
            cars.push_back(&adversary);
        player.verify_collisions(cars);
    }

    player.camera.position_camera(dt, player.forward, player.position, player.normal);

	display_car(player, player_color);
    if (gui.display_hitbox)
        display_hitbox(player, {1.0f, 0.85f, 0.05f});
    for (size_t k = 0; k < adversaries.size(); ++k)
    {
        adversary_car const& adversary = adversaries[k];
        vec3 const color = k < adversary_colors.size() ? adversary_colors[k] : vec3{0.1f, 0.25f, 0.1f};
        display_car(adversary, color);
        if (gui.display_hitbox)
            display_hitbox(adversary, {0.1f, 1.0f, 0.25f});
    }
    position_camera();
    display_countdown_overlay();
}

void scene_structure::display_gui()
{
	ImGui::Checkbox("Frame", &gui.display_frame);
	ImGui::Checkbox("Hitbox", &gui.display_hitbox);
	ImGui::Checkbox("Top View", &gui.top_view);
    if (ImGui::Button("Back to Menu")) {
        game_started = false;
        race_active = false;
        reset_race_start();
        player.camera.position_camera(1.0f, player.forward, player.position, player.normal);
    }

    if (ImGui::CollapsingHeader("Car Debug", ImGuiTreeNodeFlags_DefaultOpen)) {
        float const forward_speed = dot(player.velocity, player.forward);
        float const lateral_speed = dot(player.velocity, player.right);
        float const speed_magnitude = norm(player.velocity);
        float const forward_acceleration = dot(player.acceleration, player.forward);
        float const lateral_acceleration = dot(player.acceleration, player.right);
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
        ImGui::Text("In collision %d", player.in_colision);
        ImGui::Separator();
        display_vec3_debug("Velocity", player.velocity);
        display_vec3_debug("Acceleration", player.acceleration);
        display_vec3_debug("Forward", player.forward);
        display_vec3_debug("Normal", player.normal);
    }
}

void scene_structure::display_start_menu()
{
    if (game_started)
        return;

    ImVec2 const display_size = ImGui::GetIO().DisplaySize;
    ImVec2 const panel_size = {420.0f, 300.0f};
    ImGuiWindowFlags const flags =
        ImGuiWindowFlags_NoDecoration
        | ImGuiWindowFlags_NoMove
        | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoSavedSettings;

    ImGui::SetNextWindowPos({0.0f, 0.0f});
    ImGui::SetNextWindowSize(display_size);
    ImGui::SetNextWindowBgAlpha(0.94f);
    ImGui::Begin("Start Menu", nullptr, flags);

    ImGui::SetCursorPos({
        0.5f * (display_size.x - panel_size.x),
        0.5f * (display_size.y - panel_size.y)
    });
    ImGui::BeginGroup();
    ImGui::PushItemWidth(panel_size.x);
    ImGui::Text("Race Game");
    ImGui::Spacing();

    ImGui::ColorEdit3("Car Color", &player_color.x);
    ImGui::Spacing();

    if (ImGui::Button("Play", {panel_size.x, 44.0f})) {
        reset_race_start();
        start_countdown();
    }

    ImGui::PopItemWidth();
    ImGui::EndGroup();
    ImGui::End();
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
    if (game_started && race_active)
	    player.action_keyboard(camera_control.inputs, camera_control.window);
    else {
        player.throttle_input = 0;
        player.steering_input = 0;
    }
	camera_control.idle_frame();
}

void scene_structure::display_info()
{
	std::cout << "\nCAMERA CONTROL:" << std::endl;
	std::cout << "-----------------------------------------------" << std::endl;
	std::cout << camera_control.doc_usage() << std::endl;
	std::cout << "-----------------------------------------------\n" << std::endl;
}
