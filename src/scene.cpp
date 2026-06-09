#include "scene.hpp"
#include "utils.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace cgp;

namespace {
constexpr bool enable_adversary = true;
constexpr bool camera_follows_adversary = false;
constexpr bool enable_skybox = true;
constexpr bool environment_map = true;
constexpr bool use_obj_car_model = true;
constexpr float car_visual_length = 1.65f;
constexpr float window_environment_reflection = 0.85f;
constexpr float smoke_boost_acceleration = 38.0f;
constexpr float smoke_spawn_period = 0.055f;
constexpr float hitbox_debug_height = 0.08f;
constexpr float hitbox_point_radius = 0.06f;
constexpr float wheel_visual_scale = 0.90f;
constexpr float wheel_visual_side_offset_scale = 1.40f;
constexpr float front_wheel_visual_forward_offset = 0.15f;
constexpr float rear_wheel_visual_forward_offset = -0.25f;
constexpr float wheel_rim_outward_offset = 0.02f;
constexpr int adversary_car_count = 2;
constexpr float startline_u = 0.0f;
constexpr float track_spawn_du = 0.001f;

void display_vec3_debug(char const* label, vec3 const& value)
{
    ImGui::Text("%s: (%.3f, %.3f, %.3f)", label, value.x, value.y, value.z);
}

cgp::vec3 startline_target(terrain_structure const& terrain)
{
    vec3 const tangent = terrain.track_centerline(startline_u + track_spawn_du)
        - terrain.track_centerline(startline_u - track_spawn_du);
    return normalize_or(vec3{tangent.x, 0.0f, tangent.z}, {1.0f, 0.0f, 0.0f});
}

cgp::vec3 lateral_normal_from_target(vec3 const& target)
{
    return normalize_or(vec3{-target.z, 0.0f, target.x}, {0.0f, 0.0f, 1.0f});
}

float adversary_lateral_offset(size_t adversary_index)
{
    float constexpr spacing = 2.0f;
    float const side = adversary_index % 2 == 0 ? 1.0f : -1.0f;
    float const rank = 1.0f + static_cast<float>(adversary_index / 2);
    return side * rank * spacing;
}

struct obj_index {
    int position = -1;
    int uv = -1;
    int normal = -1;
};

struct car_model_meshes {
    mesh body;
    mesh windows;
};

obj_index parse_obj_index(std::string const& token)
{
    obj_index index;
    size_t const slash0 = token.find('/');
    if (slash0 == std::string::npos) {
        index.position = std::stoi(token) - 1;
        return index;
    }

    index.position = std::stoi(token.substr(0, slash0)) - 1;
    size_t const slash1 = token.find('/', slash0 + 1);
    if (slash1 == std::string::npos) {
        std::string const uv = token.substr(slash0 + 1);
        if (!uv.empty())
            index.uv = std::stoi(uv) - 1;
        return index;
    }

    std::string const uv = token.substr(slash0 + 1, slash1 - slash0 - 1);
    std::string const normal = token.substr(slash1 + 1);
    if (!uv.empty())
        index.uv = std::stoi(uv) - 1;
    if (!normal.empty())
        index.normal = std::stoi(normal) - 1;
    return index;
}

void append_obj_triangle(mesh& target, std::array<obj_index, 3> const& triangle, std::vector<vec3> const& positions, std::vector<vec2> const& uvs, std::vector<vec3> const& normals)
{
    uint3 connectivity;
    for (int k = 0; k < 3; ++k) {
        obj_index const& index = triangle[k];
        connectivity[k] = int(target.position.size());
        target.position.push_back(positions[index.position]);
        target.uv.push_back(index.uv >= 0 ? uvs[index.uv] : vec2{0.0f, 0.0f});
        target.normal.push_back(index.normal >= 0 ? normals[index.normal] : vec3{0.0f, 1.0f, 0.0f});
    }
    target.connectivity.push_back(connectivity);
}

void transform_car_mesh(mesh& car_mesh, vec3 const& center, float scale)
{
    car_mesh.translate(-center);
    car_mesh.scale(scale);
    car_mesh.rotate({0.0f, 1.0f, 0.0f}, Pi / 2.0f);
    car_mesh.fill_empty_field();
}

car_model_meshes create_car_model_meshes()
{
    std::ifstream stream(project::path + "assets/car.obj");
    assert_cgp(stream.is_open(), "Cannot open car.obj");

    std::vector<vec3> positions;
    std::vector<vec2> uvs;
    std::vector<vec3> normals;
    car_model_meshes model;

    bool use_window_mesh = false;
    std::string line;
    while (std::getline(stream, line)) {
        std::istringstream line_stream(line);
        std::string command;
        line_stream >> command;

        if (command == "v") {
            vec3 p;
            line_stream >> p.x >> p.y >> p.z;
            positions.push_back(p);
        }
        else if (command == "vt") {
            vec2 uv;
            line_stream >> uv.x >> uv.y;
            uvs.push_back(uv);
        }
        else if (command == "vn") {
            vec3 n;
            line_stream >> n.x >> n.y >> n.z;
            normals.push_back(n);
        }
        else if (command == "usemtl") {
            std::string material;
            line_stream >> material;
            use_window_mesh = material == "Window";
        }
        else if (command == "f") {
            std::vector<obj_index> face;
            std::string token;
            while (line_stream >> token)
                face.push_back(parse_obj_index(token));

            mesh& target = use_window_mesh ? model.windows : model.body;
            for (size_t k = 1; k + 1 < face.size(); ++k)
                append_obj_triangle(target, {face[0], face[k], face[k + 1]}, positions, uvs, normals);
        }
    }

    mesh full_mesh;
    full_mesh.push_back(model.body);
    full_mesh.push_back(model.windows);
    full_mesh.fill_empty_field();

    vec3 p_min;
    vec3 p_max;
    full_mesh.get_bounding_box_position(p_min, p_max);
    vec3 const extent = p_max - p_min;
    float const source_length = std::max(extent.z, 1e-5f);
    vec3 const center = 0.5f * (p_min + p_max);
    float const scale = car_visual_length / source_length;

    transform_car_mesh(model.body, center, scale);
    transform_car_mesh(model.windows, center, scale);
    return model;
}

mesh create_smoke_billboard_mesh()
{
    mesh quad = mesh_primitive_quadrangle(
        {-0.5f, -0.5f, 0.0f},
        { 0.5f, -0.5f, 0.0f},
        { 0.5f,  0.5f, 0.0f},
        {-0.5f,  0.5f, 0.0f});
    quad.uv = {{0.0f, 0.0f}, {1.0f, 0.0f}, {1.0f, 1.0f}, {0.0f, 1.0f}};
    quad.fill_empty_field();
    return quad;
}

}

void scene_structure::initialize()
{
    // Camera initialization
	camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
	camera_control.set_rotation_axis_y();

	display_info();
    
	global_frame.initialize_data_on_gpu(mesh_primitive_frame());
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
        ? create_car_model_meshes()
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

    smoke_drawable.initialize_data_on_gpu(create_smoke_billboard_mesh());
    smoke_drawable.texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/smoke.png");
    smoke_drawable.material.color = {0.82f, 0.82f, 0.82f};
    smoke_drawable.material.alpha = 0.65f;
    smoke_drawable.material.phong = {1.0f, 0.0f, 0.0f, 1.0f};
    smoke_drawable.material.texture_settings.two_sided = true;

    player.idx = 0;
    reset_race_start();
    player.camera.position_camera(1.0f, player.facing_direction, player.position, player.normal);
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
    player = player_car();
    player.idx = 0;
    initialize_car_on_track(player);

    adversaries.clear();
    smoke_particles.clear();
    smoke_spawn_accumulator = 0.0f;
    adversaries.resize(enable_adversary ? adversary_car_count : 0);
    vec3 const player_lateral_normal = lateral_normal_from_target(player.facing_direction);
    for (size_t k = 0; k < adversaries.size(); ++k) {
        adversary_car& adversary = adversaries[k];
        adversary.idx = static_cast<int>(k) + 1;
        adversary.position = player.position
            + adversary_lateral_offset(k) * player_lateral_normal;
        adversary.facing_direction = normalize_or(player.facing_direction, {1.0f, 0.0f, 0.0f});
    }
}

void scene_structure::initialize_car_on_track(car& vehicle, float lateral_offset)
{
    vec3 const center = terrain.track_centerline(startline_u);
    vec3 const target = startline_target(terrain);
    vec3 const lateral_normal = lateral_normal_from_target(target);

    vehicle.position = center + lateral_offset * lateral_normal;
    vehicle.facing_direction = target;
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

    if (car_window_drawable.vao != 0) {
        car_window_drawable.model.rotation = body_rotation;
        car_window_drawable.model.translation = player.position + vec3{0.0f, 0.15f, 0.0f};
        draw(car_window_drawable, environment);
    }

    rotation_transform const wheel_spin_rotation =
        rotation_transform::from_axis_angle({0, 0, 1}, -player.wheel_spin_angle);
    rotation_transform const front_wheel_steering_rotation =
        rotation_transform::from_axis_angle(up, player.steering_angle);
    std::array<vec3, 4> const wheel_offsets = {{
        {player.dimensions.wheel_forward_offset + front_wheel_visual_forward_offset, 0.00f, wheel_visual_side_offset_scale * player.dimensions.wheel_side_offset},
        {player.dimensions.wheel_forward_offset + front_wheel_visual_forward_offset, 0.00f, -wheel_visual_side_offset_scale * player.dimensions.wheel_side_offset},
        {-player.dimensions.wheel_forward_offset + rear_wheel_visual_forward_offset, 0.00f, wheel_visual_side_offset_scale * player.dimensions.wheel_side_offset},
        {-player.dimensions.wheel_forward_offset + rear_wheel_visual_forward_offset, 0.00f, -wheel_visual_side_offset_scale * player.dimensions.wheel_side_offset},
    }};

    for (size_t k = 0; k < wheel_tire_drawables.size(); ++k) {
        rotation_transform wheel_rotation = body_rotation * wheel_spin_rotation;
        if (k < 2)
            wheel_rotation = body_rotation * front_wheel_steering_rotation * wheel_spin_rotation;

        vec3 const wheel_translation = player.position + body_rotation * wheel_offsets[k] + vec3{0.0f, 0.02f, 0.0f};

        wheel_tire_drawables[k].model.rotation = wheel_rotation;
        wheel_tire_drawables[k].model.translation = wheel_translation;
        wheel_tire_drawables[k].model.scaling = wheel_visual_scale;
        draw(wheel_tire_drawables[k], environment);

        float const rim_side_sign = wheel_offsets[k].z >= 0.0f ? 1.0f : -1.0f;
        vec3 const rim_translation = wheel_translation + body_rotation * vec3{0.0f, 0.0f, rim_side_sign * wheel_rim_outward_offset};
        wheel_rim_drawables[k].model.rotation = wheel_rotation;
        wheel_rim_drawables[k].model.translation = rim_translation;
        wheel_rim_drawables[k].model.scaling = wheel_visual_scale;
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

void scene_structure::emit_smoke(float dt)
{
    if (glfwGetKey(window.glfw_window, GLFW_KEY_S) != GLFW_PRESS)
        return;

    smoke_spawn_accumulator += dt;
    while (smoke_spawn_accumulator >= smoke_spawn_period) {
        smoke_spawn_accumulator -= smoke_spawn_period;

        vec3 const forward = normalize_or(player.facing_direction, {1.0f, 0.0f, 0.0f});
        vec3 const up = normalize_or(player.normal, {0.0f, 1.0f, 0.0f});
        vec3 const right = normalize_or(cross(up, forward), {0.0f, 0.0f, -1.0f});

        smoke_particle particle;
        particle.position = player.position
            - 1.35f * forward
            + rand_uniform(-0.45f, 0.45f) * right
            + 0.25f * up;
        particle.velocity = -0.8f * forward
            + rand_uniform(-0.35f, 0.35f) * right
            + rand_uniform(0.45f, 0.85f) * up;
        particle.lifetime = rand_uniform(1.25f, 1.85f);
        particle.start_size = rand_uniform(0.45f, 0.65f);
        particle.end_size = rand_uniform(1.8f, 2.7f);
        smoke_particles.push_back(particle);
    }
}

void scene_structure::apply_player_boost(float dt)
{
    if (glfwGetKey(window.glfw_window, GLFW_KEY_S) != GLFW_PRESS)
        return;

    vec3 const forward = normalize_or(player.facing_direction, {1.0f, 0.0f, 0.0f});
    vec3 const boost_acceleration = smoke_boost_acceleration * forward;
    player.velocity += boost_acceleration * dt;
    player.acceleration += boost_acceleration;
}

void scene_structure::update_smoke(float dt)
{
    for (smoke_particle& particle : smoke_particles) {
        particle.age += dt;
        particle.position += particle.velocity * dt;
        particle.velocity += vec3{0.0f, 0.18f, 0.0f} * dt;
    }

    smoke_particles.erase(
        std::remove_if(smoke_particles.begin(), smoke_particles.end(), [](smoke_particle const& particle) {
            return particle.age >= particle.lifetime;
        }),
        smoke_particles.end());
}

void scene_structure::draw_smoke()
{
    if (smoke_particles.empty())
        return;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    vec3 const camera_position = camera_control.camera_model.position();
    for (smoke_particle const& particle : smoke_particles) {
        float const t = clamp(particle.age / particle.lifetime, 0.0f, 1.0f);
        float const size = (1.0f - t) * particle.start_size + t * particle.end_size;

        smoke_drawable.model.translation = particle.position;
        smoke_drawable.model.rotation = rotation_transform::from_vector_transform(
            {0.0f, 0.0f, 1.0f},
            normalize_or(camera_position - particle.position, {0.0f, 0.0f, 1.0f}));
        smoke_drawable.model.scaling = size;
        smoke_drawable.material.alpha = 0.65f * (1.0f - t) * (1.0f - t);
        draw(smoke_drawable, environment);
    }

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void scene_structure::position_camera()
{
    if (gui.top_view) {
		vec3 const top_position = vec3{ 0.0f, 720.0f, 25.0f };
        vec3 const track_center = vec3{ 0.0f, 0.0f,   25.0f };
        camera_control.look_at(top_position, track_center, { 0, 0, 1 });
        return;
    }

    if (camera_follows_adversary && !adversaries.empty()) {
        adversary_car const& adversary = adversaries.front();
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

    if (game_started) {
        terrain.resolve_collision(player);
        player.update(dt);
        apply_player_boost(dt);
        emit_smoke(dt);

        bool const race_started = player.throttle_input != 0 || norm(player.velocity) > 0.1f;
        for (adversary_car& adversary : adversaries) {
            terrain.resolve_collision(adversary);

            if (race_started) {
                track_projection const proj = terrain.closest_track_projection(adversary.position);
                vec3 const target = terrain.track_point_ahead(proj, adversary.lookahead_distance);
                adversary.follow_target(target);
            }
            else {
                adversary.throttle_input = 0;
                adversary.steering_input = 0;
                adversary.facing_direction = player.facing_direction;
            }

            adversary.update(dt);
        }

        std::vector<car*> cars;
        cars.reserve(1 + adversaries.size());
        cars.push_back(&player);
        for (adversary_car& adversary : adversaries)
            cars.push_back(&adversary);
        player.verify_collisions(cars);
    }
    update_smoke(dt);

    if (camera_follows_adversary && !adversaries.empty()) {
        adversary_car& adversary = adversaries.front();
        player.camera.position_camera(dt, adversary.facing_direction, adversary.position, adversary.normal);
    }
    else
        player.camera.position_camera(dt, player.facing_direction, player.position, player.normal);

	display_car(player, player_color);
    if (gui.display_hitbox)
        display_hitbox(player, {1.0f, 0.85f, 0.05f});
    for (adversary_car const& adversary : adversaries)
    {
        display_car(adversary, {0.1f, 0.25f, 0.1f});
        if (gui.display_hitbox)
            display_hitbox(adversary, {0.1f, 1.0f, 0.25f});
    }
    draw_smoke();
    position_camera();
}

void scene_structure::display_gui()
{
	ImGui::Checkbox("Frame", &gui.display_frame);
	ImGui::Checkbox("Hitbox", &gui.display_hitbox);
	ImGui::Checkbox("Top View", &gui.top_view);
    ImGui::Text("Hold S: smoke");
    if (ImGui::Button("Back to Menu")) {
        game_started = false;
        reset_race_start();
        player.camera.position_camera(1.0f, player.facing_direction, player.position, player.normal);
    }

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
        ImGui::Text("In collision %d", player.in_colision);
        ImGui::Separator();
        display_vec3_debug("Velocity", player.velocity);
        display_vec3_debug("Acceleration", player.acceleration);
        display_vec3_debug("Facing dir", player.facing_direction);
        display_vec3_debug("Normal", player.normal);
    }
}

void scene_structure::display_start_menu()
{
    if (game_started)
        return;

    char const* map_names[] = {"Grand Prix", "Oval Sprint"};
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

    if (ImGui::Combo("Map", &selected_map, map_names, IM_ARRAYSIZE(map_names))) {
        terrain.map_id = selected_map;
        initialize_track_drawables();
        reset_race_start();
        player.camera.position_camera(1.0f, player.facing_direction, player.position, player.normal);
    }

    ImGui::ColorEdit3("Car Color", &player_color.x);
    ImGui::Spacing();

    if (ImGui::Button("Play", {panel_size.x, 44.0f})) {
        reset_race_start();
        game_started = true;
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
    if (game_started)
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
