#include "car.hpp"
#include "utils.hpp"

using namespace cgp;

/****************************************************
 Camera movement definition
 ****************************************************/

camera_control::camera_control()
{
    position = {-up_offset, back_offset, 0.0f};
    distance_from_car = {-up_offset, back_offset, 0.0f};
}

void camera_control::position_camera(float dt, vec3& car_forward, vec3& car_position, vec3& car_up) 
{
    vec3 const target_direction = normalize_or(car_forward, {1.0f, 0.0f, 0.0f}); // faces the same direction as the car
    vec3 const up = normalize_or(car_up, {0.0f, 1.0f, 0.0f});
	vec3 target_position = car_position - back_offset * target_direction + up_offset * up; // positions the camera behind the car
    float position_alpha = 1.0f - std::exp(-camera_position_response * dt); // sets a delay from last position
    position = (1.0f - position_alpha) * position + position_alpha * target_position;
}

/****************************************************
 Car structure definition
 ****************************************************/

cgp::mesh car_structure::create_wheel_mesh() const
{
    return mesh_primitive_cylinder(wheel_radius, {0.0f, 0.0f, -wheel_tire_half_width}, {0.0f, 0.0f, wheel_tire_half_width}, 2, 32, false);
}

cgp::mesh car_structure::create_wheel_rim_mesh() const
{
    mesh first_disc = mesh_primitive_disc(wheel_radius, {0.0f, 0.0f, -tire_position}, {0.0f, 0.0f, -1.0f}, 32);
    mesh second_disc = mesh_primitive_disc(wheel_radius, {0.0f, 0.0f, tire_position}, {0.0f, 0.0f, 1.0f}, 32);
    first_disc.push_back(second_disc);
    first_disc.fill_empty_field();
    return first_disc;
}

/****************************************************
 Car movement definition
 ****************************************************/

car::car()
{
    idx = 0;
    in_colision = false;

    // Movement variables (car starts at rest)
    wheel_acceleration = 0.0f;
    steering_angle = 0.0f;
    angular_speed = 0.0f;
    
    collision_angular_speed = 0.0f;
    wheel_spin_angle = 0.0f; // rotation
    throttle_input = 0;
    steering_input = 0;

    position = {0.0f, 0.0f, 0.0f};
	velocity = {0.0f, 0.0f, 0.0f};
	acceleration = {0.0f, 0.0f, 0.0f};

    // Default placement
    normal = {0.0f, 1.0f, 0.0f};
    forward = {-0.0f, 0.0f, 1.0f};
    update_direction_vectors();
}

void reset_car_state(car& vehicle)
{
    vehicle.in_colision = false;
    vehicle.wheel_acceleration = 0.0f;
    vehicle.steering_angle = 0.0f;
    vehicle.angular_speed = 0.0f;
    vehicle.collision_angular_speed = 0.0f;
    vehicle.wheel_spin_angle = 0.0f;
    vehicle.throttle_input = 0;
    vehicle.steering_input = 0;
    vehicle.position = {0.0f, 0.0f, 0.0f};
    vehicle.velocity = {0.0f, 0.0f, 0.0f};
    vehicle.acceleration = {0.0f, 0.0f, 0.0f};
    vehicle.normal = {0.0f, 1.0f, 0.0f};
    vehicle.forward = {-0.0f, 0.0f, 1.0f};
    vehicle.update_direction_vectors();
}

float adversary_start_lateral_offset(size_t adversary_index, float spacing)
{
    float const side = adversary_index % 2 == 0 ? 1.0f : -1.0f;
    float const rank = 1.0f + static_cast<float>(adversary_index / 2);
    return side * rank * spacing;
}

void car::update_direction_vectors()
{
    forward = normalize_or(forward, {1.0f, 0.0f, 0.0f});
    up = normalize_or(normal, {0.0f, 1.0f, 0.0f});
    right = normalize_or(cross(up, forward), {0.0f, 0.0f, -1.0f});
}

void car::update(float dt)
{
    update_direction_vectors();

    // While pressing left or right, steering angle increases or decreases with constant steering speed.
    // If nothing is pressed, angle tends again to the center with constant speed.
    if (steering_input != 0)
        steering_angle += steering_input * constants.steering_speed * dt;
    else
        steering_angle = approach_zero(steering_angle, constants.steering_return_speed * dt);

    // Steering angle has a maximum value.
    steering_angle = clamp_value(
        steering_angle,
        -constants.max_steering_angle,
        constants.max_steering_angle);
    
    float const current_forward_speed = dot(velocity, forward);
    float const current_lateral_speed = dot(velocity, right);
    
    // Add drag factor to the acceleration. Lateral drag factor is larger.
    wheel_acceleration = throttle_input * constants.acceleration_value;
    float const forward_acceleration = wheel_acceleration - constants.drag_factor * current_forward_speed;
    float const lateral_acceleration = -constants.lateral_drag_factor * current_lateral_speed;
    acceleration = forward_acceleration * forward + lateral_acceleration * right;

    // v += a * dt
    float const updated_forward_speed = current_forward_speed + forward_acceleration * dt;
    // Byclicle turning model
    angular_speed = updated_forward_speed * std::tan(steering_angle) / dimensions.wheel_base;

    if (std::abs(angular_speed) > 1e-5f && std::abs(updated_forward_speed) > 1e-5f) {
        rotation_transform const yaw_rotation =
            rotation_transform::from_axis_angle(up, angular_speed * dt);
        forward = normalize(yaw_rotation * forward);
        update_direction_vectors();
    }

    // Collision rotation effect
    if (std::abs(collision_angular_speed) > 1e-5f) {
        rotation_transform const collision_rotation =
            rotation_transform::from_axis_angle(up, collision_angular_speed * dt);
        forward = normalize(collision_rotation * forward);
        update_direction_vectors();
        collision_angular_speed *= std::exp(-2.5f * dt);
    }

    velocity += acceleration * dt;
    position += velocity * dt;
    wheel_spin_angle += updated_forward_speed * dt / dimensions.wheel_radius; // roling 
}

std::array<vec3, 4> car::get_hitbox_samples() const
{
    float half_length = dimensions.length / 2.0;
    float half_width = dimensions.width / 2.0;

    std::array<vec3, 4> const hitbox_samples = {{
        half_length * forward + half_width * right,
        half_length * forward - half_width * right,
        -half_length * forward + half_width * right,
        -half_length * forward - half_width * right,
    }};

    return hitbox_samples;
}

/****************************************************
 Player movement definition
 ****************************************************/

void player_car::action_keyboard(input_devices* inputs, window_structure* window)
{
    assert_cgp_no_msg(inputs != nullptr);
    assert_cgp_no_msg(window != nullptr);

    throttle_input = 0;
    if (inputs->keyboard.up)
        throttle_input = 1;
    if (inputs->keyboard.down)
        throttle_input = -1;

    steering_input = 0;
    if (inputs->keyboard.left)
        steering_input = 1;
    if (inputs->keyboard.right)
        steering_input = -1;
}

void player_car::position_camera(float dt)
{
    camera.position_camera(dt, forward, position, normal);
}

/****************************************************
 Adversary movement definition
 ****************************************************/

void adversary_car::follow_direction(vec3 const& target_direction)
{
    update_direction_vectors();
    vec3 const desired_direction = normalize_or(target_direction, forward);

    // Calculate the angle between forward direction and the desired direction.
    float const heading_error = std::atan2(
        dot(cross(forward, desired_direction), up),
        dot(forward, desired_direction));
    float const target_steering_angle = clamp_value(
        heading_gain * heading_error,
        -constants.max_steering_angle,
        constants.max_steering_angle);

    // Car slows down when it turns
    float const steering_ratio = std::abs(target_steering_angle) / constants.max_steering_angle;
    float const corner_speed_ratio = clamp_value(1.0f - steering_ratio, min_corner_speed_ratio, 1.0f);
    float const target_speed = adversary_target_speed * corner_speed_ratio;
    float const current_forward_speed = dot(velocity, forward);
    throttle_input = current_forward_speed < target_speed ? 1 : 0;

    steering_input = 0;
    if (target_steering_angle > steering_angle + steering_dead_zone)
        steering_input = 1;
    if (target_steering_angle < steering_angle - steering_dead_zone)
        steering_input = -1;
}

void adversary_car::follow_target(vec3 const& target_position)
{
    follow_direction(target_position - position);
}

void adversary_car::align_with_track_tangent(vec3 const& target_tangent)
{
    follow_direction(target_tangent);
}

/**************************
 Car colision definition
 **************************/

bool is_inside_hitbox(vec3 const& point, car const& vehicle)
{
    vec3 const relative_position = point - vehicle.position;

    float const local_forward = dot(relative_position, vehicle.forward);
    float const local_right = dot(relative_position, vehicle.right);

    return std::abs(local_forward) <= vehicle.dimensions.length/2.0
        && std::abs(local_right) <= vehicle.dimensions.width/2.0;
}

bool hitboxes_overlap(car const& first, car const& second, vec3& contact_point)
{
    for (vec3 const& hitbox_sample : first.get_hitbox_samples()) {
        vec3 const world_sample = first.position + hitbox_sample;
        if (is_inside_hitbox(world_sample, second)) {
            contact_point = world_sample;
            return true;
        }
    }

    for (vec3 const& hitbox_sample : second.get_hitbox_samples()) {
        vec3 const world_sample = second.position + hitbox_sample;
        if (is_inside_hitbox(world_sample, first)) {
            contact_point = world_sample;
            return true;
        }
    }

    return false;
}

vec3 collision_normal(car const& first, car const& second)
{
    vec3 const center_direction = first.position - second.position;
    if (norm(center_direction) > 1e-5f)
        return normalize(center_direction);

    return normalize_or(first.velocity - second.velocity, first.forward);
}

void add_collision_spin(car& vehicle, vec3 const& contact_point, vec3 const& impulse)
{
    vec3 const contact_offset = contact_point - vehicle.position;
    float const torque = dot(cross(contact_offset, impulse), vehicle.up);
    float const half_length = vehicle.dimensions.length/2.0;
    float const half_width = vehicle.dimensions.width/2.0;
    float const inertia = std::max(half_length * half_length + half_width * half_width, 1e-3f);
    float constexpr spin_response = 0.35f;

    vehicle.collision_angular_speed += spin_response * torque / inertia;
    vehicle.collision_angular_speed = clamp_value(vehicle.collision_angular_speed, -8.0f, 8.0f);
}

void resolve_elastic_collision(car& first, car& second, vec3 const& contact_point)
{
    vec3 const normal = collision_normal(first, second);
    vec3 const relative_velocity = first.velocity - second.velocity;
    float const normal_relative_speed = dot(relative_velocity, normal);

    if (normal_relative_speed >= 0.0f)
        return;

    float constexpr restitution = 1.0f;
    float constexpr inverse_mass = 1.0f;
    float const impulse_magnitude =
        -(1.0f + restitution) * normal_relative_speed / (inverse_mass + inverse_mass);
    vec3 const impulse = impulse_magnitude * normal;

    first.velocity += inverse_mass * impulse;
    second.velocity -= inverse_mass * impulse;
    add_collision_spin(first, contact_point, impulse);
    add_collision_spin(second, contact_point, -impulse);
}

void player_car::verify_collisions(std::vector<car*> const& cars)
{
    for (car* vehicle : cars) {
        if (vehicle != nullptr)
            vehicle->in_colision = false;
    }

    for (size_t first_index = 0; first_index < cars.size(); ++first_index) {
        car* first = cars[first_index];
        if (first == nullptr)
            continue;

        for (size_t second_index = first_index + 1; second_index < cars.size(); ++second_index) {
            car* second = cars[second_index];
            if (second == nullptr)
                continue;

            vec3 contact_point;
            if (!hitboxes_overlap(*first, *second, contact_point))
                continue;

            resolve_elastic_collision(*first, *second, contact_point);
            first->in_colision = true;
            second->in_colision = true;
        }
    }
} 
