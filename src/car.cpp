#include "car.hpp"
#include "utils.hpp"

using namespace cgp;

/**************************
 Camera movement definition
 **************************/

camera_control::camera_control()
{
    position = {-up_offset, back_offset, 0.0f};
    distance_from_car = {-up_offset, back_offset, 0.0f};
    camera_smoothed_direction = {1.0f, 0.0f, 0.0f};
    camera_smoothed_position = position;
}

void camera_control::position_camera(float dt, vec3& car_facing_direction, vec3& car_position, vec3& car_up) 
{
    vec3 const target_direction = normalize_or(car_facing_direction, {1.0f, 0.0f, 0.0f});
    vec3 const up = normalize_or(car_up, {0.0f, 1.0f, 0.0f});
	vec3 target_position = car_position - back_offset * target_direction + up_offset * up;
    float position_alpha = 1.0f - std::exp(-camera_position_response * dt);
    position = (1.0f - position_alpha) * position + position_alpha * target_position;
}

/**************************
 Car structure definition
 **************************/

cgp::mesh car_structure::create_wheel_mesh() const
{
    return mesh_primitive_cylinder(wheel_radius, {0.0f, 0.0f, -wheel_tire_half_width}, {0.0f, 0.0f, wheel_tire_half_width}, 2, 32, false);
}

cgp::mesh car_structure::create_wheel_rim_mesh() const
{
    mesh first_disc = mesh_primitive_disc(wheel_radius, {0.0f, 0.0f, -0.055f}, {0.0f, 0.0f, -1.0f}, 32);
    mesh second_disc = mesh_primitive_disc(wheel_radius, {0.0f, 0.0f, 0.055f}, {0.0f, 0.0f, 1.0f}, 32);
    first_disc.push_back(second_disc);
    first_disc.fill_empty_field();
    return first_disc;
}

/**************************
 Car movement definition
 **************************/

car::car()
{
    // Movement variables (car starts at rest)
    wheel_acceleration = 0.0f;
    steering_angle = 0.0f;
    angular_speed = 0.0f;
    wheel_spin_angle = 0.0f;

    position = {0.0f, 0.0f, 0.0f};
	velocity = {0.0f, 0.0f, 0.0f};
	acceleration = {0.0f, 0.0f, 0.0f};

    // Car starts in xz plan facing the x axis
    normal = {0.0f, 1.0f, 0.0f};
    facing_direction = {-1.0f, 0.0f, 0.0f};
}

void car::update(float dt)
{
    if (steering_input != 0)
        steering_angle += steering_input * constants.steering_speed * dt;
    else
        steering_angle = approach_zero(steering_angle, constants.steering_return_speed * dt);

    steering_angle = clamp_value(
        steering_angle,
        -constants.max_steering_angle,
        constants.max_steering_angle);

    vec3 const up = normalize_or(normal, {0.0f, 1.0f, 0.0f});
    vec3 const forward = normalize_or(facing_direction, {1.0f, 0.0f, 0.0f});
    vec3 const right = normalize_or(cross(up, forward), {0.0f, 0.0f, -1.0f});
    float const current_forward_speed = dot(velocity, forward);
    float const current_lateral_speed = dot(velocity, right);
    wheel_acceleration = throttle_input * constants.acceleration_value;

    float const forward_acceleration = wheel_acceleration - constants.drag_factor * current_forward_speed;
    float const lateral_acceleration = -constants.lateral_drag_factor * current_lateral_speed;
    acceleration = forward_acceleration * forward + lateral_acceleration * right;

    float const updated_forward_speed = current_forward_speed + forward_acceleration * dt;
    angular_speed = updated_forward_speed * std::tan(steering_angle) / dimensions.wheel_base;

    if (std::abs(angular_speed) > 1e-5f && std::abs(updated_forward_speed) > 1e-5f) {
        rotation_transform const yaw_rotation =
            rotation_transform::from_axis_angle(up, angular_speed * dt);
        facing_direction = normalize(yaw_rotation * facing_direction);
    }

    velocity += acceleration * dt;
    wheel_spin_angle += updated_forward_speed * dt / dimensions.wheel_radius;
    position += velocity * dt;
}

float car::forward_speed() const
{
    vec3 const forward = normalize_or(facing_direction, {1.0f, 0.0f, 0.0f});
    return dot(velocity, forward);
}

float car::lateral_speed() const
{
    vec3 const forward = normalize_or(facing_direction, {1.0f, 0.0f, 0.0f});
    vec3 const up = normalize_or(normal, {0.0f, 1.0f, 0.0f});
    vec3 const right = normalize_or(cross(up, forward), {0.0f, 0.0f, -1.0f});
    return dot(velocity, right);
}

/**************************
 Player movement definition
 **************************/

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
    camera.position_camera(dt, facing_direction, position, normal);
}

/**************************
 Adversary movement definition
 **************************/

void adversary_car::follow_target(vec3 const& target_position)
{
    vec3 const forward = normalize_or(facing_direction, {1.0f, 0.0f, 0.0f});
    vec3 const up = normalize_or(normal, {0.0f, 1.0f, 0.0f});
    vec3 const to_target = normalize_or(target_position - position, forward);

    float const heading_error = std::atan2(
        dot(cross(forward, to_target), up),
        dot(forward, to_target));
    float const target_steering_angle = clamp_value(
        heading_gain * heading_error,
        -constants.max_steering_angle,
        constants.max_steering_angle);

    float const steering_ratio = std::abs(target_steering_angle) / constants.max_steering_angle;
    float const corner_speed_ratio = clamp_value(1.0f - steering_ratio, min_corner_speed_ratio, 1.0f);
    float const target_speed = adversary_target_speed * corner_speed_ratio;
    throttle_input = forward_speed() < target_speed ? 1 : 0;

    steering_input = 0;
    if (target_steering_angle > steering_angle + steering_dead_zone)
        steering_input = 1;
    if (target_steering_angle < steering_angle - steering_dead_zone)
        steering_input = -1;
}
