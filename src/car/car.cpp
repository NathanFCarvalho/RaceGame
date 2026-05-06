#include "car.hpp"
using namespace cgp;

namespace {
float approach_zero(float value, float amount)
{
    if (value > 0.0f)
        return std::max(0.0f, value - amount);
    return std::min(0.0f, value + amount);
}

float clamp_value(float value, float min_value, float max_value)
{
    return std::max(min_value, std::min(value, max_value));
}
}

car_structure::car_structure()
{
    throttle_input = 0;
    steering_input = 0;
    steering_angle = 0.0f;
    angular_speed = 0.0f;
    wheel_spin_angle = 0.0f;
	position = {0.0f, 0.0f, 0.0f};
	velocity = {0.0f, 0.0f, 0.0f};
	acceleration = {0.0f, 0.0f, 0.0f};
    normal = {0.0f, 1.0f, 0.0f};
    facing_direction = {1.0f, 0.0f, 0.0f};
}

float car_structure::forward_speed() const
{
    vec3 const forward = normalize(facing_direction);
    return dot(velocity, forward);
}

float car_structure::lateral_speed() const
{
    vec3 const forward = normalize(facing_direction);
    vec3 const right = normalize(cross(normal, forward));
    return dot(velocity, right);
}

void car_structure::update(float dt)
{
    if (steering_input != 0)
        steering_angle += steering_input * steering_speed * dt;
    else
        steering_angle = approach_zero(steering_angle, steering_return_speed * dt);

    steering_angle = clamp_value(steering_angle, -max_steering_angle, max_steering_angle);

    vec3 const forward = normalize(facing_direction);
    vec3 const right = normalize(cross(normal, forward));
    float const current_forward_speed = dot(velocity, forward);
    float const current_lateral_speed = dot(velocity, right);
    wheel_acceleration = throttle_input * acceleration_value;

    float const forward_acceleration = wheel_acceleration - drag_factor * current_forward_speed;
    float const lateral_acceleration = -lateral_drag_factor * current_lateral_speed;
    acceleration = forward_acceleration * forward + lateral_acceleration * right;

    float updated_forward_speed = current_forward_speed + forward_acceleration * dt;
    angular_speed = updated_forward_speed * std::tan(steering_angle) / wheel_base;

    if (std::abs(angular_speed) > 1e-5f && std::abs(updated_forward_speed) > 1e-5f) {
        rotation_transform yaw_rotation =
            rotation_transform::from_axis_angle(normal, angular_speed * dt);
        facing_direction = normalize(yaw_rotation * facing_direction);
    }

    velocity += acceleration * dt;
    wheel_spin_angle += updated_forward_speed * dt / wheel_radius;
    position += velocity * dt;
}

void car_structure::action_keyboard(input_devices* inputs, window_structure* window)
{
    assert_cgp_no_msg(inputs != nullptr);
    assert_cgp_no_msg(window != nullptr);

    throttle_input = 0;
    if (inputs->keyboard.up)
        throttle_input += 1;
    if (inputs->keyboard.down)
        throttle_input -= 1;

    steering_input = 0;
    if (inputs->keyboard.left)
        steering_input += 1;
    if (inputs->keyboard.right)
        steering_input -= 1;
}


// Used for debbuging
void car_structure::set_trajectory(float t){
    position = vec3(2.0f*std::cos(2 * Pi * t * 0.1), 0.0f, 2.0f*std::sin(2 * Pi * t * 0.1));
    velocity = vec3(-0.2f*std::sin(2 * Pi * t * 0.1), 0.0f, 0.2f*std::cos(2 * Pi * t * 0.1));
    if (norm(velocity) > 1e-5f)
        facing_direction = normalize(velocity);
}
