#include "car.hpp"

using namespace cgp;
car_structure::car_structure()
{
    initial_position = {0.0f, 0.0f, 0.0f};
	current_position = {0.0f, 0.0f, 0.0f};
	current_velocity = {1.0f, 0.0f, 0.0f};
	current_acceleration = {0.0f, 0.0f, 0.0f};
}

void car_structure::update(float dt)
{
    current_velocity += current_acceleration * dt;
    current_position += (current_velocity * dt + current_acceleration * dt*dt/2.0);
}

void car_structure::action_keyboard(input_devices* inputs, window_structure* window)
{
    assert_cgp_no_msg(inputs != nullptr);
    assert_cgp_no_msg(window != nullptr);

    bool const click_up = inputs->keyboard.up;
    bool const click_down = inputs->keyboard.down;

    if (click_up)     
        accelerate();
    else if (click_down) 
        decelerate();
    return;
}

void car_structure::accelerate()
{
    current_acceleration = {1.0f, 0.0f, 0.0f};
    return;
}

void car_structure::decelerate()
{
    current_acceleration = {-1.0f, 0.0f, 0.0f};
    return;
}