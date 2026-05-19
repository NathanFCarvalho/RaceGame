#pragma once

#include "cgp/cgp.hpp"

float approach_zero(float value, float amount);
float clamp_value(float value, float min_value, float max_value);
cgp::vec3 normalize_or(cgp::vec3 const& value, cgp::vec3 const& fallback);
