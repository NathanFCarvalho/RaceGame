#include "utils.hpp"

#include <algorithm>

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

cgp::vec3 normalize_or(cgp::vec3 const& value, cgp::vec3 const& fallback)
{
    float const value_norm = norm(value);
    if (value_norm > 1e-5f)
        return value / value_norm;

    float const fallback_norm = norm(fallback);
    if (fallback_norm > 1e-5f)
        return fallback / fallback_norm;

    return {1.0f, 0.0f, 0.0f};
}
