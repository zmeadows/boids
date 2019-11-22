#pragma once

#include "v2.hpp"

static constexpr size_t SCREEN_WIDTH_PIXELS = 1200;

namespace window_props {

// all units are pixels.
// static size_t width;
// static size_t height;

// static size_t config_panel_upper_left_x;
// static size_t config_panel_upper_left_y;

// sim region is always a square
static size_t sim_region_upper_left_x = 0;
static size_t sim_region_upper_left_y = 0;

// static size_t config_panel_width;
// static size_t config_panel_height;
static size_t sim_region_width = SCREEN_WIDTH_PIXELS;

// static void resize_window(size_t new_width, size_t new_height)
// {
//     width = new_width;
//     height = new_height;
// }

}  // namespace window_props

namespace boid_coordinates {

static constexpr float span = 256.f;
static constexpr float one_over_span = 1.f / span;
static constexpr float min = 0.f;
static constexpr float max = span;
static constexpr bool is_valid(V2 pos) { return pos.x >= 0.f && pos.x <= 256.f; }

inline V2 to_window_coordinates(V2 boid_pos)
{
    assert(boid_coordinates::is_valid(boid_pos));

    const float rx = (float)window_props::sim_region_width * boid_pos.x *
                     boid_coordinates::one_over_span;
    const float ry = (float)window_props::sim_region_width * boid_pos.y *
                     boid_coordinates::one_over_span;

    return {(float)window_props::sim_region_upper_left_x + rx,
            (float)window_props::sim_region_upper_left_y + ry};
}

}  // namespace boid_coordinates

