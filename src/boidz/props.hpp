#pragma once

#include "v2.hpp"

struct WindowProps {
    static constexpr int scale_int(int x, float sf)
    {
        return static_cast<int>((sf * static_cast<float>(x)));
    }

    // all units are pixels.
    int width;
    int height;

    int config_panel_upper_left_x;
    int config_panel_upper_left_y;

    int sim_region_upper_left_x;
    int sim_region_upper_left_y;

    int config_panel_width;
    int config_panel_height;
    int sim_region_width;

    static constexpr float coordinate_span = 256.f;
    static constexpr float one_over_coordinate_span = 1.f / coordinate_span;
    static constexpr bool is_valid(V2 pos)
    {
        return pos.x >= 0.f && pos.x <= coordinate_span;
    }

    WindowProps(void) = delete;

    WindowProps(size_t w, size_t h) { this->update(w, h); }

    void update(size_t new_width, size_t new_height);

    inline V2 to_window_coordinates(V2 boid_pos) const
    {
        assert(is_valid(boid_pos));

        const float rx =
            static_cast<float>(sim_region_width) * boid_pos.x * one_over_coordinate_span;
        const float ry =
            static_cast<float>(sim_region_width) * boid_pos.y * one_over_coordinate_span;

        return {(float)sim_region_upper_left_x + rx, (float)sim_region_upper_left_y + ry};
    }
};

