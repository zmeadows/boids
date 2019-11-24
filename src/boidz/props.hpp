#pragma once

#include "v2.hpp"

class WinProps {
    static WinProps s_instance;

    void update_(size_t new_width, size_t new_height);

    inline V2 boid_to_window_coordinates_(V2 boid_pos) const
    {
        assert(is_boid_onscreen(boid_pos));

        auto f = [](size_t x) { return static_cast<float>(x); };

        const float rx = f(_sim_region_width) * (boid_pos.x * one_over_boid_span);
        const float ry = f(_sim_region_height) * (boid_pos.y * one_over_boid_span);

        return {f(_sim_region_upper_left_x) + rx, f(_sim_region_upper_left_y) + ry};
    }

    int _window_width = -1;
    int _window_height = -1;

    int _left_panel_upper_left_x = -1;
    int _left_panel_upper_left_y = -1;
    int _left_panel_width = -1;
    int _left_panel_height = -1;

    int _right_panel_upper_left_x = -1;
    int _right_panel_upper_left_y = -1;
    int _right_panel_width = -1;
    int _right_panel_height = -1;

    int _sim_region_upper_left_x = -1;
    int _sim_region_upper_left_y = -1;
    int _sim_region_width = -1;
    int _sim_region_height = -1;

public:
    static int window_width(void) { return s_instance._window_width; }
    static int window_height(void) { return s_instance._window_height; }

    static int left_panel_upper_left_x(void) { return s_instance._left_panel_upper_left_x; }
    static int left_panel_upper_left_y(void) { return s_instance._left_panel_upper_left_y; }
    static int left_panel_width(void) { return s_instance._left_panel_width; }
    static int left_panel_height(void) { return s_instance._left_panel_height; }

    static int right_panel_upper_left_x(void) { return s_instance._right_panel_upper_left_x; }
    static int right_panel_upper_left_y(void) { return s_instance._right_panel_upper_left_y; }
    static int right_panel_width(void) { return s_instance._right_panel_width; }
    static int right_panel_height(void) { return s_instance._right_panel_height; }

    static int sim_region_upper_left_x(void) { return s_instance._sim_region_upper_left_x; }
    static int sim_region_upper_left_y(void) { return s_instance._sim_region_upper_left_y; }
    static int sim_region_width(void) { return s_instance._sim_region_width; }
    static int sim_region_height(void) { return s_instance._sim_region_height; }

    static constexpr float boid_span = 256.f;
    static constexpr float one_over_boid_span = 1.f / boid_span;

    static constexpr bool is_boid_onscreen(V2 pos)
    {
        return (pos.x > 0.f && pos.x < boid_span) && (pos.y > 0.f && pos.y < boid_span);
    }

    static inline void update(size_t new_width, size_t new_height)
    {
        return s_instance.update_(new_width, new_height);
    }

    static inline V2 boid_to_window_coordinates(V2 boid_pos)
    {
        return s_instance.boid_to_window_coordinates_(boid_pos);
    }
};

