#include "props.hpp"

WinProps WinProps::s_instance;

static constexpr int scale_int(int x, float sf)
{
    return static_cast<int>((sf * static_cast<float>(x)));
}

void WinProps::update_(size_t new_width, size_t new_height)
{
    _window_width = new_width;
    _window_height = new_height;

    _sim_region_width = _window_height;

    const float remaining_width = _window_width - _sim_region_width;
    const float panel_width = scale_int(remaining_width, 0.5);

    _left_panel_width = panel_width;
    _right_panel_width = panel_width;

    _sim_region_upper_left_x = _left_panel_width;
    _sim_region_upper_left_y = 0;
    _sim_region_height = _window_height;

    _left_panel_upper_left_x = 0;
    _left_panel_upper_left_y = 0;
    _left_panel_height = _window_height;

    _right_panel_upper_left_x = _window_width - _right_panel_width;
    _right_panel_upper_left_y = 0;
    _right_panel_height = _window_height;
}

