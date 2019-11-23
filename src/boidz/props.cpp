#include "props.hpp"

void WindowProps::update(size_t new_width, size_t new_height)
{
    width = new_width;
    height = new_height;

    config_panel_upper_left_x = 0;
    config_panel_upper_left_y = 0;

    config_panel_width = scale_int(new_width, 0.2);
    config_panel_height = new_height;

    sim_region_width = new_width - config_panel_width;
    sim_region_upper_left_x = config_panel_width;
    sim_region_upper_left_y = 0;
}

