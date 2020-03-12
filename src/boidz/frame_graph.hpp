#pragma once

class TimeGraph {
    bool m_cycled_once;
    float m_average_frame_time;
    int m_next_idx;
    int m_frames_to_keep;
    std::vector<float> m_frame_times;

public:
    TimeGraph(void)
        : m_cycled_once(false),
          m_average_frame_time(0.f),
          m_next_idx(0),
          m_frames_to_keep(60 * 3),
          m_frame_times(m_frames_to_keep, 0.f)
    {
    }

    void attach_new_time_delta(float dt)
    {
        m_frame_times[m_next_idx] = dt;
        m_next_idx = (m_next_idx + 1) % m_frames_to_keep;
        m_average_frame_time =
            std::accumulate(m_frame_times.begin(), m_frame_times.end(), 0.f) / m_frames_to_keep;

        if (m_next_idx == 0) {
            m_cycled_once = true;
        }
    }

    void draw(const char* label)
    {
        if (m_cycled_once) {
            char buffer[256];
            sprintf(buffer, "%.*f", 6, m_average_frame_time);
            ImGui::PlotLines(label, m_frame_times.data(), m_frames_to_keep, m_next_idx, buffer);
        }
        else {
            ImGui::PlotLines(label, m_frame_times.data(), m_next_idx, 0);
        }
    }
};
