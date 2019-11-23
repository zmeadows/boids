// dear imgui: standalone example application for GLFW + OpenGL 3, using programmable
// pipeline If you are new to dear imgui, see examples/README.txt and documentation at the
// top of imgui.cpp. (GLFW is a cross-platform general purpose library for handling
// windows, inputs, OpenGL/Vulkan graphics context creation, etc.)

#include <stdio.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"

// About OpenGL function loaders: modern OpenGL doesn't have a standard header file and
// requires individual function pointers to be loaded manually. Helper libraries are often
// used for this purpose! Here we are supporting a few common ones: gl3w, glew, glad. You
// may use another loader/header of your choice (glext, glLoadGen, etc.), or chose to
// manually implement your own.
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>  // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>  // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>  // Initialize with gladLoadGL()
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif

// Include glfw3.h after our OpenGL definitions
#include <GLFW/glfw3.h>

// [Win32] Our example includes a copy of glfw3.lib pre-compiled with VS2010 to maximize
// ease of testing and compatibility with old VS compilers. To link with VS2010-era
// libraries, VS2015+ requires linking with legacy_stdio_definitions.lib, which we do
// using this pragma. Your own project should not be affected, as you are likely to link
// with a newer binary of GLFW that is adequate for your version of Visual Studio.
#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

#include "boid_collection.hpp"
#include "props.hpp"
#include "quad_tree.hpp"
#include "v2.hpp"

void draw(const BoidCollection& boids, const WindowProps& props)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, props.width, props.height, 0, 100, -100);

    const std::vector<V2>& positions = boids.positions();

    glBegin(GL_POINTS);
    for (const V2& pos : positions) {
        const V2 wpos = props.to_window_coordinates(pos);
        glColor3f(1.f, 1.f, 1.f);
        glVertex2f(wpos.x, wpos.y);
    }
    glEnd();
}

void draw_debug_layout(const WindowProps& props)
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, props.width, props.height, 0, 100, -100);

    glBegin(GL_LINE_STRIP);
    // config panel
    glColor3f(1.f, 0.f, 0.f);
    glVertex2f(1 + props.config_panel_upper_left_x, props.config_panel_upper_left_y);
    glVertex2f(props.config_panel_upper_left_x + props.config_panel_width,
               props.config_panel_upper_left_y);
    glVertex2f(props.config_panel_upper_left_x + props.config_panel_width,
               props.config_panel_upper_left_y + props.config_panel_height - 1);
    glVertex2f(1 + props.config_panel_upper_left_x,
               props.config_panel_upper_left_y + props.config_panel_height - 1);
    glVertex2f(1 + props.config_panel_upper_left_x, props.config_panel_upper_left_y);
    glEnd();

    glBegin(GL_LINE_STRIP);
    glColor3f(0.f, 0.f, 1.f);
    glVertex2f(1 + props.sim_region_upper_left_x, props.sim_region_upper_left_y);
    glVertex2f(props.sim_region_upper_left_x + props.sim_region_width,
               props.sim_region_upper_left_y);
    glVertex2f(props.sim_region_upper_left_x + props.sim_region_width,
               props.sim_region_upper_left_y + props.config_panel_height - 1);
    glVertex2f(1 + props.sim_region_upper_left_x,
               props.sim_region_upper_left_y + props.config_panel_height - 1);
    glVertex2f(1 + props.sim_region_upper_left_x, props.sim_region_upper_left_y);
    glEnd();
}

struct BoidSim {
    BoidCollection boids;
    QuadTree grid;
    RuleParameters params;

    // TODO: pass in frame time
    void tick(const WindowProps& props)
    {
        boids.update(1.f / 60.f, params, grid);
        draw(boids, props);
    }
};

static BoidSim g_sim;

void tick(const WindowProps& props)
{
    glClearColor(0.05f, 0.05f, 0.05f, 1.0f);  // Set background color to black and
    glClear(GL_COLOR_BUFFER_BIT);
    g_sim.tick(props);
}

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

int main(int, char**)
{
    // Setup window
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) return 1;

        // Decide GL+GLSL versions
#if __APPLE__
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    // glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(1280, 720, "WeBoids", NULL, NULL);
    if (window == NULL) return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // Enable vsync

    WindowProps props(1280, 720);

    // Initialize OpenGL loader
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
    bool err = gl3wInit() != 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
    bool err = glewInit() != GLEW_OK;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
    bool err = gladLoadGL() == 0;
#else
    bool err = false;  // If you use IMGUI_IMPL_OPENGL_LOADER_CUSTOM, your loader is
                       // likely to requires some form of initialization.
#endif
    if (err) {
        fprintf(stderr, "Failed to initialize OpenGL loader!\n");
        return 1;
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsClassic();

    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    ImGui::GetStyle().WindowRounding = 0.0f;
    ImGui::GetStyle().ChildRounding = 0.0f;
    ImGui::GetStyle().FrameRounding = 0.0f;
    ImGui::GetStyle().GrabRounding = 0.0f;
    ImGui::GetStyle().PopupRounding = 0.0f;
    ImGui::GetStyle().ScrollbarRounding = 0.0f;

    // Main loop
    while (!glfwWindowShouldClose(window)) {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if
        // dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your
        // main application.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to
        // your main application. Generally you may always pass all inputs to dear imgui,
        // and hide them from your application based on those two flags.
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        {
            ImGui::SetNextWindowPos(
                ImVec2(props.config_panel_upper_left_x, props.config_panel_upper_left_y),
                ImGuiSetCond_Always);

            ImGui::SetNextWindowSize(
                ImVec2(props.config_panel_width, props.config_panel_height),
                ImGuiSetCond_Always);

            ImGui::Begin(
                "weboids", 0,
                ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoMove |
                    ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                    ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoTitleBar);

            RuleParameters& params = g_sim.params;

            ImGui::Checkbox("##CoM_Checkbox", &params.enabled.com);
            ImGui::SameLine();
            ImGui::InputFloat("Center Of Mass", &params.value.com, 0.01f, 1.0f, "%.8f");

            ImGui::Checkbox("##Density_Checkbox", &params.enabled.density);
            ImGui::SameLine();
            ImGui::InputFloat("Density", &params.value.density, 0.01f, 1.0f, "%.8f");

            ImGui::Checkbox("##Confine_Checkbox", &params.enabled.confine);
            ImGui::SameLine();
            ImGui::InputFloat("Confine", &params.value.confine, 0.01f, 1.0f, "%.8f");

            ImGui::Checkbox("##AvgVel_Checkbox", &params.enabled.avg_vel);
            ImGui::SameLine();
            ImGui::InputFloat("Avg. Velocity", &params.value.avg_vel, 0.01f, 1.0f,
                              "%.8f");

            ImGui::Checkbox("##Gravity_Checkbox", &params.enabled.gravity);
            ImGui::SameLine();
            ImGui::InputFloat("Gravity", &params.value.gravity, 0.01f, 1.0f, "%.8f");

            ImGui::End();
        }

        // Rendering
        ImGui::Render();

        int display_w, display_h;
        glfwMakeContextCurrent(window);
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        props.update(display_w, display_h);
        tick(props);
        draw_debug_layout(props);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwMakeContextCurrent(window);
        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}

