/* 
 * lightviewer.cpp
 * 
 * Created on: Nov 15, 2018 10:39
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightview/lightviewer.hpp"

#include <iostream>

#include "lightview/viewer_utils.hpp"

using namespace librav;

namespace
{
static void glfw_error_callback(int error, const char *description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}
} // namespace

//---------------------------------------------------------------------//

LightViewer::~LightViewer()
{
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window_);
    glfwTerminate();
}

bool LightViewer::SetupViewer(int width, int hight, std::string title, bool dark_color)
{
    // Setup window_
    glfwSetErrorCallback(glfw_error_callback);

    if (!glfwInit())
        return false;

        // Decide GL+GLSL versions
#if __APPLE__
    // GL 3.2 + GLSL 150
    const char *glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);           // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char *glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    // Create window_ with graphics context
    window_ = glfwCreateWindow(width, hight, title.c_str(), NULL, NULL);
    if (window_ == NULL)
        return false;

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // Enable vsync

    // Initialize OpenGL loader
    if (gl3wInit() != 0)
    {
        std::cerr << "Failed to initialize OpenGL loader!" << std::endl;
        return false;
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    // Setup Platform/Renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Setup Style
    if (dark_color)
        ImGui::StyleColorsDark();
    else
        ImGui::StyleColorsClassic();

    clear_color_ = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    return true;
}

void LightViewer::PreHouseKeeping()
{
    // Poll and handle events (inputs, window_ resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
    // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void LightViewer::PostHousekeeping()
{
    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwMakeContextCurrent(window_);
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color_.x, clear_color_.y, clear_color_.z, clear_color_.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwMakeContextCurrent(window_);
    glfwSwapBuffers(window_);
}

void LightViewer::Start()
{
    std::cout << "started run" << std::endl;

    bool show_demo_window = true;
    bool show_another_window = false;

    while (!glfwWindowShouldClose(window_))
    {
        PreHouseKeeping();

        // 1. Show the big demo window_ (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        if (show_demo_window)
            ImGui::ShowDemoWindow(&show_demo_window);

        // 2. Show a simple window_ that we create ourselves. We use a Begin/End pair to created a named window_.
        {
            static float f = 0.0f;
            static int counter = 0;

            ImGui::Begin("Hello, world!"); // Create a window_ called "Hello, world!" and append into it.

            ImGui::Text("This is some useful text.");          // Display some text (you can use a format strings too)
            ImGui::Checkbox("Demo Window", &show_demo_window); // Edit bools storing our window_ open/close state
            ImGui::Checkbox("Another Window", &show_another_window);

            ImGui::SliderFloat("float", &f, 0.0f, 1.0f);              // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::ColorEdit3("clear color", (float *)&clear_color_); // Edit 3 floats representing a color

            if (ImGui::Button("Button")) // Buttons return true when clicked (most widgets return true when edited/activated)
                counter++;
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::End();
        }

        {
            ImGui::Text("Hello, world %d", 123);
            if (ImGui::Button("Save"))
            {
                // do stuff
            }
            // ImGui::InputText("string", buf, IM_ARRAYSIZE(buf));
            // ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
        }

        // 3. Show another simple window_.
        if (show_another_window)
        {
            ImGui::Begin("Another Window", &show_another_window); // Pass a pointer to our bool variable (the window_ will have a closing button that will clear the bool when clicked)
            ImGui::Text("Hello from another window_!");
            if (ImGui::Button("Close Me"))
                show_another_window = false;
            ImGui::End();
        }

        PostHousekeeping();
    }
}