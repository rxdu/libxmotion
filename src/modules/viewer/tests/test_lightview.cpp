#include <GLFW/glfw3.h>

#include <iostream>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

int main(int argc, char* argv[]) {
  std::cout << "start gui" << std::endl;

  GLFWwindow* window;

  if (!glfwInit()) return -1;
  window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
  if (!window) {
    std::cerr << "Could not create OpenGL window" << std::endl;
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);

  //   ImGui::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL(window, false);

  /* Loop until the user closes the window */
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    /* Render here */
    // glClear(GL_COLOR_BUFFER_BIT);

    {
      static float f = 0.0f;
      static int counter = 0;

      ImGui::Begin("Hello, world!");  // Create a window called "Hello, world!"
                                      // and append into it.

      ImGui::Text("This is some useful text.");  // Display some text (you can
                                                 // use a format strings too)
                                                 //   ImGui::Checkbox(
                                                 //       "Demo Window",
      //       &show_demo_window);  // Edit bools storing our window open/close
      //       state
      //   ImGui::Checkbox("Another Window", &show_another_window);

      ImGui::SliderFloat(
          "float", &f, 0.0f,
          1.0f);  // Edit 1 float using a slider from 0.0f to 1.0f
                  //   ImGui::ColorEdit3(
                  //       "clear color",
      //       (float*)&clear_color);  // Edit 3 floats representing a color

      if (ImGui::Button(
              "Button"))  // Buttons return true when clicked (most widgets
                          // return true when edited/activated)
        counter++;
      ImGui::SameLine();
      ImGui::Text("counter = %d", counter);

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                  1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::End();
    }

    ImGui::Render();
    // int display_w, display_h;
    // glfwGetFramebufferSize(window, &display_w, &display_h);
    // glViewport(0, 0, display_w, display_h);
    // glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    // glClear(GL_COLOR_BUFFER_BIT);
    // ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

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