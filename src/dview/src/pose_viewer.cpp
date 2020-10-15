/* 
 * pose_viewer.cpp
 * 
 * Created on: Jan 07, 2019 04:46
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "pose_viewer/pose_viewer.hpp"

#include <cassert>

#include "cav_common/cav_datalink.hpp"

#include "drawstuff/drawstuff.h"

using namespace ivnav;

namespace
{
void drawCube(float orientation[3], float position[3], float scale[3], float tone)
{
    // vertices
    GLfloat vertices[] =
        {
            -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1,
            1, -1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1,
            -1, -1, -1, -1, -1, 1, 1, -1, 1, 1, -1, -1,
            -1, 1, -1, -1, 1, 1, 1, 1, 1, 1, 1, -1,
            -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1,
            -1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, 1};

    // Define color gray
    GLfloat colors[72];
    for (int i = 0; i < 72; i++)
    {
        colors[i] = tone;
    }

    glPushMatrix();
    glRotatef(orientation[0], 1, 0, 0);
    glRotatef(orientation[1], 0, 1, 0);
    glRotatef(orientation[2], 0, 0, 1);
    glTranslatef(position[0], position[1], position[2]);
    glScalef(scale[0], scale[1], scale[2]);

    //We have a color array and a vertex array
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glVertexPointer(3, GL_FLOAT, 0, vertices);
    glColorPointer(3, GL_FLOAT, 0, colors);

    //Send data : 24 vertices
    glDrawArrays(GL_QUADS, 0, 24);

    //Cleanup states
    glDisableClientState(GL_COLOR_ARRAY);
    glDisableClientState(GL_VERTEX_ARRAY);
    glPopMatrix();
}

void drawSphere(int lats, int longs, GLfloat x, GLfloat y, GLfloat z)
{
    glPushMatrix();
    glTranslatef(x, y + (GLfloat)0.15, z);
    glScalef((GLfloat)0.47, (GLfloat)0.47, (GLfloat)0.47);

    int i, j;

    for (i = 0; i <= lats; i++)
    {
        double lat0 = M_PI * (-0.5 + (double)(i - 1) / lats);
        double z0 = sin(lat0);
        double zr0 = cos(lat0);

        double lat1 = M_PI * (-0.5 + (double)i / lats);
        double z1 = sin(lat1);
        double zr1 = cos(lat1);

        glColor3f((GLfloat)1, (GLfloat)0.4, (GLfloat)0.1);
        glBegin(GL_QUAD_STRIP);

        for (j = 0; j <= longs; j++)
        {
            double lng = 2 * M_PI * (double)(j - 1) / longs;
            double x2 = cos(lng);
            double y2 = sin(lng);

            glNormal3f((GLfloat)(x2 * zr0), (GLfloat)(y2 * zr0), (GLfloat)z0);
            glVertex3f((GLfloat)(x2 * zr0), (GLfloat)(y2 * zr0), (GLfloat)z0);
            glNormal3f((GLfloat)(x2 * zr1), (GLfloat)(y2 * zr1), (GLfloat)z1);
            glVertex3f((GLfloat)(x2 * zr1), (GLfloat)(y2 * zr1), (GLfloat)z1);
        }
        glEnd();
    }
    glPopMatrix();
}
} // namespace

PoseViewer::PoseViewer() : LightViewer()
{
    // setup communication link
    data_link_ = std::make_shared<LCMLink>();
    if (!data_link_->good())
        std::cerr << "ERROR: Failed to initialize LCM." << std::endl;
    data_link_ready_ = true;

    data_link_->subscribe(CAV_COMMON_CHANNELS::VEHICLE_ESTIMATIONS_CHANNEL, &PoseViewer::HandleVehicleEstimationsMsg, this);
    data_link_->subscribe(CAV_COMMON_CHANNELS::EGO_VEHICLE_STATE_CHANNEL, &PoseViewer::HandleEgoVehicleStateMsg, this);
}

void PoseViewer::HandleEgoVehicleStateMsg(const ivnav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleState *msg)
{
    ego_vehicle_state_ = VehicleState(msg->id, {msg->position[0], msg->position[1], msg->theta}, msg->speed);
    ego_state_updated_ = true;
}

void PoseViewer::HandleVehicleEstimationsMsg(const ivnav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleEstimations *msg)
{
    surrounding_vehicles_.clear();
    for (int i = 0; i < msg->vehicle_num; ++i)
    {
        auto est = msg->estimations[i];
        surrounding_vehicles_.emplace_back(VehicleState(est.id, {est.position[0], est.position[1], est.theta}, est.speed));
    }
}

void PoseViewer::Start()
{
    std::cout << "pose viewer started" << std::endl;

    bool show_another_window = false;
    float rotateCube = 0;

    while (!glfwWindowShouldClose(window_))
    {
        PreHouseKeeping();

        // process LCM callbacks first
        data_link_->handleTimeout(0);

        //-------------------------------------------------------------//

        static float f = 0.0f;
        static int counter = 0;

        // static bool show_center_line = false;
        // static bool use_jetcolor_bg = false;
        // static bool centered_at_ego_vehicle = true;
        // static float zoom_in_ratio = 0.56f;
        // static bool save_image = false;
        static float cube_color_tone = 0.56f;
        static int ori_x = 30;
        static int ori_y = 0;
        static int ori_z = 0;
        static int pos_x = 0;
        static int pos_y = -10;
        static int pos_z = -20;

        //-------------------------------------------------------------//

        ImGui::SetNextWindowPos(ImVec2(0, 0), 0, ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(100, 700), ImGuiCond_FirstUseEver);

        ImGui::Begin("Settings:");

        // ImGui::Text("Road Map:");
        // ImGui::Checkbox("Show Centerline", &show_center_line);
        // ImGui::Checkbox("Jetcolor Map", &use_jetcolor_bg);

        // ImGui::Text("Traffic:");
        // ImGui::Checkbox("Centered at Ego Vehicle", &centered_at_ego_vehicle);
        // ImGui::SliderFloat("Zoom-in Ratio", &zoom_in_ratio, 0.0f, 1.0f);
        ImGui::SliderFloat("Cube color tune", &cube_color_tone, 0.0f, 1.0f);
        ImGui::SliderInt("Position x", &pos_x, -100, 100);
        ImGui::SliderInt("Position y", &pos_y, -100, 100);
        ImGui::SliderInt("Position z", &pos_z, -100, 100);
        ImGui::SliderInt("Orientation x", &ori_x, -100, 100);
        ImGui::SliderInt("Orientation y", &ori_y, -100, 100);
        ImGui::SliderInt("Orientation z", &ori_z, -100, 100);
        // ImGui::Text("Save Result:");
        // static char img_name_buf[32] = "traffic_viewer";
        // ImGui::InputText("", img_name_buf, IM_ARRAYSIZE(img_name_buf));
        // ImGui::SameLine();
        // if (ImGui::Button("Save Image"))
        //     save_image = true;
        // ImGui::SameLine();

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

        ImGui::End();

        //-------------------------------------------------------------//

        int display_w, display_h;
        PrepareOpenGLDraw(display_w, display_h);

        //-------------------------------------------------------------//

        {
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();

            gluPerspective(60, (double)display_w / (double)display_h, 0.1, 3000);

            glMatrixMode(GL_MODELVIEW);

            glClearColor(clear_color_.x, clear_color_.y, clear_color_.z, clear_color_.w);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glLoadIdentity();

            // Positions the "camera"
            // Both respectively X, Y and Z
            // int orientation[3] = {60, 0, 0};
            // int position[3] = {0, -20, -15};
            // int orientation[3] = {30, 0, 0};
            // int position[3] = {0, -10, -15};
            int orientation[3] = {ori_x, ori_y, ori_z};
            int position[3] = {pos_x, pos_y, pos_z};

            glRotatef((GLfloat)orientation[0], (GLfloat)1, 0, 0);
            glRotatef((GLfloat)orientation[1], 0, (GLfloat)1, 0);
            glRotatef((GLfloat)orientation[2], 0, 0, (GLfloat)1);
            glTranslatef((GLfloat)position[0], (GLfloat)position[1], (GLfloat)position[2]);

            // draw floor
            float orientationFloor[3] = {0, 0, 0}; // Radians
            float positionFloor[3] = {0, 0, 0};
            float scaleFloor[3] = {15, (float)0.05, 13};
            drawCube(orientationFloor, positionFloor, scaleFloor, (float)0.35);

            // draw sphere
            // drawSphere(10, 10, 3, 0, 0);

            // draw cube
            float orientationCube[3] = {0, rotateCube, 0}; // Radians
            float positionCube[3] = {0, 1, 2};
            float scaleCube[3] = {1, 1, 1};
            drawCube(orientationCube, positionCube, scaleCube, (float)0.75);
            // drawCube(orientationCube, positionCube, scaleCube, cube_color_tone);
            rotateCube += (float)0.2;
        }

        //-------------------------------------------------------------//

        RenderData();
    }
}