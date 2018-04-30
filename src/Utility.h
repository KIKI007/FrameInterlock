//
// Created by ziqwang on 25.04.18.
//

#ifndef FRAMEINTERLOCK_UTILITY_H
#define FRAMEINTERLOCK_UTILITY_H
#include <iostream>
#include <cstring>
#include <memory>

#include "igl/viewer/Viewer.h"
#include <igl/readOFF.h>

#include "FrameMesh.h"
#include "FrameInterface.h"
#include "FrameInterlocking.h"

using std::string;
using Eigen::MatrixXd;
using Eigen::MatrixXi;

extern igl::viewer::Viewer viewer;
extern std::string shared_ptr;
extern std::shared_ptr<ColorCoding> colorcoder;
extern std::shared_ptr<FrameInterface> frame_interface;


struct Parameter
{
    bool render_frame_mesh;

    bool render_joint_knots;

    bool render_show_pillar_index;

    bool render_show_joint_index;

    bool render_show_joint_face_index;

    double render_joint_size_;

    double render_pillar_radius;

    int render_num_joint_voxel;

    int render_foucus_joint_index;

    int render_add_pillar_f0;

    int render_add_pillar_f1;
}para;

void draw_frame_mesh()
{
    if(frame_interface)
    {
        MatrixXd renderV, renderC;
        MatrixXi renderF;

        frame_interface->cube_size_ = para.render_joint_size_;
        frame_interface->radius_ = para.render_pillar_radius;
        frame_interface->draw(renderV, renderF, renderC, para.render_frame_mesh, para.render_joint_knots);

        viewer.data.clear();
        viewer.data.set_mesh(renderV, renderF);
        viewer.data.set_colors(renderC);

        if(para.render_foucus_joint_index >= 0 && para.render_foucus_joint_index < frame_interface->joint_voxels_.size())
        {
            MatrixXd V;MatrixXi F;MatrixXd C;
            VoxelizedRenderCube render_cube(*frame_interface->joint_voxels_[para.render_foucus_joint_index]);
            render_cube.hx = render_cube.hy = render_cube.hz = para.render_joint_size_;
            render_cube.rendering(V, F, C);
            for(int id = 0; id < V.rows(); id++)
            {
                V.row(id) += frame_interface->frame_mesh_->points_[para.render_foucus_joint_index];
            }
            viewer.core.align_camera_center(V, F);
            viewer.core.camera_zoom = 0.8;
        }
        else
            {
            viewer.core.align_camera_center(renderV);
            viewer.core.camera_zoom = 2;
        }

        viewer.core.show_lines = false;

        void show_pillar_index();
        void show_joints_index();
        void show_joints_face_index();
        if(para.render_show_pillar_index) show_pillar_index();
        if(para.render_show_joint_index) show_joints_index();
        if(para.render_show_joint_face_index) show_joints_face_index();
    }

    return;
}

void write_fpuz()
{
    string path = igl::file_dialog_save();
    if(path != "" && frame_interface)
    {
        frame_interface->write_fpuz(path);
    }
    return;
}

void show_pillar_index()
{
    for(int id = 0; id < frame_interface->pillars_.size(); id++)
    {
        std::shared_ptr<FramePillar> pillar = frame_interface->pillars_[id];
        viewer.data.add_label((pillar->end_points_cood[0] + pillar->end_points_cood[1])/2, std::to_string(pillar->index));
    }
}

void show_joints_index()
{
    int id = 0;
    for(Vector3d pos : frame_interface->frame_mesh_->points_)
    {
        viewer.data.add_label(pos, std::to_string(id++));
    }
}

void add_pillar()
{
    if(frame_interface)
    {
        if(para.render_add_pillar_f0 >= 0 && para.render_add_pillar_f1 >= 0)
        {
            frame_interface->add_pillar(para.render_add_pillar_f0, para.render_add_pillar_f1);
        }
    }
}

void show_joints_face_index()
{
    int id = 0;
    for(Vector3d pos : frame_interface->frame_mesh_->points_)
    {
        int dX[6] = {1, -1, 0, 0, 0, 0};
        int dY[6] = {0, 0, 1, -1, 0, 0};
        int dZ[6] = {0, 0, 0, 0, 1, -1};
        for(int nrm = 0; nrm < 6; nrm++)
        {
            Vector3d normal(dX[nrm], dY[nrm], dZ[nrm]);
            Vector3d npos = pos + normal * para.render_num_joint_voxel / 2 * para.render_joint_size_;
            viewer.data.add_label(npos, std::to_string(id * 6 + nrm));
        }
        id++;
    }
}

void read_fpuz(){
    string path = "";
    path = igl::file_dialog_open();
    if(path != "") {
        frame_interface.reset();
        frame_interface = std::make_shared<FrameInterface>(FrameInterface(para.render_pillar_radius, para.render_joint_size_, -1, colorcoder));
        frame_interface->read_fpuz(path);
        draw_frame_mesh();
    }
};

void read_mesh_file()
{
    string path = "";
    path = igl::file_dialog_open();
    if(path != "")
    {
        MeshVertices V;
        MeshFaces F;
        if(igl::readOBJ(path, V, F))
        {
            std::shared_ptr<FrameMesh> frameMesh = std::make_shared<FrameMesh>(FrameMesh(colorcoder, para.render_pillar_radius));
            frameMesh->set_mesh(V, F);

            frame_interface.reset();
            frame_interface = std::make_shared<FrameInterface>(FrameInterface(para.render_pillar_radius, para.render_joint_size_, para.render_num_joint_voxel, colorcoder));
            frame_interface->set_frame_mesh(frameMesh);
            frame_interface->init_joints();

            draw_frame_mesh();
        }
    }
}

void init()
{
    shared_ptr = LIBIGL_PATH;
    shared_ptr += "/tutorial/shared/";

    para.render_num_joint_voxel = 4;
    para.render_joint_size_ = 0.005;
    para.render_pillar_radius = 0.003;
    para.render_joint_knots = true;
    para.render_frame_mesh = true;
    para.render_show_pillar_index = false;
    para.render_show_joint_face_index = false;
    para.render_show_joint_index = false;
    para.render_foucus_joint_index = -1;
    para.render_add_pillar_f0 = -1;
    para.render_add_pillar_f1 = -1;
    Vector3d colorTable[10] = {
            Vector3d(0.9, 0.2, 0.2),   //  1: Red
            Vector3d(0.2, 0.2, 0.9),   //  2: Blue
            Vector3d(0.9, 0.9, 0.5),   //  3: Yellow
            Vector3d(0.9, 0.5, 0.2),   //  4: Orange
            Vector3d(0.7, 0.2, 0.9),   //  5: Purple
            Vector3d(0.2, 0.9, 0.9),   //  6: Cyan
            Vector3d(0.5, 0.3, 0.2),   //  7: Brown
            Vector3d(0.9, 0.2, 0.6),   //  8: Pink
            Vector3d(0.6, 0.6, 0.7),   //  9: Gray
            Vector3d(0.9, 0.6, 0.5)}; // 10:LightSalmon

    veccolor render_colorTab;
    for (int id = 0; id < 10; id++)
    {
        std::shared_ptr<nanogui::Color> pColor = std::make_shared<nanogui::Color>(nanogui::Color(255 * colorTable[id][0],255 * colorTable[id][1], 255 * colorTable[id][2], 255));
        render_colorTab.push_back(pColor);
    }

    colorcoder = std::make_shared<ColorCoding>(ColorCoding(render_colorTab));
}

void test()
{
    if(frame_interface)
    {
        FrameInterlocking interlock(frame_interface);
        frame_interface = interlock.output_present_frame();
        draw_frame_mesh();
    }
    return;
}

#endif //FRAMEINTERLOCK_UTILITY_H
