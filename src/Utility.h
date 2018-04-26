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

using std::string;
using Eigen::MatrixXd;
using Eigen::MatrixXi;

extern igl::viewer::Viewer viewer;
extern std::string shared_ptr;
extern std::shared_ptr<ColorCoding> colorcoder;


struct Parameter
{
    bool render_frame_mesh;

    bool render_joint_knots;

    bool render_modified_frame_mesh_;
}para;

void test()
{
    std::shared_ptr<FrameMesh> frameMesh = std::make_shared<FrameMesh>(FrameMesh(colorcoder, 0.01));
    MeshVertices V;
    MeshFaces F;
    igl::readOBJ(shared_ptr + "bunny_quad.obj", V, F);
    //igl::readOBJ(shared_ptr + "cube.obj", V, F);
    //igl::readOBJ(shared_ptr + "DeTri.obj", V, F);
    frameMesh->set_mesh(V, F);
    MatrixXd renderV, renderC;
    MatrixXi renderF;

    FrameInterface interface(0.002, 0.005, colorcoder);
    interface.set_frame_mesh(frameMesh);
    interface.init_joints(4);
    interface.draw(renderV, renderF, renderC);
    viewer.data.clear();
    viewer.data.set_mesh(renderV, renderF);
    viewer.data.set_colors(renderC);

    return;
}

void init()
{
    shared_ptr = LIBIGL_PATH;
    shared_ptr += "/tutorial/shared/";

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

#endif //FRAMEINTERLOCK_UTILITY_H
