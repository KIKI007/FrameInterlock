#include "Utility.h"

igl::viewer::Viewer viewer;

string shared_ptr;

std::shared_ptr<ColorCoding> colorcoder;
std::shared_ptr<FrameInterface> frame_interface;

int main() {
    init();
    nanogui::init();
    viewer.callback_init = [&](igl::viewer::Viewer &viewer)
    {
        viewer.ngui->addWindow(Eigen::Vector2i(220, 10), "Frame Render");
        viewer.ngui->addGroup("Parameter");
        viewer.ngui->addVariable("Joint size", para.render_joint_size_);
        viewer.ngui->addVariable("Pillar radius", para.render_pillar_radius);
        viewer.ngui->addVariable("Joint num voxel", para.render_num_joint_voxel);

        viewer.ngui->addGroup("Render Option");
        viewer.ngui->addVariable("Render Joints", para.render_joint_knots);
        viewer.ngui->addVariable("Render Frame", para.render_frame_mesh);

        viewer.ngui->addGroup("Render Index");
        viewer.ngui->addVariable("Show Pillar Index", para.render_show_pillar_index);
        viewer.ngui->addVariable("Show Joints Index", para.render_show_joint_index);
        viewer.ngui->addVariable("Show Joints Face Index", para.render_show_joint_face_index);
        viewer.ngui->addVariable("Focus", para.render_foucus_joint_index);

        viewer.ngui->addGroup("I/O");
        viewer.ngui->addButton("Read .obj", [](){read_mesh_file();});
        viewer.ngui->addButton("Read .fpuz", [](){});
        viewer.ngui->addButton("Write .fpuz", [](){write_fpuz();});

        viewer.ngui->addGroup("Render");
        viewer.ngui->addButton("Draw", [](){draw_frame_mesh();});

        viewer.screen->performLayout();
        return true;
    };
    viewer.launch(true, false);
    return 0;
}