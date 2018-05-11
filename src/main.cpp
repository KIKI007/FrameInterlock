#include "Utility.h"

igl::viewer::Viewer viewer;

string shared_ptr;

std::shared_ptr<ColorCoding> colorcoder;
std::shared_ptr<FrameInterface> frame_interface;
std::shared_ptr<FrameInterlocking> frame_interlock;
std::shared_ptr<FrameInterfaceAnimation> frame_animation;

int main() {
    init();
    nanogui::init();
    viewer.callback_init = [&](igl::viewer::Viewer &viewer)
    {
        viewer.ngui->addWindow(Eigen::Vector2i(220, 10), "Frame Render");
        viewer.ngui->addGroup("Parameter");
        viewer.ngui->addVariable("Joint size", para.render_joint_size_);
        viewer.ngui->addVariable("Joint num voxel", para.render_num_joint_voxel);

        viewer.ngui->addGroup("Render Option");
        viewer.ngui->addVariable("Render Joints", para.render_joint_knots);
        viewer.ngui->addVariable<bool>("Render Joints Sphere", [&](bool value){
            if(value)
            {
                para.render_joint_sphere_ = true;
                para.render_frame_mesh = false;
            }
            else
            {
                para.render_joint_sphere_ = false;
            }
        }, [&](){
            return para.render_joint_sphere_;
        });
        viewer.ngui->addVariable("Render Frame", para.render_frame_mesh);

        viewer.ngui->addGroup("Render Index");
        viewer.ngui->addVariable("Show Pillar Index", para.render_show_pillar_index);
        viewer.ngui->addVariable("Show Joints Index", para.render_show_joint_index);
        viewer.ngui->addVariable("Show Joints Face Index", para.render_show_joint_face_index);
        viewer.ngui->addVariable("Focus", para.render_foucus_joint_index);

        viewer.ngui->addGroup("Modify Frame");
        viewer.ngui->addVariable("Joints Face 0", para.render_add_pillar_f0);
        viewer.ngui->addVariable("Joints Face 1", para.render_add_pillar_f1);
        viewer.ngui->addButton("Add Pillar", [](){add_pillar();});
        viewer.ngui->addButton("Delete Pillar", [](){
            if(frame_interface) frame_interface->delete_pillar(para.render_add_pillar_f0);
            draw_frame_mesh();
        });

        viewer.ngui->addGroup("I/O");
        viewer.ngui->addButton("Read .obj", [](){read_mesh_file();});
        viewer.ngui->addButton("Write .obj", [](){write_frame_obj();});
        viewer.ngui->addButton("Read .fpuz", [](){read_fpuz();});
        viewer.ngui->addButton("Write .fpuz", [](){write_fpuz();});


        viewer.ngui->addGroup("Render");
        viewer.ngui->addButton("Draw", [](){draw_frame_mesh();});
        viewer.ngui->addButton("Test", [](){graph_test();});

        viewer.ngui->addWindow(Eigen::Vector2i(470, 10), "Interlock");
        auto children_choose = viewer.ngui->addVariable<int>("Child ID", [&](int id){
            if(frame_interlock && id >= 0)
            {
                TreeNode *parent = frame_interlock->tree_->present_node_->parent;
                if(parent && parent->children.size() > id)
                {
                    frame_interface = frame_interlock->tree_->output_frame(parent->children[id].get());
                    draw_frame_mesh();
                    para.interlock_children_id = id;
                    std::cout << parent->children[id]->relation << std::endl;
                }
            }
        },[&](){
            return para.interlock_children_id;
        });
        children_choose->setSpinnable(true);

        viewer.ngui->addButton("generate_key", []{generate_key();});
        viewer.ngui->addButton("generate children", []{generate_children();});
        viewer.ngui->addButton("automatic", []{automatic_generate();});
        viewer.ngui->addButton("go back", go_back);

        viewer.ngui->addGroup("Graph");
        viewer.ngui->addButton("Draw Full Graph", [](){
            if (frame_interface)
                write_show_puzzle_blocking_graph(frame_interface);
        });
        viewer.ngui->addButton("Draw Simplified Graph", [](){
            if (frame_interface)
                write_show_puzzle_blocking_graph_simplified(frame_interface);
        });
        viewer.ngui->addButton("Draw Graph Debug", []()
        {
            if (frame_interface)
                write_show_puzzle_blocling_graph_debug(frame_interlock.get());
        });
        viewer.ngui->addGroup("Animation");
        viewer.ngui->addVariable<bool>("Animation", [&](bool value){
            if(value)
            {
                if(init_animation())
                {
                    para.is_animation = true;
                }
                else
                {
                    para.is_animation = false;
                }
            }
            else
            {
                para.is_animation = value;
            }
            }, [](){return para.is_animation;});

        auto slider = viewer.ngui->addVariable<double>("ratio", [&](double ratio)
        {
            if(para.is_animation)
            {
                para.animation_ratio = ratio;
                draw_animation();
            }
        }, [&](){return para.animation_ratio;});
        slider->setSpinnable(true);
        slider->setMinMaxValues(0, 1);
        slider->setValueIncrement(0.005);

        viewer.ngui->addButton("Animation", []()
        {
            if(para.is_animation)
            {
                for(int id = 0; id < 100; id++)
                {
                    para.animation_ratio = id / 100.0;
                    draw_animation();

                    usleep(100000);
                }
            }
        });

        viewer.screen->performLayout();
        return true;
    };
    viewer.launch(true, false);
    return 0;
}