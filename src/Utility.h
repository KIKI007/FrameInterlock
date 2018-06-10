//
// Created by *** on 25.04.18.
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
#include "graph/InterlockSDChecking.h"
#include "FrameInterfaceAnimation.h"

using std::string;
using Eigen::MatrixXd;
using Eigen::MatrixXi;

extern igl::viewer::Viewer viewer;
extern std::string shared_ptr;
extern std::shared_ptr<ColorCoding> colorcoder;
extern std::shared_ptr<FrameInterface> frame_interface;
extern std::shared_ptr<FrameInterlocking> frame_interlock;
extern std::shared_ptr<FrameInterfaceAnimation> frame_animation;


struct Parameter
{
    bool render_frame_mesh;

    bool render_joint_knots;

    bool render_show_pillar_index;

    bool render_show_joint_index;

    bool render_joint_sphere_;

    bool render_show_joint_face_index;

    double render_joint_size_;

    double render_pillar_radius;

    int render_num_joint_voxel;

    int render_foucus_joint_index;

    int render_add_pillar_f0;

    int render_add_pillar_f1;

    int interlock_children_id;

    bool is_animation;

    double animation_ratio;
}para;

void draw_frame_mesh()
{
    if(frame_interface)
    {
        MatrixXd renderV, renderC;
        MatrixXi renderF;

        frame_interface->cube_size_ = para.render_joint_size_;
        frame_interface->radius_ = para.render_pillar_radius;
        frame_interface->draw(renderV, renderF, renderC, para.render_frame_mesh, para.render_joint_knots, para.render_joint_sphere_);
        //frame_interface->draw(renderV, renderF, renderC, false, para.render_joint_knots, false);
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
            viewer.core.camera_zoom = 1.3;
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
            draw_frame_mesh();
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
        frame_interface->read_fpuz(path, para.render_joint_size_);
        frame_interlock = std::make_shared<FrameInterlocking>(FrameInterlocking(frame_interface));
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

            auto normalized_mesh = [&](MeshVertices &vertices)
            {
                Vector3d center(0, 0, 0);
                for(vector<double> vec : vertices)
                {
                    for(int kd = 0; kd < 3; kd++)
                        center[kd] += vec[kd];
                }

                center/= vertices.size();
                for(vector<double> &vec : vertices)
                {
                    for(int kd = 0; kd < 3; kd++)
                        vec[kd] -= center[kd];
                }

                double max_cood[3] = {0, 0, 0};
                for(vector<double> vec: vertices)
                {
                    for(int kd = 0; kd < 3; kd++)
                        if(max_cood[kd] < vec[kd])
                            max_cood[kd] = vec[kd];
                }
                double max_length = std::max(max_cood[0], std::max(max_cood[1], max_cood[2]));
                for(vector<double> &vec : vertices)
                {
                    for(int kd = 0; kd < 3; kd++)
                        vec[kd] /= max_length;
                }

                return;
            };

            normalized_mesh(V);

            std::shared_ptr<FrameMesh> frameMesh = std::make_shared<FrameMesh>(FrameMesh(colorcoder, para.render_pillar_radius));
            frameMesh->set_mesh(V, F);

            frame_interface.reset();
            frame_interface = std::make_shared<FrameInterface>(FrameInterface(para.render_pillar_radius, para.render_joint_size_, para.render_num_joint_voxel, colorcoder));
            frame_interface->set_frame_mesh(frameMesh);
            frame_interface->init_joints_from_frame_mesh();
            frame_interlock = std::make_shared<FrameInterlocking>(FrameInterlocking(frame_interface));
            draw_frame_mesh();
        }
    }
}

void init()
{
    shared_ptr = LIBIGL_PATH;
    shared_ptr += "/tutorial/shared/";

    para.render_num_joint_voxel = 3;
    para.render_joint_size_ = 0.02;
    para.render_joint_knots = true;
    para.render_frame_mesh = true;
    para.render_show_pillar_index = false;
    para.render_joint_sphere_ = false;
    para.render_show_joint_face_index = false;
    para.render_show_joint_index = false;
    para.render_foucus_joint_index = -1;
    para.render_add_pillar_f0 = -1;
    para.render_add_pillar_f1 = -1;
    para.interlock_children_id = 0;
    para.is_animation = false;
    para.animation_ratio = 0;

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
    frame_interlock = nullptr;
    frame_animation = nullptr;
}

void go_back()
{
    if(frame_interlock)
    {
        if(frame_interlock->tree_->present_node_)
        {
            TreeNode *present = frame_interlock->tree_->present_node_;
            if(present ->parent)
            {
                frame_interlock->tree_->present_node_ = present->parent;
                frame_interface = frame_interlock->output_present_frame();
                draw_frame_mesh();
            }
        }
    }
    return;
}

void write_frame_obj()
{
    if(frame_interface)
    {
        string path = "";
        path = igl::file_dialog_save();
        if(path != "")
        {

            for(int id = path.size() - 1; id >= 0; id--)
            {
                if(path[id - 1] == '/')
                {
                    path.erase(id, path.size());
                    break;
                }
            }
            frame_interface->write_obj(path, para.render_frame_mesh, para.render_joint_knots);
        }
    }
}
void write_show_puzzle_blocking_graph(std::shared_ptr<FrameInterface> interf)
{
    if(interf)
    {
        std::shared_ptr<Assembly> assembly = interf->output_assembly();
        InterlockSDChecking checker;
        Vector3d vec[3] = {
                Vector3d(1, 0, 0),
                Vector3d(0, 1, 0),
                Vector3d(0, 0, 1)};
        string caption[3] = {"X Direction", "Y Direction", "Z Direction"};
        string openfile;
        for(int id = 0; id < 3; id++)
        {
            std::shared_ptr<DirectedGraph> graph =  checker.init_sd_graph(vec[id], *assembly);
            string dot_file = "tmp_" + std::to_string(id) + ".dot";
            string png_file = "tmp_" + std::to_string(id) + ".png";
            graph->output_dot(dot_file, caption[id]);
            string command = "dot -Tpng " + dot_file + " -o " + png_file;
            system(command.c_str());
            openfile += " " + png_file;
        }

        string command = "open" + openfile;
        system(command.c_str());
    }
    return;
}


void write_show_puzzle_blocling_graph_debug(FrameInterlocking *frame_interlock)
{
    if(frame_interlock && frame_interlock->tree_ && frame_interlock->tree_->present_node_)
    {
        InterlockSDChecking checker;
        Vector3d vec[3] = {
                Vector3d(1, 0, 0),
                Vector3d(0, 1, 0),
                Vector3d(0, 0, 1)};
        string caption[3] = {"X Direction", "Y Direction", "Z Direction"};
        string openfile;
        for(int id = 0; id < 3; id++)
        {
            std::shared_ptr<DirectedGraph> graph = frame_interlock->tree_->present_node_->graph[id];
            string dot_file = "tmp_" + std::to_string(id) + ".dot";
            string png_file = "tmp_" + std::to_string(id) + ".png";
            graph->output_dot(dot_file, caption[id]);
            string command = "dot -Tpng " + dot_file + " -o " + png_file;
            system(command.c_str());
            openfile += " " + png_file;
        }

        string command = "open" + openfile;
        system(command.c_str());
    }
}

void write_show_puzzle_blocking_graph_simplified(std::shared_ptr<FrameInterface> interf)
{
    if(interf)
    {
        std::shared_ptr<Assembly> assembly = interf->output_assembly();
        InterlockSDChecking checker;
        Vector3d vec[3] = {
                Vector3d(1, 0, 0),
                Vector3d(0, 1, 0),
                Vector3d(0, 0, 1)};
        string caption[3] = {"X Direction", "Y Direction", "Z Direction"};
        string openfile;
        for(int id = 0; id < 3; id++)
        {
            std::shared_ptr<DirectedGraph> graph =  checker.simplified_sd_graph(vec[id], *assembly);
            string dot_file = "tmp_" + std::to_string(id) + ".dot";
            string png_file = "tmp_" + std::to_string(id) + ".png";
            graph->output_dot(dot_file, caption[id]);
            string command = "dot -Tpng " + dot_file + " -o " + png_file;
            system(command.c_str());
            openfile += " " + png_file;
        }

        string command = "open" + openfile;
        system(command.c_str());
    }
    return;
}

void generate_children()
{
    if(frame_interlock)
    {
        if(frame_interlock->tree_ == nullptr) frame_interlock->init_tree();
        frame_interlock->tree_->generate_children(frame_interlock->tree_->present_node_);
        para.interlock_children_id = 0;
        if(frame_interlock->tree_->present_node_->children.size() > 0)
        {
            frame_interlock->tree_->present_node_ = frame_interlock->tree_->present_node_->children[0].get();
            frame_interface = frame_interlock->output_present_frame();
            draw_frame_mesh();
        }
    }
}

void automatic_generate()
{
    string path = "/Users/***/Desktop/WorkSpace/frame_chair";
    if(frame_interlock)
    {
        //for(int id = 0; id < 100; id++)
        //{
            frame_interlock.reset();
            frame_interlock = std::make_shared<FrameInterlocking>(FrameInterlocking(frame_interface));
            frame_interface = frame_interlock->generate_interlocking();
            draw_frame_mesh();
            //string file_name = path + "/chair_" + std::to_string(id) + ".fpuz";
            //if(finterface)
                //finterface->write_fpuz(file_name);
        //}
    }
    return;
}

void generate_key()
{
    if(frame_interlock)
    {
        if(frame_interlock->tree_ == nullptr) frame_interlock->init_tree();
        frame_interlock->tree_->generate_key(frame_interlock->tree_->root_.get());
        para.interlock_children_id = 0;
        if(frame_interlock->tree_->root_->children.size() > 0)
        {
            frame_interlock->tree_->present_node_ = frame_interlock->tree_->root_->children[0].get();
            frame_interface = frame_interlock->output_present_frame();
            draw_frame_mesh();
        }
    }

}

void graph_test()
{
//    UndirectedGraph graph(18);
//    graph.add_edge(0, 1);
//    graph.add_edge(0, 2);
//    graph.add_edge(0, 3);
//    graph.add_edge(1, 2);
//    graph.add_edge(3, 4);
//    graph.add_edge(0, 1);
//    graph.add_edge(0, 2);
//    graph.add_edge(1, 3);
//    graph.add_edge(1, 4);
//    graph.add_edge(1, 6);
//    graph.add_edge(1, 2);
//    graph.add_edge(3, 5);
//    graph.add_edge(4, 5);

//    MatrixXd mat(26, 2);
//    mat << 1, 2,
//            2, 3,
//            2, 4,
//            2, 5,
//            2, 6,
//            3, 4,
//            5, 6,
//            5, 7,
//            6, 7,
//            7, 8,
//            7, 11,
//            8, 12,
//            8, 14,
//            8, 15,
//            8, 11,
//            8, 9,
//            9, 10,
//            9, 11,
//            10, 11,
//            10, 16,
//            10, 17,
//            10, 18,
//            12, 13,
//            13, 14,
//            13, 15,
//            17, 18;
//    for(int id = 0; id < 26; id++)
//    {
//        graph.add_edge(mat(id, 0) - 1, mat(id ,1) - 1);
//    }
//
//    std::map<int, bool> cutpoints;
//    graph.tarjan_cut_points(cutpoints);
//    for(auto it: cutpoints)
//    {
//        std::cout << it.first << " ";
//    }
//    std::cout << std::endl;

    DirectedGraph graph(6);
    MatrixXi mat(6, 2);
    mat <<  1, 2,
            1, 3,
            2, 3,
            2, 4,
            3, 4,
            3, 6;
    for(int id = 0; id < 6; id++)
    {
        graph.add_edge(mat(id, 0) - 1, mat(id ,1) - 1);
    }

    vector<DirectGraphNodeValence> valences;
    graph.compute_node_valence(valences);
    for(DirectGraphNodeValence valence: valences)
    {
        std::cout << valence.index + 1 << " " << valence.valence_in << " " << valence.valence_out << std::endl;
    }
    return;
}

bool init_animation()
{
//    if(frame_interface)
//    {
//        TreeNode *present = frame_interlock->tree_->present_node_;
//        if(present == nullptr)
//            return  false;
//        if(present->num_pillar_finished != frame_interface->pillars_.size())
//            return false;
//
//        frame_interface->cube_size_ = para.render_joint_size_;
//
//        frame_interface->radius_ = para.render_pillar_radius;
//        frame_animation = std::make_shared<FrameInterfaceAnimation>(FrameInterfaceAnimation(*frame_interface));
//        vector<int> sequences;
//
//        for(int id = 0; id < present->disassembly_order.size(); id++)
//            sequences.push_back(present->disassembly_order[id]->index);
//        frame_animation->set_disassembling_sequences(sequences);
//        vecVector3d directions = present->disassembling_directions;
//        frame_animation->set_disassembling_direction(directions);
//        return true;
//    }
//    else

    if(frame_interface)
    {
        frame_interface->cube_size_ = para.render_joint_size_;
        frame_interface->radius_ = para.render_pillar_radius;
        frame_animation = std::make_shared<FrameInterfaceAnimation>(FrameInterfaceAnimation(*frame_interface));
        if(frame_animation->compute_animation_sequences())
            return true;
    }

    return false;
}

void close_animation()
{
    frame_animation.reset();
}

void draw_animation()
{
    if(para.is_animation && frame_animation)
    {
        MatrixXd V;
        MatrixXi F;
        MatrixXd C;
        frame_animation->draw_animation(V, F, C, para.animation_ratio);
        viewer.data.clear();
        viewer.data.set_mesh(V, F);
        viewer.data.set_colors(C);
        viewer.draw();
        glfwSwapBuffers(viewer.window);
    }
}

#endif //FRAMEINTERLOCK_UTILITY_H
