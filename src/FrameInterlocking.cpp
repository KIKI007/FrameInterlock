//
// Created by ziqwang on 28.04.18.
//

#include "FrameInterlocking.h"

FrameInterlocking::FrameInterlocking(std::shared_ptr<FrameInterface> frame_interface)
{
    frame_interface_ = frame_interface;
}

void FrameInterlocking::init_tree()
{
    //reset frame interface's joints to its original style
    if(frame_interface_) frame_interface_->reset_joints_to_origin();

    tree_ = std::make_shared<FrameInterlockingTree>();
    tree_->interface = frame_interface_.get();
    tree_->map_joint_pillars_.resize(frame_interface_->joint_voxels_.size());

    for(int id = 0; id < frame_interface_->pillars_.size(); id++)
    {
        int e0 = frame_interface_->pillars_[id]->cube_id[0];
        int e1 = frame_interface_->pillars_[id]->cube_id[1];
        tree_->map_joint_pillars_[e0].push_back(frame_interface_->pillars_[id].get());
        tree_->map_joint_pillars_[e1].push_back(frame_interface_->pillars_[id].get());
    }


    std::shared_ptr<TreeNode> root_ = std::make_shared<TreeNode>();
    root_->parent = nullptr;
    root_->brother = nullptr;
    root_->num_pillar_finished = 0;

    //build puzzle
    int voxel_num = frame_interface_->voxel_num_;
    for(int id = 0; id < frame_interface_->joint_voxels_.size(); id++)
    {
        array_3i array = array_3i(boost::extents[voxel_num][voxel_num][voxel_num]);

        for(int ix = 0; ix < frame_interface_->voxel_num_; ix++)
            for(int iy = 0; iy < frame_interface_->voxel_num_; iy++)
                for(int iz = 0; iz < frame_interface_->voxel_num_; iz++)
                    array[ix][iy][iz] = frame_interface_->pillars_.size();

        std::shared_ptr<VoxelizedPuzzle> puzzle =
                std::make_shared<VoxelizedPuzzle>(VoxelizedPuzzle(frame_interface_->voxel_num_,
                                                                  frame_interface_->voxel_num_,
                                                                  frame_interface_->voxel_num_,
                                                                  id,
                                                                  array,
                                                                  frame_interface_->colorcoder_));
        root_->puzzles.push_back(puzzle);
        root_->num_pillar_left_in_joints.push_back(tree_->map_joint_pillars_[id].size());
    }

    for(int kd = 0; kd < 3; kd++)
        root_->graph[kd] = std::make_shared<DirectedGraph>(DirectedGraph(1));

    tree_->root_ = root_;
    tree_->present_node_ = root_.get();

    select_key_pillar();


    return;
}

void FrameInterlocking::select_key_pillar()
{
    double maxY = -std::numeric_limits<double>::infinity();
    FramePillar *key;
    for(std::shared_ptr<FramePillar> pillar: frame_interface_->pillars_)
    {
        double valueY = pillar->end_points_cood[0][1] + pillar->end_points_cood[1][1];
        if(maxY < valueY)
        {
            maxY = valueY;
            key = pillar.get();
        }
    }
    std::cout << "Key is :\t" << key->index << std::endl;
    tree_->root_->candidate_pillar.push_back(key);
}

std::shared_ptr<FrameInterface> FrameInterlocking::generate_interlocking()
{
    std::srand(150);

    if(tree_ == nullptr)
        init_tree();

    int it_times = 0;
    int num_pillar = frame_interface_->pillars_.size();
    while(tree_->present_node_ && tree_->present_node_->num_pillar_finished < num_pillar)
    {
//        if(tree_->present_node_->num_pillar_finished >= 1 && tree_->present_node_->disassembly_order.back()->index == 17)
//            break;
        tree_->generate_children(tree_->present_node_);
        if(tree_->present_node_->children.size() > 0)
        {
            tree_->present_node_ = tree_->present_node_->children[0].get();
        }
        else
        {
            if(tree_->present_node_->brother != nullptr)
            {
                tree_->present_node_ = tree_->present_node_->brother;
            }
            else
            {
                TreeNode *parent = tree_->present_node_;
                do {
                    parent = parent->parent;
                    if(parent)
                    {
                        tree_->present_node_ = parent->brother;
                    }
                    else
                    {
                        tree_->present_node_ = nullptr;
                        break;
                    }
                }while(tree_->present_node_ == nullptr);
                if(parent)
                    parent->children.clear();
            }
        }

        if(tree_->present_node_)
            std::cout << it_times ++ << "\t, part \t" << tree_->present_node_->num_pillar_finished << "\tFinished!!" << std::endl;
    }


//    if(tree_->present_node_ && tree_->present_node_->num_pillar_finished == num_pillar)
//    {
//        return tree_->output_frame(tree_->present_node_);
//    }
//    else
//    {
//        return nullptr;
//    }
    return tree_->output_frame(tree_->present_node_);
}

std::shared_ptr<FrameInterface> FrameInterlocking::output_present_frame() {
    if(tree_ && tree_->present_node_)
    {
        return  tree_->output_frame(tree_->present_node_);
    }
    else
    {
        return  nullptr;
    }
}
