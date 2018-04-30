//
// Created by ziqwang on 28.04.18.
//

#include "FrameInterlockingTree.h"

void FrameInterlockingTree::generate_key(TreeNode *node)
{
    if(node)
    {
        
    }
}

std::shared_ptr<FrameInterface> FrameInterlockingTree::output_frame(TreeNode *node) {

    std::shared_ptr<FrameInterface> frame_interface = std::make_shared<FrameInterface>(FrameInterface(*interface));

    int num_joints = node->puzzles.size();

    for(int id = 0;id < num_joints; id++)
    {
        if(node->puzzles[id])
        {
            frame_interface->joint_voxels_[id] = node->puzzles[id]->output_assembly_interface(false, false);
        }
    }

    return frame_interface;
}
