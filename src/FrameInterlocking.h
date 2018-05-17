//
// Created by ziqwang on 28.04.18.
//

#ifndef FRAMEINTERLOCK_FRAMEINTERLOCKING_H
#define FRAMEINTERLOCK_FRAMEINTERLOCKING_H

#include "FrameInterface.h"
#include "FrameInterlockingTree.h"
#include "voxel/VoxelizedPuzzle.h"

class FrameInterlocking
{
public:
    FrameInterlocking(std::shared_ptr<FrameInterface> frame_interface);

public:

    void init_tree();

    void select_key_pillar();

    void output_information(TreeNode *node);

public:

    std::shared_ptr<FrameInterface> generate_interlocking();

    std::shared_ptr<FrameInterface> output_present_frame();

public:

    std::shared_ptr<FrameInterface> frame_interface_;

    std::shared_ptr<FrameInterlockingTree> tree_;

};


#endif //FRAMEINTERLOCK_FRAMEINTERLOCKING_H
