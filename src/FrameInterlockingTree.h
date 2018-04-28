//
// Created by ziqwang on 28.04.18.
//

#ifndef FRAMEINTERLOCK_FRAMEINTERLOCKINGTREE_H
#define FRAMEINTERLOCK_FRAMEINTERLOCKINGTREE_H

#include "voxel/VoxelizedPuzzle.h"
#include "FrameMesh.h"
class TreeNode
{
    vector<std::shared_ptr<VoxelizedPuzzle>> puzzles;

    int present_joints_id;

    vector<std::shared_ptr<TreeNode>> children;

    TreeNode *parent;

    TreeNode *brother;
};

class FrameInterlockingTree
{

public:

    FrameInterlockingTree();

public:

    void generate_children();

public:

    std::shared_ptr<TreeNode> root_;

    TreeNode *present_node_;

public:

    std::shared_ptr<FrameMesh> fram_mesh_;
};


#endif //FRAMEINTERLOCK_FRAMEINTERLOCKINGTREE_H
