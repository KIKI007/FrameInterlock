//
// Created by ziqwang on 28.04.18.
//

#ifndef FRAMEINTERLOCK_FRAMEINTERLOCKINGTREE_H
#define FRAMEINTERLOCK_FRAMEINTERLOCKINGTREE_H

#include "voxel/VoxelizedPuzzle.h"
#include "FrameMesh.h"
#include "FrameInterface.h"
#include "graph/DirectedGraph.h"
class TreeNode
{
public:
    vector<std::shared_ptr<VoxelizedPuzzle>> puzzles;

    int num_pillar_finished;

    vector<FramePillar *> candidate_pillar;

    std::map<int, int> pillar_visited_order;

    std::map<int, bool> pillar_in_candidate;

public:

    vector<std::shared_ptr<TreeNode>> children;

    TreeNode *parent;

    TreeNode *brother;

    std::shared_ptr<DirectedGraph> graph;
};

class FrameInterlockingTree
{

public:

    FrameInterlockingTree(){interface = nullptr;};

public:

    bool generate_children(TreeNode *node){return false;}

    void generate_key(TreeNode *node);

    std::shared_ptr<FrameInterface> output_frame(TreeNode *node);

public:

    std::shared_ptr<TreeNode> root_;

    TreeNode *present_node_;

public:

    FrameInterface *interface;

    vector<vector<FramePillar *>> map_joint_pillars_;
};


#endif //FRAMEINTERLOCK_FRAMEINTERLOCKINGTREE_H
