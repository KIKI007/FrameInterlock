//
// Created by ziqwang on 28.04.18.
//

#ifndef FRAMEINTERLOCK_FRAMEINTERLOCKINGTREE_H
#define FRAMEINTERLOCK_FRAMEINTERLOCKINGTREE_H

#include "voxel/VoxelizedPuzzle.h"
#include "voxel/VoxelizedPartition.h"

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

    vector<FramePillar *> disassembly_order;

public:

    vector<std::shared_ptr<TreeNode>> children;

    TreeNode *parent;

    TreeNode *brother;

    std::shared_ptr<DirectedGraph> graph[3];
};

class FrameInterlockingTree
{

public:

    FrameInterlockingTree(){interface = nullptr;};

public:

    bool generate_children(TreeNode *node);

    bool generate_children(TreeNode *node, FramePillar *cpillar);

    bool generate_key(TreeNode *node);

    std::shared_ptr<FrameInterface> output_frame(TreeNode *node);

public:

    void get_pillars_fixed_voxels(vecVector3i &new_fixed_voxels,
                                  vecVector3i &remain_fixed_voxels,
                                  int joint_id,
                                  int new_part_id,
                                  const std::map<int, int> &visited);

    void split_graph(const DirectedGraph& in_graph,
                     DirectedGraph& out_graph,
                     FramePillar* pillar,
                     TreeNode *node,
                     int nrm);

    void select_candidates(TreeNode *node);

public:

    std::shared_ptr<TreeNode> root_;

    TreeNode *present_node_;

public:

    FrameInterface *interface;

    vector<vector<FramePillar *>> map_joint_pillars_;
};


#endif //FRAMEINTERLOCK_FRAMEINTERLOCKINGTREE_H
