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

    vector<int> num_pillar_left_in_joints;

    vector<int> pillar_contact_voxels[2];

    vecVector3d disassembling_directions;

public:

    vector<std::shared_ptr<TreeNode>> children;

    TreeNode *parent;

    TreeNode *brother;

    std::shared_ptr<DirectedGraph> graph[3];

    int relation;
};

class FrameInterlockingTree
{

public:

    FrameInterlockingTree(){interface = nullptr;};

public:

    bool generate_children(TreeNode *node);

    bool generate_children(TreeNode *node, FramePillar *cpillar);


    void generate_vpuzzlegraph(VPuzzleGraph &graph, TreeNode *node, FramePillar *cpillar);

    void seperate_concept_design(int kd,
                                 VPuzRemainVolumePartitionDat& concept,
                                 VPuzRemainVolumePartitionDat& plan,
                                 FramePillar *cpillar);

public:

    bool generate_key(TreeNode *node);

    void generate_key_plan(TreeNode *node, FramePillar *key_pillar, vector<VPuzRemainVolumePartitionDat> &plan);

    std::shared_ptr<FrameInterface> output_frame(TreeNode *node);

    void compute_legal_disassembling_direction(TreeNode *node, FramePillar *pillar, vector<int> &legal_nrm);

    void filter_remaining_volume_partition_plan(vector<VPuzRemainVolumePartitionDat> &outer_plan);

    int get_voxel_number(TreeNode* node, int joint_id);

    int get_pillar_contact_region(TreeNode *node, FramePillar *pillar, int *contact_voxels);

public:

    void accept_partition_plan(TreeNode *node, FramePillar *pillar, const vector<FinalPartiResult> &final_parti, Vector3d direction);

    void sort_children(TreeNode *node);

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

    int max_number_of_children_;

    int max_variation_of_voxel_in_joint;

    bool balance_inner_relation;

    FrameInterface *interface;

    vector<vector<FramePillar *>> map_joint_pillars_;
};


#endif //FRAMEINTERLOCK_FRAMEINTERLOCKINGTREE_H
