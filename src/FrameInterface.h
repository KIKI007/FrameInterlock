//
// Created by ziqwang on 23.04.18.
//

#ifndef FRAMEINTERLOCK_FRAMEINTERFACE_H
#define FRAMEINTERLOCK_FRAMEINTERFACE_H

#include "voxel/VoxelizedInterface.h"
#include "voxel/VoxelizedRenderCube.h"
#include "voxel/VoxelizedRenderSphere.h"
#include "FrameMesh.h"

struct FramePillar
{
    std::shared_ptr<VoxelizedInterface> cube[2];

    //0 +X  1 -X    2 +Y    3 -Y   4 +Z    5 -Z
    int pos_in_cube_face[2];

    double angle_from_face_normal[2];

    Vector3d end_points_cood[2];

    int index;
};

struct FrameInterfaceRenderUnit{
    MatrixXd V;
    MatrixXi F;
    MatrixXd C;
    RowVector3i dF;
    RowVector3d dV;
    bool visible;
};

class FrameInterface {
public:
    FrameInterface(double radius, double cube_size, std::shared_ptr<ColorCoding> &colorcoder);

public:

    void set_frame_mesh(std::shared_ptr<FrameMesh> frame_mesh);

    void set_jonts(std::vector<std::shared_ptr<VoxelizedInterface>> joint_voxels);

public:





    void draw(MatrixXd &V, MatrixXi &F, MatrixXd &C);

public:

    void create_void_joints(int voxel_num);

    void fill_one_face_of_joints(std::shared_ptr<VoxelizedInterface> joint, int nrm, int index);

    void assigned_pillar_to_each_face(vector<vector<std::shared_ptr<FramePillar>>> &pillar_in_cube_face, int cube_voxel_num);

    void remove_duplicate_pillar(vector<vector<std::shared_ptr<FramePillar>>> &pillar_in_cube_face);

    void init_joints(int cube_voxel_num);

public:

    std::shared_ptr<FrameMesh> frame_mesh_;

public:

    std::vector<std::shared_ptr<VoxelizedInterface>> joint_voxels_;

    std::vector<std::shared_ptr<FramePillar>> pillars_;


public:

    double radius_;

    double cube_size_;

    std::shared_ptr<ColorCoding> colorcoder_;
};


#endif //FRAMEINTERLOCK_FRAMEINTERFACE_H
