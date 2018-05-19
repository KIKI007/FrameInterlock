//
// Created by ziqwang on 23.04.18.
//

#ifndef FRAMEINTERLOCK_FRAMEINTERFACE_H
#define FRAMEINTERLOCK_FRAMEINTERFACE_H

#include "voxel/VoxelizedInterface.h"
#include "voxel/VoxelizedRenderCube.h"
#include "voxel/VoxelizedRenderSphere.h"
#include "FrameMesh.h"
#include <fstream>
#include <map>

struct FramePillar
{
    //0 +X  1 -X    2 +Y    3 -Y   4 +Z    5 -Z
    int pos_in_cube_face[2];

    VoxelizedInterface* cube[2];

    int cube_id[2];

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
    int pillar_index;
    bool visible;
};

class FrameInterface {
public:

    FrameInterface(const FrameInterface& interface);

    FrameInterface(double radius, double cube_size, int voxel_num, std::shared_ptr<ColorCoding> &colorcoder);

public:

    void set_frame_mesh(std::shared_ptr<FrameMesh> frame_mesh);

    void set_jonts(std::vector<std::shared_ptr<VoxelizedInterface>> joint_voxels);

public:

    void draw(MatrixXd &V, MatrixXi &F, MatrixXd &C, bool is_draw_frame, bool is_draw_joints, bool is_draw_joints_sphere);

    void draw_frame_mesh(vector<FrameInterfaceRenderUnit> &unit);

    void draw_joints(vector<FrameInterfaceRenderUnit> &render_list, bool is_draw_joints_sphere);

    void draw_pillar(FramePillar *pillar, FrameInterfaceRenderUnit &unit);

public:

    void create_void_joints();

    void fill_one_face_of_joints(VoxelizedInterface* joint, int nrm, int index);

    void assigned_pillar_to_each_face(vector<vector<std::shared_ptr<FramePillar>>> &pillar_in_cube_face, int cube_voxel_num);

    void remove_duplicate_pillar(vector<vector<std::shared_ptr<FramePillar>>> &pillar_in_cube_face);

    void init_joints_from_frame_mesh();

    void reset_joints_to_origin();

    std::shared_ptr<Assembly> output_assembly();

public:

    void add_pillar(int f0, int f1);

    void delete_pillar(int p0);

public:

    void write_fpuz(string file_name);

    void read_fpuz(string file_name, double &cube_size);

    void write_obj(string file_name, bool is_draw_frame, bool is_draw_joints);

public:

    std::shared_ptr<FrameMesh> frame_mesh_;

public:

    std::vector<std::shared_ptr<VoxelizedInterface>> joint_voxels_;

    std::vector<std::shared_ptr<FramePillar>> pillars_;


public:

    int voxel_num_;

    double radius_;

    double cube_size_;

    std::shared_ptr<ColorCoding> colorcoder_;
};


#endif //FRAMEINTERLOCK_FRAMEINTERFACE_H
