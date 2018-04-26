//
// Created by ziqwang on 23.04.18.
//

#include "FrameInterface.h"

FrameInterface::FrameInterface(double radius, double cube_size, std::shared_ptr<ColorCoding> &colorcoder)
{
    radius_ = radius;
    cube_size_ = cube_size;
    colorcoder_ = colorcoder;
}

void FrameInterface::set_frame_mesh(std::shared_ptr<FrameMesh> frame_mesh) {
    frame_mesh_ = frame_mesh;
}

void FrameInterface::set_jonts(std::vector<std::shared_ptr<VoxelizedInterface>> joint_voxels) {
    joint_voxels_ = joint_voxels;
}

void FrameInterface::draw_frame_mesh(FrameInterfaceRenderUnit &frame_unit)
{
    FrameMesh render_mesh(colorcoder_, radius_);
    vecVector3d points;
    for(int id = 0; id < pillars_.size(); id++)
    {
        points.push_back(pillars_[id]->end_points_cood[0]);
        points.push_back(pillars_[id]->end_points_cood[1]);
    }

    render_mesh.set_vertices(points);
    for(int id = 0; id < pillars_.size(); id++)
    {
        render_mesh.add_edge(2 * id, 2 * id + 1);
    }

    render_mesh.draw(frame_unit.V, frame_unit.F, frame_unit.C);

    int num_pillar = pillars_.size();
    int num_face_pillar = frame_unit.C.rows() / num_pillar;
    colorcoder_->request(num_pillar);
    for(int id = 0; id < pillars_.size(); id++)
    {
        for(int jd = 0; jd < num_face_pillar; jd++)
        {
            frame_unit.C.row(id * num_face_pillar + jd) = colorcoder_->get(pillars_[id]->index);
        }
    }

    frame_unit.dV = RowVector3d(0, 0, 0);
    frame_unit.visible = true;
}

void FrameInterface::draw_joints(vector<FrameInterfaceRenderUnit> &render_list)
{
    for(int id = 0; id < joint_voxels_.size(); id++)
    {
        std::shared_ptr<VoxelizedInterface> voxel_interface = joint_voxels_[id];
        FrameInterfaceRenderUnit unit;
        VoxelizedRenderCube renderCube(*voxel_interface);
        renderCube.hx = renderCube.hy = renderCube.hz = cube_size_;
        renderCube.rendering(unit.V, unit.F, unit.C);
        unit.dV = frame_mesh_->points_[id]
                  - Vector3d(cube_size_ * voxel_interface->Nx /2,
                             cube_size_ * voxel_interface->Ny /2,
                             cube_size_ * voxel_interface->Nz /2);
        unit.visible = true;
        render_list.push_back(unit);
    }
}

void FrameInterface::draw(MatrixXd &V, MatrixXi &F, MatrixXd &C, bool is_draw_frame, bool is_draw_joints)
{

    vector<FrameInterfaceRenderUnit> render_unit;

    if(is_draw_frame)
    {
        FrameInterfaceRenderUnit frame_unit;
        draw_frame_mesh(frame_unit);
        render_unit.push_back(frame_unit);
    }

    if(is_draw_joints)
    {
        draw_joints(render_unit);
    }

    //render
    int nV = 0;
    int nF = 0;
    for(FrameInterfaceRenderUnit &unit : render_unit)
    {
        if(unit.visible)
        {
            unit.dF = RowVector3i(nV, nV, nV);
            nV += unit.V.rows();
            nF += unit.F.rows();
        }
    }

    V = MatrixXd(nV, 3);
    C = MatrixXd(nF, 3);
    F = MatrixXi(nF, 3);

    int iV = 0, iF = 0;
    for(FrameInterfaceRenderUnit unit : render_unit) {
        if(unit.visible == false) continue;
        for (int id = 0; id < unit.V.rows(); id++) {
            V.row(iV++) = unit.V.row(id) + unit.dV;
        }

        for (int id = 0; id < unit.F.rows(); id++) {
            C.row(iF) = unit.C.row(id);
            F.row(iF++) = unit.F.row(id) + unit.dF;
        }
    }

    return;
}







void FrameInterface::create_void_joints(int voxel_num) {

    joint_voxels_.clear();
    for(int id = 0; id < frame_mesh_->points_.size(); id++)
    {
        std::shared_ptr<VoxelizedInterface> voxel_interface
                =std::make_shared<VoxelizedInterface>(VoxelizedInterface(voxel_num, voxel_num, voxel_num, colorcoder_));

        for(int ix = 0; ix < voxel_num; ix++)
        {
            for(int iy = 0; iy < voxel_num; iy++)
            {
                for(int iz = 0; iz < voxel_num; iz ++)
                {
                    voxel_interface->set_grid_part_index(Vector3i(ix, iy, iz), 0);
                }
            }
        }

        joint_voxels_.push_back(voxel_interface);
    }
}

void FrameInterface::init_joints(int cube_voxel_num)
{
    create_void_joints(cube_voxel_num);

    //6 * number of cube
    vector<vector<std::shared_ptr<FramePillar>>> pillar_in_cube_face;
    assigned_pillar_to_each_face(pillar_in_cube_face, cube_voxel_num);

    std::sort(pillars_.begin(), pillars_.end(), [&](std::shared_ptr<FramePillar> a, std::shared_ptr<FramePillar> b)
    {
        return a->angle_from_face_normal[0] + a->angle_from_face_normal[1] > b->angle_from_face_normal[0] + b->angle_from_face_normal[1];
    });

    remove_duplicate_pillar(pillar_in_cube_face);

    for(int id = 0; id < pillars_.size(); id++)
    {
        pillars_[id]->index = id;
        for(int kd = 0; kd < 2; kd++)
        {
            std::shared_ptr<VoxelizedInterface> joints = pillars_[id]->cube[kd];
            int nrm = pillars_[id]->pos_in_cube_face[kd];
            fill_one_face_of_joints(joints, nrm, id);
        }
    }

}

void FrameInterface::assigned_pillar_to_each_face(vector<vector<std::shared_ptr<FramePillar>>> &pillar_in_cube_face, int cube_voxel_num)
{
    int dX[6] = {1, -1, 0 ,0 ,0 ,0};
    int dY[6] = {0, 0, 1, -1, 0, 0};
    int dZ[6] = {0, 0, 0, 0, 1, -1};
    int sign[2] = {1, -1};
    std::shared_ptr<DirectedGraphNode> n0, n1;
    int e0, e1;
    Vector3d v0, v1;

    pillar_in_cube_face.resize(frame_mesh_->points_.size() * 6);
    for(int id = 0; id < frame_mesh_->graph_->nodeLists_.size(); id++)
    {
        n0 = frame_mesh_->graph_->nodeLists_[id];
        e0 = n0->index_;
        v0 = frame_mesh_->points_[e0];
        for(int jd = 0; jd < n0->neighborList_.size(); jd++)
        {
            n1 = n0->neighborList_[jd].node.lock();
            e1 = n1->index_;
            v1 = frame_mesh_->points_[e1];
            if(e0 < e1)
            {
                std::shared_ptr<FramePillar> pillar = std::make_shared<FramePillar>();
                for(int kd = 0; kd < 2; kd++)
                {
                    Vector3d vec = (v1 - v0) * sign[kd];
                    vec /= vec.norm();
                    pillar->angle_from_face_normal[kd] = -std::numeric_limits<double>::infinity();
                    for(int nrm = 0; nrm < 6; nrm ++)
                    {
                        Vector3d normal(dX[nrm], dY[nrm], dZ[nrm]);
                        if(pillar->angle_from_face_normal[kd] < normal.dot(vec))
                        {
                            pillar->angle_from_face_normal[kd] = normal.dot(vec);
                            pillar->pos_in_cube_face[kd] = nrm;
                            if(kd == 0)
                            {
                                pillar->end_points_cood[kd] = normal * cube_voxel_num / 2.0 * cube_size_ + v0;
                                pillar->cube[kd] = joint_voxels_[e0];
                                pillar->cube_id[kd] = e0;
                            }
                            else
                            {
                                pillar->end_points_cood[kd] = normal * cube_voxel_num / 2.0 * cube_size_+ v1;
                                pillar->cube[kd] = joint_voxels_[e1];
                                pillar->cube_id[kd] = e1;
                            }
                        }
                    }

                    int nrm = pillar->pos_in_cube_face[kd];
                    if(kd == 0)
                    {
                        pillar_in_cube_face[e0 * 6 + nrm].push_back(pillar);
                    }
                    else
                    {
                        pillar_in_cube_face[e1 * 6 + nrm].push_back(pillar);
                    }
                }
                pillar->index = pillars_.size();
                pillars_.push_back(pillar);
            }
        }
    }
}

void FrameInterface::fill_one_face_of_joints(std::shared_ptr<VoxelizedInterface> joint, int nrm, int index)
{
    int X[2], Y[2], Z[2];
    X[0] = Y[0] = Z[0] = 1;
    X[1] = joint->Nx - 2;
    Y[1] = joint->Ny - 2;
    Z[1] = joint->Nz - 2;

    switch(nrm)
    {
        case 0:
        {
            X[0] = X[1] = joint->Nx - 1;
            break;
        }
        case 1:
        {
            X[0] = X[1] = 0;
            break;
        }
        case 2:
        {
            Y[0] = Y[1] = joint->Ny - 1;
            break;
        }
        case 3:
        {
            Y[0] = Y[1] = 0;
            break;
        }
        case 4:
        {
            Z[0] = Z[1] = joint->Nz - 1;
            break;
        }
        case 5:
        {
            Z[0] = Z[1] = 0;
            break;
        }
    }

    for(int ix = X[0]; ix <= X[1]; ix++)
    {
        for(int iy = Y[0]; iy <= Y[1]; iy++)
        {
            for(int iz = Z[0]; iz <= Z[1]; iz++)
            {
                joint->set_grid_part_index(Vector3i(ix, iy, iz), index);
            }
        }
    }
    return;
}

void FrameInterface::remove_duplicate_pillar(vector<vector<std::shared_ptr<FramePillar>>> &pillar_in_cube_face)
{
    std::vector<bool> pillar_accept;
    pillar_accept.resize(pillars_.size());
    for(int id = 0; id < pillars_.size(); id++)
        pillar_accept[id] = true;

    std::vector<std::shared_ptr<FramePillar>> new_pillars;
    for(int id = 0; id < pillars_.size(); id++)
    {
        //std::cout << pillars_[id]->end_points_cood[0] << std::endl << pillars_[id]->end_points_cood[1] << std::endl;
        if(!pillar_accept[pillars_[id]->index])
            continue;

        bool accept_this_pillar = false;
        std::shared_ptr<FramePillar> pu = pillars_[id];

        for(int jd = 0; jd < pillar_in_cube_face.size(); jd++)
        {
            bool pillar_in_this_cube_face = false;

            for (int kd = 0; kd < pillar_in_cube_face[jd].size(); kd++)
            {
                if (pillar_in_cube_face[jd][kd] == pu)
                {
                    pillar_in_this_cube_face = true;
                    accept_this_pillar = true;
                    break;
                }
            }

            if (pillar_in_this_cube_face)
            {
                for (int kd = 0; kd < pillar_in_cube_face[jd].size(); kd++)
                {
                    if (pillar_in_cube_face[jd][kd] != pu)
                    {
                        pillar_accept[pillar_in_cube_face[jd][kd]->index] = false;
                    }
                }
                pillar_in_cube_face.erase(pillar_in_cube_face.begin() + jd);
                jd--;
            }
        }

        if(accept_this_pillar)
            new_pillars.push_back(pillars_[id]);
    }

    pillars_ = new_pillars;
}

void FrameInterface::write_fpuz(string file_name)
{
    std::ofstream fout(file_name);
    if(fout.fail()) return;
    int vertices_num = frame_mesh_->points_.size();
    fout << vertices_num << std::endl;
    for(int id = 0; id < frame_mesh_->points_.size(); id++)
    {
        fout << frame_mesh_->points_[id].transpose() << std::endl;
    }

    int pillar_num = pillars_.size();
    fout << pillar_num << std::endl;
    for(int id = 0; id < pillar_num; id++)
    {
        fout << pillars_[id]->pos_in_cube_face[0] + pillars_[id]->cube_id[0] * 6 << " "
             << pillars_[id]->pos_in_cube_face[1] + pillars_[id]->cube_id[1] * 6 << std::endl;
    }

    int num_joints = vertices_num;
    int cube_voxel_num = joint_voxels_.front()->Nx;
    fout << num_joints << " " << cube_voxel_num << std::endl;

    for(int id = 0; id < num_joints; id++)
    {
        std::shared_ptr<VoxelizedInterface> voxel_interface = joint_voxels_[id];
        for(int iz = 0; iz < voxel_interface->Nz; iz++)
        {
            for(int iy = 0; iy < voxel_interface->Ny; iy++)
            {
                for(int ix = 0; ix < voxel_interface->Nx; ix++)
                {
                    fout << (*voxel_interface->voxel_)[ix][iy][iz] + 1 << " ";
                }
            }
        }
        fout << std::endl;
    }
    fout.close();
}

void FrameInterface::read_fpuz(string file_name)
{
    std::ifstream fin(file_name);
    if(fin.fail()) return;
    int vertices_num;
    fin >> vertices_num;

    vecVector3d points;
    for(int id = 0; id < vertices_num; id++)
    {
        double x, y ,z;
        fin >> x >> y >> z;
        points.push_back(Eigen::Vector3d(x, y, z));
    }
    frame_mesh_.reset();
    frame_mesh_ = std::make_shared<FrameMesh>(FrameMesh(colorcoder_, radius_));
    frame_mesh_->set_vertices(points);

    //create joints
    joint_voxels_.clear();
    for(int id = 0; id < vertices_num; id++)
    {

    }

    int pillar_num;
    fin >> pillar_num;

    pillars_.clear();
    for(int id = 0; id < pillar_num; id++)
    {
        FramePillar pillar;
        int f0, f1;
        int e0, e1;
        fin >> f0 >> f1;
        e0 = f0 / 6;
        e1 = f1 / 6;
    }
}
