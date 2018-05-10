//
// Created by ziqwang on 23.04.18.
//

#include "FrameInterface.h"

FrameInterface::FrameInterface(double radius, double cube_size, int voxel_num, std::shared_ptr<ColorCoding> &colorcoder)
{
    radius_ = radius;
    cube_size_ = cube_size;
    voxel_num_ = voxel_num;
    colorcoder_ = colorcoder;

    frame_mesh_ = nullptr;

}

void FrameInterface::set_frame_mesh(std::shared_ptr<FrameMesh> frame_mesh) {
    frame_mesh_ = frame_mesh;
}

void FrameInterface::set_jonts(std::vector<std::shared_ptr<VoxelizedInterface>> joint_voxels) {
    joint_voxels_ = joint_voxels;
}

void FrameInterface::draw_frame_mesh(vector<FrameInterfaceRenderUnit> &frame_unit)
{
    for(int id = 0; id < pillars_.size(); id++)
    {
        FrameInterfaceRenderUnit unit;
        draw_pillar(pillars_[id].get(), unit);
        unit.visible = true;
        frame_unit.push_back(unit);
    }
    return;
}


void FrameInterface::draw_pillar(FramePillar *pillar, FrameInterfaceRenderUnit &unit)
{

    auto get_face_vertices = [&](vecVector3d &points, Vector3d &direction, FramePillar *pillar, int k)
    {
        points.clear();
        VoxelizedInterface *voxel_interface = joint_voxels_[pillar->cube_id[k]].get();
        int face_id = pillar->pos_in_cube_face[k];
        Vector3d center = pillar->end_points_cood[k];
        Vector3d dX((double)cube_size_ * voxel_interface->Nx /2, 0, 0);
        Vector3d dY(0, (double)cube_size_ * voxel_interface->Ny /2, 0);
        Vector3d dZ(0, 0, (double)cube_size_ * voxel_interface->Nz /2);
        switch(face_id)
        {
            case 0:
            {
                //+X
                points.push_back(center - dY - dZ);
                points.push_back(center - dY + dZ);
                points.push_back(center + dY + dZ);
                points.push_back(center + dY - dZ);
                direction = dX * 2;
                break;
            }
            case 1:
            {
                //-X
                points.push_back(center - dY - dZ);
                points.push_back(center + dY - dZ);
                points.push_back(center + dY + dZ);
                points.push_back(center - dY + dZ);
                direction = -dX * 2;
                break;
            }
            case 2:
            {
                //+Y
                points.push_back(center - dX - dZ);
                points.push_back(center + dX - dZ);
                points.push_back(center + dX + dZ);
                points.push_back(center - dX + dZ);
                direction = dY * 2;
                break;
            }
            case 3:
            {
                //-Y
                points.push_back(center - dX - dZ);
                points.push_back(center - dX + dZ);
                points.push_back(center + dX + dZ);
                points.push_back(center + dX - dZ);
                direction = -dY * 2;
                break;
            }
            case 4:
            {
                //+Z
                points.push_back(center - dX - dY);
                points.push_back(center - dX + dY);
                points.push_back(center + dX + dY);
                points.push_back(center + dX - dY);
                direction = dZ * 2;
                break;
            }
            case 5:
            {
                //-Z
                points.push_back(center - dX - dY);
                points.push_back(center + dX - dY);
                points.push_back(center + dX + dY);
                points.push_back(center - dX + dY);
                direction = -dZ * 2;
                break;
            }
        }
    };

    auto add_quad_into_mesh = [&](int v0, int v1, int v2, int v3, vecVector3i &faces)
    {
        faces.push_back(Vector3i(v0, v1, v2));
        faces.push_back(Vector3i(v2, v3, v0));
    };

    auto add_tri_into_mesh = [&](int v0, int v1, int v2, vecVector3i &faces)
    {
        faces.push_back(Vector3i(v0, v1, v2));
    };

    vecVector3d points[2];
    Vector3d direction[2];
    for(int kd = 0; kd < 2; kd++)
    {
        get_face_vertices(points[kd], direction[kd], pillar, kd);
    }

    double eps = 1e-5;
    auto four_points_in_one_plane =[&](Vector3d p0, Vector3d p1, Vector3d p2, Vector3d p3)->bool
    {
        Eigen::Matrix3d mat;
        mat.col(0) = p1 - p0;
        mat.col(1) = p2 - p0;
        mat.col(2) = p3 - p0;
        if(std::abs(mat.determinant()) < eps)
            return true;
        else
            return false;
    };

    int conn[4] = {0, 0, 0, 0};

    bool find_correspond = false;
    for(int id = 0; id < 4; id++)
    {
        Vector3d p0 = points[0][id];
        Vector3d p1 = points[0][(id + 1) % 4];
        for(int jd = 0; jd < 4; jd++)
        {
            Vector3d p2 = points[1][jd];
            Vector3d p3 = points[1][(jd + 3) % 4];
            if((p0 - p1).cross(p2 - p3).norm() < eps && (p0 -p2).cross(p1- p3).norm() <eps)
            {
                for(int kd = 0; kd < 4; kd++)
                    conn[(id + kd)%4] = (jd - kd + 4) % 4 + 4;
                find_correspond = true;
                break;
            }
        }
        if(find_correspond)
            break;
    }
    vecVector3i faces;

    if(direction[0].cross(direction[1]).norm() < eps)
    {
        unit.V = MatrixXd(8, 3);
        for(int id = 0; id < 2; id++)
        {
            for(int jd = 0; jd < 4; jd++)
                unit.V.row(id * 4 + jd) = points[id][jd].transpose();
        }

        //create bottom
        for(int kd = 0; kd < 2; kd++)
        {
            add_quad_into_mesh(4 * kd , 4 * kd + 1, 4 * kd + 2, 4 * kd + 3, faces);
        }

        //create outside
        for(int id = 0; id < 4; id++)
        {
            add_quad_into_mesh((id + 1)%4, id, conn[id] , conn[(id + 1) % 4], faces);
        }
    }
    else
    {
        Vector3d edgevec = direction[1].cross(direction[0]);
        vecVector3d plist;
        for(int id = 0; id < 4; id++)
        {
            Vector3d p0 = points[0][id];
            Vector3d p1 = points[0][(id + 1)%4];
            if((p1 - p0).cross(edgevec).norm() < eps && (p1 - p0).dot(edgevec) > 0)
            {
                for(int kd = 0; kd < 4; kd++)
                {
                    plist.push_back(points[0][(id + kd)%4]);
                }
                for(int kd = 0; kd < 4; kd++)
                {
                    plist.push_back(points[1][conn[(id + kd)%4] - 4]);
                }
                for(int kd = 0; kd < 4; kd++)
                {
                    plist.push_back(points[0][(id + kd)%4] + direction[0]);
                }
                break;
            }
        }

        unit.V = MatrixXd(plist.size(), 3);
        for(int id = 0; id < plist.size(); id++)
            unit.V.row(id) = plist[id];

        add_quad_into_mesh(0, 1, 2, 3, faces);
        add_quad_into_mesh(4, 7, 6, 5, faces);
        add_quad_into_mesh(2, 6, 7, 3, faces);

        add_quad_into_mesh(0, 8, 9, 1, faces);
        add_quad_into_mesh(8, 11, 10, 9, faces);
        add_quad_into_mesh(10, 11, 4 ,5, faces);

        add_quad_into_mesh(0, 3, 11, 8, faces);
        add_quad_into_mesh(3, 7, 4, 11, faces);

        add_quad_into_mesh(1, 9, 10, 2, faces);
        add_quad_into_mesh(10, 5, 6, 2, faces);
    }

    unit.F = MatrixXi(faces.size(), 3);
    unit.C = MatrixXd(faces.size(), 3);
    colorcoder_->request(pillars_.size() + 2);
    for(int id = 0; id < faces.size(); id++) {
        unit.F.row(id) = faces[id];
        unit.C.row(id) = colorcoder_->get(pillar->index);
    }
    unit.dV = Vector3d(0, 0, 0);
    unit.pillar_index = pillar->index;

    return;
}

void FrameInterface::draw_joints(vector<FrameInterfaceRenderUnit> &render_list, bool is_draw_joints_sphere)
{
    for(int id = 0; id < joint_voxels_.size(); id++)
    {
        std::shared_ptr<VoxelizedInterface> voxel_interface = joint_voxels_[id];
        if(is_draw_joints_sphere)
        {
            FrameInterfaceRenderUnit unit;
            VoxelizedRenderSphere render_sphere(*voxel_interface);
            render_sphere.sphere_radius = cube_size_ / 2 * 0.7;
            render_sphere.cylinder_length = cube_size_;
            render_sphere.hwdith = cube_size_;
            render_sphere.rendering(unit.V, unit.F, unit.C);

            unit.dV = frame_mesh_->points_[id]
                      - Vector3d(cube_size_ * voxel_interface->Nx /2,
                                 cube_size_ * voxel_interface->Ny /2,
                                 cube_size_ * voxel_interface->Nz /2);
            unit.visible = true;
            render_list.push_back(unit);
        }
        else
        {
            vecMatrixXd Vs;
            vecMatrixXd Cs;
            vecMatrixXi Fs;
            vector<int> indexs;
            VoxelizedRenderCube render_cube(*voxel_interface);
            render_cube.hx = render_cube.hy = render_cube.hz = cube_size_;
            render_cube.rendering(Vs, Fs, Cs, indexs);
            for(int jd = 0; jd < Vs.size(); jd++)
            {
                FrameInterfaceRenderUnit unit;
                unit.V = Vs[jd];
                unit.F = Fs[jd];
                unit.C = Cs[jd];
                unit.dV = frame_mesh_->points_[id]
                          - Vector3d(cube_size_ * voxel_interface->Nx /2,
                                     cube_size_ * voxel_interface->Ny /2,
                                     cube_size_ * voxel_interface->Nz /2);
                unit.visible = true;
                unit.pillar_index = indexs[jd];
                render_list.push_back(unit);
            }
        }
    }
    return;
}

void FrameInterface::draw(MatrixXd &V, MatrixXi &F, MatrixXd &C, bool is_draw_frame, bool is_draw_joints,  bool is_draw_joints_sphere)
{
    vector<FrameInterfaceRenderUnit> render_unit;

    if(is_draw_frame)
    {
        vector<FrameInterfaceRenderUnit> frame_unit;
        draw_frame_mesh(frame_unit);
        render_unit = frame_unit;
    }

    if(is_draw_joints)
    {
        draw_joints(render_unit, is_draw_joints_sphere);
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



void FrameInterface::create_void_joints() {

    joint_voxels_.clear();
    for(int id = 0; id < frame_mesh_->points_.size(); id++)
    {
        std::shared_ptr<VoxelizedInterface> voxel_interface
                =std::make_shared<VoxelizedInterface>(VoxelizedInterface(voxel_num_, voxel_num_, voxel_num_, colorcoder_));

        for(int ix = 0; ix < voxel_num_; ix++)
        {
            for(int iy = 0; iy < voxel_num_; iy++)
            {
                for(int iz = 0; iz < voxel_num_; iz ++)
                {
                    voxel_interface->set_grid_part_index(Vector3i(ix, iy, iz), 0);
                }
            }
        }

        joint_voxels_.push_back(voxel_interface);
    }
}

void FrameInterface::init_joints_from_frame_mesh()
{
    create_void_joints();

    //6 * number of cube
    vector<vector<std::shared_ptr<FramePillar>>> pillar_in_cube_face;
    assigned_pillar_to_each_face(pillar_in_cube_face, voxel_num_);

    std::sort(pillars_.begin(), pillars_.end(), [&](std::shared_ptr<FramePillar> a, std::shared_ptr<FramePillar> b)
    {
        return a->angle_from_face_normal[0] + a->angle_from_face_normal[1] > b->angle_from_face_normal[0] + b->angle_from_face_normal[1];
    });

    remove_duplicate_pillar(pillar_in_cube_face);

    for(auto &joint : joint_voxels_)
    {
        for(int ix = 0; ix < voxel_num_; ix++)
        {
            for(int iy = 0; iy < voxel_num_; iy++)
            {
                for(int iz = 0; iz < voxel_num_; iz ++)
                {
                    joint->set_grid_part_index(Vector3i(ix, iy, iz), pillars_.size());
                }
            }
        }
    }

    for(int id = 0; id < pillars_.size(); id++)
    {
        pillars_[id]->index = id;
        for(int kd = 0; kd < 2; kd++)
        {
            VoxelizedInterface* joint = pillars_[id]->cube[kd];
            int nrm = pillars_[id]->pos_in_cube_face[kd];
            fill_one_face_of_joints(joint, nrm, pillars_[id]->index);
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
                                pillar->cube[kd] = joint_voxels_[e0].get();
                                pillar->cube_id[kd] = e0;
                            }
                            else
                            {
                                pillar->end_points_cood[kd] = normal * cube_voxel_num / 2.0 * cube_size_+ v1;
                                pillar->cube[kd] = joint_voxels_[e1].get();
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

void FrameInterface::fill_one_face_of_joints(VoxelizedInterface* joint, int nrm, int index)
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
    fout << vertices_num << " " << cube_size_ << std::endl;
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

void FrameInterface::read_fpuz(string file_name, double &cube_size)
{
    std::ifstream fin(file_name);
    if(fin.fail()) return;
    int vertices_num;
    fin >> vertices_num >> cube_size;
    cube_size_ = cube_size;

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

    int pillar_num;
    fin >> pillar_num;

    vector<std::pair<int, int>> pillar_connection;
    for(int id = 0; id < pillar_num; id++)
    {
        FramePillar pillar;
        int f0, f1;
        int e0, e1;
        fin >> f0 >> f1;
        e0 = f0 / 6;
        e1 = f1 / 6;
        frame_mesh_->add_edge(e0, e1);
        std::pair<int, int> conn;
        conn.first = f0;
        conn.second = f1;
        pillar_connection.push_back(conn);
    }

    int num_joints, num_voxel_in_joint;
    fin >> num_joints >> num_voxel_in_joint;
    voxel_num_ = num_voxel_in_joint;
//    for(int id = 0; id < num_joints; id++)
//    {
//        std::shared_ptr<VoxelizedInterface> voxel_interface;
//        voxel_interface = std::make_shared<VoxelizedInterface>(
//                VoxelizedInterface(num_voxel_in_joint, num_voxel_in_joint, num_voxel_in_joint, colorcoder_));
//        for(int iz = 0; iz < num_voxel_in_joint; iz++)
//        {
//            for(int iy = 0; iy < num_voxel_in_joint; iy++)
//            {
//                for(int ix = 0; ix < num_voxel_in_joint; ix++)
//                {
//                    int index = 0;
//                    fin >> index;
//                    voxel_interface->set_grid_part_index(Vector3i(ix, iy, iz), index - 1);
//                }
//            }
//        }
//        joint_voxels_.push_back(voxel_interface);
//    }

    //build pillar
    int dX[6] = {1, -1, 0 ,0 ,0 ,0};
    int dY[6] = {0, 0, 1, -1, 0, 0};
    int dZ[6] = {0, 0, 0, 0, 1, -1};
    for(int id = 0; id < pillar_num; id++)
    {
        std::shared_ptr<FramePillar> pillar;
        pillar = std::make_shared<FramePillar>(FramePillar());
        int f0, f1;
        int e0, e1;
        f0 = pillar_connection[id].first;
        f1 = pillar_connection[id].second;
        e0 = f0 / 6;
        e1 = f1 / 6;
        pillar->index = id;
        pillar->pos_in_cube_face[0] = f0 % 6;
        pillar->pos_in_cube_face[1] = f1 % 6;
        pillar->cube_id[0] = e0;
        pillar->cube_id[1] = e1;
        pillar->end_points_cood[0] = points[e0] + Vector3d(dX[f0 % 6], dY[f0 % 6], dZ[f0 % 6]) * num_voxel_in_joint / 2 * cube_size_;
        pillar->end_points_cood[1] = points[e1] + Vector3d(dX[f1 % 6], dY[f1 % 6], dZ[f1 % 6]) * num_voxel_in_joint / 2 * cube_size_;
        pillars_.push_back(pillar);
    }

    create_void_joints();
    for(auto &joint : joint_voxels_)
    {
        for(int ix = 0; ix < voxel_num_; ix++)
        {
            for(int iy = 0; iy < voxel_num_; iy++)
            {
                for(int iz = 0; iz < voxel_num_; iz ++)
                {
                    joint->set_grid_part_index(Vector3i(ix, iy, iz), pillars_.size());
                }
            }
        }
    }

    for(int id = 0; id < pillars_.size(); id++)
    {
        for(int kd = 0; kd < 2; kd++)
        {
            VoxelizedInterface* joint = joint_voxels_[pillars_[id]->cube_id[kd]].get();
            pillars_[id]->cube[kd] = joint;
            int nrm = pillars_[id]->pos_in_cube_face[kd];
            fill_one_face_of_joints(joint, nrm, pillars_[id]->index);
        }
    }


    fin.close();
}

void FrameInterface::add_pillar(int f0, int f1)
{
    int num_voxel_in_joint = joint_voxels_.front()->Nx;
    int dX[6] = {1, -1, 0 ,0 ,0 ,0};
    int dY[6] = {0, 0, 1, -1, 0, 0};
    int dZ[6] = {0, 0, 0, 0, 1, -1};
    std::shared_ptr<FramePillar> pillar;
    pillar = std::make_shared<FramePillar>(FramePillar());
    int e0, e1;
    e0 = f0 / 6;
    e1 = f1 / 6;
    pillar->cube[0] = joint_voxels_[e0].get();
    pillar->cube[1] = joint_voxels_[e1].get();
    pillar->index = pillars_.size();
    pillar->pos_in_cube_face[0] = f0 % 6;
    pillar->pos_in_cube_face[1] = f1 % 6;
    pillar->cube_id[0] = e0;
    pillar->cube_id[1] = e1;
    pillar->end_points_cood[0] = frame_mesh_->points_[e0] + Vector3d(dX[f0 % 6], dY[f0 % 6], dZ[f0 % 6]) *  (double)num_voxel_in_joint / 2 * cube_size_;
    pillar->end_points_cood[1] = frame_mesh_->points_[e1] + Vector3d(dX[f1 % 6], dY[f1 % 6], dZ[f1 % 6]) * (double )num_voxel_in_joint / 2 * cube_size_;
    pillars_.push_back(pillar);

    fill_one_face_of_joints(joint_voxels_[e0].get(), f0 % 6, pillar->index);
    fill_one_face_of_joints(joint_voxels_[e1].get(), f1 % 6, pillar->index);
}

FrameInterface::FrameInterface(const FrameInterface &interface)
{
    radius_ = interface.radius_;
    cube_size_  = interface.cube_size_;
    voxel_num_ = interface.voxel_num_;
    colorcoder_ = interface.colorcoder_;

    frame_mesh_ = std::make_shared<FrameMesh>(FrameMesh(colorcoder_, radius_));
    if(interface.frame_mesh_)
    {
        frame_mesh_->set_vertices(interface.frame_mesh_->points_);
        for(auto nodeU : interface.frame_mesh_->graph_->nodeLists_)
        {
            for(auto edgeV: nodeU->neighborList_)
            {
                auto nodeV = edgeV.node.lock();
                if(nodeU->index_ < nodeV->index_)
                    frame_mesh_->add_edge(nodeU->index_, nodeV->index_);
            }
        }
    }

    for(int id = 0; id < interface.joint_voxels_.size(); id++)
    {
        std::shared_ptr<VoxelizedInterface> joint = std::make_shared<VoxelizedInterface>(
                VoxelizedInterface(voxel_num_, voxel_num_, voxel_num_, colorcoder_));
        for(int ix = 0; ix < voxel_num_; ix++)
        {
            for(int iy = 0; iy < voxel_num_;iy ++)
            {
                for(int iz = 0; iz < voxel_num_; iz++)
                {
                    joint->set_grid_part_index(Vector3i(ix, iy, iz),
                                               interface.joint_voxels_[id]->get_grid_part_index(Vector3i(ix, iy, iz)));
                }
            }
        }

        joint_voxels_.push_back(joint);
    }

    for(int id = 0; id < interface.pillars_.size(); id++)
    {
        std::shared_ptr<FramePillar> pillar = std::make_shared<FramePillar>();
        pillar->index = interface.pillars_[id]->index;
        for(int kd = 0; kd < 2; kd++)
        {
            pillar->end_points_cood[kd] = interface.pillars_[id]->end_points_cood[kd];
            pillar->cube_id[kd] = interface.pillars_[id]->cube_id[kd];
            pillar->cube[kd] = joint_voxels_[pillar->cube_id[kd]].get();
            pillar->pos_in_cube_face[kd] = interface.pillars_[id]->pos_in_cube_face[kd];
            pillar->angle_from_face_normal[kd] = interface.pillars_[id]->angle_from_face_normal[kd];
        }
        pillars_.push_back(pillar);
    }

    return;
}


std::shared_ptr<Assembly> FrameInterface::output_assembly() {
    if(pillars_.size() == 0)
    return nullptr;

    int ix, iy, iz;
    int nix, niy, niz;
    int ID, nID;


    /* allocate space for assembly class */
    std::shared_ptr<Assembly> assembly_;
    int pillar_num = pillars_.size();
    assembly_ = std::make_shared<Assembly>(Assembly(pillar_num + 1));


    /* Matrix for recording joints between different part */
    //voxel model only have 6 normal directions:
    //000001b: ( 1,  0,  0)
    //000010b: (-1,  0,  0)
    //000100b: ( 0,  1,  0)
    //001000b: ( 0, -1,  0)
    //010000b: ( 0,  0,  1)
    //100000b: ( 0,  0, -1)
    MatrixXi Joint(pillar_num + 1, pillar_num + 1);
    Joint.setZero();

    /* compute joint between different part */
    // Going through all voxels.
    // Checking its up, down, left, right, front, back.
    int dX[6] = {1, -1,  0,  0, 0,  0};
    int dY[6] = {0,  0,  1, -1, 0,  0};
    int dZ[6] = {0,  0,  0,  0, 1, -1};

    for(int JointID = 0; JointID < joint_voxels_.size(); JointID++)
    {
        VoxelizedInterface *interface = joint_voxels_[JointID].get();
        for(ix = 0; ix < interface->Nx; ix++)
        {
            for(iy = 0; iy < interface->Ny; iy++)
            {
                for(iz = 0; iz < interface->Nz; iz++)
                {
                    ID = (*(interface->voxel_))[ix][iy][iz];
                    if(ID != -1)
                    {
                        for(int id = 0; id < 6; id++)
                        {
                            //(nix, niy, niz) is adjacent voxel of (ix, iy, iz)
                            nix = ix + dX[id];
                            niy = iy + dY[id];
                            niz = iz + dZ[id];

                            //check whether is outside of the model
                            if(nix < 0 || niy < 0 || niz < 0) continue;
                            if(nix >= interface->Nx || niy >= interface->Ny || niz >= interface->Nz) continue;

                            //create joint in the matrix
                            nID = (*(interface->voxel_))[nix][niy][niz];
                            if(nID != -1 && ID < nID)
                            {
                                Joint(ID, nID) |= (1 << id);
                            }
                        }
                    }
                }
            }
        }
    }


    //create joint in the assembly.
    for(int id = 0; id < pillar_num + 1; id++)
    {
        for(int jd = id + 1; jd < pillar_num + 1; jd++)
        {
            vecVector3d nrms;
            if(Joint(id, jd) & 1) nrms.push_back(Vector3d(1, 0, 0));
            if(Joint(id, jd) & 2) nrms.push_back(Vector3d(-1, 0, 0));
            if(Joint(id, jd) & 4) nrms.push_back(Vector3d(0, 1, 0));
            if(Joint(id, jd) & 8) nrms.push_back(Vector3d(0, -1, 0));
            if(Joint(id, jd) & 16) nrms.push_back(Vector3d(0, 0, 1));
            if(Joint(id, jd) & 32) nrms.push_back(Vector3d(0, 0, -1));
            if(nrms.size() > 0) assembly_->add_contact(id, jd, nrms);
        }
    }

    return assembly_;
}

void FrameInterface::write_obj(string file_name, bool is_draw_frame, bool is_draw_joints)
{
    if(file_name != "")
    {
        vector<int> pillars_element_count;
        pillars_element_count.resize(pillars_.size(), 0);

        vector<FrameInterfaceRenderUnit> render_unit;
        if(is_draw_frame)
        {
            vector<FrameInterfaceRenderUnit> frame_unit;
            draw_frame_mesh(frame_unit);
            render_unit = frame_unit;
        }

        if(is_draw_joints)
        {
            draw_joints(render_unit, false);
        }

        //render
        for(FrameInterfaceRenderUnit &unit : render_unit)
        {
            if(unit.visible)
            {
                int index = unit.pillar_index;
                int eid = pillars_element_count[index] ++;
                string name = file_name + "part_" + std::to_string(index) + "_elrement_" + std::to_string(eid) + ".obj";
                for (int id = 0; id < unit.V.rows(); id++) {
                    unit.V.row(id) = unit.V.row(id) + unit.dV;
                }
                igl::writeOBJ(name, unit.V, unit.F);
            }
        }
    }

    return;
}



