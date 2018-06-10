//
// Created by *** on 06.05.18.
//

#include "FrameInterfaceAnimation.h"


void FrameInterfaceAnimation::draw_animation(MatrixXd &V, MatrixXi &F, MatrixXd &C, double ratio)
{
    int num_pillar = pillars_.size();
    int animation_step = (int)(ratio * num_pillar);

    vector<FrameInterfaceRenderUnit> render_unit;

    std::map<int, bool> would_render;
    int would_move = disassembling_sequences_[animation_step];
    for(int id = animation_step; id < num_pillar; id++)
    {
        would_render.insert(std::make_pair(disassembling_sequences_[id], true));
    }

    double move_ratio = ratio * num_pillar - animation_step;
    for(FrameInterfaceRenderUnit unit : units_)
    {
        if(would_render.find(unit.pillar_index) != would_render.end())
        {
            if(unit.pillar_index == would_move)
            {
                for(int id = 0; id < unit.V.rows(); id++)
                {
                    unit.V.row(id) += directions_[animation_step] * max_length * move_ratio;
                }
            }
            render_unit.push_back(unit);
        }
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
    for(FrameInterfaceRenderUnit unit : render_unit)
    {
        for (int id = 0; id < unit.V.rows(); id++) {
            V.row(iV++) = unit.V.row(id);
        }

        for (int id = 0; id < unit.F.rows(); id++) {
            C.row(iF) = unit.C.row(id);
            F.row(iF++) = unit.F.row(id) + unit.dF;
        }
    }
    return;
}

bool FrameInterfaceAnimation::compute_animation_sequences()
{
    //following the order of Y
    disassembling_sequences_.clear();
    directions_.clear();

    std::shared_ptr<DirectedGraph>  graph[3];
    std::shared_ptr<Assembly> assembly = output_assembly();
    InterlockSDChecking checker;
    Vector3d vec[3] = {
            Vector3d(1, 0, 0),
            Vector3d(0, 1, 0),
            Vector3d(0, 0, 1)};
    for(int XYZ = 0; XYZ < 3; XYZ++)
    {
        graph[XYZ] =  checker.init_sd_graph(vec[XYZ], *assembly);
        graph[XYZ]->nodeLists_.pop_back();
    }

    auto is_graph_empty = [&]() ->bool
    {
        for(int XYZ = 0; XYZ < 3; XYZ++)
            if(graph[XYZ]->nodeLists_.empty())
                return true;
        return false;
    };

    std::map<int, bool> visited;
    while(!is_graph_empty())
    {
        vector<AnimationMovablePart> movable_parts;
        for(int XYZ = 0; XYZ < 3; XYZ++)
        {
            vector<DirectGraphNodeValence> valences;
            graph[XYZ]->compute_node_valence(valences);
            compute_movable_parts(valences, visited, movable_parts, XYZ);
        }

        if(movable_parts.empty())
        {
            std::cout << "DeadLock!!!" << std::endl;
            disassembling_sequences_.clear();
            directions_.clear();
            return false;
        }

        std::sort(movable_parts.begin(), movable_parts.end(), [&](const AnimationMovablePart &iA, const AnimationMovablePart &iB)
        {
            Vector3d pA = frame_mesh_->points_[pillars_[iA.part_id]->cube_id[0]] +
                          frame_mesh_->points_[pillars_[iA.part_id]->cube_id[1]];
            Vector3d pB = frame_mesh_->points_[pillars_[iB.part_id]->cube_id[0]] +
                          frame_mesh_->points_[pillars_[iB.part_id]->cube_id[1]];
            return pA[1] > pB[1];
        });

        int index = movable_parts.front().part_id;
        disassembling_sequences_.push_back(index);
        directions_.push_back(movable_parts.front().move_direction);
        visited[index] = true;
        for(int XYZ = 0; XYZ < 3; XYZ++)
            graph[XYZ]->remove_node(index);
    }
    return true;
}

void FrameInterfaceAnimation::compute_movable_parts(const vector<DirectGraphNodeValence> &valences,
                                                    const std::map<int, bool> &visited,
                                                    vector<AnimationMovablePart> &movable_parts,
                                                    int XYZ) {
    int dX[3] = {1, 0 ,0};
    int dY[3] = {0, 1, 0};
    int dZ[3] = {0, 0, 1};
    for(int id = 0; id < valences.size(); id++)
    {
        int index = valences[id].index;
        if(visited.find(index) != visited.end()) continue;
        if(valences[id].valence_in == 0)
        {
            AnimationMovablePart ampart;
            ampart.part_id = index;
            ampart.move_direction = Vector3d(dX[XYZ], dY[XYZ], dZ[XYZ]);
            if(is_collision_free(index, 2 * XYZ, visited))
                movable_parts.push_back(ampart);
        }
        if(valences[id].valence_out == 0)
        {
            AnimationMovablePart ampart;
            ampart.part_id = index;
            ampart.move_direction = -Vector3d(dX[XYZ], dY[XYZ], dZ[XYZ]);
            if(is_collision_free(index, 2 * XYZ + 1, visited))
                movable_parts.push_back(ampart);
        }
    }
    return;
}

bool FrameInterfaceAnimation::is_collision_free(int index, int nrm, const std::map<int, bool> &visited) {

    FramePillar *u = pillars_[index].get();
    int neighbor = 0;
    for(int id = 0; id < pillars_.size(); id++)
    {
        if(visited.find(pillars_[id]->index) == visited.end())
        {
            FramePillar *v = pillars_[id].get();
            if(u->index == v->index) continue;
            for(int ix = 0; ix < 2; ix++)
            {
                for(int iy = 0; iy < 2; iy++)
                {
                    if(u->cube_id[ix] == v->cube_id[iy] && v->pos_in_cube_face[iy] == nrm)
                    {
                        return false;
                    }
                }
            }
        }
    }

    auto opposite_face = [](int nrm) -> int
    {
        int XYZ = nrm / 2;
        int sign = (nrm % 2 + 1) % 2;
        return 2 * XYZ + sign;
    };

    for(int kd = 0; kd < 2; kd++)
    {
        int cube_face_nrm = u->pos_in_cube_face[kd];
        if(opposite_face(nrm) != cube_face_nrm) continue;
        for(int id = 0; id < pillars_.size(); id++)
        {
            if(visited.find(pillars_[id]->index) == visited.end())
            {
                FramePillar *v = pillars_[id].get();
                if (u->index == v->index) continue;
                for (int iy = 0; iy < 2; iy++)
                {
                    if (u->cube_id[kd] == v->cube_id[iy])
                    {
                        return false;
                    }
                }
            }
        }
    }

    return true;
}



void FrameInterfaceAnimation::write_animation_script(string path_name)
{
    if(path_name != "")
    {

        vector<string> obj_file_names;
        vector<FrameInterfaceRenderUnit> outlist;
        for(int id = 0; id < pillars_.size(); id++)
        {
            int index  = id;
            vector<FrameInterfaceRenderUnit> list;
            for(int jd = 0; jd < units_.size(); jd++)
            {
                FrameInterfaceRenderUnit &unit = units_[jd];
                if(unit.pillar_index == index)
                {
                    list.push_back(unit);
                }
            }
            FrameInterfaceRenderUnit combine_unit;
            merge_into_one_unit(list, combine_unit);
            outlist.push_back(combine_unit);
        }

        //output obj
        for(FrameInterfaceRenderUnit &unit : outlist)
        {
            int index = unit.pillar_index;
            string obj_name = "part_" + std::to_string(index) + ".obj";
            igl::writeOBJ(path_name + obj_name, unit.V, unit.F);
            obj_file_names.push_back(obj_name);
        }

        //output animation.motion.txt
        std::ofstream fout;
        fout.open(path_name + "animation.motion.txt");
        fout << "Objects " << obj_file_names.size() << std::endl;
        for(int id = 0; id < obj_file_names.size(); id++)
        {
            fout << obj_file_names[id] << " ";
        }
        fout << std::endl;

        for(int id = 0; id < disassembling_sequences_.size(); id++)
        {
            int index = disassembling_sequences_[id];
            for(int jd = 0; jd < outlist.size(); jd++)
            {
                FrameInterfaceRenderUnit &unit = outlist[jd];
                if(unit.pillar_index == index)
                {

                    fout << "Begin Action 130" << std::endl;
                    fout << "Move id " << jd + 1
                         << " [" << directions_[id][0] * 0.2
                         << ", " << directions_[id][1] * 0.2
                         << ", " << directions_[id][2] * 0.2 << " ]"
                         << std::endl;
                    fout << "End" << std::endl << std::endl;
                }
            }
        }
    }
}

void
FrameInterfaceAnimation::merge_into_one_unit(vector<FrameInterfaceRenderUnit> &list, FrameInterfaceRenderUnit &unit)
{
    int nV = 0;
    int nF = 0;
    for(FrameInterfaceRenderUnit &unit : list)
    {
        if(unit.visible)
        {
            unit.dF = RowVector3i(nV, nV, nV);
            nV += unit.V.rows();
            nF += unit.F.rows();
        }
    }

    MatrixXd V = MatrixXd(nV, 3);
    MatrixXd C = MatrixXd(nF, 3);
    MatrixXi F = MatrixXi(nF, 3);

    int iV = 0, iF = 0;
    for(FrameInterfaceRenderUnit &unit : list)
    {
        for (int id = 0; id < unit.V.rows(); id++) {
            V.row(iV++) = unit.V.row(id);
        }

        for (int id = 0; id < unit.F.rows(); id++) {
            C.row(iF) = unit.C.row(id);
            F.row(iF++) = unit.F.row(id) + unit.dF;
        }
    }

    unit.V = V;
    unit.C = C;
    unit.F = F;
    unit.pillar_index = list.front().pillar_index;
    return;
}
