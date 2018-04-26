//
// Created by ziqwang on 23.04.18.
//

#include "FrameMesh.h"

FrameMesh::FrameMesh(std::shared_ptr<ColorCoding> colorcode, double radius)
{
    colorcode_ = colorcode;
    radius_ = radius;
}

void FrameMesh::set_vertices(vecVector3i &points)
{
    clear();
    points_ = points;
    graph_ = std::make_shared<UndirectedGraph>(UndirectedGraph(points.size()));
}

void FrameMesh::add_edge(int e0, int e1) {
    graph_->add_edge(e0, e1, 0);
}

void FrameMesh::set_mesh(MeshVertices &V, MeshFaces &F) {
    clear();

    for(int id = 0; id < V.size(); id++)
    {
        points_.push_back(Vector3d(V[id][0], V[id][1], V[id][2]));
    }
    graph_ = std::make_shared<UndirectedGraph>(UndirectedGraph(points_.size()));


    for(int id = 0; id < F.size(); id++)
    {
        int size = F[id].size();
        for(int jd = 0; jd < F[id].size(); jd++)
        {
            int e0 = F[id][jd];
            int e1 = F[id][(jd + 1) % size];
            graph_->add_edge(e0, e1);
        }
    }

    return;
}

void FrameMesh::draw(Eigen::MatrixXd &V, MatrixXi &F, MatrixXd &C) {

    vector<FrameUnit> render_unit;
    Vector3d v0, v1;

    FrameUnit cylinder;
    loadCylinderX(cylinder);

    auto build_frame = [&](const Vector3d &v0, const Vector3d &v1) -> MatrixXd
    {
        Vector3d n0 = (v1 - v0) / (v1 - v0).norm();
        Vector3d n1 = n0.cross(Vector3d(1, 0, 0));
        Vector3d n2 = n0.cross(Vector3d(0, 1, 0));
        if(n2.norm() > n1.norm()) n1 = n2;
        n1 /= n1.norm();
        n2 = n0.cross(n1);
        Eigen::MatrixXd mat(3, 3);
        mat << n0.transpose(),
                n1.transpose(),
                n2.transpose();
        return mat;
    };


    auto reshape_cylinder = [&](FrameUnit &unit, const MatrixXd &frame, double length)
    {
        for(int kd = 0; kd < unit.V.rows(); kd++)
        {
            unit.V.row(kd)[0] *= length;
            unit.V.row(kd)= unit.V.row(kd) * frame;
        }
        return;
    };


    for(int id = 0; id < graph_->nodeLists_.size(); id++)
    {
        int e0 = graph_->nodeLists_[id]->index_;
        for(int jd = 0; jd < graph_->nodeLists_[id]->neighborList_.size(); jd++)
        {
            int e1 = graph_->nodeLists_[id]->neighborList_[jd].node.lock()->index_;
            if(e0 < e1)
            {
                //build frame
                v0 = points_[e0];
                v1 = points_[e1];
                MatrixXd frame;
                frame = build_frame(v0, v1);

                //change frame & length
                FrameUnit unit = cylinder;
                reshape_cylinder(unit, frame, (v1 - v0).norm());

                unit.dV = (v0 + v1)/2;

                render_unit.push_back(unit);
            }
        }
    }

    //render
    int nV = 0;
    int nF = 0;
    for(FrameUnit &unit : render_unit)
    {
        unit.dF = RowVector3i(nV, nV, nV);
        nV += unit.V.rows();
        nF += unit.F.rows();
    }

    V = MatrixXd(nV, 3);
    C = MatrixXd(nF, 3);
    F = MatrixXi(nF, 3);

    int iV = 0, iF = 0;
    for(FrameUnit unit : render_unit) {
        for (int id = 0; id < unit.V.rows(); id++) {
            V.row(iV++) = unit.V.row(id) + unit.dV;
        }

        for (int id = 0; id < unit.F.rows(); id++) {
            C.row(iF) = unit.color;
            F.row(iF++) = unit.F.row(id) + unit.dF;
        }
    }
    return;
}

void FrameMesh::clear() {
    points_.clear();
    graph_.reset();
}

bool FrameMesh::loadCylinderX(FrameUnit &cylinderX) {
    string path = LIBIGL_PATH;
    path += "/tutorial/shared/xcylinder.obj";
    if(igl::readOBJ(path, cylinderX.V, cylinderX.F))
    {
        double scale = 1.0 / 1.6;
        for(int id = 0; id < cylinderX.V.rows(); id++)
        {
            cylinderX.V(id, 0) *= scale;
            cylinderX.V(id, 1) = cylinderX.V(id, 1) / 0.424264 * radius_;
            cylinderX.V(id, 2) = cylinderX.V(id, 2) / 0.424264 * radius_;
        }
        cylinderX.dV = Vector3d(0, 0, 0);
        cylinderX.dF = Vector3i(0, 0, 0);
        cylinderX.color = Vector3d(colorcode_->frame_color_[0],
                                   colorcode_->frame_color_[1],
                                   colorcode_->frame_color_[2]);
        return true;
    }
    else
    {
        return false;
    }
}
