//
// Created by ziqwang on 23.04.18.
//

#ifndef FRAMEINTERLOCK_FRAMEMESH_H
#define FRAMEINTERLOCK_FRAMEMESH_H

#include<Eigen/StdVector>
#include "Eigen/Dense"
#include "ColorCoding.h"
#include "graph/UndirectedGraph.h"

using Eigen::MatrixXd;
using Eigen::MatrixXi;
using Eigen::Vector3d;
using Eigen::Vector3i;
using Eigen::RowVector3d;
using Eigen::RowVector3i;

typedef std::vector<std::vector<double>> MeshVertices;
typedef std::vector<std::vector<int >> MeshFaces;

typedef std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> vecVector3i;

struct FrameUnit
{
    MatrixXd V;
    MatrixXi F;
    RowVector3d dV;
    RowVector3i dF;
    RowVector3d color;
};

class FrameMesh {
public:
    FrameMesh(std::shared_ptr<ColorCoding> colorcode, double radius);

public:

    void set_vertices(vecVector3i &points);

    void add_edge(int e0, int e1);

    void set_mesh(MeshVertices &V, MeshFaces &F);

    void clear();

    bool loadCylinderX(FrameUnit &cylinderX);

public:

    void draw(Eigen::MatrixXd &V, MatrixXi &F, MatrixXd &C);

public:

    double radius_;

    std::shared_ptr<ColorCoding> colorcode_;

    std::shared_ptr<UndirectedGraph> graph_;

    vecVector3i points_;
};


#endif //FRAMEINTERLOCK_FRAMEMESH_H
