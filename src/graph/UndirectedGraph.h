//
// Created by ziqwang on 23.04.18.
//

#ifndef FRAMEINTERLOCK_UNDIRECTEDGRAPH_H
#define FRAMEINTERLOCK_UNDIRECTEDGRAPH_H

#include "DirectedGraph.h"
class UndirectedGraph : public DirectedGraph {
public:
    UndirectedGraph(int num) : DirectedGraph(num)
    {

    }
public:

    bool is_neighbor(int idA, int idB)
    {

        for(int id = 0; id < nodeLists_[idA]->neighborList_.size(); id++)
        {
            std::shared_ptr<DirectedGraphNode> u = nodeLists_[idA]->neighborList_[id].node.lock();
            if(u->index_ == idB)
                return true;
        }
        return false;
    }

    void add_edge(int idA, int idB, double weight = 0)
    {
        DirectedGraphEdge edge;
        if(!is_neighbor(idA, idB))
        {
            edge.weight = weight;
            edge.node = nodeLists_[idB];
            nodeLists_[idA]->neighborList_.push_back(edge);
        }

        if(!is_neighbor(idB, idA))
        {
            edge.weight = weight;
            edge.node = nodeLists_[idA];
            nodeLists_[idB]->neighborList_.push_back(edge);

        }
        return;
    }

    void output_dot(std::string filename, string caption)
    {
        std::ofstream fout(filename);
        fout << "graph {\n";
        if(caption != "") fout << "label=\"" << caption << "\"";
        std::shared_ptr<DirectedGraphNode> p, q;
        if(nodeLists_.size() == 1)
        {
            p = nodeLists_[0];
            fout << p->label << "\n}";
        }
        else
        {
            for(int id = 0; id < nodeLists_.size(); id++)
            {
                p = nodeLists_[id];
                for(int jd = id + 1; jd < p->neighborList_.size(); jd++)
                {
                    q = p->neighborList_[jd].node.lock();
                    fout << p->label << " -- " << q->label << "\n";
                }
            }
            fout << "}";
        }
    }
};


#endif //FRAMEINTERLOCK_UNDIRECTEDGRAPH_H
