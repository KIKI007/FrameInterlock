//
// Created by ziqwang on 01.03.18.
//

#ifndef TI_STABLE_DIRECTEDGRAPH_H
#define TI_STABLE_DIRECTEDGRAPH_H

#include <memory>
#include "vector"
#include <fstream>
#include <string>
#include "map"
#include "../voxel/VoxelElement.h"
using std::string;
using std::vector;

struct DirectGraphNodeValence
{
    int valence_in;
    int valence_out;
    int index;
};

struct DirectedGraphNode;
struct DirectedGraphEdge
{
    double weight;
    std::shared_ptr<VoxelsList> list;
    std::weak_ptr<DirectedGraphNode> node;
};

struct DirectedGraphNode
{
    int index_;
    string label;
    vector<DirectedGraphEdge> neighborList_;
};

class DirectedGraph {
public:
    DirectedGraph(int num = 0)
    {
        init(num);
    }

public:

    int size(){return nodeLists_.size();}

    void init(int num)
    {
        for(int id = 0; id < num; id++)
        {
            std::shared_ptr<DirectedGraphNode> p = std::make_shared<DirectedGraphNode>();
            p->index_ = id;
            p->label = std::to_string(id);
            nodeLists_.push_back(p);
        }
    }

public:

    void virtual add_edge(int idA, int idB)
    {
        for(auto u : nodeLists_[idA]->neighborList_)
        {
            if(u.node.lock()->index_ == idB)
                return;
        }

        DirectedGraphEdge edge;
        edge.weight = 0;
        edge.list = nullptr;
        edge.node = nodeLists_[idB];
        nodeLists_[idA]->neighborList_.push_back(edge);
    }

    void virtual add_edge(int idA, int idB, std::shared_ptr<VoxelsList> list)
    {
        for(auto u : nodeLists_[idA]->neighborList_)
        {
            if(u.node.lock()->index_ == idB)
                return;
        }

        DirectedGraphEdge edge;
        edge.list = list;
        edge.weight = 0;
        edge.node = nodeLists_[idB];
        nodeLists_[idA]->neighborList_.push_back(edge);
        return;
    }

    void virtual add_edge(int idA, int idB, pEmt voxel)
    {
        for(auto u : nodeLists_[idA]->neighborList_)
        {
            if(u.node.lock()->index_ == idB && u.list) {
                u.list->push_back(voxel);
                return;
            }
        }
        DirectedGraphEdge edge;
        edge.list = std::make_shared<VoxelsList>();
        edge.list->push_back(voxel);
        edge.node = nodeLists_[idB];
        edge.weight = 0;
        nodeLists_[idA]->neighborList_.push_back(edge);
        return;
    }

    void remove_node(int ID)
    {
        for(int id = 0; id < nodeLists_.size(); id++)
        {
            DirectedGraphNode *u = nodeLists_[id].get();
            for(int jd = 0; jd < u->neighborList_.size(); jd++)
            {
                DirectedGraphEdge edge = u->neighborList_[jd];
                if(edge.node.lock()->index_ == ID)
                {
                    u->neighborList_.erase(u->neighborList_.begin() + jd);
                    jd--;
                }
            }
        }

        for(int id = 0; id < nodeLists_.size(); id++)
        {
            if(nodeLists_[id]->index_ == ID)
            {
                nodeLists_.erase(nodeLists_.begin() + id);
                id--;
            }
        }

        return;
    }

    void compute_node_valence(vector<DirectGraphNodeValence> &valences)
    {
        std::map<int, int> node_index_to_id;
        for(int id = 0; id < nodeLists_.size(); id++)
        {
            DirectGraphNodeValence valence;
            valence.index = nodeLists_[id]->index_;
            valence.valence_in = 0;
            valence.valence_out = 0;
            node_index_to_id[valence.index] = id;
            valences.push_back(valence);
        }

        for(int id = 0; id < nodeLists_.size(); id++)
        {
            DirectedGraphNode *u = nodeLists_[id].get();
            int IDu = node_index_to_id[u->index_];
            for(int jd = 0; jd < u->neighborList_.size(); jd++)
            {
                DirectedGraphNode *v = u->neighborList_[jd].node.lock().get();
                int IDv = node_index_to_id[v->index_];
                valences[IDu].valence_out++;
                valences[IDv].valence_in++;
            }
        }

        return;
    }

    void virtual output_dot(std::string filename, string caption)
    {
        std::ofstream fout(filename);
        fout << "digraph {\n";
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
                for(int jd = 0; jd < p->neighborList_.size(); jd++)
                {
                    q = p->neighborList_[jd].node.lock();
                    fout << p->label << " -> " << q->label << "\n";
                }
            }
            fout << "}";
        }
    }

public:

    string to_string(vector<int> indexs)
    {
        if(indexs.size() == 1) return std::to_string(indexs.back());
        string str = "\"";
        for(int id = 0; id < indexs.size() - 1; id++)
            str += std::to_string(indexs[id]) + ", ";
        str+= std::to_string(indexs.back()) + "\"";
        return str;
    }

public:
    vector< std::shared_ptr<DirectedGraphNode> > nodeLists_;
};


#endif //TI_STABLE_DIRECTEDGRAPH_H
