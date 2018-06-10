//
// Created by *** on 02.05.18.
//

#include "UndirectedGraph.h"

void UndirectedGraph::tarjan_cut_points(std::map<int, bool> &cutpoints)
{
    tarjan_init();
    int num = nodeLists_.size();
    int root_node = -1;
    for(int id = 0; id < num; id++)
    {
        if(cutpoints.find(id) != cutpoints.end()) continue;
        if(DFN_[id] == 0)
        {
            tarjan_dfs(id, cutpoints);
        }
    }
}

void UndirectedGraph::tarjan_dfs(int u, std::map<int, bool> &cutpoints)
{
    std::shared_ptr<DirectedGraphNode> p, q;

    DFN_[u] = LOW_[u] = ++Dindex;

    p = nodeLists_[u];

    int children = 0;
    for(int jd = 0; jd < p->neighborList_.size(); jd++)
    {
        q = p->neighborList_[jd].node.lock();
        int v = q->index_;
        if(!DFN_[v])
        {
            children++;
            //un-visited node
            PARENT_[v] = u;
            tarjan_dfs(v, cutpoints);
            LOW_[u] = std::min(LOW_[u], LOW_[v]);
            if(PARENT_[u] != -1 && LOW_[v] >= DFN_[u])
                cutpoints.insert(std::make_pair(u, true));
            if(PARENT_[u] == -1 && children >= 2)
            {
                cutpoints.insert(std::make_pair(u, true));
            }
        }
        else if(v != PARENT_[u])
        {
            LOW_[u] = std::min(LOW_[u], DFN_[v]);
        }
    }
}

void UndirectedGraph::tarjan_init()
{
    int num = nodeLists_.size();

    DFN_.resize(num, 0);
    LOW_.resize(num, 0);
    PARENT_.resize(num, 0);
    for(int &em : PARENT_) em = -1;

    Dindex = 0;
}
