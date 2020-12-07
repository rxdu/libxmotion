/* 
 * tree_impl.hpp
 * 
 * Created on: Jan 07, 2019 09:21
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef TREE_IMPL_HPP
#define TREE_IMPL_HPP

#include <type_traits>

namespace robotnav
{
template <typename State, typename Transition, typename StateIndexer>
Tree<State, Transition, StateIndexer>::Tree(const Tree<State, Transition, StateIndexer> &other)
{
    for (auto &pair : other.vertex_map_)
    {
        auto vertex = pair.second;
        for (auto &edge : vertex->edges_to_)
            this->AddEdge(edge.src_->state_, edge.dst_->state_, edge.trans_);
    }
}

template <typename State, typename Transition, typename StateIndexer>
Tree<State, Transition, StateIndexer>::Tree(Tree<State, Transition, StateIndexer> &&other)
{
    vertex_map_ = std::move(other.vertex_map_);
}

template <typename State, typename Transition, typename StateIndexer>
Tree<State, Transition, StateIndexer> &Tree<State, Transition, StateIndexer>::operator=(const Tree<State, Transition, StateIndexer> &other)
{
    Tree<State, Transition, StateIndexer> temp = other;
    std::swap(*this, temp);
    return *this;
}

template <typename State, typename Transition, typename StateIndexer>
Tree<State, Transition, StateIndexer> &Tree<State, Transition, StateIndexer>::operator=(Tree<State, Transition, StateIndexer> &&other)
{
    std::swap(vertex_map_, other.vertex_map_);
    return *this;
}

template <typename State, typename Transition, typename StateIndexer>
Tree<State, Transition, StateIndexer>::~Tree()
{
    for (auto &vertex_pair : vertex_map_)
        delete vertex_pair.second;
};

template <typename State, typename Transition, typename StateIndexer>
typename Tree<State, Transition, StateIndexer>::vertex_iterator Tree<State, Transition, StateIndexer>::AddVertex(State state)
{
    // Note: only allow adding a vertex without an edge when tree is empty
    if (!TreeType::vertex_map_.empty())
        return TreeType::vertex_end();
    root_ = TreeType::ObtainVertexFromVertexMap(state);
    return root_;
}

template <typename State, typename Transition, typename StateIndexer>
void Tree<State, Transition, StateIndexer>::RemoveVertex(int64_t state_id)
{
    auto it = vertex_map_.find(state_id);

    // remove if specified vertex exists
    if (it != vertex_map_.end())
    {
        // removing root of a non-empty tree will leave the tree in an undefined state
        if (vertex_map_.size() != 1)
        {
            assert(it != root_);

            // remove from other vertices that connect to the vertex to be deleted
            auto vtx = vertex_iterator(it);
            for (auto eit = vtx->parent_vertex_->edges_to_.begin(); eit != vtx->parent_vertex_->edges_to_.end(); eit++)
                if ((*eit).dst_ == vtx)
                {
                    vtx->parent_vertex_->edges_to_.erase(eit);
                    break;
                }
        }

        // remove from vertex map
        auto vptr = it->second;
        vertex_map_.erase(it);
        delete vptr;
    }
}

template <typename State, typename Transition, typename StateIndexer>
int32_t Tree<State, Transition, StateIndexer>::GetVertexDepth(int64_t state_id)
{
    auto vtx = TreeType::FindVertex(state_id);

    if (vtx != TreeType::vertex_end())
    {
        int32_t depth = 0;
        auto parent = vtx->parent_vertex_;
        while (parent != TreeType::vertex_end())
        {
            ++depth;
            parent = parent->parent_vertex_;
        }
        return depth;
    }

    return -1;
}

template <typename State, typename Transition, typename StateIndexer>
typename Tree<State, Transition, StateIndexer>::vertex_iterator Tree<State, Transition, StateIndexer>::GetParentVertex(int64_t state_id)
{
    auto vtx = TreeType::FindVertex(state_id);

    assert(vtx != TreeType::vertex_end());

    if (vtx == root_)
        return TreeType::vertex_end();
    else
        return vtx->parent_vertex_;
}

template <typename State, typename Transition, typename StateIndexer>
void Tree<State, Transition, StateIndexer>::RemoveSubtree(int64_t state_id)
{
    auto vtx = TreeType::FindVertex(state_id);

    // remove if specified vertex exists
    if (vtx != TreeType::vertex_end())
    {
        // remove from other vertices that connect to the vertex to be deleted
        for (auto eit = vtx->parent_vertex_->edges_to_.begin(); eit != vtx->parent_vertex_->edges_to_.end(); eit++)
            if ((*eit).dst_ == vtx)
            {
                vtx->parent_vertex_->edges_to_.erase(eit);
                break;
            }

        // remove all subsequent vertices
        std::vector<vertex_iterator> child_vertices;
        child_vertices.push_back(vtx);
        std::vector<vertex_iterator> direct_children = vtx->GetNeighbours();
        while (!direct_children.empty())
        {
            // add direct children
            child_vertices.insert(child_vertices.end(), direct_children.begin(), direct_children.end());
            std::vector<vertex_iterator> all_children;
            for (auto &vtx : direct_children)
            {
                auto chds = vtx->GetNeighbours();
                if (!chds.empty())
                    all_children.insert(all_children.end(), chds.begin(), chds.end());
            }
            direct_children = all_children;
        }
        for (auto &vtx : child_vertices)
        {
            // remove from vertex map
            auto vptr = TreeType::vertex_map_[vtx->GetVertexID()];
            TreeType::vertex_map_.erase(vtx);
            delete vptr;
        }
    }
}

template <typename State, typename Transition, typename StateIndexer>
void Tree<State, Transition, StateIndexer>::AddEdge(State sstate, State dstate, Transition trans)
{
    bool tree_empty = TreeType::vertex_map_.empty();

    auto src_vertex = TreeType::ObtainVertexFromVertexMap(sstate);
    auto dst_vertex = TreeType::ObtainVertexFromVertexMap(dstate);

    // set root if tree is empty or a parent vertex is connected to root_
    if (tree_empty || (dst_vertex == root_))
        root_ = src_vertex;

    // update transition if edge already exists
    auto it = src_vertex->FindEdge(dstate);
    if (it != src_vertex->edge_end())
    {
        it->trans_ = trans;
        return;
    }

    dst_vertex->parent_vertex_ = src_vertex;
    src_vertex->edges_to_.emplace_back(src_vertex, dst_vertex, trans);
}

template <typename State, typename Transition, typename StateIndexer>
bool Tree<State, Transition, StateIndexer>::RemoveEdge(State sstate, State dstate)
{
    auto src_vertex = FindVertex(sstate);
    auto dst_vertex = FindVertex(dstate);

    if ((src_vertex != vertex_end()) && (dst_vertex != vertex_end()))
    {
        for (auto it = src_vertex->edges_to_.begin(); it != src_vertex->edges_to_.end(); ++it)
        {
            if (it->dst_ == dst_vertex)
            {
                src_vertex->edges_to_.erase(it);
                dst_vertex->parent_vertex_ = TreeType::vertex_end();
                return true;
            }
        }
    }

    return false;
}

template <typename State, typename Transition, typename StateIndexer>
std::vector<typename Tree<State, Transition, StateIndexer>::edge_iterator> Tree<State, Transition, StateIndexer>::GetAllEdges() const
{
    std::vector<typename Tree<State, Transition, StateIndexer>::edge_iterator> edges;
    for (auto &vertex_pair : vertex_map_)
    {
        auto vertex = vertex_pair.second;
        for (auto it = vertex->edge_begin(); it != vertex->edge_end(); ++it)
            edges.push_back(it);
    }
    return edges;
}

template <typename State, typename Transition, typename StateIndexer>
void Tree<State, Transition, StateIndexer>::ResetAllVertices()
{
    for (auto &vertex_pair : vertex_map_)
        vertex_pair.second->ClearVertexSearchInfo();
}

template <typename State, typename Transition, typename StateIndexer>
void Tree<State, Transition, StateIndexer>::ClearAll()
{
    for (auto &vertex_pair : vertex_map_)
        delete vertex_pair.second;
    vertex_map_.clear();
    root_ = TreeType::vertex_end();
}

template <typename State, typename Transition, typename StateIndexer>
typename Tree<State, Transition, StateIndexer>::vertex_iterator Tree<State, Transition, StateIndexer>::ObtainVertexFromVertexMap(State state)
{
    int64_t state_id = GetStateIndex(state);
    auto it = vertex_map_.find(state_id);

    if (it == vertex_map_.end())
    {
        auto new_vertex = new Vertex(state, state_id);
        new_vertex->search_parent_ = vertex_end();
        vertex_map_.insert(std::make_pair(state_id, new_vertex));
        return vertex_iterator(vertex_map_.find(state_id));
    }

    return vertex_iterator(it);
}
} // namespace robotnav

#endif /* TREE_IMPL_HPP */
