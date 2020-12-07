/* 
 * tree_edge_impl.hpp
 * 
 * Created on: Jan 07, 2019 09:21
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef TREE_EDGE_IMPL_HPP
#define TREE_EDGE_IMPL_HPP

namespace robotnav
{
template <typename State, typename Transition, typename StateIndexer>
bool Tree<State, Transition, StateIndexer>::Edge::operator==(const Tree<State, Transition, StateIndexer>::Edge &other)
{
    if (src_ == other.src_ && dst_ == other.dst_ && trans_ == other.trans_)
        return true;
    return false;
}

template <typename State, typename Transition, typename StateIndexer>
void Tree<State, Transition, StateIndexer>::Edge::PrintEdge()
{
    std::cout << "Edge_t: src - " << src_->GetVertexID() << " , dst - " << dst_->GetVertexID() << " , cost - " << trans_ << std::endl;
}

} // namespace robotnav

#endif /* TREE_EDGE_IMPL_HPP */
