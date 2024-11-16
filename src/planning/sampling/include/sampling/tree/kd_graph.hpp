/*
 * kd_graph.hpp
 *
 * Created on: Jan 02, 2019 09:36
 * Description: a graph with a internal kd-tree to support NN queries
 *              used for RRG algorithm
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef KD_GRAPH_HPP
#define KD_GRAPH_HPP

#include <limits>
#include <algorithm>

#include "spatial/spatial.hpp"
#include "spatial/point_multiset.hpp"
#include "spatial/neighbor_iterator.hpp"

#include "graph/graph.hpp"
#include "sampling/base/tree_adapter.hpp"

namespace xmotion {
template <typename Space>
class KdGraph
    : public Graph<std::shared_ptr<typename Space::StateType>, double>,
      public TreeAdapter<Space> {
 public:
  using TreeType = Graph<std::shared_ptr<typename Space::StateType>, double>;
  using AdapterType = TreeAdapter<Space>;
  using StateType = typename TreeAdapter<Space>::StateType;
  using PathType = typename TreeAdapter<Space>::PathType;

  // inherit constructors
  using Graph<std::shared_ptr<typename Space::StateType>, double>::Graph;
  using TreeAdapter<Space>::TreeAdapter;

  // setup Kd-tree types
  struct StateAccessor {
    double operator()(spatial::dimension_type dim,
                      std::shared_ptr<StateType> state) const {
      return (*state)[dim];
    }
  };

  using KdTreeType = spatial::point_multiset<
      Space::DimensionSize, std::shared_ptr<StateType>,
      spatial::accessor_less<StateAccessor, std::shared_ptr<StateType>>>;

 public:
  void AddTreeRootNode(std::shared_ptr<StateType> sstate) final {
    TreeType::AddVertex(sstate);
    kdtree_.insert(sstate);
  }

  void ConnectTreeNodes(std::shared_ptr<StateType> sstate,
                        std::shared_ptr<StateType> dstate, double dist) final {
    if (TreeType::FindVertex(sstate) == TreeType::vertex_end())
      kdtree_.insert(sstate);
    if (TreeType::FindVertex(dstate) == TreeType::vertex_end())
      kdtree_.insert(dstate);
    TreeType::AddUndirectedEdge(sstate, dstate, dist);
  }

  void DisconnectTreeNodes(std::shared_ptr<StateType> sstate,
                           std::shared_ptr<StateType> dstate) final {
    TreeType::RemoveUndirectedEdge(sstate, dstate);
  }

  void PrintTreeInfo() {
    std::cout << "nodes in graph: " << TreeType::GetTotalVertexNumber()
              << " , nodes in kd tree: " << kdtree_.size() << std::endl;
  }

  std::size_t GetTotalTreeNodeNumber() final {
    return TreeType::GetTotalVertexNumber();
  }

  // Note: this function is ill-defined for a graph, can be ignored
  PathType TraceBackToRoot(std::shared_ptr<StateType> state) final {
    PathType path;
    // path.push_back(state);
    // auto parent = TreeType::GetParentVertex(state);
    // while (parent != TreeType::vertex_end())
    // {
    //     path.push_back(parent->state_);
    //     parent = TreeType::GetParentVertex(parent->vertex_id_);
    // }
    // std::reverse(path.begin(), path.end());
    return path;
  }

  std::shared_ptr<StateType> FindNearest(
      std::shared_ptr<StateType> state) final {
    spatial::neighbor_iterator<KdTreeType> iter =
        spatial::neighbor_begin(kdtree_, state);
    return (*iter);
  }

  std::vector<std::shared_ptr<StateType>> FindNear(
      std::shared_ptr<StateType> state, double radius) final {
    std::vector<std::shared_ptr<StateType>> near_states;

    spatial::neighbor_iterator<KdTreeType> iter =
        spatial::neighbor_begin(kdtree_, state);

    for (auto it = iter; it != kdtree_.end(); ++it) {
      if (distance(it) <= radius)
        near_states.push_back(*it);
      else
        break;
    }

    return near_states;
  }

 private:
  KdTreeType kdtree_;
};
}  // namespace xmotion

#endif /* KD_GRAPH_HPP */
