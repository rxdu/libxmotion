/*
 * basic_tree.hpp
 *
 * Created on: Dec 30, 2018 22:10
 * Description:
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef BASIC_TREE_HPP
#define BASIC_TREE_HPP

#include <limits>
#include <memory>
#include <algorithm>

#include "graph/tree.hpp"
#include "sampling/base/tree_adapter.hpp"

namespace robosw {
template <typename Space>
class BasicTree
    : public Tree<std::shared_ptr<typename Space::StateType>, double>,
      public TreeAdapter<Space> {
 public:
  using TreeType = Tree<std::shared_ptr<typename Space::StateType>, double>;
  using AdapterType = TreeAdapter<Space>;
  using StateType = typename TreeAdapter<Space>::StateType;
  using PathType = typename TreeAdapter<Space>::PathType;

  // inherit constructors
  using Tree<std::shared_ptr<typename Space::StateType>, double>::Tree;
  using TreeAdapter<Space>::TreeAdapter;

 public:
  void AddTreeRootNode(std::shared_ptr<StateType> sstate) final {
    TreeType::AddVertex(sstate);
  }

  void ConnectTreeNodes(std::shared_ptr<StateType> sstate,
                        std::shared_ptr<StateType> dstate, double dist) final {
    TreeType::AddEdge(sstate, dstate, dist);
  }

  void DisconnectTreeNodes(std::shared_ptr<StateType> sstate,
                           std::shared_ptr<StateType> dstate) final {
    TreeType::RemoveEdge(sstate, dstate);
  }

  std::size_t GetTotalTreeNodeNumber() final {
    return TreeType::GetTotalVertexNumber();
  }

  PathType TraceBackToRoot(std::shared_ptr<StateType> state) final {
    PathType path;
    path.push_back(state);
    auto parent = TreeType::GetParentVertex(state);
    while (parent != TreeType::vertex_end()) {
      path.push_back(parent->state);
      parent = TreeType::GetParentVertex(parent->vertex_id);
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  std::shared_ptr<StateType> FindNearest(
      std::shared_ptr<StateType> state) final {
    double min_dist = std::numeric_limits<double>::max();
    std::shared_ptr<StateType> min_state = nullptr;
    for (auto it = TreeType::vertex_begin(); it != TreeType::vertex_end();
         ++it) {
      double dist = AdapterType::space->EvaluateDistance(state, it->state);
      if (dist < min_dist) {
        min_dist = dist;
        min_state = it->state;
      }
    }
    return min_state;
  }

  std::vector<std::shared_ptr<StateType>> FindNear(
      std::shared_ptr<StateType> state, double radius) final {
    // TODO
    std::vector<std::shared_ptr<StateType>> empty;
    return empty;
  }
};
}  // namespace robosw

#endif /* BASIC_TREE_HPP */
