/*
 * @file introspection_io.hpp
 * @brief Offline sink for MPPI introspection snapshots: one JSON object per
 * line (JSONL), consumable by plotting scripts or converted to viewer
 * formats. Deliberately dependency-free — the family's insight tooling
 * converts offline artifacts into existing viewers rather than building
 * live pipelines.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_INTROSPECTION_IO_HPP
#define XMNAV_MPPI_INTROSPECTION_IO_HPP

#include <ostream>

#include "xmnav/mppi/mppi.hpp"

namespace xmotion {
namespace mppi_io_detail {

template <typename Derived>
void WriteMatrix(std::ostream &os, const Eigen::MatrixBase<Derived> &m) {
  os << '[';
  for (Eigen::Index r = 0; r < m.rows(); ++r) {
    if (r != 0) os << ',';
    os << '[';
    for (Eigen::Index c = 0; c < m.cols(); ++c) {
      if (c != 0) os << ',';
      os << m(r, c);
    }
    os << ']';
  }
  os << ']';
}

}  // namespace mppi_io_detail

// Append one snapshot as a single JSONL record.
template <int StateDim, int ControlDim>
void AppendSnapshotJsonl(std::ostream &os,
                         const MppiSnapshot<StateDim, ControlDim> &snap) {
  os << "{\"plan\":" << snap.plan_index << ",\"ess\":"
     << snap.effective_sample_size << ",\"best_cost\":" << snap.best_cost
     << ",\"nominal_states\":";
  mppi_io_detail::WriteMatrix(os, snap.nominal_states);
  os << ",\"nominal_controls\":";
  mppi_io_detail::WriteMatrix(os, snap.nominal_controls);
  os << ",\"candidates\":[";
  for (std::size_t i = 0; i < snap.candidates.size(); ++i) {
    const auto &c = snap.candidates[i];
    if (i != 0) os << ',';
    os << "{\"cost\":" << c.cost << ",\"weight\":" << c.weight
       << ",\"states\":";
    mppi_io_detail::WriteMatrix(os, c.states);
    os << '}';
  }
  os << "]}\n";
}

}  // namespace xmotion

#endif  // XMNAV_MPPI_INTROSPECTION_IO_HPP
