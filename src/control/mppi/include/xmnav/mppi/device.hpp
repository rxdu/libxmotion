/*
 * @file device.hpp
 * @brief Host/device annotation for MPPI functor cores.
 *
 * Model and cost cores shared between the CPU and CUDA rollout backends
 * (see docs/typst/mppi.typ: "the same functor code compiles for both
 * backends") carry XMNAV_HD so nvcc emits device code for them; host-only
 * compilers see plain inline functions.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_MPPI_DEVICE_HPP
#define XMNAV_MPPI_DEVICE_HPP

#if defined(__CUDACC__)
#define XMNAV_HD __host__ __device__
#else
#define XMNAV_HD
#endif

#endif  // XMNAV_MPPI_DEVICE_HPP
