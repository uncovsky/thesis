# pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>

template< typename value_t >
using Matrix2D = Eigen::SparseMatrix< value_t, Eigen::RowMajor >;

template< typename value_t >
using Matrix3D = std::vector< Matrix2D< value_t > >;



