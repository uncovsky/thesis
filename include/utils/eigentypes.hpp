# pragma once

#include "Eigen/Core"
#include "Eigen/SparseCore"

using Matrix2D = Eigen::SparseMatrix< double, 
                                      Eigen::RowMajor >;

using Matrix3D = std::vector< Matrix2D >;

