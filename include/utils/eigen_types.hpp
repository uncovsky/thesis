# pragma once

#include <Eigen/Core>
#include <Eigen/SparseCore>

/* typedefs for matrices used for the sparse model, and geometry. */
template< typename value_t >
using Matrix2D = Eigen::SparseMatrix< value_t, Eigen::RowMajor >;

template< typename value_t >
using Matrix3D = std::vector< Matrix2D< value_t > >;


/* this is a global typedef for the structure used to store points throughout
 * all headers, can be replaced as long as the structure extends operator[] and
 * size() methods ~ TODO: mayby specify as a concept instead
 */
template< typename value_t > 
using Point = std::vector< value_t > ;

