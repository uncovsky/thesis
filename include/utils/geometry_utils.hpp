# pragma once

# include <cmath>
# include <vector> 
# include <set>
# include "utils/eigen_types.hpp"


/*
 * helper fucntions for linear algebra and miscellaneous component-wise
 * operations ( implemented with vectors for later Storm porting ) 
 */


template < typename value_t >
value_t dot_product( const Point< value_t > &lhs , 
                     const Point< value_t > &rhs) {
    assert( lhs.size() == rhs.size() );
    value_t result(0);
    for ( size_t i = 0; i < rhs.size(); i++ ){
        result += lhs[i] * rhs[i];
    }

    return result;
}



template < typename value_t > 
void multiply( value_t scalar, 
               Point< value_t > &vec ) {
    for ( value_t &elem : vec ){
        elem *= scalar;
    }
}

template < typename value_t > 
void add( value_t scalar, 
          Point< value_t > &vec ) {
    for ( value_t &elem : vec ){
        elem += scalar;
    }
}

template < typename value_t > 
void multiply( Point< value_t > &lhs , 
               const Point< value_t > &rhs ) {

    assert( lhs.size() == rhs.size() );
    for ( size_t i = 0; i < rhs.size(); i++ ){
        lhs[i] *= rhs[i];
    }
}

/* component-wise division */
template < typename value_t > 
void divide( Point< value_t > &lhs , 
             const Point< value_t > &rhs ) {
    assert( lhs.size() == rhs.size() );
    for ( size_t i = 0; i < rhs.size(); i++ ){
        lhs[i] /= rhs[i];
    }
}

template < typename value_t > 
void add( Point< value_t > &lhs, 
                            const Point< value_t > &rhs ) {
    assert ( lhs.size() == rhs.size() );
    for ( size_t i = 0; i < lhs.size(); i++ ) {
        lhs[i] += rhs[i];
    }
}

template < typename value_t > 
void subtract( Point< value_t > &lhs, 
                                 const Point< value_t > &rhs ) {
    assert ( lhs.size() == rhs.size() );
    for ( size_t i = 0; i < lhs.size(); i++ ) {
        lhs[i] -= rhs[i];
    }

}

template < typename value_t > 
Point< value_t > norm( const Point< value_t > &lhs ) {

    return multiply( 1.0/dot_product( lhs, lhs ), lhs );
}


template < typename value_t >
value_t euclidean_distance( const Point< value_t > &lhs , 
                     const Point< value_t > &rhs) {
    assert( lhs.size() == rhs.size() );

    Point< value_t > diff( lhs );
    // get diff
    subtract( diff, rhs );

    return std::sqrt( dot_product( diff, diff ) );
}



// get distance of x from line segment [beg, end]
template < typename value_t > 
value_t line_segment_distance( const Point< value_t > &beg,
                               const Point< value_t > &end,
                               const Point< value_t > &x ) {

    Point< value_t > line( end ), delta( x );
    
    // line is vector of the line segment, delta is vector from x to beg
    subtract( line, beg );
    subtract( delta, beg );

    // get projection on line segment
    value_t norm = dot_product( line, line ) + 0.000001;
    value_t coeff = std::clamp( dot_product( delta, line ) / norm , value_t( 0 ), value_t( 1 ) );

    // get projection, ( line * coeff + x1 )
    multiply( coeff, line );
    add( line, beg );

    // calculate distance of this projection from x
    return euclidean_distance( line, x );
}

/* helper functions for 2D convex hull and minkowski sum 
 * checks if point is in ccw halfspace determined by line x1->x2 
 */
template < typename value_t > 
double ccw( const Point< value_t > &x1,
            const Point< value_t > &x2,
            const Point< value_t > &p ){
    value_t res = ( x2[0] - x1[0] ) * ( p[1] - x1[1] ) - ( x2[1] - x1[1] ) * ( p[0] - x1[0] );
    return static_cast< double > ( res );
}

// returns a vector of extreme points along respective dimensions in input
// all points in input must be of the same dimensions
template < typename value_t > 
std::vector< std::pair< Point< value_t >, Point< value_t > > >  
get_extreme_points( const std::vector< Point< value_t > > &vertices ) {

    if ( vertices.empty() )
        return {};

    std::vector< std::pair< Point< value_t >, Point< value_t > > > res;

    for ( size_t dimension = 0; dimension < vertices.begin()->size(); ++dimension ) {
         auto [ min_it, max_it ] = std::minmax_element( 
                                   vertices.begin(), vertices.end(),
                                   [dimension]( const Point< value_t >& lhs, const Point< value_t >& rhs) 
                                            { return lhs[dimension] < rhs[dimension]; } );
        res.emplace_back( *min_it, *max_it );
    }

    return res;
}

