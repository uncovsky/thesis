# pragma once

#include <iostream>
# include <algorithm> 

# include "utils/geometry_utils.hpp"


/* helper classes for geometrical operations, points are again vectors of
 * values, while line segments are used in quickhull and polygon class ( could
 * expand to facets in general, but considering two dimensionsional polygons so
 * far ) */

template < typename value_t >
using Point = std::vector< value_t >;

template < typename value_t >
struct LineSegment {
    Point< value_t > x1, x2;

        LineSegment( const Point< value_t >& _x1, const Point< value_t >& _x2 ) : x1(), x2() { 
            // order the two points lexicographically
            x1 = std::min( _x1, _x2 );
            x2 = std::max( _x1, _x2 );
        }

        value_t point_distance( const Point< value_t >& y ) const {
            
            std::vector< value_t > line = subtract( x2, x1 );
            value_t norm = dot_product( line, line );

            std::vector< value_t > delta = subtract( y, x1 );

            // get projection on line segment
            value_t coeff = std::clamp( dot_product( delta, line ) / norm , 0.0, 1.0 );

            Point< value_t > proj = add( x1 , multiply( coeff, line ) );

            return euclidean_distance( proj, y );

        }

        std::pair< Point< value_t >, Point< value_t > > get_points() const {
            return { x1, x2 };
        }

        bool operator<( const LineSegment &rhs ) const {
            auto [ y1, y2 ] = rhs.get_points();
            return ( x1 < y1 ) || ( ( x1 == y1 ) && ( x2 < y2 ) );
        }

        bool operator==( const LineSegment &rhs ) const {
            auto [ y1, y2 ] = rhs.get_points();
            return ( x1 == y1 ) && ( x2 == y2 );
        }
};

// returns a vector of extreme points along respective dimensions in input
// all points in input must be of the same dimensions
template < typename value_t > 

std::vector< std::pair< Point< value_t >, Point< value_t > > >  
get_extreme_points( const std::set< Point< value_t > > &vertices ) {

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


// get the id of halfspace formed by line segment ab point c is located on
// works only for 2d
// if positive, the point c is located in ccw direction from line segment a,b
// if zero, points are colinear
template < typename value_t >
double ccw ( const Point< value_t > &a, 
             const Point< value_t > &b, 
             const Point< value_t > &c ) {

    assert( a.size() == 2 );
    value_t res = ( b[0] - a[0] ) * ( c[1] - a[1] ) - ( b[1] - a[1] ) * ( c[0] - a[0] );

    return static_cast< double > ( res );
}


