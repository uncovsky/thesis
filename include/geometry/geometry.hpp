# pragma once

# include <algorithm> 
# include <cassert>
# include <iostream>

# include "utils/geometry_utils.hpp"


/* helper classes for geometrical operations, points are again vectors of
 * values, while line segments are used in quickhull and polygon class ( could
 * expand to facets in general, but considering two dimensionsional polygons so
 * far ) */

template < typename value_t >
using Point = std::vector< value_t >;


/* oriented line segment from point x1 to x2 
 * the orientation is relevant when deciding in which halfspace a given point
 * is located w.r.t to the line passing through x1,x2 ( we consider clockwise &
 * counterclockwise halfspaces, see method ccw() )
 * this is used in the quickhull algorithm to filter out points already bounded
 * by given hull vertices
 */
template < typename value_t >
struct LineSegment {
    Point< value_t > x1, x2;

    LineSegment( const Point< value_t >& x1, const Point< value_t >& x2 ) : x1( x1 ), x2( x2 ) {  }

    size_t get_dimensions( ) const {
        assert ( x1.size() == x2.size() );
        return x1.size();
    }

    value_t point_distance( const Point< value_t >& y ) const {
        
        std::vector< value_t > line = subtract( x2, x1 );
        std::vector< value_t > delta = subtract( y, x1 );

        // get projection on line segment
        value_t coeff = std::clamp( dot_product( delta, line ) , value_t( 0 ), value_t( 1 ) );

        Point< value_t > proj = add( x1 , multiply( coeff, line ) );

        return euclidean_distance( proj, y );

    }

    // check whether a point p is located in ccw direction
    // positive result -> true, zero -> lies on the line, negative -> lies
    // in cw halfspace
    double ccw( const Point< value_t > &p ){
            assert( get_dimensions() == 2 );
            value_t res = ( x2[0] - x1[0] ) * ( p[1] - x1[1] ) - ( x2[1] - x1[1] ) * ( p[0] - x1[0] );
            return static_cast< double > ( res );
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

