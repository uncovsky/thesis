# pragma once 
# include <vector>
# include <algorithm> 
# include <set>
# include "utils/geometry_utils.hpp"


/*
 * Class storing information about a pareto curve ( set of vectors for a given
 * state value pair ), along with updates implemented
 * so far implemented for 1 or 2 dimensions
 *
 */

template < typename value_t >
class ParetoCurve {

    using Point = std::vector< value_t >;
    // replace with facet later
    // for higher dimensions maybe solve QP for distance? or 
    struct LineSegment {
        Point x1, x2;
        LineSegment( const Point& x1, const Point& x2 ) : x1( x1 ), x2( x2 ) {  }

        value_t point_distance( const Point& y ) {
            
            std::vector< value_t > line = subtract( x2, x1 );
            std::vector< value_t > delta = subtract( y, x1 );

            // get projection on line segment
            value_t coeff = std::clamp( dot_product( delta, line ), 0.0, 1.0 );

            Point proj = add( x1 , multiply( coeff, line ) );

            return euclidean_distance( proj, y );

        }

        std::pair< Point, Point > get_points( ) const {
            return { x1, x2 };
        }
    };

    std::set< Point > vertices;
    std::set< LineSegment > facets;


public:

    ParetoCurve( const std::set< Point > &v ) : vertices( v ) {  }
    ParetoCurve( std::set< Point > &&v ) : vertices( v ) {  }


    std::set< Point > get_vertices( ) const {
        return vertices;
    }

    void scalar_multiply( value_t mult ) {
        for ( Point &p : vertices ) {
            p = multiply( mult, p );
        }    
    }

    // this := minkowski sum of this and rhs
    void add_curve( const ParetoCurve &rhs ) {
        std::set< Point > new_vertices;
        std::set< Point > rhs_vertices = rhs.get_vertices();
        for ( const auto &v1 : vertices ) {
            for ( const auto &v2 : rhs_vertices ) {
                new_vertices.insert( add( v1, v2 ) );
            }
        }

        remove_dominated( new_vertices );
        vertices = new_vertices;
    }

    // downward closure for 1/2 dimensions
    // preconditions:
    //  array vertices contains only nondominated elements
    //  vertices form a convex polygon ( so convex_hull has been called
    //  beforehand )
    // reference point contains minimal values for each objective
    void downward_closure( Point reference_point ) {
        assert ( reference_point.size() < 3 );

        if ( reference_point.size() == 0 )
            return;

        std::set< LineSegment > new_facets;

        if ( reference_point.size() == 1 ) {

           // only one nondom vertex is possible 
            if ( !vertices.empty() ) { 
                new_facets.emplace_back( vertices[0], reference_point );
            }
        }

        else {

            Point max_x_point, max_y_point;

            for ( size_t dimension : {0, 1} ) {
                Point res = *std::max_element( 
                                  vertices.begin(), vertices.end(),
                                  []( const Point& lhs, const Point& rhs) 
                                            { return lhs[0] < rhs[0]; } );

                if ( dimension == 1 ) { max_x_point = res; }
                else                  { max_y_point = res; }

            }

            new_facets = std::move( facets );

            // just add two line segments from extremal points of the curve for now
            // need to also delete facets that are now interior, todo later, 
            // but shouldn't matter with distance calculations 
            Point facet_x = { max_x_point[0], reference_point[1] };
            Point facet_y = { reference_point[0], max_y_point[1] };

            new_facets.emplace_back( facet_x, max_x_point );
            new_facets.emplace_back( facet_y, max_y_point );

        }
    }
};
