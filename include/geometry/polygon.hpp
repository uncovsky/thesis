# pragma once 

# include <algorithm> 
# include <fstream>
# include <set>
# include <string>

# include "geometry/geometry.hpp"
# include "geometry/pareto.hpp"
# include "geometry/quickhull.hpp"
# include "utils/geometry_utils.hpp"


/*
 * polygon class, which will be used to store
 * the pareto curve for each state-value pair
 * along with updates implemented
 * so far implemented for 1 or 2 dimensions
 *
 */

template < typename value_t >
class Polygon {

    // replace with facet later
    // for higher dimensions maybe solve QP for distance?
    //
    using Point = std::vector< value_t >;

    std::set< Point > vertices;
    std::set< LineSegment< value_t > > facets;


public:

    Polygon( const std::set< Point > &v ) : vertices( v ) {  }
    Polygon( std::set< Point > &&v ) : vertices( std::move( v ) ) {  }


    std::set< Point > get_vertices( ) const {
        return vertices;
    }

    std::set< Point >& get_vertices( ) {
        return vertices;
    }

    void scalar_multiply( value_t mult ) {
        std::set< Point > new_vertices;
        for ( const Point &p : vertices ) {
            new_vertices.insert( multiply( mult, p ) );
        }    

        vertices = new_vertices;
    }

    // this := minkowski sum of this and rhs
    void add_curve( const Polygon &rhs ) {
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

        std::set< LineSegment< value_t > > new_facets;

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
                                  [dimension]( const Point& lhs, const Point& rhs) 
                                            { return lhs[dimension] < rhs[dimension]; } );

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

    void convex_hull() {
        std::vector< Point > result = quickhull( vertices );
        std::set< LineSegment< value_t > > new_facets;

        if ( result.size() > 1 ) {
            for ( size_t i = 0; i < result.size() - 1; i++ ){
                // vertices are sorted in ccw order
                new_facets.emplace(  result[i], result[i+1] );
            }

            new_facets.emplace ( result.back(), result[0] );
        }

        std::set< Point > new_vertices( result.begin(), result.end() );

        vertices = new_vertices;
        facets = new_facets;
    }

    void write_to_file( std::string filename ) {
        std::ofstream str( filename, std::ios::out );
        str << "VERTICES:\n";
        for ( const auto &vert : vertices ) {
            for ( value_t val : vert ) {
                str << val << ";";
            }
            str << "\n";
        }
        str << "\n\n\nFACETS:\n";
        for ( const auto &facet : facets ) {
            auto [x, y] = facet.get_points();

            for ( auto a : x )
                str << a << " ";

            str << " ; ";

            for ( auto a : y )
                str << a << " ";
            str << "\n";
            }
    
    }
};
