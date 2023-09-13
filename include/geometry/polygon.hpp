# pragma once 

# include <algorithm> 
# include <iostream>
# include <fstream>
# include <set>
# include <sstream>
# include <string>

# include "geometry/geometry.hpp"
# include "geometry/pareto.hpp"
# include "geometry/quickhull.hpp"
# include "utils/geometry_utils.hpp"


/*
 * 2D polygon class to track the pareto curve, the curve is saved as its 
 * vertices ( nondominated points after each update ).
 *
 * We make a few assumptions about the data:
 *  - facets are only initialized after a call to convex_hull ( which also sets
 *  vertices to ones of the hull )
 *  - convex hull operation sorts the vertices lexicographically
 *
 */

template < typename value_t >
class Polygon {

    std::vector< Point< value_t > > vertices;
    std::vector< LineSegment< value_t > > facets;


public:

    Polygon( ) : vertices( ), facets( ) {  }

    Polygon( const std::vector< Point< value_t > > &v ) : vertices( v ),
                                            facets ( ) {  }

    Polygon( std::vector< Point< value_t > > &&v ) :  vertices( std::move( v ) ), 
                                           facets ( ) {  }
    Polygon( const std::vector< Point< value_t > > &v,
             const std::vector< LineSegment< value_t > > &f ) : vertices( v ),
                                                                facets ( f ) {  }

    bool operator==( Polygon& rhs ) const {
        bool equal_f = facets == rhs.get_facets();
        bool equal_v = vertices == rhs.get_vertices();

        return equal_f && equal_v;
    }


    const std::vector< Point< value_t > >& get_vertices( ) const {
        return vertices;
    }

    const std::vector< LineSegment< value_t > >& get_facets( ) const {
        return facets;
    }

    std::vector< LineSegment< value_t > >& get_facets( ) {
        return facets;
    }

    std::vector< Point< value_t > >& get_vertices( ) {
        return vertices;
    }

    size_t get_dimension() const {
        if ( vertices.empty() ) 
            return 0;
        return vertices[0].size();
    }

    /* multiplication & addition, both element-wise and single elements */
    void multiply_scalar( value_t mult ) {
        for ( Point< value_t > &p : vertices ) {
            multiply( mult, p );
        }    
    }

    void multiply_vector( const Point< value_t > &mult ) {
        for ( Point< value_t > &p : vertices ) {
            multiply( p, mult );
        }    
    }

    void shift_scalar( value_t shift ) {
        for ( Point< value_t > &p : vertices ) {
            // foreach i p[i] += shift
            add( shift, p );
        }    

    }

    void shift_vector( const std::vector< value_t > &shift ) {
        for ( Point< value_t > &p : vertices ) {
            // p += shift
            add( p, shift );
        }    

    }

    // TODO: use the fact that both polygons are convex, so its possible to
    // compute in linear time instead of O(mn)
    void minkowski_sum( const Polygon &rhs ) {
        std::vector< Point< value_t > > new_vertices;
        const std::vector< Point< value_t > > &rhs_vertices = rhs.get_vertices();

        if ( rhs_vertices.empty() ) { 
            return; 
        }
        else if ( vertices.empty() ) { 
            vertices = rhs_vertices;
            return;
        }
        for ( const auto &v1 : vertices ) {
            for ( const auto &v2 : rhs_vertices ) {
                Point< value_t > v1_copy( v1 );
                add( v1_copy, v2  );
                new_vertices.emplace_back( v1_copy );
            }
        }

        vertices = std::move( new_vertices );
    }

    /* downward closure for 1/2 dimensions
     * preconditions:
     *  array vertices contains only nondominated elements
     *  vertices form a convex polygon ( so convex_hull has been called
     *  beforehand )
     * reference point contains minimal values for each objective
     */
    void downward_closure( const Point< value_t > &reference_point ) {

        assert( get_dimension() == reference_point.size() );

        if ( get_dimension() > 2 ) {
            std::cout << "Higher dimension downward closures are currently unsupported." << std::endl;
            // assert( false );
            return;
        }

        /* 1d downward closure is irrelevant */
        if ( reference_point.size() < 2 )
            return;

        if ( vertices.empty() )
            return;

        auto extreme_points = get_extreme_points( vertices );

        Point< value_t > max_x_point = extreme_points[0].second;
        Point< value_t > max_y_point = extreme_points[1].second;


        // add two line segments from extremal points of the curve 
        Point< value_t > facet_x = { max_x_point[0], reference_point[1] };
        Point< value_t > facet_y = { reference_point[0], max_y_point[1] };
        facets.emplace_back( facet_x, max_x_point );
        facets.emplace_back( facet_y, max_y_point );
    }



    // computes convex hull of vertices, removing all but the vertices forming
    // the hull. also correctly initializes facets 
    void convex_hull() {

        if ( vertices.empty() )
            return;
        if ( get_dimension() > 2 ) {
            std::cout << "Higher dimension convex hulls are currently unsupported." << std::endl;
            assert( false );
        }

        // handle one dimensional case, since only nondominated points, only
        // one vertex is possible
        if ( get_dimension() == 1 ) {
            facets = { LineSegment< value_t >( vertices[0], vertices[0] ) };
            return;
        }
        

        std::vector< Point< value_t > > result = quickhull( vertices );
        std::vector< LineSegment< value_t > > new_facets;

        size_t vertex_count = result.size();

        for ( size_t i = 0; i < vertex_count - 1; i++ ) {
            new_facets.emplace_back( result[i], result[i + 1] );
        }

        if ( vertex_count > 1 )
            new_facets.emplace_back( result.back(), result[0] );

        std::vector< Point< value_t > > new_vertices( result.begin(), result.end() );

        vertices = std::move( new_vertices );
        facets = std::move( new_facets );
    }

    // more efficient + automatically remove dominated solutions
    // eps determines the granularity of the hull
    void upper_right_hull( double eps=1e-4 ){
        if ( vertices.empty() )
            return;
        if ( get_dimension() > 2 ) {
            std::cout << "Higher dimension convex hulls are currently unsupported." << std::endl;
            assert( false );
        }

        // keep only max element
        if ( get_dimension() == 1 ) {
            vertices = { *std::max_element( vertices.begin(), vertices.end() ) };
            facets = { LineSegment< value_t >( vertices[0], vertices[0] ) };
            return;
        }

        std::sort( vertices.begin(), vertices.end() );

        std::vector< Point< value_t > > hull = { vertices.back() } ;

        for ( auto it = vertices.rbegin() + 1; it != vertices.rend(); it++ ){
            Point< value_t > pt = *it;
            // need increasing y
            if ( pt[1] <= hull.back()[1] ) { continue; }
            else if ( hull.size() < 2 ) { hull.push_back( pt ); }
            else { 
                size_t i = hull.size() - 1;
                while ( ( hull.size() >= 2 ) && ( ccw( hull[i - 1], hull[i], pt ) <= eps ) ){
                    hull.pop_back();
                    i--;
                }
                hull.push_back( pt );
            }
        }

        facets.clear();

        for ( size_t i = 0; i < hull.size() - 1; i++ ) {
            facets.emplace_back( hull[i], hull[i + 1] );
        }

        vertices = std::move( hull );

    }

    void pareto( const Point< value_t >& ref_point ){
        upper_right_hull();
        downward_closure( ref_point );
    }

    // precondition: pareto function has been called beforehand, facets are
    // correctly initialized, input point is not dominated by any point on
    // *this pareto curve
    value_t point_distance( const Point< value_t >& point ) const {
        if ( facets.empty() )
            return 0;

        // handle one dimensional case, only one vertex is possible again
        if ( get_dimension() == 1 ) {
            return point[0] - vertices[0][0];
        }

        LineSegment< value_t > first_facet = facets[0];
        value_t min_distance = first_facet.point_distance( point );

        for ( const auto &ls : facets  ) {
           min_distance = std::min( min_distance, ls.point_distance( point ) );
        }

        return min_distance;
    }

    /* computes hausdorff distance of two polygons, assuming *this is contained
     * in upper_polygon entirely and facets of *this are initialized properly
     * ( convex hull call preceded this )
     */
    value_t hausdorff_distance( const Polygon& upper_polygon ) {
        value_t max_distance = 0;

        for ( const auto &v : upper_polygon.get_vertices() ) {
            max_distance = std::max( max_distance, point_distance( v ) );
        }
        return max_distance;
    }


    std::string to_string( ) const {
        std::stringstream str;

        for ( const auto &vert : vertices ) {
            for ( value_t val : vert ) {
                str << val << " ";
            }
            str << "\n";
        }
        /*
        str << "\nFACETS:\n";
        for ( const auto &facet : facets ) {
            auto [x, y] = facet.get_points();

            for ( auto a : x )
                str << a << " ";

            str << " ; ";

            for ( auto a : y )
                str << a << " ";
            str << "\n";
            }
        */

        return str.str();
    }

    void write_to_file( std::string filename ) const {
        std::ofstream str( filename, std::ios::out );
        str << to_string();
    }
};
