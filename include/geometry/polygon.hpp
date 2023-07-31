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
 * 2D Convex polygon class ( implemented as a collection of points and
 * associated facets / line segments )
 * implemented:
 * convex hull operation that removes redundant points 
 * (i.e only keeps vertices of the convex hull) and sets facets (line segments)
 *
 * downward closure operation that simply adds two facets from extreme points after the
 * convex hull operation has run
 */

template < typename value_t >
class Polygon {

    // replace with facet later
    // for higher dimensions maybe solve QP for distance?
    using Point = std::vector< value_t >;

    std::set< Point > vertices;
    std::set< LineSegment< value_t > > facets;


public:

    Polygon( ) : vertices( ), facets( ) {  }

    Polygon( const std::set< Point > &v ) : vertices( v ),
                                            facets ( ) {  }

    Polygon( std::set< Point > &&v ) :  vertices( std::move( v ) ), 
                                        facets ( ) {  }
    Polygon( const std::set< Point > &v,
             const std::set< LineSegment< value_t > > &f ) : vertices( v ),
                                                             facets ( f ) {  }

    bool operator==( const Polygon& rhs ) const {
        bool equal_f = facets == rhs.get_facets();
        bool equal_v = vertices == rhs.get_vertices();

        return equal_f && equal_v;
    }


    const std::set< Point >& get_vertices( ) const {
        return vertices;
    }

    const std::set< LineSegment< value_t > >& get_facets( ) const {
        return facets;
    }

    std::set< LineSegment< value_t > >& get_facets( ) {
        return facets;
    }

    std::set< Point >& get_vertices( ) {
        return vertices;
    }

    // hacky solution since points tracked as std::vectors,
    // could change them to Eigen::Vectors / arrs and template the dims
    size_t get_dimension() const {
        if ( vertices.empty() ) 
            return 0;
        
        Point vertex = *( vertices.begin() );

        return vertex.size();
    }

    /* multiplication & addition, both element-wise and single elements */

    void multiply_scalar( value_t mult ) {
        std::set< Point > new_vertices;
        for ( const Point &p : vertices ) {
            new_vertices.insert( multiply( mult, p ) );
        }    

        vertices = std::move( new_vertices );
    }

    void multiply_vector( const Point &mult ) {
        std::set< Point > new_vertices;
        for ( const Point &p : vertices ) {
            new_vertices.insert( multiply( mult, p ) );
        }    

        vertices = std::move( new_vertices );
    }

    void shift_scalar( value_t shift ) {
        std::set< Point > new_vertices;
        for ( const Point &p : vertices ) {
            new_vertices.insert( add( shift, p ) );
        }    

        vertices = std::move( new_vertices );
    }

    void shift_vector( const std::vector< value_t > &shift ) {
        std::set< Point > new_vertices;
        for ( const Point &p : vertices ) {
            new_vertices.insert( add( shift, p ) );
        }    

        vertices = std::move( new_vertices );
    }

    void minkowski_sum( const Polygon &rhs ) {
        std::set< Point > new_vertices;
        std::set< Point > rhs_vertices = rhs.get_vertices();

        if ( rhs_vertices.empty() ) { 
            return; 
        }
        else if ( vertices.empty() ) { 
            vertices = rhs_vertices;
            return;
        }
        for ( const auto &v1 : vertices ) {
            for ( const auto &v2 : rhs_vertices ) {
                new_vertices.insert( add( v1, v2 ) );
            }
        }

        vertices = new_vertices;
    }

    // downward closure for 1/2 dimensions
    // preconditions:
    //  array vertices contains only nondominated elements
    //  vertices form a convex polygon ( so convex_hull has been called
    //  beforehand )
    // reference point contains minimal values for each objective
    void downward_closure( const Point &reference_point ) {
        assert ( reference_point.size() < 3 );

        if ( reference_point.size() == 0 )
            return;

        std::set< LineSegment< value_t > > new_facets;

        if ( reference_point.size() == 1 ) {

           // only one nondom vertex is possible 
            if ( !vertices.empty() ) { 
                new_facets.emplace( *(vertices.begin() ), reference_point );
            }
        }

        else {

            auto extreme_points = get_extreme_points( vertices );

            Point max_x_point = extreme_points[0].second;
            Point max_y_point = extreme_points[1].second;

            new_facets = std::move( facets );

            // just add two line segments from extremal points of the curve for now
            // need to also delete facets that are now interior, todo later, 
            // but shouldn't matter with distance calculations 
            Point facet_x = { max_x_point[0], reference_point[1] };
            Point facet_y = { reference_point[0], max_y_point[1] };

            new_facets.emplace( facet_x, max_x_point );
            new_facets.emplace( facet_y, max_y_point );

        }

        facets = std::move( new_facets );
    }


    // computes convex hull of vertices, removing all but the vertices forming
    // the hull. also correctly initializes facets 
    void convex_hull() {
        if ( vertices.empty() )
            return;

        // handle one dimensional case, facets are just points
        if ( get_dimension() == 1 ) {
            auto [ min_x, max_x ] = get_extreme_points( vertices )[0];

            if ( vertices.size() == 1 ) {
                vertices = { min_x };
                facets = { LineSegment< value_t > { min_x, min_x } };
            }
            else {
                vertices = { min_x, max_x };
                facets = { LineSegment< value_t > { min_x, min_x },
                           LineSegment< value_t > { max_x, max_x }} ;
            }

            return;
        }
        

        std::vector< Point > result = quickhull( vertices );
        std::set< LineSegment< value_t > > new_facets;

        size_t vertex_count = result.size();

        for ( size_t i = 0; i < vertex_count; i++ ) {
            new_facets.emplace( result[i], result[ ( i + 1 ) % vertex_count] );
        }

        std::set< Point > new_vertices( result.begin(), result.end() );

        vertices = new_vertices;
        facets = new_facets;
    }

    void pareto( const Point& ref_point ){
        convex_hull();
        downward_closure( ref_point );
    }

    // precondition: convex hull has been called beforehand, facets are
    // correctly initialized
    value_t point_distance( const Point& point ) const {
        if ( facets.empty() )
            return 0;

        // handle one dimensional case, 
        // at most 2 vertices after calling convex_hull()
        if ( get_dimension() == 1 ) {
            auto [ min_x, max_x ] = get_extreme_points( vertices )[0];
            return std::min( min_x[0] - point[0], max_x[0] - point[0] );
        }

        LineSegment< value_t > first_facet = *( facets.begin() );

        value_t min_distance = first_facet.point_distance( point );

        for ( const auto &ls : facets  ) {
           min_distance = std::min( min_distance, ls.point_distance( point ) );
        }

        return min_distance;
    }

    // computes hausdorff distance of two polygons, assuming *this is contained
    // in upper_polygon entirely and facets of *this are initialized properly
    // ( convex hull call preceded this )
    value_t hausdorff_distance( const Polygon& upper_polygon ) {
        value_t max_distance = 0;

        for ( const auto &v : upper_polygon.get_vertices() ) {
            max_distance = std::max( max_distance, point_distance( v ) );
        }
        return max_distance;
    }


    std::string to_string( ) const {
        std::stringstream str;

        str << "VERTICES:\n";
        for ( const auto &vert : vertices ) {
            for ( value_t val : vert ) {
                str << val << ";";
            }
            str << "\n";
        }
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

        return str.str();
    }

    void write_to_file( std::string filename ) const {
        std::ofstream str( filename, std::ios::out );
        str << to_string();
    }
};
