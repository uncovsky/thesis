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

    bool operator==( Polygon& rhs ) {

        std::sort( facets.begin(), facets.end() );
        std::sort( vertices.begin(), vertices.end() );

        std::sort( rhs.get_facets().begin(), rhs.get_facets().end() );
        std::sort( rhs.get_vertices().begin(), rhs.get_vertices().end() );

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

    // hacky solution since points tracked as std::vectors,
    // could change them to Eigen::Vectors / arrs and template the dims
    size_t get_dimension() const {
        if ( vertices.empty() ) 
            return 0;
        
        Point< value_t > vertex = *( vertices.begin() );

        return vertex.size();
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

    // TODO more efficiently ( use the fact both are convex )
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

    // downward closure for 1/2 dimensions
    // preconditions:
    // nondominated elems removed, convex_hull called bforehand
    void downward_closure( const Point< value_t > &reference_point ) {
        assert ( reference_point.size() < 3 );

        // nothing to be done in 0d/1d case
        if ( reference_point.size() < 2 )
            return;
        if ( vertices.empty() )
            return;

        std::vector< LineSegment< value_t > > new_facets;

        auto extreme_points = get_extreme_points( vertices );

        Point< value_t > max_x_point = extreme_points[0].second;
        Point< value_t > max_y_point = extreme_points[1].second;

    }


    // computes convex hull of vertices, removing all but the vertices forming
    // the hull. also correctly initializes facets 
    void convex_hull() {
        if ( vertices.empty() )
            return;

        // handle one dimensional case, facets are just points
        if ( get_dimension() == 1 ) {
            auto [ min_x, max_x ] = get_extreme_points( vertices )[0];

            if ( min_x == max_x ) {
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

        std::vector< Point< value_t > > result = quickhull( vertices );
        std::vector< LineSegment< value_t > > new_facets;

        size_t vertex_count = result.size();

        // vertices are sorted in ccw order
        for ( size_t i = 0; i < vertex_count - 1; i++ ) {
            new_facets.emplace_back( result[i], result[i + 1] );
        }

        if ( vertex_count > 1 )
            new_facets.emplace_back( result.back(), result[0] );

        vertices = result;
        facets = new_facets;
    }

    void pareto( const Point< value_t >& ref_point ){
        convex_hull();
        downward_closure( ref_point );
    }

    // precondition: convex hull has been called beforehand, facets are
    // correctly initialized
    value_t point_distance( const Point< value_t >& point ) const {
        if ( facets.empty() )
            return 0;

        // handle one dimensional case, 
        // at most 2 vertices after calling convex_hull()
        if ( get_dimension() == 1 ) {
            auto [ min_x, max_x ] = get_extreme_points( vertices )[0];
            return std::min( min_x[0] - point[0], max_x[0] - point[0] );
        }

        LineSegment< value_t > first_facet = facets[0];
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
