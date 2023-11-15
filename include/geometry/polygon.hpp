# pragma once 

# include <algorithm> 
# include <iostream>
# include <fstream>
# include <sstream>
# include <string>

# include "utils/prng.hpp"
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
class ParetoCurve {

    // class storing facet information, currently only 1d ( line segments ) support
    struct Facet {
        std::vector< Point< value_t > > points;
        value_t point_distance( const Point< value_t >& y ) const {
        
            if ( points.size() != 2 ) {
                std::cout << "error: only two dimensional polygons are currently supported.\n";
                throw std::runtime_error("invalid facet operation.");
            }

            std::vector< value_t > line( points[1] ), delta( y );
            
            // line is vector of the line segment, delta is vector from x1 to y
            subtract( line, points[0] );
            subtract( delta, points[0] );

            // get projection on line segment
            value_t coeff = std::clamp( dot_product( delta, line ) 
                                      , value_t( 0 )
                                      , value_t( 1 ) );

            // get projection, ( line * coeff + x1 )
            multiply( coeff, line );
            add( line, points[0] );

            // calculate distance of this projection (saved in line) from y
            return euclidean_distance( line, y );

        }

        Facet() : points() {}
        Facet( const std::vector< Point< value_t > > &_points ) : points( _points ) {
            // keep points in lexicographic order
            std::sort( points.begin(), points.end() );
        }
            

        bool operator==( const Facet& other ) const{
            return points == other.points;
        }
    };

    std::vector< Point< value_t > > vertices;
    std::vector< Facet > facets;

public:

    ParetoCurve( ) : vertices( ), facets( ), maximizing_indices() {  }

    ParetoCurve( const std::vector< Point< value_t > > &v ) : vertices( v )
                                                            , facets ( ) {}
                                                        

    ParetoCurve( std::vector< Point< value_t > > &&v ) : vertices( std::move( v ) )
                                                       , facets ( ) {  }

    bool operator==( const ParetoCurve& rhs ) const {
        bool equal_f = facets == rhs.get_facets();
        bool equal_v = vertices == rhs.get_vertices();

        return equal_f && equal_v;
    }


    const std::vector< Point< value_t > >& get_vertices( ) const {
        return vertices;
    }

    const std::vector< Facet >& get_facets( ) const {
        return facets;
    }

    std::vector< Facet >& get_facets( ) {
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

    size_t size() const {
        return vertices.size();
    }

    Point< value_t > get_vertex( size_t i ) const {
        return vertices[i];
    }


    void multiply_scalar( value_t mult ) {
        for ( Point< value_t > &p : vertices ) {
            // p = mult p
            multiply( mult, p );
        }    
    }

    void multiply_vector( const std::vector< value_t > &mult ) {
        for ( Point< value_t > &p : vertices ) {
            multiply( p, mult );
        }    

    }

    void shift_vector( const std::vector< value_t > &shift ) {
        for ( Point< value_t > &p : vertices ) {
            // p += shift
            add( p, shift );
        }    

    }

    /* precondition:
     *  convex hull called ( vertices sorted + convex polygon )
     */
    void init_facets() {
        facets.clear();

        if ( vertices.size() == 1 ) {
            facets.push_back( Facet( { vertices[0], vertices[0] } ) );
            return;
        }
        
        for ( size_t i = 0; i < vertices.size() - 1; i++ ) {
            facets.push_back( Facet({ vertices[i], vertices[i + 1] }) );
        }
    }

    // TODO: use the fact that both polygons are convex, so its possible to
    // compute in linear time instead of O(mn)
    void minkowski_sum( const ParetoCurve &rhs ) {
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
     * convex hull called beforehand
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

        Point< value_t > max_x_point = vertices.back();
        Point< value_t > max_y_point = vertices[0];


        // add two line segments from extremal points of the curve 
        Point< value_t > facet_x = { max_x_point[0], reference_point[1] };
        Point< value_t > facet_y = { reference_point[0], max_y_point[1] };
        facets.push_back( Facet({ facet_x, max_x_point }) );
        facets.push_back( Facet({ facet_y, max_y_point }) );
    }

    // precondition: pareto function has been called beforehand, facets are
    // correctly initialized, input point is not dominated by any point on
    // *this pareto curve
    value_t point_distance( const Point< value_t >& point ) const {

        if ( facets.empty() ){
            throw std::runtime_error("Distance from empty pareto curve");
        }

        // handle one dimensional case, only one vertex is possible again
        if ( get_dimension() == 1 ) {
            return point[0] - vertices[0][0];
        }

        Facet first_facet = facets[0];
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
    value_t hausdorff_distance( const ParetoCurve& upper_polygon ) {
        value_t max_distance = 0;

        for ( const auto &v : upper_polygon.get_vertices() ) {
            max_distance = std::max( max_distance, point_distance( v ) );
        }
        return max_distance;
    }


    std::string to_string() const {
        std::stringstream str;

        for ( const auto &vert : vertices ) {
            for ( value_t val : vert ) {
                str << val << " ";
            }
            str << "\n";
        }
        return str.str();
    }

    void write_to_file( std::string filename ) const {
        std::ofstream str( filename, std::ios::out );
        str << to_string();
    }
};
