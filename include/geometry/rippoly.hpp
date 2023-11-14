# pragma once 

# include <algorithm> 
# include <iostream>
# include <fstream>
# include <sstream>
# include <string>

# include "utils/prng.hpp"
# include "utils/geometry_utils.hpp"

template < typename value_t >
class ParetoCurve {

    std::vector< Point< value_t > > vertices;
    std::set< size_t > maximizing_indices;

public:


    ParetoCurve( ) : vertices( ), maximizing_indices( ) {  }

    ParetoCurve( const std::vector< Point< value_t > > &v ) : vertices( v )
                                                            , maximizing_indices ( ) {  }

    ParetoCurve( const std::vector< Point< value_t > > &v,
                 const std::set< size_t > &idxs ) : vertices( v )
                                                     , maximizing_indices ( idxs ) {  }

    const std::vector< Point< value_t > >& get_vertices( ) const {
        return vertices;
    }

    const std::set< size_t >& get_indices( ) const {
        return maximizing_indices;
    }

    std::set< size_t >& get_indices( ) {
        return maximizing_indices;
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

    // TODO: use the fact that both polygons are convex, so its possible to
    // compute in linear time instead of O(mn)
    /*
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
    */


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

        vertices.push_back( facet_x );
        vertices.push_back( facet_y );
    }



    // preconditions ~ convex hull and downward closure called beforand
    // point lies outside this curve
    value_t point_distance( const Point< value_t >& point ) const {

        if ( vertices.size() < 2 ){
            throw std::runtime_error("Distance from empty pareto curve");
        }

        // handle one dimensional case, only one vertex is possible again
        if ( get_dimension() == 1 ) {
            return point[0] - vertices[0][0];
        }

        value_t min_distance( line_segment_distance( vertices[0], vertices[1], point ) );
        for ( size_t i = 1; i < vertices.size() - 1 ; i++ ) {
            min_distance = std::min( min_distance, line_segment_distance( vertices[i], vertices[i+1], point ) );
        }
        return min_distance;
    }

    /* computes hausdorff distance of two polygons, assuming *this is contained
     * in upper entirely and convex hull call preceded this )
     */
    value_t hausdorff_distance( const ParetoCurve& upper ) {
        value_t max_distance = 0;

        for ( const auto &v : upper.get_vertices() ) {
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



