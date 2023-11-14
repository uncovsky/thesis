
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
                 const std::vector< size_t > &idxs ) : vertices( v )
                                                     , maximizing_indices ( idxs ) {  }

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

    value_t point_distance( const Point< value_t >& y ) const {
    
        if ( points.size() != 2 ) {
            std::cout << "error: only two dimensional polygons are currently supported.\n";
            throw std::runtime_error("invalid distance operation.");
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
            min_distance = min( min_distance, line_segment_distance( vertices[i], vertices[i+1], point ) );
        }
        return min_distance;
    }

    /* computes hausdorff distance of two polygons, assuming *this is contained
     * in upper entirely and convex hull call preceded this )
     */
    value_t hausdorff_distance( const ParetoCurve& upper ) {
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


template< typename value_t >
ParetoCurve< value_t > minkowski_sum( const std::vector< ParetoCurve< value_t > * > &curves,
                                               const std::vector< double > &weights ){

        std::vector< Point< value_t > > resulting_vertices;

        // indices into vertex array of each polygon 
        std::vector< size_t > offsets( args.size(), 0 );

        // helper lambdas for indexing using both arrays
        auto get_ith_vertex = [ & ] ( size_t polygon_idx, size_t vertex_idx ){
            Point< value_t > pt = args[ polygon_idx ]->get_vertex( vertex_idx );
            multiply( weights[ polygon_idx ], pt );
            return pt;
        };

        auto polygon_done = [ & ]( size_t polygon_idx ){
            return offsets[ polygon_idx ] == args[ polygon_idx ]->size() - 1; 
        };

        auto sum_unfinished = [ & ](){
            for ( size_t i = 0; i < offsets.size(); i++ ) {
                if ( !polygon_done( i ) ) { return true; }
            }
            return false;
        };

        unfinished = false;

        while ( unfinished ){

            // check here, because need to add last vertex after all offsets reach
            // final index
            unfinished = sum_unfinished();
            Point< value_t > next = { 0, 0 };
            for ( size_t i = 0; i < args.size(); i++ ){
                // select current point in polygon i and add it to next vertex
                Point< value_t > added = get_ith_vertex( i, offsets[i] );
                next[0] += added[0];
                next[1] += added[1];
            }

            resulting_vertices.push_back( next );

            // track all edges with minimal polar angle, to remove colinear
            // points
            std::vector< size_t > incremented_indices = {};
            value_t min_dy( -1 );

            /* investigate the next edge of each polygon, select those edges
             * that correspond to the least polar angle and mark them for the
             * next shift */ 
            for ( size_t i = 0; i < args.size(); i++ ){

                if ( polygon_done( i ) ) { continue; }

                Point< value_t > Pcurr = get_ith_vertex( i, offsets[i] );
                Point< value_t > Pnext = get_ith_vertex( i, offsets[i] + 1 );

                value_t dy = ( Qcurr[1] - Qnext[1] ) / ( Qnext[0] - Qcurr[1] );

                if ( ( min_dy == -1 ) || ( dy < min_dy ) ){
                    min_dy = dy;
                    incremented_indices = { i };
                }

                else if ( dy == min_dy ) {
                    incremented_indices.push_back( i );
                }

            }
            // move all vertices corresponding to edges with least polar angles
            for ( size_t idx : incremented_indices ){
                offsets[ idx ]++;
            }
        }

    return ParetoCurve( resulting_vertices );
}

    void upper_right_hull( double eps ){

