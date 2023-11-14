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

    ParetoCurve( ) : vertices( ), facets( ) {  }

    ParetoCurve( const std::vector< Point< value_t > > &v ) : vertices( v )
                                                        , facets ( ) {  }

    ParetoCurve( std::vector< Point< value_t > > &&v ) : vertices( std::move( v ) )
                                                   , facets ( ) {  }

    ParetoCurve( const std::vector< Point< value_t > > &v,
             const std::vector< Facet > &f ) : vertices( v )
                                             , facets ( f ) {  }

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


    void minkowski_sum( const std::vector< const ParetoCurve * > &args ) {

        if ( args.empty() ){
            return;
        }

        if ( args[0]->get_dimension() > 2 ) {
            throw std::runtime_error("only 1d/2d operations supported for now");
        }

        if ( args[0]->get_dimension() == 2 ){
            multiple_minkowski_sum( args );
            return;
        }

        // else 1d
        Point< value_t > new_pt = { 0 };
        for ( size_t i = 0; i < args.size(); i++ ){
            // only one point possible
            Point< value_t > other = args[i]->get_vertices()[0];
            new_pt[ 0 ] += other[ 0 ];
        }

        vertices = { new_pt };
        init_facets();

    }

    /* preconditions: all entries in args are correctly initalized, i.e.
     * upper_right_hull() has been called, vertices are sorted and 
     * form a convex polygon that is nonempty
     * args are all also 2d polygons
     * see e.g. : https://cp-algorithms.com/geometry/minkowski.html
     *
     * linear time 2D minkowski sum for the bound update
     *
     */
    void multiple_minkowski_sum( const std::vector< ParetoCurve< value_t > * > &args ){


        std::vector< Point< value_t > > resulting_vertices;

        // indices into vertex array of each polygon 
        std::vector< size_t > offsets( args.size(), 0 );

        // helper lambdas for indexing using both arrays
        auto get_ith_vertex = [ & ] ( size_t polygon_idx, size_t vertex_idx ){
            return args[ polygon_idx ]->get_vertices()[ vertex_idx ];
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

        while ( sum_unfinished() ){
            Point< value_t > next = { 0, 0 };
            for ( size_t i = 0; i < args.size(); i++ ){
                // select current point in polygon i and add it to next vertex
                Point< value_t > added = get_ith_vertex( i, offsets[i] );
                next[0] += added[0];
                next[1] += added[1];
            }

            resulting_vertices.push_back( next );

            size_t next_idx = args.size();
            std::vector< size_t > incremented_indices = {};

            /* investigate the next edge of each polygon, select those edges
             * that correspond to the least polar angle and mark them for the
             * next shift */ 
            for ( size_t i = 0; i < args.size(); i++ ){

                if ( polygon_done( i ) ) { continue; }

                // set to first unfinished polygon 
                if ( next_idx == args.size() ) {
                    next_idx = i;
                    incremented_indices = { i };
                    continue;
                }

                Point< value_t > Pcurr = get_ith_vertex( next_idx, offsets[ next_idx ] );
                Point< value_t > Pnext = get_ith_vertex( next_idx, offsets[ next_idx ] + 1 );

                Point< value_t > Qcurr = get_ith_vertex( i, offsets[i] );
                Point< value_t > Qnext = get_ith_vertex( i, offsets[i] + 1 );

                subtract( Qnext, Qcurr );
                double ccw_val = ccw( Pnext, Pcurr, Qnext );

                if ( approx_zero( ccw_val ) ) { incremented_indices.push_back( i ); }
                else if ( ccw_val < 0 ) { 
                    next_idx = i;
                    incremented_indices = { i };
                }

            }

            // move all vertices corresponding to edges with least polar angles
            for ( size_t idx : incremented_indices ){
                offsets[ idx ]++;
            }
        }

        // add last vertex
        Point< value_t > next = { 0, 0 };
        for ( size_t i = 0; i < args.size(); i++ ){
            next[0] += args[i]->get_vertices().back()[0];
            next[1] += args[i]->get_vertices().back()[1];
        }
        resulting_vertices.push_back( next );

        vertices = std::move( resulting_vertices );
        init_facets();
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

        //TODO: use the fact that convex hull sorts
        auto extreme_points = get_extreme_points( vertices );

        Point< value_t > max_x_point = extreme_points[0].second;
        Point< value_t > max_y_point = extreme_points[1].second;


        // add two line segments from extremal points of the curve 
        Point< value_t > facet_x = { max_x_point[0], reference_point[1] };
        Point< value_t > facet_y = { reference_point[0], max_y_point[1] };
        facets.push_back( Facet({ facet_x, max_x_point }) );
        facets.push_back( Facet({ facet_y, max_y_point }) );
    }


    // more efficient + automatically remove dominated solutions
    // eps determines the granularity of the hull
    void upper_right_hull( double eps ){
        if ( vertices.empty() )
            return;
        if ( get_dimension() > 2 ) {
            std::cout << "Higher dimension convex hulls are currently unsupported." << std::endl;
            assert( false );
        }

        // keep only max element
        if ( get_dimension() == 1 ) {
            vertices = { *std::max_element( vertices.begin(), vertices.end() ) };
            init_facets();
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
                /* if last vertex of teh hulls lies in CW direction from
                 * pt->hull[i-1], remove the last element of the hull
                 * repeat
                 */
                while ( ( hull.size() >= 2 ) && 
                        ( ccw( pt, hull[i - 1], hull[i] ) <= eps / 100 ) ) {
                    hull.pop_back();
                    i--;
                }
                hull.push_back( pt );
            }
        }

        vertices = std::move( hull );
        init_facets();
    }

    void pareto( const Point< value_t >& ref_point, double prec ){
        upper_right_hull( prec );
        downward_closure( ref_point );
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
