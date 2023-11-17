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
class Polygon {

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
            value_t norm = dot_product( line, line ) + 0.000001;
            value_t coeff = std::clamp( dot_product( delta, line ) / norm
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
        }
            
    };

    std::vector< Point< value_t > > vertices;
    std::vector< Facet > facets;

public:

    Polygon( ) : vertices( ), facets( ) {  }

    Polygon( const std::vector< Point< value_t > > &v ) : vertices( v )
                                                        , facets ( ) {  }

    Polygon( std::vector< Point< value_t > > &&v ) : vertices( std::move( v ) )
                                                   , facets ( ) {  }

    Polygon( const std::vector< Point< value_t > > &v,
             const std::vector< Facet > &f ) : vertices( v )
                                             , facets ( f ) {  }

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
    void minkowski_sum( const Polygon &rhs, value_t weight ) {
        std::vector< Point< value_t > > new_vertices;
        const std::vector< Point< value_t > > &rhs_vertices = rhs.get_vertices();

        if ( rhs_vertices.empty() ) { 
            return; 
        }
        else if ( vertices.empty() ) { 
            for ( const auto &v2 : rhs_vertices ) {
                Point< value_t > v2_copy( v2 );
                multiply( weight, v2_copy );
                new_vertices.emplace_back( v2_copy );
            }
            vertices = std::move(new_vertices);
            return;
        }
        for ( const auto &v1 : vertices ) {
            for ( const auto &v2 : rhs_vertices ) {
                Point< value_t > v1_copy( v1 );
                Point< value_t > v2_copy( v2 );
                multiply( weight, v2_copy );
                add( v1_copy, v2_copy  );
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
    value_t hausdorff_distance( const Polygon& upper_polygon ) {
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
std::vector< Point< value_t > > upper_right_hull( std::vector< Point< value_t > > &vertices, double eps ){
    if ( vertices.empty() )
        return {};
    if ( vertices[0].size() > 2 ) {
        std::cout << "Higher dimension convex hulls are currently unsupported." << std::endl;
        assert( false );
    }

    // keep only max element
    if ( vertices[0].size() == 1 ) {
        return { *std::max_element( vertices.begin(), vertices.end() ) };
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
            while ( ( hull.size() >= 2 ) && ( ccw( pt, hull[i - 1], hull[i] ) <= eps ) ){
                hull.pop_back();
                i--;
            }
            hull.push_back( pt );
        }
    }

    return hull;
}



template< typename value_t > 
Polygon< value_t > hull_union( std::vector< Polygon< value_t > * > curves,
                 double eps ){
    std::vector< Point< value_t > > vertices;
    for ( auto ptr : curves ){
        auto& curve_vrt = ptr->get_vertices();
        std::copy( curve_vrt.begin(), curve_vrt.end(), std::back_inserter( vertices ) ) ;
    }

    std::vector< Point< value_t > > hull_vertices = upper_right_hull( vertices, eps );
    Polygon< value_t > result( hull_vertices );
    return result;
}


template< typename value_t >
Polygon< value_t > naive_minkowski_sum( const std::vector< Polygon< value_t > * > &args,
                                        const std::vector< double > &probs ) {

    Polygon< value_t > result;
    for ( size_t i = 0; i < args.size(); i++ ) {
        result.minkowski_sum( *args[i], probs[i] );
    }

    auto vertices = upper_right_hull( result.get_vertices(), 0 );
    return Polygon< value_t > ( std::move( vertices ) );
}




/* update operations */
template < typename value_t >
Polygon< value_t > weighted_minkowski_sum( const std::vector< Polygon< value_t > * > &args,
                             const std::vector< double > &probs ) {
    if ( args.empty() ){
        throw std::runtime_error("empty curve operation - minkowski sum");
    }

    if ( args[0]->get_dimension() > 2 ) {
        throw std::runtime_error("only 1d/2d operations supported for now");
    }

    if ( args[0]->get_dimension() == 2 ){
        return multiple_minkowski_sum( args, probs );
    }

    // else 1d
    Point< value_t > new_pt = { 0 };
    for ( size_t i = 0; i < args.size(); i++ ){
        // only one point possible
        Point< value_t > other = args[i]->get_vertex(0);
        multiply( probs[i], other );
        new_pt[ 0 ] += other[ 0 ];
    }

    return Polygon< value_t > ( { new_pt } );
}


/* preconditions: all entries in args are correctly initalized, i.e.
 * contain convex pareto curves that are sorted lexicographically
 *
 *
 * linear time 2D minkowski sum for the bound update
 */
template< typename value_t >
Polygon< value_t > multiple_minkowski_sum( const std::vector< Polygon< value_t > * > &curves,
                                           const std::vector< double > &probs ){

    std::vector< Point< value_t > > resulting_vertices;

    // indices into vertex array of each polygon 
    std::vector< size_t > offsets( curves.size(), 0 );

    // helper lambdas for indexing using both arrays
    auto get_ith_vertex = [ & ] ( size_t polygon_idx, size_t vertex_idx ){
        auto pt = curves[ polygon_idx ]->get_vertex( vertex_idx );
        multiply( probs[polygon_idx], pt );
        return pt;
    };

    // whether this polygon has an edge that was not added yet
    auto polygon_done = [ & ]( size_t polygon_idx ){
        return offsets[ polygon_idx ] == curves[ polygon_idx ]->size() - 1; 
    };

    auto sum_unfinished = [ & ](){
        for ( size_t i = 0; i < offsets.size(); i++ ) {
            if ( !polygon_done( i ) ) { return true; }
        }
        return false;
    };

    bool unfinished = true;

    while ( unfinished ){

        // check here, because need to add last vertex after all offsets reach
        // final index
        unfinished = sum_unfinished();
        Point< value_t > next = { 0, 0 };
        for ( size_t i = 0; i < curves.size(); i++ ){
            // select current point in polygon i and add it to next vertex
            Point< value_t > added = get_ith_vertex( i, offsets[i] );
            next[0] += added[0];
            next[1] += added[1];
        }

        resulting_vertices.push_back( next );

        // track all edges with minimal polar angle, to remove colinear
        // points
        std::vector< size_t > incremented_indices = {};
        value_t max_dy( -1 );

        /* investigate the next edge of each polygon, select those edges
         * that correspond to the least polar angle change and move them
         * next shift */ 
        for ( size_t i = 0; i < curves.size(); i++ ){

            // if no more edges
            if ( polygon_done( i ) ) { continue; }

            // investigate polar angle of this edge
            Point< value_t > Pcurr = get_ith_vertex( i, offsets[i] );
            Point< value_t > Pnext = get_ith_vertex( i, offsets[i] + 1 );
            value_t dy = ( Pcurr[1] - Pnext[1] ) / ( Pnext[0] - Pcurr[0] );

            // if maximal => least change, since the curves are ordered 
            // with decreasing x after convex hull calls
            if ( ( max_dy == -1 ) || ( dy > max_dy ) ){
                max_dy = dy;
                incremented_indices = { i };
            }

            // if multiple edges have minimal, we will move them all
            else if ( dy == max_dy ) {
                incremented_indices.push_back( i );
            }

        }
        // move all vertices corresponding to edges with least polar angles
        for ( size_t idx : incremented_indices ){
            offsets[ idx ]++;
        }
    }
    return Polygon< value_t >( resulting_vertices );
}
