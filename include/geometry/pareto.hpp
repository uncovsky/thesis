# pragma once

#include <algorithm>
#include <iostream>
#include <set>
#include <vector>
#include "utils/eigen_types.hpp"
#include "utils/geometry_utils.hpp"


/* several simple helper functions for manipulating points, and sets of points
 * when computing pareto curves
 *
 * std::vector is used to storm points in storm
 * as well, could also use an Eigen::Array with component-wise ops built in */

// checks whether point lhs is pareto dominated by rhs
template < typename value_t >
bool is_dominated( const Point< value_t > &lhs, 
                   const Point< value_t > &rhs ) {
    bool dominates_i = false;
    bool dominated_i = false;
    for (size_t i = 0; i < lhs.size(); i++){
        if (rhs[i] < lhs[i]) 
            dominates_i = true;
        else if (lhs[i] < rhs[i])
            dominated_i = true;
    }

    return dominated_i && !dominates_i;
}


template < typename value_t >
bool strictly_non_dominated( const Point< value_t > &lhs, 
                             const Point< value_t > &rhs ) {
    return !is_dominated( lhs, rhs ) && lhs != rhs;
}

// brute force to remove dominated points from the input set
// should be fine for small num of dimensions
template < typename value_t >
void remove_dominated( std::vector< Point< value_t > > &input){

    std::vector< std::vector< value_t> > nondominated_elems;

    for ( size_t i = 0; i < input.size(); i++ ) {
        bool dominated = false;

        for ( size_t it = i + 1; it < input.size(); it++ ) {
            if ( is_dominated( input[i], input[it] ) ) {
                dominated = true;
                break;
            }
        }

        if ( !dominated )
            nondominated_elems.push_back( input[i] );
        }

    input = nondominated_elems;
}


// simple divide & conquer algorithm to remove nondominated vertices
template< typename value_t >
std::set< size_t > remove_dominated_rec( const std::vector< Point< value_t > > &input,
                                                  size_t start,
                                                  size_t end ) {
    if ( start == end ) {
        return std::set( { start } );
    }

    size_t mid = start + ( end - start ) / 2; 

    // split into two and get nondominated
    std::set< size_t > l = remove_dominated_rec( input, start, mid );
    std::set< size_t > r = remove_dominated_rec( input, mid + 1, end );


    // check for dominated elements across both parts
    for ( auto it = l.begin(); it != l.end();  ) {
        bool dominated = false;
        for ( auto r_it = r.begin(); r_it != r.end(); ) {
            if ( is_dominated( input[*it], input[*r_it] ) ) { dominated = true; break; }
            if ( is_dominated( input[*r_it], input[*it] ) || ( input[*r_it] == input[*it] ) ) { r_it = r.erase( r_it ); }
            else { r_it++; }
        }

        if ( dominated ) { it = l.erase( it ); }
        else{ it++; }
    }
 
    std::set_union( l.begin(), l.end(),
                    r.cbegin(), r.cend(),
                    std::inserter( l, l.begin() ));
    return l;
}


template< typename value_t >
void remove_dominated_alt( std::vector< Point< value_t > > &input ){

    if ( input.empty() )
        return;
    std::set< size_t > nondom_indices = remove_dominated_rec( input, 0, input.size() - 1 );

    std::vector< Point< value_t > > nondom_points;
    for ( size_t i : nondom_indices )
        nondom_points.emplace_back( input[i] );

    input = std::move( nondom_points );
}

template < typename value_t > 
bool dominates_set( const Point< value_t > &point,
                    const std::vector< Point< value_t > > &candidates ) {
    
    return std::all_of( candidates.begin(), candidates.end(),
                        [&] ( auto candidate_pt ) { return is_dominated( candidate_pt, point ); });
}


/* precondition
 *  ~ vertices sorted by descending x value and dominated pts removed
 *  in our case only called after pareto() operator has run
 *  and vertices sorted using Polygon::upper_right_hull()
 */
template< typename value_t >
value_t hypervolume_indicator( const std::vector< Point< value_t > > &vertices,
                               const Point< value_t > &ref_point ) {

    if ( ref_point.size() > 2 ){
        throw std::runtime_error("Only 1D/2D hypervolume supported.");
    }
    if ( ref_point.size() == 1 ) {
        return vertices[0][0] - ref_point[0];
    }

    Point< value_t > copy( ref_point );
    value_t res( 0 );

    // calculate disjoint areas
    for ( const auto &pt : vertices ) {
        res += ( pt[0] - ref_point[0] ) * ( pt[1] - copy[1] );
        copy = pt;
    }

    return res;
}
