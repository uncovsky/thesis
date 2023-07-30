# pragma once

#include <vector>
#include <set>
#include <algorithm>
#include "utils/geometry_utils.hpp"


/* several simple helper functions for manipulating points, and sets of points
 * when computing pareto curves
 *
 * std::vector is used to storm points in storm
 * as well, could also use an Eigen::Array with component-wise ops built in */

// checks whether point lhs is pareto dominated by rhs

template < typename value_t >
bool is_dominated( const std::vector< value_t > &lhs, 
                   const std::vector< value_t > &rhs ) {
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
bool strictly_non_dominated( const std::vector< value_t > &lhs, 
                             const std::vector< value_t > &rhs ) {
    return !is_dominated( lhs, rhs ) && lhs != rhs;
}


// brute force to remove dominated points from the input set
// should be fine for small num of dimensions
template < typename value_t >
void remove_dominated( std::set< std::vector< value_t > > &input){

    std::vector< std::vector< value_t> > nondominated_elems;

    for ( auto it = input.begin(); it != input.end() ; ) {

        bool dominated = false;

        for ( auto it_dom = input.begin(); it_dom != input.end(); ++it_dom ) {
            if ( is_dominated( *it, *it_dom ) ) {
                dominated = true;
                break;
            }
        }

        if ( dominated ) { it = input.erase(it); }
        else { ++it; }
    }
}

// lhs := ND( union( lhs, rhs ) )
// for update rule in pql
template < typename value_t > 
void nondominated_union( std::set< std::vector< value_t > > &lhs,
                         const std::set< std::vector< value_t > > &rhs ) {

    std::set_union( lhs.cbegin(), lhs.cend() , 
                    rhs.cbegin(), rhs.cend() ,
                    std::inserter( lhs, lhs.begin() ) );
    remove_dominated( lhs );
}


// returns true if element value is dominated by one in given set, or removes
// all vertices dominated by it from input_set
// used in pareto action heuristic
// input condition : input_set does not contain any dominated vectors (i.e
// ND(input_set) = input_set )
template < typename value_t >
bool pareto_check_remove( const std::vector< value_t > &value,
                          std::set< std::vector< value_t > > &input_set ) {

    for ( auto it = input_set.begin(); it  < input_set.end(); ){

        // if value is dominated, then from input condition can't dominate any
        // member of input set, safe to return
        if ( is_dominated( value, *it ) )
            return true;

        if ( is_dominated( *it, value ) ){ it = input_set.remove(it); }
        else  { it++; }
    
    }

    return false;
}
