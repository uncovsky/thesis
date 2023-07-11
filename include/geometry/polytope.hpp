#include <vector>
#include <set>
#include <algorithm>
#include "utils/geometry_utils.hpp"



/* represents a half space determined by a normal vector of a hyperplane + a
 * max dot product from it (distance_diff) */
template < typename value_t > 
class Halfspace {

    // normal vector of the hyperplane
    std::vector < value_t > normal_vec;

    // max dot product with 
    value_t max_distance; 

public:
    Halfspace( const std::vector< value_t > &normal_vec, 
               value_t distance) : 
            normal_vec(normal_vec), max_distance(max_distance) {}

    
    value_t distance( const std::vector< value_t > &point ) const {
        value_t dist = dot_product( point, normal_vec ) / norm(normal_vec);
        return std::max(0, dist - max_distance);
    }

    bool on_boundary( const std::vector< value_t > &point ) const{
        return dot_product( point, normal_vec ) == max_distance;
    } 
};


/* represents a convex d-dimensional polytope as an intersection of 
 * given half spaces */ 

template < typename value_t > 
class Polytope {

    typedef std::vector< value_t > Point;

    // halfspaces that form the polytope
    std::vector< Halfspace< value_t > > halfspaces;
    
    // vertices of the polytope (intersections of the halfspaces)
    std::vector< Point > points;
};


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
    return !is_dominated(lhs, rhs) && lhs != rhs;
}


// brute force to remove dominated points from the input set
// todo a bit smarter
template < typename value_t >
void remove_dominated( std::set< std::vector< value_t > > &input){

    std::vector< std::vector< value_t> > nondominated_elems;

    for (auto it = input.begin(); it != input.end();){

        bool dominated = false;

        for (auto it_dom = input.begin(); it_dom != input.end(); ++it_dom){
            if (is_dominated(*it, *it_dom)){
                dominated = true;
                break;
            }
        }

        if (dominated) { it = input.erase(it); }
        else { ++it; }
    }
}

// ND(union of lhs and rhs)
// for update rule in pql
template < typename value_t > 
void nondominated_union( std::set< std::vector< value_t > > &lhs,
                                                       const std::set< std::vector< value_t > > &rhs ) {

    std::set_union(lhs.cbegin(), lhs.cend(), 
                   rhs.cbegin(), rhs.cend(),
                   std::inserter(lhs, lhs.begin()));
    remove_dominated(lhs);
}
