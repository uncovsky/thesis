#include <vector>
#include <set>
#include <algorithm>
#include "utils/geometry_utils.hpp"



/* represents a half space determined by a normal vector of a hyperplane + a
 * distance from it (in the direction of the normal vector) */ 
template < typename value_t > 
class Halfspace {

    // normal vector of the hyperplane
    std::vector < value_t > normal_vec;

    // euclidean distance from the hyperplane
    value_t distance_diff; 

public:
    Halfspace( const std::vector< value_t > &normal_vec, 
               value_t distance) : 
            normal_vec(normal_vec), distance_diff(distance_diff) {}

    
    value_t distance( const std::vector< value_t > &point ) const {
        value_t dist = dot_product( point, normal_vec ) / norm(normal_vec);
        return std::max(0, dist - distance_diff);
    }

    bool on_frontier( const std::vector< value_t > &point ) const{
        return dot_product( point, normal_vec ) == distance_diff;
    } 
};


/* represents a d-dimensional polytope as an intersection of 
 * given half spaces */ 

template < typename value_t > 
class Polytope {

    // halfspaces that form the polytope
    std::vector< Halfspace< value_t > > halfspaces;
    
    // vertices of the polytope (intersections of the halfspaces)
    std::vector< value_t > points;
};


// checks whether lhs is strictly non dominated by vector rhs
// if not true we don't want to insert into the set
template < typename value_t >
bool strictly_non_dominated( const std::vector< value_t > &lhs, 
                             const std::vector< value_t > &rhs ) {
    bool not_dominated = true;
    bool neq = true;
    for ( size_t i = 0; i < lhs.size(); i++ ) {
        not_dominated &= lhs[i] >= rhs[i];
        neq &= lhs[i] != rhs[i];
    }

    return not_dominated & neq;
}

template < typename value_t > 
std::set< std::vector< value_t > > nondominated_union( const std::set< std::vector< value_t > > &lhs,
                                                       const std::set< std::vector< value_t > > &rhs ) {
    std::set< std::vector< value_t > > res;

    std::set_union(lhs.cbegin(), lhs.cen(), 
                   rhs.cbegin(), rhs.cend(),
                   std::inserter(res, res.begin()),
                   [](const auto &x, const auto &y){ return !strictly_not_dominated(x, y); });

    return res;
}
