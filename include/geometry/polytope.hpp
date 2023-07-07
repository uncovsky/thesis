#include <vector>
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
