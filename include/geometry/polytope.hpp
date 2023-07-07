#include <vector>



/* represents a half space determined by a normal vector of a hyperplane + a
 * distance from it (in the direction of the normal vector) */ 
template < typename value_t > 
class Halfspace {

    // normal vector of the hyperplane
    std::vector < value_t > normal_vec;

    // how far away from the hyperplane does the space start
    value_t distance_diff; 

public:
    Halfspace(std::vector< value_t > normal_vec, value_t distance, bool positive_side=true) : 
            normal_vec(normal_vec), distance_diff(distance_diff) {}

    
    value_t distance( std::vector< value_t > point ) const {
          
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
