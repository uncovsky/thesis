# pragma once
#include "geometry/polygon.hpp"



/* class used to store upper and lower bounds on the objective value
 * for every state action pair. ( the over/under approximations of the pareto
 * curve )
 *
 * technically these are just pairs of Polygon< T > objects with some
 * additional helper functions ~ like calculating distance, etc.
 */
template < typename value_t > 
class Bounds{

    Polygon< value_t > lower_bound;
    Polygon< value_t > upper_bound;

    bool distance_valid = false;
    value_t distance = 0;

public:

    Bounds() : lower_bound(), upper_bound(){}

    Bounds ( const std::vector< std::vector< value_t > > &lower_pts, 
             const std::vector< std::vector< value_t > >&upper_pts ) : lower_bound( lower_pts ),
                                                                       upper_bound( upper_pts ){}
    Bounds ( const Polygon< value_t > &lower, 
             const Polygon< value_t > &upper ) : lower_bound( lower ),
                                                 upper_bound( upper ){}

    Bounds ( Polygon< value_t > &&lower, 
             Polygon< value_t > &&upper ) : lower_bound( std::move( lower ) ),
                                            upper_bound( std::move( upper ) ){}

    Polygon< value_t > &lower() {
        return lower_bound; 
    }

    Polygon< value_t > &upper() {
        return upper_bound; 
    }

    const Polygon< value_t > &lower() const {
        return lower_bound; 
    }

    const Polygon< value_t > &upper() const {
        return upper_bound; 
    }

    // helper functions for multiplying/adding scalars/vectors to all entries
    void multiply_bounds( value_t mult ) {
        lower_bound.multiply_scalar( mult );
        upper_bound.multiply_scalar( mult );
    }

    void multiply_bounds( const std::vector<value_t> &mult ) {
        lower_bound.multiply_vector( mult );
        upper_bound.multiply_vector( mult );
    }

    void shift_bounds( const std::vector< value_t > &shift ) {
        lower_bound.shift_vector( shift );
        upper_bound.shift_vector( shift );
    }

    void init_facets() {
        lower_bound.init_facets();
    }

    void downward_closure( const Point< value_t > &pt ) {
        lower_bound.downward_closure( pt );
    }

    // input conditions -> this is a state bound set by set_bound() function,
    // i.e. facets and downward closure intiialzied
    value_t bound_distance(){
        if ( !distance_valid ) {
            distance = lower_bound.hausdorff_distance( upper_bound );
            distance_valid = true;
        }
        return distance;
    }

    friend std::ostream &operator<<( std::ostream& os, const Bounds< value_t > &b ) {
        os << "lower bound:\n" << b.lower().to_string() << "\n";
        os << "upper bound:\n" << b.upper().to_string() << "\n";
        return os;
    }
};


