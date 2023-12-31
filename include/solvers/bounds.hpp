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

    bool hausdorff_valid = false;
    value_t hausdorff_dist = 0;

    std::vector< Point< value_t > > furthest_points = {};

public:

    Bounds() : lower_bound(), upper_bound(){}

    Bounds( const Bounds< value_t > &other ) : lower_bound( other.lower() )
                                             , upper_bound( other.upper() )
                                             , hausdorff_valid( other.is_hausdorff_valid() ){}

    Bounds ( const std::vector< std::vector< value_t > > &lower_pts, 
             const std::vector< std::vector< value_t > >&upper_pts ) : lower_bound( lower_pts ),
                                                                       upper_bound( upper_pts ){}
    Bounds ( const Polygon< value_t > &lower, 
             const Polygon< value_t > &upper ) : lower_bound( lower ),
                                                 upper_bound( upper ){}

    Bounds ( Polygon< value_t > &&lower, 
             Polygon< value_t > &&upper ) : lower_bound( std::move( lower ) ),
                                            upper_bound( std::move( upper ) ){}

    /* access to underlying polygons / curves */
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

    /* helper functions that execute the same operation on the relevant
     * polygon / polygons */ 
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

    value_t hypervolume( const Point< value_t > &ref_point ) const {
        return upper_bound.hypervolume( ref_point );
    }
    /* 
     * methods for distance calculations / caching them 
     */ 
    bool is_hausdorff_valid() const {
        return hausdorff_valid;
    }

    // input conditions -> this is a state bound set by set_bound() function,
    // i.e. facets and downward closure intiialzied
    value_t hausdorff_distance(){
        if ( !hausdorff_valid ) {
            auto res = lower_bound.hausdorff_distance( upper_bound );
            
            hausdorff_dist = res.first;
            furthest_points = res.second;
            hausdorff_valid = true;
        }
        return hausdorff_dist;
    }
    
    std::vector< Point< value_t > > get_furthest_points() {
        if ( !hausdorff_valid ) {
            hausdorff_distance();
        }
        return furthest_points;
    }

    /* 
     * output to stream
     */
    friend std::ostream &operator<<( std::ostream& os, const Bounds< value_t > &b ) {
        os << "lower bound:\n" << b.lower().to_string() << "\n";
        os << "upper bound:\n" << b.upper().to_string() << "\n";
        return os;
    }
};


