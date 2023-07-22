# pragma once 
# include <vector>
# include <set>
# include "utils/geometry_utils.hpp"


/*
 * Class storing information about a pareto curve ( set of vectors for a given
 * state value pair ), along with updates implemented
 *
 *
 */

template < typename value_t >
class ParetoCurve {

    using Point = std::vector< value_t >;

    std::set< Point > vertices;

public:

    ParetoCurve( const std::set< Point > &v ) : vertices( v ) {  }
    ParetoCurve( std::set< Point > &&v ) : vertices( v ) {  }


    std::set< Point > get_vertices( ) const {
        return vertices;
    }

    void scalar_multiply( value_t mult ) {
        for ( Point &p : vertices ) {
            p = multiply( mult, p );
        }    
    }

    void add_curve( const ParetoCurve &rhs ) {
        std::set< Point > new_vertices;
        std::set< Point > rhs_vertices = rhs.get_vertices();
        for ( const auto &v1 : vertices ) {
            for ( const auto &v2 : rhs_vertices ) {
                new_vertices.insert( add( v1, v2 ) );
            }
        }

        remove_dominated( new_vertices );
        vertices = new_vertices;
    }
};
