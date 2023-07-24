# include "geometry/polygon.hpp"
# include "geometry/pareto.hpp"

/*
 * two dimensional pareto curve object that will be stored for every
 * state-action pair in the solver
 */

template< typename value_t > 
class ParetoCurve {

    using Point = std::vector< value_t >;

    Polygon< value_t > curve;

    // remove dominated points from the underlying polygon
    void remove_dominated() {
        std::vector< Point > &vertices = curve.get_vertices();
        remove_dominated( vertices );
    }

public:

    ParetoCurve() : curve() {}

    ParetoCurve( const std::vector< Point > &vertices ) : curve( vertices ) {}
    ParetoCurve( std::vector< Point > &&vertices ) : curve( std::move(vertices ) ) {}
    
    Polygon< value_t > &get_curve() const {
        return curve;
    }


    void add_curve( const ParetoCurve &rhs ) {
        curve.minkowski_sum( rhs.get_curve() );
        remove_dominated();
    }

    void add_scalar( value_t scalar ) {
        curve.shift_scalar( scalar );
    }

    void multiply_scalar( value_t scalar ) {
        curve.multiply_scalar( scalar );
    }

    void nondominated_union( const ParetoCurve &rhs ) {
        std::vector< Point > &vertices = curve.get_vertices();
        nondominated_union( vertices, rhs.get_curve().get_vertices() );
    }

    void pareto_op( const Point &ref_point ){
        curve.convex_hull();
        curve.downward_closure( ref_point );
    }

};
