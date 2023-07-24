# pragma once
# include "geometry/pareto_curve.hpp"


template< typename value_t > 
class Bounds{
    
    ParetoCurve< value_t > lower_bound;
    ParetoCurve< value_t > upper_bound;

};
