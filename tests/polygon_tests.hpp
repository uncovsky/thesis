# pragma once

# include "geometry/polygon.hpp"
# include "models/env_wrapper.hpp"
# include "utils/geometry_utils.hpp"
# include <cassert>


void test_euclidean_distance(){
    assert( euclidean_distance< double > ( { 0, 0 }, { 0, 0 } )  == 0 );
    assert( euclidean_distance< double > ( { 0, 10 }, { 0, 0 } )  == 10 );
    assert( euclidean_distance< double > ( { 12, 4 }, { 0, 0 } )  == std::sqrt( 160 ) );
    assert( euclidean_distance< double > ( { 12, 4 }, { 2, 2 } )  == std::sqrt( 104 ) );
    assert( euclidean_distance< double > ( { 1, 5 }, { -10, 14 } )  == std::sqrt( 202 ) );
}

void test_minkowski_sum(){

    Polygon< double > poly( { { 1.0, 0.0 }, { -1.0, 0.0 }, { 0.0, 2.0 } });
    Polygon< double > poly2( { { 0.0, -1.0 }, { 1.0, 1.0 }, { -1.0, 1.0 } });

    Polygon< double > res;
    res.minkowski_sum( { &poly, &poly2 } );

    poly.minkowski_sum( poly2 );
    poly.upper_right_hull( 0.0 );

    std::cout << res.to_string();
    std::cout << poly.to_string();

}
