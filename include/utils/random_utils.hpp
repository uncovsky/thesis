#pragma once
#include <cmath>


template < typename prob_t >
bool approx_equal(prob_t x, prob_t y, prob_t eps=1e-7){
    return std::abs(x - y) < eps;
}

template < typename prob_t >
bool approx_zero(prob_t x, prob_t eps=1e-7){
    return approx_equal(x, 0.0);
}
