#pragma once
#include <cmath>

template < typename prob_t >
bool approx_zero(prob_t x, prob_t eps=1e-7){
    return std::abs(x - 0) < eps;
}
