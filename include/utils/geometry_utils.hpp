#pragma once

#include <vector> 
#include <set>


template < typename value_t >
value_t dot_product( const std::vector< value_t > &lhs , 
                     const std::vector< value_t > &rhs) {
    assert( lhs.size() == rhs.size() );
    value_t result(0);
    for ( size_t i = 0; i < rhs.size(); i++ ){
        result += lhs[i] * rhs[i];
    }

    return result;
}


template < typename value_t > 
std::vector< value_t > multiply( value_t scalar, 
                                 const std::vector< value_t > vec ) {
    std::vector< value_t > res( vec ); 
    for ( value_t &elem : res ){
        elem *= scalar;
    }

    return res;
}

template < typename value_t > 
std::vector< value_t > add( const std::vector< value_t > lhs, 
                            const std::vector< value_t > rhs ) {
    assert ( lhs.size() == rhs.size() );
    std::vector< value_t > res( lhs ); 

    for ( size_t i = 0; i < lhs.size(); i++ ) {
        res[i] += rhs[i];
    }

    return res;
}

template < typename value_t > 
std::vector< value_t > norm( const std::vector< value_t > lhs ) {

    return multiply( 1/dot( lhs, lhs ), lhs );
}
