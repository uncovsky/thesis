#pragma once

#include <cmath>
#include <vector> 
#include <set>


/*
 * helper fucntions for linear algebra and miscellaneous component-wise
 * operations ( implemented with vectors for later Storm porting ) 
 */


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
                                 const std::vector< value_t > &vec ) {
    std::vector< value_t > res( vec ); 
    for ( value_t &elem : res ){
        elem *= scalar;
    }

    return res;
}

template < typename value_t > 
std::vector< value_t > add( value_t scalar, 
                                 const std::vector< value_t > &vec ) {
    std::vector< value_t > res( vec ); 
    for ( value_t &elem : res ){
        elem += scalar;
    }

    return res;
}

/* component-wise multiplication */
template < typename value_t > 
std::vector< value_t > multiply( const std::vector< value_t > &lhs , 
                                 const std::vector< value_t > &rhs ) {

    assert( lhs.size() == rhs.size() );
    std::vector< value_t > res( lhs ); 
    for ( size_t i = 0; i < rhs.size(); i++ ){
        res[i] *= rhs[i];
    }
    return res;
}

/* component-wise division */
template < typename value_t > 
std::vector< value_t > divide( const std::vector< value_t > &lhs , 
                                 const std::vector< value_t > &rhs ) {

    assert( lhs.size() == rhs.size() );
    std::vector< value_t > res( lhs ); 
    for ( size_t i = 0; i < rhs.size(); i++ ){
        res[i] /= rhs[i];
    }
    return res;
}

template < typename value_t > 
std::vector< value_t > add( const std::vector< value_t > &lhs, 
                            const std::vector< value_t > &rhs ) {
    assert ( lhs.size() == rhs.size() );
    std::vector< value_t > res( lhs ); 

    for ( size_t i = 0; i < lhs.size(); i++ ) {
        res[i] += rhs[i];
    }

    return res;
}

template < typename value_t > 
std::vector< value_t > subtract( const std::vector< value_t > &lhs, 
                                 const std::vector< value_t > &rhs ) {
    assert ( lhs.size() == rhs.size() );
    std::vector< value_t > res( lhs ); 

    for ( size_t i = 0; i < lhs.size(); i++ ) {
        res[i] -= rhs[i];
    }

    return res;
}

template < typename value_t > 
std::vector< value_t > norm( const std::vector< value_t > &lhs ) {

    return multiply( 1/dot_product( lhs, lhs ), lhs );
}


template < typename value_t >
value_t euclidean_distance( const std::vector< value_t > &lhs , 
                     const std::vector< value_t > &rhs) {
    assert( lhs.size() == rhs.size() );

    std::vector< value_t > diff = subtract( lhs, rhs );

    return std::sqrt( dot_product( diff, diff ) );
}

