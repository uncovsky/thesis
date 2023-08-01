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
void multiply( value_t scalar, 
               std::vector< value_t > &vec ) {
    for ( value_t &elem : vec ){
        elem *= scalar;
    }
}

template < typename value_t > 
void add( value_t scalar, 
          std::vector< value_t > &vec ) {
    for ( value_t &elem : vec ){
        elem += scalar;
    }
}

template < typename value_t > 
void multiply( std::vector< value_t > &lhs , 
                                 const std::vector< value_t > &rhs ) {

    assert( lhs.size() == rhs.size() );
    for ( size_t i = 0; i < rhs.size(); i++ ){
        lhs[i] *= rhs[i];
    }
}

/* component-wise division */
template < typename value_t > 
void divide( std::vector< value_t > &lhs , 
             const std::vector< value_t > &rhs ) {
    assert( lhs.size() == rhs.size() );
    for ( size_t i = 0; i < rhs.size(); i++ ){
        lhs[i] /= rhs[i];
    }
}

template < typename value_t > 
void add( std::vector< value_t > &lhs, 
                            const std::vector< value_t > &rhs ) {
    assert ( lhs.size() == rhs.size() );
    for ( size_t i = 0; i < lhs.size(); i++ ) {
        lhs[i] += rhs[i];
    }
}

template < typename value_t > 
void subtract( std::vector< value_t > &lhs, 
                                 const std::vector< value_t > &rhs ) {
    assert ( lhs.size() == rhs.size() );
    for ( size_t i = 0; i < lhs.size(); i++ ) {
        lhs[i] -= rhs[i];
    }

}

template < typename value_t > 
std::vector< value_t > norm( const std::vector< value_t > &lhs ) {

    return multiply( 1/dot_product( lhs, lhs ), lhs );
}


template < typename value_t >
value_t euclidean_distance( const std::vector< value_t > &lhs , 
                     const std::vector< value_t > &rhs) {
    assert( lhs.size() == rhs.size() );

    std::vector< value_t > diff( lhs );
    // get diff
    subtract( diff, rhs );

    return std::sqrt( dot_product( diff, diff ) );
}

