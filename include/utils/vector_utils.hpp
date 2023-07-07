#pragma once

#include <vector> 



template < typename value_t >
value_t dot_product( const std::vector< value_t > &lhs , 
                     const std::vector< value_t > &rhs) {
    assert( lhs.size() == rhs.size() );
    value_t result(0);
    for ( size_t i = 0; i < rhs.size(); i++ ){
        result += lhs[ i ] * rhs[ i ];
    }

    return result;
}

