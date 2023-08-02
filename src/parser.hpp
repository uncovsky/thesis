# pragma once

# include <memory>
# include <set>
# include <string>
# include <vector>
# include "utils/eigen_types.hpp"

/*
 * parser for explicit model files ( mdps )
 */


class PrismParser {

    // action - successor - probability
    using asp_triplet = std::tuple< size_t, size_t, double > ;
    std::map< size_t, asp_triplet > state_records;

    std::string filename, line;

    int shift_next( const std::string token )
    int require( const std::string token );
    int check( const std::string token );


}
