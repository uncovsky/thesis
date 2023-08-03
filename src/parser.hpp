# pragma once

# include <cctype>
# include <istream>
# include <memory>
# include <set>
# include <string>
# include <sstream>
# include <vector>
# include "utils/eigen_types.hpp"
# include "utils/random_utils.hpp"

/*
 * parser for explicit model files ( mdps )
 */

struct ParsingError {

    size_t line_num;
    std::string msg;

    const char *what() {
        std::string line_msg = "Error on line " + std::to_string( line_num ) + " - ";
        return ( std::to_string(line_num) + msg ).c_str();
    }

    ParsingError( size_t line_num, const std::string &msg ) : line_num( line_num ),
                                                              msg( msg ) {}
};

// for each state keep a triplet list
struct TripletList { 

    // map ( S x ) A -> double to ensure probabilities sum to approx 1
    std::map< size_t , double > prob_sums;

    std::map< std::pair< size_t, size_t > , double > triplets;

    void add_triplet( size_t a, size_t s, double prob ) {

        prob_sums[a] += prob;
        auto idx = std::make_pair( a, s );

        if ( contains( a, s ) ) {
            throw ParsingError( 0, "Duplicate transition" );
        }

        triplets[ idx ] = prob;
    }

    bool contains( size_t a, size_t s ) {
        auto idx = std::make_pair( a, s );
        return triplets.find( idx ) != triplets.end() ;
    }

    bool valid_probabilities() {
        for ( const auto &[ a, prob ] : triplets ) {
            if ( !approx_equal( prob, 1.0 ) )
                return false;
        }
        
        return true;
    }

    Matrix2D<double> build_matrix(){
        //TODO
        return Matrix2D<double>();
    }
};


class PrismParser {

    
    // map each state to its transitions, later build matrix
    std::map< size_t, TripletList > transition_info;
    // all rewards ( dimensions )
    std::vector< std::map< size_t, TripletList > > reward_structures;

    std::map< size_t, size_t> state_to_index;
    std::map< size_t, size_t> action_to_index;

    // line and iterators inside
    std::string line;

    size_t line_number;
    std::string::const_iterator curr, end;
    

    bool eol() const;
    char get_token( );

    size_t translate( bool state );
    std::string load_number();
    double load_float();

    void require( char token );
    void require( int (* callback )( char ));

    bool check( char token );
    bool check( int (* callback)( char ));

    bool remove_all( int (* callback)( char ) );
    std::tuple< size_t, size_t, size_t > match_triplet();
    void match_transition();

public:
    // only transition rewards supported
    void parse_transition_file( const std::string &filename );
    void parse_reward_file( const std::string &filename );
};

size_t string_to_ull( const std::string input ) {
    std::istringstream iss(input);
    size_t size;
    iss >> size;

    return size;
}

