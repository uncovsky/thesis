# pragma once

# include <algorithm>
# include <cctype>
# include <fstream>
# include <memory>
# include <set>
# include <string>
# include <sstream>
# include <vector>
# include "models/mdp.hpp"
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
        return ( line_msg + msg ).c_str();
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

    bool valid_probabilities() const {
        for ( const auto &[ a, prob ] : triplets ) {
            if ( !approx_equal( prob, 1.0 ) )
                return false;
        }
        
        return true;
    }

    std::pair< double, double> get_min_max_value() {
        if ( triplets.empty() )
            return { 0, 0 };
        const auto [ min, max ] = std::minmax_element( triplets.begin(), triplets.end());
        // get associated values
        return std::make_pair( min->second, max->second );
    }

    Matrix2D<double> build_matrix(){
        size_t max_a = 0, max_s = 0;
        std::vector< Eigen::Triplet< double > > eigen_triplets;
        for ( const auto& [ key, val ] : triplets ){

            auto [ a, s ] = key;
            max_a = std::max( max_a, a );
            max_s = std::max( max_s, s );

            eigen_triplets.emplace_back( a, s, val );
        }

        Matrix2D<double> result( max_a + 1, max_s + 1 );
        result.setFromTriplets( eigen_triplets.begin(), eigen_triplets.end() );

        return result;
    }
};


class PrismParser {

    
    // map each state to its transitions, later build matrix
    std::map< size_t, TripletList > transition_info;

    // reward structures ( multiple dimensions )
    // reduced to SxA rewards from SxAxS rewards in the match_reward() function
    // since the transition reward files contain SxAxS
    std::vector< TripletList > reward_info;

    std::vector< std::pair< double, double > > reward_bounds;

    std::map< size_t, size_t> state_to_index;
    std::map< size_t, size_t> action_to_index;

    // line and iterators inside
    std::string line;

    size_t line_num;
    std::string::const_iterator curr, end;
    

    bool eol() const;
    char get_token( );

    size_t translate( const std::string &name, bool state );
    std::string load_number();
    double load_float();

    void require( char token );
    void require( int (* callback )( int ));

    bool check( char token );
    bool check( int (* callback)( int ));

    bool remove_all( int (* callback)( int ) );

    std::tuple< size_t, size_t, size_t > match_triplet();
    void match_reward();
    void match_transition();

    bool ignore_line( const std::string &line );
    size_t string_to_ull( const std::string &input );


public:
    // only transition rewards supported
    void parse_transition_file( const std::string &filename );
    void parse_reward_file( const std::string &filename );
    MDP< double > build_model( size_t initial_state );
};


