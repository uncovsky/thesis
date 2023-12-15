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
# include "utils/prng.hpp"

/*
 * Parser for the prism explicit MDP format
 * See: https://www.prismmodelchecker.org/manual/Appendices/ExplicitModelFiles
 *
 * Additionally, multiple reward dimensions in one reward file are supported,
 * so the format currently is:
 *  
 *  source_state action succ_state rew1 rew2 rew3 ... 
 *  
 * all lines don't have to have the same number of dimensions, but the largest
 * number of reward dimensions on some line in the file is taken as the
 * dimension of reward in the resulting model, and zeroes are added to all
 * missing entries throughout the file.
 *
 * It is also possible to load multiple reward files as multiple dimensions of
 * the resulting model reward.
 * 
 * Constructs an MDP model given transition files ( .tra ) and potentially
 * multiple transition reward files ( .trew ).
 *
 */


/* struct thrown as exception when parsing errors occur */
struct ParseError {

    size_t line_num;
    std::string msg;

    const char *what() const {
        std::string line_msg = "Error on line " + std::to_string( line_num ) + " - ";
        std::cout << line_msg + msg << std::endl;
        return ( line_msg + msg ).c_str();
    }

    ParseError( size_t line_num, const std::string &msg ) : line_num( line_num ),
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
            throw ParseError( 1, "Duplicate transition" );
        }

        triplets[ idx ] = prob;
    }

    // query whether the underlying map contains a given (a,s) tuple
    bool contains( size_t a, size_t s ) {
        auto idx = std::make_pair( a, s );
        return triplets.find( idx ) != triplets.end() ;
    }

    // check whether all the probability sums over successors ( for each action )
    // are approximately equal to 1
    bool valid_probabilities() const {
        for ( const auto &[ a, prob ] : prob_sums ) {
            if ( !approx_equal( prob, 1.0 ) )
                return false;
        }
        return true;
    }

    /* if used to represent rewards ( i.e the value in the triplet map is not a
     * probability but arbitrary double ), which is used while parsing .trew
     * files, this returns the highest/lowest double value, this is used when
     * setting reward bounds when constructing the final MDP */
    std::pair< double, double> get_min_max_value() {
        if ( triplets.empty() )
            return { 0, 0 };
        const auto [ min, max ] = std::minmax_element( triplets.begin(), 
                                                       triplets.end(), 
                                                       []( const auto &a, const auto &b ) { return a.second < b.second; } );
        // get associated values
        return std::make_pair( min->second, max->second );
    }

    /* build a 2D matrix out of the triplet map, used when constructing the
     * final model */
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
        result.makeCompressed();
        return result;
    }
};


class PrismParser {

    set[set[int]]

    // map each state to its transitions, later build matrix
    std::map< size_t, TripletList > transition_info;

    // reward structures ( multiple dimensions )
    // reduced to SxA rewards from SxAxS rewards in the match_reward() function
    // since the transition reward files contain SxAxS
    std::vector< TripletList > reward_info;


    bool translate_indices = false;

    /* if translate indices is set, 
     * names ( in the actual files ) of states and actions will be mapped to
     * their respective indices ( first available number ) that will be used
     * when indexing the triplet lists and the resulting MDPs */
    std::map< size_t, size_t> state_to_index;
    std::map< size_t, size_t> action_to_index;

    // line which is currently parsed
    size_t line_num = 0;

    // how many dimensions of reward are currently loaded 
    size_t reward_dimension = 0;

    // line and iterators inside
    std::string line;
    std::string::const_iterator curr, end;
    

    bool eol() const;
    char get_token();

    /* translate state/action name to its index in aforementioned maps */
    size_t translate( const std::string &name, bool state );

    /* helper functions for parsing */
    std::string load_unsigned();
    std::string load_digits();
    double load_float();

    bool ignore_line( const std::string &line );
    size_t string_to_ull( const std::string &input );

    void require( char token );
    void require( int (* callback )( int ));

    bool check( char token );
    bool check( int (* callback)( int ));

    bool remove_all( int (* callback)( int ) );

    
    // match an S,A,S triplet
    std::tuple< size_t, size_t, size_t > match_triplet();

    // match line of transition/reward file
    void match_reward();
    void match_transition();

    void update_expected_reward( size_t s_id, size_t a_id, size_t succ_id,
                                 double reward, size_t idx );

    // only transition rewards supported
    void parse_transition_file( const std::string &filename );
    void parse_reward_file( const std::string &filename );



public:
    // constructs a model using the transition and reward info 
    // stored in this object
    MDP< double > build_model( size_t initial_state );

    // function used to parse and build a model from transition and reward
    // load files -> build_model
    MDP< double > parse_model( const std::string &transition_files,
                               const std::vector< std::string > &reward_files,
                               size_t initial_state );
};


