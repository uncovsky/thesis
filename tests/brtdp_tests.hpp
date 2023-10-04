# pragma once

# include "models/env_wrapper.hpp"
# include "models/mdp.hpp"
# include "solvers/brtdp.hpp"
# include "solvers/config.hpp"
# include "utils/eigen_types.hpp"
# include "parser.hpp"

# include <cassert>
# include <set>

void test_brtdp( const std::string &transition_file,
                 const std::vector< std::string > &reward_files,
                 std::vector< double > discount_params,
                 size_t starting_state=0,
                 double precision=0.1 ) {

    // construct parser object
    PrismParser parser;

    // parse the provided files
    auto mdp = parser.parse_model( transition_file,
                                   reward_files,
                                   starting_state );

    // construct a wrapper object around the mdp ( initializes the object
    // bounds, and the solver uses it to interact with the model )
    EnvironmentWrapper< size_t, size_t, std::vector<double>, double> env_wrap( &mdp );
    ExplorationSettings< double > config;

    config.discount_params = discount_params;
    config.precision = precision;

    // construct the solver object using the wrapper and discount parameters
    BRTDPSolver brtdp( std::move( env_wrap ), config );

    // solve up to given precision
    auto start_bound = brtdp.solve();

    // output the lower/upper bounds of the starting state
    std::cout << start_bound;
}


MDP< double > build_simple_mdp( ) {

    Matrix3D< double > rewards;
    Matrix3D< double > transitions;


    // make an example mdp with 5 states and two actions total
    for ( size_t i = 0 ; i < 5 ; i++ ) { 
        transitions.push_back( Matrix2D< double >(2, 5) );
    }

    // actions 0 and 1 detrministically change state to 1/2
    transitions[0].insert(0, 1) = 1.0;
    transitions[0].insert(1, 2) = 1.0;

    // action zero from state 1 changes either to 3/4 with 0.5 prob
    transitions[1].insert(0, 3) = 0.5;
    transitions[1].insert(0, 4) = 0.5;

    transitions[2].insert(0, 4) = 1.0;

    // terminal states 3 & 4
    transitions[3].insert(0, 3) = 1.0;
    transitions[4].insert(0, 4) = 1.0;

    rewards.push_back( Matrix2D< double >(2, 5) );
    rewards.push_back( Matrix2D< double >(2, 5) );

    // first component of reward for action 0/1 in state 0
    rewards[0].insert( 0, 0 ) = 3.0;
    rewards[0].insert( 1, 0 ) = 1.0;

    // second component
    rewards[1].insert( 0, 0 ) = 1.0;
    rewards[1].insert( 1, 0 ) = 1.0;

    // rewards from states 1 and 2 are (1, 1)
    rewards[0].insert( 0, 1 ) = 1.0;
    rewards[1].insert( 0, 1 ) = 1.0;

    rewards[0].insert( 0, 2 ) = 1.0;
    rewards[1].insert( 0, 2 ) = 1.0;

    // terminal states have rewards (0, 1), (1, 0)
    rewards[0].insert( 0, 3 ) = 1.0;
    rewards[1].insert( 0, 3 ) = 0.0;
    rewards[0].insert( 0, 4 ) = 0.0;
    rewards[1].insert( 0, 4 ) = 1.0;

    // third argument is the reward bounds
    MDP<double> mdp( transitions,
                     rewards,
                     std::make_pair ( std::vector< double > ( { 0.0, 0.0 } ), std::vector< double > ( { 3.0, 1.0 } ) ),
                    0
                );
    return mdp;

}


MDP< double > test2 () {

    Matrix3D< double > rewards;
    Matrix3D< double > transitions;

    for ( size_t i = 0 ; i < 2 ; i++ ) { 
        transitions.push_back( Matrix2D< double >(2, 2) );
    }

    transitions[0].insert(0, 0) = 1.0;
    transitions[0].insert(1, 1) = 1.0;

    transitions[1].insert(0, 0) = 1.0;
    transitions[1].insert(1, 1) = 1.0;


    rewards.push_back( Matrix2D< double >(2, 2) );
    rewards.push_back( Matrix2D< double >(2, 2) );

    rewards[0].insert( 0, 0 ) = 1.0;
    rewards[0].insert( 0, 1 ) = 1.0;
    rewards[0].insert( 1, 0 ) = 0.0;
    rewards[0].insert( 1, 1 ) = 0.0;

    rewards[1].insert( 0, 0 ) = 0.0;
    rewards[1].insert( 0, 1 ) = 0.0;
    rewards[1].insert( 1, 0 ) = 1.0;
    rewards[1].insert( 1, 1 ) = 1.0;


    MDP<double> mdp( transitions,
                     rewards,
                     std::make_pair ( std::vector< double > ( { 0.0, 0.0 } ), std::vector< double > ( { 1.0, 1.0 } ) ),
                    0
                );
    return mdp;
}


template < typename value_t >
bool approx_set_inclusion( const std::vector< std::vector< value_t > > &lhs, 
                           const std::vector< std::vector< value_t > > &rhs ) {

    for ( const auto &point : lhs ) {
        bool has_counterpart = false;
        for ( const auto &other : rhs ) {
            // assuming sets only contain points of same dimension
            bool matched = true;

            if ( point.size() != other.size() )
                return false;

            // is approx equal across all components 
            for ( size_t i = 0; i < point.size(); i++) {
                matched &= approx_equal( point[i], other[i] );
            }

            if ( matched )
                has_counterpart = true;
        }

        if ( !has_counterpart ) {
            std::cout << point[0] << point[1] << std::endl;
            return false;
        }
    }

    return true;
}


template < typename value_t > 
bool approx_set_equality( const std::vector< std::vector< value_t > > &lhs, 
                          const std::vector< std::vector< value_t > > &rhs ) {
    return approx_set_inclusion( lhs, rhs ) && approx_set_inclusion( rhs, lhs );
}



void test_brtdp_on_simple_mdp() {
    auto mdp = build_simple_mdp();
    EnvironmentWrapper< size_t, size_t, std::vector<double>, double> env_wrap( &mdp );

    // env, discount params and precision ( unused for now )
    //
    ExplorationSettings< double > config;
    config.precision = 1e-12;
    config.discount_params = { 0.75, 0.75 };

    BRTDPSolver brtdp( std::move( env_wrap ) , config );
    Bounds< double > result = brtdp.solve();

   assert( approx_set_equality( result.lower().get_vertices(), std::vector< std::vector< double > > ( { { 4.875, 2.875 }, { 1.75, 4 } } ) ) );

    assert( approx_set_equality( result.lower().get_vertices(), std::vector< std::vector< double > > ( { { 3, 1 } } ) ) );
    auto mdp2 = test2();

    config.precision = 0.01;
    config.discount_params = { 0.85, 0.85 };

    EnvironmentWrapper< size_t, size_t, std::vector<double>, double> env_wrap3( &mdp2 );
    BRTDPSolver brtdp3( std::move( env_wrap3 ) , config );
    result = brtdp3.solve();

    // short 1d test
    test_brtdp( "../tests/parser_files/model1.tra",  // transition file
                { 
                  "../tests/parser_files/model1.trew",  // reward files
                }, 
                { 0.5 }, // discount params
                0,  // id of starting state
                0.01 // precision
                  );
}
