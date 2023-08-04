# pragma once

# include "models/env_wrapper.hpp"
# include "models/mdp.hpp"
# include "solvers/brtdp.hpp"
# include "utils/eigen_types.hpp"

# include <cassert>
# include <set>

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
    BRTDPSolver brtdp( std::move( env_wrap ) , { 0.75, 0.75 } );
    Bounds< double > result = brtdp.solve( 1e-12 );

   assert( approx_set_equality( result.lower().get_vertices(), std::vector< std::vector< double > > ( { { 4.875, 2.875 }, { 1.75, 4 } } ) ) );

    EnvironmentWrapper< size_t, size_t, std::vector<double>, double> env_wrap2( &mdp );
    BRTDPSolver brtdp2( std::move( env_wrap2 ) , { 0, 0 } );
    result = brtdp2.solve( 1e-12 );

    assert( approx_set_equality( result.lower().get_vertices(), std::vector< std::vector< double > > ( { { 3, 1 } } ) ) );
    auto mdp2 = test2();
    EnvironmentWrapper< size_t, size_t, std::vector<double>, double> env_wrap3( &mdp2 );
    BRTDPSolver brtdp3( std::move( env_wrap3 ) , { 0.5, 0.5 } );
    result = brtdp3.solve( 0.2 );
}
