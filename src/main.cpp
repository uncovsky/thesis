#include "models/mdp.hpp"
#include "solvers/brtdp.hpp"
#include "geometry/polygon.hpp"
#include "models/env_wrapper.hpp"
#include <iostream>
#include "geometry/pareto.hpp"


int main(){

    Matrix3D< double > rewards;
    Matrix3D< double > transitions;


    // make an example mdp with 5 states and two actions total
    for ( size_t i = 0 ; i < 5 ; i++ ) { 
        transitions.push_back( Matrix2D< double >() );
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

    rewards.push_back( Matrix2D< double >( ) );
    rewards.push_back( Matrix2D< double >( ) );

    // first component of reward for action 0/1 in state 0
    rewards[0].insert( 0, 0 ) = 3.0;
    rewards[0].insert( 0, 1 ) = 1.0;

    // second component
    rewards[1].insert( 0, 0 ) = 1.0;
    rewards[1].insert( 0, 1 ) = 1.0;

    // rewards from states 1 and 2 are (1, 1)
    rewards[0].insert( 1, 1 ) = 1.0;
    rewards[1].insert( 1, 0 ) = 1.0;

    rewards[0].insert( 2, 0 ) = 1.0;
    rewards[1].insert( 2, 0 ) = 1.0;

    // terminal states have rewards (0, 1), (1, 0)
    rewards[0].insert( 3, 0 ) = 1.0;
    rewards[1].insert( 3, 0 ) = 0.0;
    rewards[0].insert( 4, 0 ) = 0.0;
    rewards[1].insert( 4, 0 ) = 1.0;

    // third argument is the reward bounds, just placeholders right now
    MDP<double> mdp( transitions,
                  rewards,
                  std::make_pair ( std::vector< double > ( { 0.0, 0.0 } ), std::vector< double > ( { 3.0, 1.0 } ) ),
                  0
                );


    EnvironmentWrapper< size_t, size_t, std::vector<double>, double> env_wrap( &mdp );

    // env, discount params and precision ( unused for now )
    BRTDPSolver brtdp( env_wrap, { 0.75, 0.75 }, 0.005  );
    brtdp.solve( 10, 10 );

}
