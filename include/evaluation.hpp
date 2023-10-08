# pragma once

#include "benchmarks/frozen_lake.hpp"
#include "benchmarks/sea_treasure.hpp"
#include "benchmarks/racetrack.hpp"
#include "benchmarks/resource_gathering.hpp"
#include "models/environment.hpp"
#include "models/env_wrapper.hpp"
#include "solvers/brtdp.hpp"
#include "solvers/chvi.hpp"

#include <fstream>
#include <iostream>
#include <string>

// helper function for evaluation
template < typename state_t, typename action_t, typename value_t >
void run_benchmark( Environment< state_t, action_t, std::vector< value_t > >  *env,
                    const ExplorationSettings< value_t > &config ){


    EnvironmentWrapper< state_t, action_t, std::vector< value_t >, value_t > envw( env );
    EnvironmentWrapper< state_t, action_t, std::vector< value_t >, value_t > chvi_envw( env );

    BRTDPSolver brtdp( std::move( envw ), config );
    CHVIExactSolver chvi( std::move( chvi_envw ), config );

    brtdp.solve();
    chvi.solve();
}


void eval_racetrack( const std::string &dir ){
    
    ExplorationSettings< double > config;

    config.action_heuristic = ActionSelectionHeuristic::Pareto;
    config.state_heuristic = StateSelectionHeuristic::BRTDP;
    config.min_trajectory = 100;
    config.max_trajectory = 5000;
    config.max_episodes = 10000;
    config.discount_params = { 1, 1 };
    config.trace = false;
    config.precision = 1.0;
    config.filename = dir + "racetrack_easy";
    config.lower_bound_init = { -1000, -1000 };
    config.upper_bound_init = { 0, 0 };
    config.lower_bound_init_term = { 0, 0 };
    config.upper_bound_init_term = { 0, 0 };


    Racetrack easy;
    easy.from_file("../benchmarks/race_easy.track");
    run_benchmark( &easy, config );

    config.lower_bound_init = { -1000, -1000 };
    config.upper_bound_init = { -10, -10 };
    config.lower_bound_init_term = { 0, 0 };
    config.upper_bound_init_term = { 0, 0 };

    config.filename = dir + "ring_easy";
    easy.from_file("../benchmarks/ring_easy.track");
    run_benchmark( &easy, config );

}


void eval_treasure( const std::string &dir="" ){

    ExplorationSettings< double > config;
    config.min_trajectory = 100;
    config.action_heuristic = ActionSelectionHeuristic::Pareto;
    config.max_trajectory = 5000;
    config.max_episodes = 10000;
    config.discount_params = { 0.95, 0.95 };
    config.trace = false;
    config.precision = 0.1;

    // at least no treasure and <100 fuel spent
    config.lower_bound_init = { 0, -100 };
    // at most this amount of treasure and <0 fuel lost
    config.upper_bound_init = { 125, 0 };

    config.lower_bound_init_term = { 0, 0 };
    config.upper_bound_init_term = { 0, 0 };

    DeepSeaTreasure dst, dst_convex;
    dst.from_file( "../benchmarks/sea_treasure1.txt" );
    dst_convex.from_file( "../benchmarks/sea_treasure_convex.txt" );

    config.filename = dir + "treasure_concave";
    run_benchmark( &dst, config );
    config.filename = dir + "treasure_convex";
    run_benchmark( &dst_convex, config );
}

void eval_frozenlake( const std::string &dir ) {
    ExplorationSettings< double > config;
    config.min_trajectory = 100;
    config.max_trajectory = 5000;
    config.max_episodes = 10000;
    config.discount_params = { 0.95, 0.95 };
    config.trace = false;
    config.precision = 0.1;
    config.filename = dir + "easy_lake";
    config.lower_bound_init = { 0, -3 };
    config.upper_bound_init = { 1, 0 };
    config.lower_bound_init_term = { 0, 0 };
    config.upper_bound_init_term = { 0, 0 };

    FrozenLake lake;

    run_benchmark( &lake, config );

    PRNG gen;
    std::set< Coordinates > randpits;
    for ( int i = 0; i < 25; ++i ) {
       randpits.insert( Coordinates( gen.rand_int( 1, 25), gen.rand_int( 1, 25 ) ) );
    }

    for ( auto &x : randpits ) {
        std::cout << x << "\n";
    }


    config.discount_params = { 0.99, 0.99 };
    FrozenLake lake2( 30, 30, randpits, 0.3 );
    config.filename = dir + "hard_lake";
    run_benchmark( &lake2, config );
}

void evaluate_benchmarks( const std::string &dir="") {
    eval_racetrack( dir );
    eval_treasure( dir );
    eval_frozenlake( dir );
}
