# pragma once

#include "benchmarks/frozen_lake.hpp"
#include "benchmarks/sea_treasure.hpp"
#include "benchmarks/racetrack.hpp"
#include "benchmarks/resource_gathering.hpp"
#include "models/environment.hpp"
#include "models/env_wrapper.hpp"
#include "solvers/brtdp.hpp"
#include "solvers/chvi.hpp"

#include "parser.hpp"

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

void eval_uav( const std::string &dir ){
    PrismParser parser;
    auto MDP = parser.parse_model( "../benchmarks/uav/out.tra",
                      {
                      "../benchmarks/uav/out1.trew",
                      "../benchmarks/uav/out2.trew"
                      },
                      0 );

    auto taskgraph = parser.parse_model( "../benchmarks/pareto_taskgraph/out.tra",
                      {
                      "../benchmarks/pareto_taskgraph/rew1.trew",
                      "../benchmarks/pareto_taskgraph/rew2.trew"
                      },
                      0 );
    ExplorationSettings< double > config;

    config.action_heuristic = ActionSelectionHeuristic::Pareto;
    config.state_heuristic = StateSelectionHeuristic::BRTDP;
    config.max_depth = 5000;
    config.filename = dir + "uav";
    config.max_episodes = 30000;
    config.discount_params = { 0.99, 0.99 };
    config.trace = false;
    config.precision = 1e-5;
    config.lower_bound_init = { -20, -20 };
    config.upper_bound_init = { 0, 0 };
    config.lower_bound_init_term = {};
    config.upper_bound_init_term = {};

    run_benchmark( &MDP, config );
    config.filename = dir + "taskgraph";
    run_benchmark( &taskgraph, config );
}

void eval_racetrack( const std::string &dir ){
    
    ExplorationSettings< double > config;

    config.action_heuristic = ActionSelectionHeuristic::Pareto;
    config.state_heuristic = StateSelectionHeuristic::BRTDP;
    config.max_depth = 5000;
    config.max_episodes = 1000;
    config.discount_params = { 1, 1 };
    config.trace = false;
    config.precision = 1e-4;
    config.lower_bound_init = { -1000, -1000 };
    config.upper_bound_init = { 0, 0 };
    config.lower_bound_init_term = { 0, 0 };
    config.upper_bound_init_term = { 0, 0 };


    Racetrack easy;
    config.filename = dir + "racetrack-easy";
    config.trace = true;
    easy.from_file("../benchmarks/racetracks/racetrack-easy.track");
    run_benchmark( &easy, config );


    config.lower_bound_init = { -1000, -1000 };
    config.upper_bound_init = { -10, -10 };
    config.filename = dir + "racetrack-hard";
    easy.from_file("../benchmarks/racetracks/racetrack-hard.track");
    run_benchmark( &easy, config );

    config.lower_bound_init = { -1000, -1000 };
    config.upper_bound_init = { 0, 0 };
    config.filename = dir + "racetrack-ring";
    easy.from_file("../benchmarks/racetracks/racetrack-ring.track");
    run_benchmark( &easy, config );
}


void eval_treasure( const std::string &dir="" ){

    ExplorationSettings< double > config;
    config.action_heuristic = ActionSelectionHeuristic::Pareto;
    config.max_depth = 5000;
    config.max_episodes = 10000;
    config.discount_params = { 0.99, 0.99 };
    config.trace = true;
    config.precision = 0.1;

    // at least no treasure and <100 fuel spent
    config.lower_bound_init = { 0, -100 };
    // at most this amount of treasure and <0 fuel lost
    config.upper_bound_init = { 125, 0 };

    /*
    config.lower_bound_init_term = { 0, 0 };
    config.upper_bound_init_term = { 0, 0 };
    */

    DeepSeaTreasure dst, dst_convex;
    dst.from_file( "../benchmarks/treasures/treasure-concave.txt" );
    dst_convex.from_file( "../benchmarks/treasures/treasure-convex.txt" );

    config.filename = dir + "treasure-concave";
    run_benchmark( &dst, config );
    config.filename = dir + "treasure-convex";
    run_benchmark( &dst_convex, config );
}

void eval_frozenlake( const std::string &dir ) {
    ExplorationSettings< double > config;
    config.max_depth = 5000;
    config.max_episodes = 10000;
    config.discount_params = { 0.95, 0.95 };
    config.trace = false;
    config.precision = 0.1;
    config.filename = dir + "lake-easy";
    config.lower_bound_init = { 0, -3 };
    config.upper_bound_init = { 1, 0 };
    config.lower_bound_init_term = { 0, 0 };
    config.upper_bound_init_term = { 0, 0 };

    FrozenLake lake;

    run_benchmark( &lake, config );

    PRNG gen;
    std::set< Coordinates > randpits;
    config.discount_params = { 0.99, 0.99 };
    FrozenLake lake2( 25, 25, {
                       Coordinates(1, 5),
                       Coordinates(1, 8),
                       Coordinates(1, 12),
                       Coordinates(2, 16),
                       Coordinates(3, 21),
                       Coordinates(5, 10),
                       Coordinates(6, 4),
                       Coordinates(6, 24),
                       Coordinates(7, 4),
                       Coordinates(9, 2),
                       Coordinates(9, 15),
                       Coordinates(10, 4),
                       Coordinates(13, 7),
                       Coordinates(15, 22),
                       Coordinates(16, 6),
                       Coordinates(16, 10),
                       Coordinates(19, 5),
                       Coordinates(19, 7),
                       Coordinates(19, 24),
                       Coordinates(20, 8),
                       Coordinates(22, 9),
                       Coordinates(23, 1),
                       Coordinates(23, 19),
                       Coordinates(24, 21),
                       Coordinates(24, 8)
            }, 0.3 );
    config.filename = dir + "lake-hard";
    run_benchmark( &lake2, config );
}

void evaluate_benchmarks( const std::string &dir="") {
    eval_uav( dir );
    eval_treasure( dir );
    eval_racetrack( dir );
    eval_frozenlake( dir );
}
