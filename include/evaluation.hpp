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


void eval_racetrack(){
    
    ExplorationSettings< double > config;
    config.min_trajectory = 100;
    config.max_trajectory = 5000;
    config.max_episodes = 10000;
    config.discount_params = { 1, 1 };
    config.trace = true;
    config.precision = 1.0;
    config.filename = "racetrack_easy";
    config.lower_bound_init = { -1000, -1000 };
    config.upper_bound_init = { 0, 0 };
    config.lower_bound_init_term = { 0, 0 };
    config.upper_bound_init_term = { 0, 0 };


    Racetrack easy;
    easy.from_file("../benchmarks/race_easy.track");
    run_benchmark( &easy, config );

    easy.from_file("../benchmarks/barto-small.track");
    // run_benchmark( &easy, config );


}


void eval_treasure(){

    ExplorationSettings< double > config;
    config.min_trajectory = 100;
    config.action_heuristic = ActionSelectionHeuristic::Pareto;
    config.max_trajectory = 5000;
    config.max_episodes = 10000;
    config.discount_params = { 0.99, 0.99 };
    config.trace = true;
    config.precision = 0.1;
    config.filename = "treasure_concave";

    // at least no treasure and <100 fuel spent
    config.lower_bound_init = { 0, -100 };
    // at most this amount of treasure and <0 fuel lost
    config.upper_bound_init = { 125, 0 };

    config.lower_bound_init_term = { 0, 0 };
    config.upper_bound_init_term = { 0, 0 };

    DeepSeaTreasure dst, dst_convex;
    dst.from_file( "../benchmarks/sea_treasure1.txt" );
    dst_convex.from_file( "../benchmarks/sea_treasure_convex.txt" );

    run_benchmark( &dst, config );
    config.filename = "treasure_convex";
    run_benchmark( &dst_convex, config );
}

void eval_frozenlake() {
    ExplorationSettings< double > config;
    config.min_trajectory = 1000;
    config.max_trajectory = 5000;
    config.max_episodes = 1000;
    config.discount_params = { 0.95, 0.95 };
    config.trace = true;
    config.precision = 0.1;
    config.filename = "easy_lake";
    config.lower_bound_init = { 0, -4 };
    config.upper_bound_init = { 1, 0 };
    config.lower_bound_init_term = { 0, 0 };
    config.upper_bound_init_term = { 0, 0 };

    FrozenLake lake;

    run_benchmark( &lake, config );
}

void evaluate_benchmarks() {
    eval_racetrack();
    eval_treasure();
    eval_frozenlake();
}
