# pragma once

#include "models/environment.hpp"
#include "models/env_wrapper.hpp"

#include "solvers/brtdp.hpp"
#include "solvers/chvi.hpp"
#include "solvers/config.hpp"

#include "parser.hpp"

#include <fstream>
#include <iostream>
#include <string>

// an example evalutaion on a custom explicit transition file
void eval_example() {

    // construct parser object
    PrismParser parser;

    /* construct sparse mdp model
     * first argument is the transition file, followed by a vector of reward
     * files (up to two)
     *
     * the last argument specifies the id of the starting state, this can be
     * found out by exporting the labels in prism and loooking at the id of the
     * state with the init label */

    auto uav5 = parser.parse_model( "../benchmarks/uav/uav5.tra",
                      {
                      "../benchmarks/uav/uav51.trew",
                      "../benchmarks/uav/uav52.trew"
                      },
                      0 );

    // construct configuration object, see include/solvers/config.hpp
    ExplorationSettings< double > config;

    // enable debug output
    config.trace = true;
    
    // no limit on depth or episodes, time limit of 300 seconds
    config.max_depth = 0;
    config.max_episodes = 0;
    config.max_seconds = 300;

    // epsilon and depth constant
    config.precision = 0.01;
    config.depth_constant = 50;

    // discounting
    config.discount_param = 0.99;

    // empty initial bounds -> automatic initialization
    config.lower_bound_init = {};
    config.upper_bound_init = {};
    
    // use DB heuristic
    config.action_heuristic = ActionSelectionHeuristic::Hausdorff;
    
    // speicify if objectives should be max/min
    config.directions = { OptimizationDirection::MINIMIZE, OptimizationDirection::MINIMIZE };


    // output filename and title for results in csv table
    config.filename = "uav5";

    // runs SBPCA and SCHVI once on this benchmark, using the specified config
    // outputs curve & result data into out/results.csv
    // out/filename_curve-brtdp and out/filename_curve-chvi
    run_benchmark( &uav5, config, 1 );
}
